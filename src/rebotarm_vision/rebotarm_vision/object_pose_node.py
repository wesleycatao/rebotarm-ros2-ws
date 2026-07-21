from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rebotarm_msgs.msg import SegmentedObject, SegmentedObjectArray

from .geometry import apply_homography, clamp, matrix3, yaw_to_quaternion


class ObjectPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_object_pose")
        dynamic_list = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("input_topic", "/rebotarm/vision/objects_2d")
        self.declare_parameter("output_topic", "/rebotarm/vision/objects")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("image_height", 480.0)
        self.declare_parameter("use_workspace_homography", False)
        self.declare_parameter("calibration_pixel_points", [], dynamic_list)
        self.declare_parameter("calibration_xy_points", [], dynamic_list)
        self.declare_parameter(
            "workspace_pixel_vertices",
            [0.0, 0.0, 639.0, 0.0, 639.0, 479.0, 0.0, 479.0],
        )
        self.declare_parameter(
            "workspace_xy_vertices",
            [0.42, 0.20, 0.42, -0.20, 0.18, -0.20, 0.18, 0.20],
        )
        self.declare_parameter("pixel_to_base_homography", [0.0] * 9)
        self.declare_parameter("base_x_from_image_v", [0.42, 0.18])
        self.declare_parameter("base_y_from_image_u", [0.20, -0.20])
        self.declare_parameter("table_z", 0.0)
        self.declare_parameter("default_object_height", 0.035)
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("x_limits", [0.16, 0.46])
        self.declare_parameter("y_limits", [-0.24, 0.24])
        self.declare_parameter("z_limits", [-0.02, 0.20])
        self.declare_parameter("use_operation_limits", False)
        self.declare_parameter("operation_limit_shape", "polygon")
        self.declare_parameter("operation_limit_xy_vertices", [], dynamic_list)
        self.declare_parameter("operation_limit_circle", [0.30, 0.0, 0.18])
        self.declare_parameter("operation_limit_margin_m", 0.0)
        self.declare_parameter("edge_margin_px", 18.0)
        self.declare_parameter("pick_roll", 0.0)
        self.declare_parameter("pick_pitch", 1.5708)
        self.declare_parameter("yaw_offset", 0.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(SegmentedObjectArray, output_topic, 10)
        self._current_image_width = float(self.get_parameter("image_width").value)
        self._current_image_height = float(self.get_parameter("image_height").value)
        self.create_subscription(SegmentedObjectArray, input_topic, self._on_objects, 10)
        self.get_logger().info(f"projecting {input_topic} into {output_topic}")

    def _on_objects(self, message: SegmentedObjectArray) -> None:
        self._current_image_width, self._current_image_height = self._image_size_from_message(message)
        output = SegmentedObjectArray()
        output.header = message.header
        output.header.frame_id = str(self.get_parameter("target_frame").value)
        output.image_width = float(self._current_image_width)
        output.image_height = float(self._current_image_height)
        output.objects = [self._project(obj) for obj in message.objects]
        output.objects.sort(key=lambda item: item.score, reverse=True)
        self._publisher.publish(output)

    def _image_size_from_message(self, message: SegmentedObjectArray) -> tuple[float, float]:
        width = float(getattr(message, "image_width", 0.0))
        height = float(getattr(message, "image_height", 0.0))
        if width > 0.0 and height > 0.0:
            return width, height
        return (
            float(self.get_parameter("image_width").value),
            float(self.get_parameter("image_height").value),
        )

    def _project(self, source: SegmentedObject) -> SegmentedObject:
        obj = SegmentedObject()
        obj.header = source.header
        obj.header.frame_id = str(self.get_parameter("target_frame").value)
        obj.object_id = source.object_id
        obj.class_id = source.class_id
        obj.class_name = source.class_name
        obj.confidence = source.confidence
        obj.center_pixel = source.center_pixel
        obj.area_px = source.area_px
        obj.yaw = source.yaw + float(self.get_parameter("yaw_offset").value)
        obj.axis_yaw = source.axis_yaw + float(self.get_parameter("yaw_offset").value)
        obj.gripper_yaw = obj.yaw
        obj.direction_sign = source.direction_sign
        obj.direction_sense = source.direction_sense
        obj.direction_start_pixel = source.direction_start_pixel
        obj.direction_end_pixel = source.direction_end_pixel
        obj.object_number = source.object_number
        obj.bounding_box_pixels = source.bounding_box_pixels
        obj.clearance_left_px = source.clearance_left_px
        obj.clearance_right_px = source.clearance_right_px
        obj.clearance_up_px = source.clearance_up_px
        obj.clearance_down_px = source.clearance_down_px
        obj.contour_pixels = source.contour_pixels
        obj.orientation = source.orientation
        obj.grasp_zone_pixels = source.grasp_zone_pixels
        obj.grasp_inset_zone_pixels = source.grasp_inset_zone_pixels
        obj.preferred_grasp_pixel = source.preferred_grasp_pixel
        obj.grasp_inset_ratio = source.grasp_inset_ratio
        obj.pickable = source.pickable
        obj.pickable_reason = source.pickable_reason
        obj.gripper_width_estimate = source.gripper_width_estimate

        projected = self._pixel_to_base(source.center_pixel.x, source.center_pixel.y)
        object_height = float(self.get_parameter("default_object_height").value)
        if projected is None:
            obj.reachable = False
            obj.reject_reason = "projection_failed"
            obj.score = 0.0
            return obj

        x, y = projected
        z = float(self.get_parameter("table_z").value) + object_height * 0.5
        obj.pose.position = Point(x=float(x), y=float(y), z=float(z))
        obj.pose.orientation = yaw_to_quaternion(
            obj.gripper_yaw,
            float(self.get_parameter("pick_roll").value),
            float(self.get_parameter("pick_pitch").value),
        )
        obj.dimensions = self._estimate_dimensions(source, object_height)
        obj.pick_width = float(min(obj.dimensions.x, obj.dimensions.y))

        reachable, reason = self._reachable(obj)
        obj.reachable = reachable
        obj.reject_reason = reason
        obj.score = self._score(source, obj) if reachable else 0.0
        return obj

    def _pixel_to_base(self, u: float, v: float) -> tuple[float, float] | None:
        if bool(self.get_parameter("use_workspace_homography").value):
            homography = self._workspace_homography()
            if homography is not None:
                return apply_homography(homography, u, v)

        homography = matrix3(self.get_parameter("pixel_to_base_homography").value)
        if homography and any(abs(value) > 1e-12 for value in homography):
            return apply_homography(homography, u, v)

        width, height = self._current_image_width, self._current_image_height
        x_top, x_bottom = [float(value) for value in self._parameter_list("base_x_from_image_v")]
        y_left, y_right = [float(value) for value in self._parameter_list("base_y_from_image_u")]
        u_ratio = clamp(u / max(width - 1.0, 1.0), 0.0, 1.0)
        v_ratio = clamp(v / max(height - 1.0, 1.0), 0.0, 1.0)
        x = x_top + (x_bottom - x_top) * v_ratio
        y = y_left + (y_right - y_left) * u_ratio
        return x, y

    def _workspace_homography(self) -> list[float] | None:
        calibration = self._calibration_homography()
        if calibration is not None:
            return calibration

        pixels = [float(value) for value in self._parameter_list("workspace_pixel_vertices")]
        xy = [float(value) for value in self._parameter_list("workspace_xy_vertices")]
        if len(pixels) != 8 or len(xy) != 8:
            self.get_logger().warn("workspace homography expects 4 pixel vertices and 4 XY vertices", throttle_duration_sec=5.0)
            return None
        src = np.array(pixels, dtype=np.float32).reshape((4, 2))
        dst = np.array(xy, dtype=np.float32).reshape((4, 2))
        try:
            homography = cv2.getPerspectiveTransform(src, dst)
        except cv2.error as exc:
            self.get_logger().warn(f"workspace homography failed: {exc}", throttle_duration_sec=5.0)
            return None
        return [float(value) for value in homography.reshape(-1)]

    def _calibration_homography(self) -> list[float] | None:
        pixels = [float(value) for value in self._parameter_list("calibration_pixel_points")]
        xy = [float(value) for value in self._parameter_list("calibration_xy_points")]
        if len(pixels) < 6 or len(xy) < 6:
            return None
        pair_count = min(len(pixels), len(xy)) // 2
        pair_count = min(pair_count, 4)
        if pair_count < 3:
            return None
        src = np.array(pixels[: pair_count * 2], dtype=np.float32).reshape((pair_count, 2))
        dst = np.array(xy[: pair_count * 2], dtype=np.float32).reshape((pair_count, 2))
        try:
            if pair_count == 3:
                affine = cv2.getAffineTransform(src, dst)
                homography = np.vstack([affine, np.array([0.0, 0.0, 1.0], dtype=np.float32)])
            elif pair_count == 4:
                homography = cv2.getPerspectiveTransform(src, dst)
            else:
                found, _mask = cv2.findHomography(src, dst, method=0)
                homography = found
        except cv2.error as exc:
            self.get_logger().warn(f"calibration transform failed: {exc}", throttle_duration_sec=5.0)
            return None
        if homography is None:
            self.get_logger().warn("calibration transform failed", throttle_duration_sec=5.0)
            return None
        return [float(value) for value in homography.reshape(-1)]

    def _estimate_dimensions(self, obj: SegmentedObject, object_height: float) -> Vector3:
        if bool(self.get_parameter("use_workspace_homography").value) and obj.bounding_box_pixels:
            projected = [
                self._pixel_to_base(point.x, point.y)
                for point in obj.bounding_box_pixels
            ]
            if all(point is not None for point in projected):
                xs = [float(point[0]) for point in projected if point is not None]
                ys = [float(point[1]) for point in projected if point is not None]
                return Vector3(
                    x=max(0.01, max(xs) - min(xs)),
                    y=max(0.01, max(ys) - min(ys)),
                    z=float(object_height),
                )

        width, height = self._current_image_width, self._current_image_height
        x_top, x_bottom = [float(value) for value in self._parameter_list("base_x_from_image_v")]
        y_left, y_right = [float(value) for value in self._parameter_list("base_y_from_image_u")]
        meters_per_px_x = abs(x_bottom - x_top) / max(height, 1.0)
        meters_per_px_y = abs(y_right - y_left) / max(width, 1.0)

        if obj.contour_pixels:
            xs = [point.x for point in obj.contour_pixels]
            ys = [point.y for point in obj.contour_pixels]
            size_x = max(0.01, (max(ys) - min(ys)) * meters_per_px_x)
            size_y = max(0.01, (max(xs) - min(xs)) * meters_per_px_y)
        else:
            side = math.sqrt(max(float(obj.area_px), 1.0))
            size_x = max(0.01, side * meters_per_px_x)
            size_y = max(0.01, side * meters_per_px_y)
        return Vector3(x=float(size_x), y=float(size_y), z=float(object_height))

    def _reachable(self, obj: SegmentedObject) -> tuple[bool, str]:
        min_confidence = float(self.get_parameter("min_confidence").value)
        if obj.confidence < min_confidence:
            return False, "low_confidence"

        x_limits = [float(value) for value in self._parameter_list("x_limits")]
        y_limits = [float(value) for value in self._parameter_list("y_limits")]
        z_limits = [float(value) for value in self._parameter_list("z_limits")]
        p = obj.pose.position
        if not (x_limits[0] <= p.x <= x_limits[1]):
            return False, "x_out_of_bounds"
        if not (y_limits[0] <= p.y <= y_limits[1]):
            return False, "y_out_of_bounds"
        if not (z_limits[0] <= p.z <= z_limits[1]):
            return False, "z_out_of_bounds"
        if bool(self.get_parameter("use_operation_limits").value) and not self._inside_operation_limits(p.x, p.y):
            return False, "operation_limit"
        return True, ""

    def _inside_operation_limits(self, x: float, y: float) -> bool:
        shape = str(self.get_parameter("operation_limit_shape").value).lower()
        margin = max(0.0, float(self.get_parameter("operation_limit_margin_m").value))
        if shape == "circle":
            circle = [float(value) for value in self._parameter_list("operation_limit_circle")]
            if len(circle) < 3:
                return True
            cx, cy, radius = circle[:3]
            return math.hypot(float(x) - cx, float(y) - cy) <= max(radius - margin, 0.0)

        values = [float(value) for value in self._parameter_list("operation_limit_xy_vertices")]
        if len(values) < 6:
            return True
        if len(values) % 2 == 1:
            values = values[:-1]
        polygon = np.array(values, dtype=np.float32).reshape((-1, 2))
        distance = cv2.pointPolygonTest(polygon, (float(x), float(y)), True)
        return distance >= margin

    def _score(self, source: SegmentedObject, projected: SegmentedObject) -> float:
        width, height = self._current_image_width, self._current_image_height
        edge_margin = float(self.get_parameter("edge_margin_px").value)
        u = float(source.center_pixel.x)
        v = float(source.center_pixel.y)
        distance_to_edge = min(u, v, width - u, height - v)
        edge_score = clamp(distance_to_edge / max(edge_margin, 1.0), 0.0, 1.0)
        area_score = clamp(float(source.area_px) / max(width * height * 0.05, 1.0), 0.0, 1.0)
        p = projected.pose.position
        reach_center = 1.0 - clamp(abs(p.y) / 0.24, 0.0, 1.0) * 0.35
        return float(source.confidence * 0.55 + edge_score * 0.20 + area_score * 0.15 + reach_center * 0.10)

    def _parameter_list(self, name: str) -> list:
        value = self.get_parameter(name).value
        if value is None:
            return []
        return list(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
