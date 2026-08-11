from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rebotarm_msgs.msg import SegmentedObject, SegmentedObjectArray
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from .geometry import (
    clamp,
    local_to_image_point,
    normalize_angle,
    polygon_area,
    rotated_iou,
    shrink_polygon,
    stable_id,
)
from .image_tools import bgr_to_image_msg, image_msg_to_bgr
from .vision_profile import CalibratedProfile, VisionProfile, profile_to_segmentation_params


@dataclass(frozen=True)
class OrientedBox:
    center_x: float
    center_y: float
    long_axis: tuple[float, float]
    short_axis: tuple[float, float]
    half_long: float
    half_short: float
    min_long: float
    max_long: float
    min_short: float
    max_short: float
    corners: tuple[tuple[float, float], ...]


class SegmentationNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_segmentation")
        dynamic_list = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("objects_topic", "/rebotarm/vision/objects_2d")
        self.declare_parameter("debug_image_topic", "/rebotarm/vision/debug_image")
        self.declare_parameter("process_service", "/rebotarm_segmentation/process_frame")
        self.declare_parameter("process_mode", "manual")
        self.declare_parameter("max_processing_fps", 1.0)
        self.declare_parameter("use_orientation_profiles", False)
        self.declare_parameter("vertical_profile_json", "{}")
        self.declare_parameter("horizontal_profile_json", "{}")
        self.declare_parameter("calibrated_profile_json", "{}")
        self.declare_parameter("gripper_width_px", 0.0)
        self.declare_parameter("grasp_inset_ratio", 0.10)
        self.declare_parameter("backend", "auto")
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("image_size", 416)
        self.declare_parameter("target_classes", [], dynamic_list)
        self.declare_parameter("auxiliary_attribute", "")
        self.declare_parameter("min_area_px", 500.0)
        self.declare_parameter("max_area_px", 0.0)
        self.declare_parameter("max_area_ratio", 0.50)
        self.declare_parameter("min_bbox_long_side_px", 0.0)
        self.declare_parameter("min_bbox_short_side_px", 0.0)
        self.declare_parameter("max_bbox_long_side_px", 0.0)
        self.declare_parameter("max_bbox_short_side_px", 0.0)
        self.declare_parameter("max_contour_points", 48)
        self.declare_parameter("opencv_mode", "hsv")
        self.declare_parameter("center_method", "moments")
        self.declare_parameter("direction_method", "min_area_rect")
        self.declare_parameter("direction_sense_method", "none")
        self.declare_parameter("shape_filter", "any")
        self.declare_parameter("shape_epsilon_ratio", 0.035)
        self.declare_parameter("circle_min_circularity", 0.72)
        self.declare_parameter("square_aspect_tolerance", 0.22)
        self.declare_parameter("lying_aspect_ratio_threshold", 1.8)
        self.declare_parameter("draw_clearance", True)
        self.declare_parameter("use_workspace_mask", False)
        self.declare_parameter("draw_workspace_limit", True)
        self.declare_parameter("workspace_limit_shape", "polygon")
        self.declare_parameter("workspace_limit_pixels", [], dynamic_list)
        self.declare_parameter("workspace_circle_pixel", [320.0, 240.0, 180.0])
        self.declare_parameter("workspace_mask_margin_px", 0.0)
        self.declare_parameter("hsv_lower", [0, 40, 40])
        self.declare_parameter("hsv_upper", [179, 255, 255])
        self.declare_parameter("rgb_lower", [0, 0, 0])
        self.declare_parameter("rgb_upper", [255, 255, 255])
        self.declare_parameter("brightness_threshold", 110)
        self.declare_parameter("blur_kernel", 5)
        self.declare_parameter("brightness_invert", False)
        self.declare_parameter("canny_low", 45)
        self.declare_parameter("canny_high", 130)
        self.declare_parameter("edge_aperture_size", 3)
        self.declare_parameter("edge_dilate_iterations", 1)
        self.declare_parameter("edge_erode_iterations", 0)
        self.declare_parameter("roi_margin_px", 0)
        self.declare_parameter("morph_kernel", 5)
        self.declare_parameter("morph_open_iterations", 1)
        self.declare_parameter("morph_close_iterations", 1)
        self.declare_parameter("min_solidity", 0.0)
        self.declare_parameter("min_extent", 0.0)
        self.declare_parameter("max_extent", 1.0)
        self.declare_parameter("min_aspect_ratio", 0.0)
        self.declare_parameter("max_aspect_ratio", 0.0)

        image_topic = str(self.get_parameter("image_topic").value)
        objects_topic = str(self.get_parameter("objects_topic").value)
        debug_topic = str(self.get_parameter("debug_image_topic").value)
        process_service = str(self.get_parameter("process_service").value)

        self._model = None
        self._model_names: dict[int, str] = {}
        self._latest_frame: np.ndarray | None = None
        self._latest_header = None
        self._last_process_time = 0.0
        self._profile_overrides: dict[str, Any] | None = None
        self._load_model()
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        self._objects_pub = self.create_publisher(SegmentedObjectArray, objects_topic, 10)
        self._debug_pub = self.create_publisher(Image, debug_topic, 10)
        self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_service(Trigger, process_service, self._on_process_frame)
        self.get_logger().info(
            f"segmentation listening on {image_topic}, publishing {objects_topic}"
        )
        self.get_logger().info(f"manual trigger service: {process_service}")

    def _load_model(
        self,
        backend: str | None = None,
        model_path: str | None = None,
        *,
        strict: bool = False,
    ) -> tuple[bool, str]:
        backend = str(
            self.get_parameter("backend").value if backend is None else backend
        ).lower()
        model_path = str(
            self.get_parameter("model_path").value if model_path is None else model_path
        ).strip()
        if backend == "opencv" or not model_path:
            if strict and backend == "yolo" and not model_path:
                return False, "model_path é obrigatório para o backend YOLO"
            self._model = None
            self._model_names = {}
            mode = str(self.get_parameter("opencv_mode").value)
            self.get_logger().info(f"using OpenCV {mode} contour segmentation")
            return True, "backend OpenCV ativo"
        try:
            from ultralytics import YOLO  # type: ignore

            model = YOLO(model_path)
            names = getattr(model, "names", {})
            model_names = {int(k): str(v) for k, v in dict(names).items()}
            self._model = model
            self._model_names = model_names
            self.get_logger().info(f"loaded YOLO segmentation model: {model_path}")
            return True, f"modelo YOLO carregado: {model_path}"
        except Exception as exc:
            if backend == "yolo":
                self.get_logger().error(f"failed to load YOLO model: {exc}")
            if strict:
                return False, f"falha ao carregar modelo YOLO: {exc}"
            self.get_logger().warning("falling back to OpenCV contour segmentation")
            self._model = None
            self._model_names = {}
            return True, f"fallback OpenCV: {exc}"

    def _on_parameters_changed(self, parameters) -> SetParametersResult:
        """Reload YOLO atomically when its runtime parameters are changed."""
        changed = {parameter.name: parameter.value for parameter in parameters}
        if not {"backend", "model_path"}.intersection(changed):
            return SetParametersResult(successful=True)
        backend = str(changed.get("backend", self.get_parameter("backend").value)).lower()
        model_path = str(changed.get("model_path", self.get_parameter("model_path").value))
        if backend not in {"auto", "opencv", "yolo"}:
            return SetParametersResult(successful=False, reason=f"backend inválido: {backend}")
        success, reason = self._load_model(backend, model_path, strict=True)
        return SetParametersResult(successful=success, reason="" if success else reason)

    def _on_image(self, message: Image) -> None:
        try:
            frame = image_msg_to_bgr(message)
        except Exception as exc:
            self.get_logger().warning(f"failed to decode image: {exc}", throttle_duration_sec=2.0)
            return

        self._latest_frame = frame
        self._latest_header = message.header
        if str(self.get_parameter("process_mode").value).lower() != "continuous":
            return
        if not self._processing_interval_elapsed():
            return
        self._process_frame(frame, message.header)

    def _on_process_frame(self, _request, response):
        if self._latest_frame is None or self._latest_header is None:
            response.success = False
            response.message = "nenhum frame recebido ainda"
            return response
        count = self._process_frame(self._latest_frame, self._latest_header)
        response.success = True
        response.message = f"frame processado; objetos={count}"
        return response

    def _processing_interval_elapsed(self) -> bool:
        max_fps = max(0.0, float(self.get_parameter("max_processing_fps").value))
        if max_fps <= 0.0:
            return True
        now = float(self.get_clock().now().nanoseconds) * 1e-9
        if now - self._last_process_time < (1.0 / max_fps):
            return False
        self._last_process_time = now
        return True

    def _process_frame(self, frame: np.ndarray, header) -> int:
        backend = str(self.get_parameter("backend").value).lower()
        if backend == "opencv" or self._model is None:
            objects, debug = self._segment_with_opencv(frame, header)
        else:
            objects, debug = self._segment_with_yolo(frame, header)

        array = SegmentedObjectArray()
        array.header = header
        array.image_width = float(frame.shape[1])
        array.image_height = float(frame.shape[0])
        array.objects = objects
        self._objects_pub.publish(array)
        self._debug_pub.publish(bgr_to_image_msg(debug, header))
        return len(objects)

    def _segment_with_yolo(self, frame: np.ndarray, header) -> tuple[list[SegmentedObject], np.ndarray]:
        confidence = float(self.get_parameter("confidence_threshold").value)
        image_size = int(self.get_parameter("image_size").value)
        try:
            results = self._model(frame, imgsz=image_size, conf=confidence, verbose=False)
        except Exception as exc:
            self.get_logger().warning(f"YOLO inference failed: {exc}", throttle_duration_sec=2.0)
            return self._segment_with_opencv(frame, header)

        objects: list[SegmentedObject] = []
        debug = frame.copy()
        if not results:
            return objects, debug
        result = results[0]
        masks = getattr(result, "masks", None)
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return objects, debug

        names = getattr(result, "names", self._model_names)
        classes = boxes.cls.detach().cpu().numpy() if hasattr(boxes.cls, "detach") else np.asarray(boxes.cls)
        confidences = boxes.conf.detach().cpu().numpy() if hasattr(boxes.conf, "detach") else np.asarray(boxes.conf)
        box_xyxy = boxes.xyxy.detach().cpu().numpy() if hasattr(boxes.xyxy, "detach") else np.asarray(boxes.xyxy)

        # Detection-only model: build binary masks from the axis-aligned bounding boxes.
        if masks is None:
            mask_data = self._masks_from_boxes(box_xyxy, frame.shape[:2])
        else:
            raw_masks = masks.data.detach().cpu().numpy() if hasattr(masks.data, "detach") else np.asarray(masks.data)
            mask_data = []
            for raw_mask in raw_masks:
                mask = (raw_mask > 0.5).astype(np.uint8)
                if mask.shape[:2] != frame.shape[:2]:
                    mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
                mask_data.append(mask)

        for index, mask in enumerate(mask_data):
            class_id = int(classes[index]) if index < len(classes) else -1
            class_name = str(names.get(class_id, f"class_{class_id}")) if isinstance(names, dict) else f"class_{class_id}"
            target_classes = {
                str(value).strip().lower()
                for value in self._parameter_list("target_classes")
                if str(value).strip()
            }
            if target_classes and class_name.strip().lower() not in target_classes:
                continue
            conf = float(confidences[index]) if index < len(confidences) else 1.0
            obj = self._object_from_mask(
                mask,
                header,
                index,
                class_id,
                class_name,
                conf,
                frame=frame,
            )
            if obj is not None:
                objects.append(obj)
        self._finalize_objects(objects, frame.shape[1], frame.shape[0])
        self._draw_workspace_limit(debug)
        for obj in objects:
            self._draw_object(debug, obj, (0, 220, 80))
        return objects, debug

    @staticmethod
    def _masks_from_boxes(boxes_xyxy: np.ndarray, shape: tuple[int, int]) -> list[np.ndarray]:
        """Create filled rectangle masks from axis-aligned YOLO detection boxes."""
        masks: list[np.ndarray] = []
        height, width = shape[:2]
        for box in boxes_xyxy:
            x1, y1, x2, y2 = box.astype(int)
            x1 = max(0, min(x1, width - 1))
            y1 = max(0, min(y1, height - 1))
            x2 = max(0, min(x2, width - 1))
            y2 = max(0, min(y2, height - 1))
            mask = np.zeros((height, width), dtype=np.uint8)
            if x2 > x1 and y2 > y1:
                cv2.rectangle(mask, (x1, y1), (x2, y2), 255, thickness=cv2.FILLED)
            masks.append(mask)
        return masks

    def _segment_with_opencv(self, frame: np.ndarray, header) -> tuple[list[SegmentedObject], np.ndarray]:
        profiles = self._orientation_profiles()
        if profiles:
            objects: list[SegmentedObject] = []
            debug = frame.copy()
            colors = {"vertical": (0, 220, 80), "horizontal": (0, 180, 255)}
            previous_overrides = self._profile_overrides
            try:
                for label, profile in profiles:
                    self._profile_overrides = profile
                    objects.extend(self._segment_with_opencv_profile(frame, header, label))
            finally:
                self._profile_overrides = previous_overrides
            objects = self._deduplicate_objects(objects)
            self._finalize_objects(objects, frame.shape[1], frame.shape[0])
            self._draw_workspace_limit(debug)
            for obj in objects:
                self._draw_object(debug, obj, colors.get(obj.orientation, (0, 180, 255)))
            return objects, debug

        objects = self._segment_with_opencv_profile(frame, header, "")
        self._finalize_objects(objects, frame.shape[1], frame.shape[0])
        debug = frame.copy()
        self._draw_workspace_limit(debug)
        for obj in objects:
            self._draw_object(debug, obj, (0, 180, 255))
        return objects, debug

    def _segment_with_opencv_profile(self, frame: np.ndarray, header, profile_label: str) -> list[SegmentedObject]:
        mask = self._build_opencv_mask(frame)

        kernel_size = max(1, int(self._param("morph_kernel")))
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        open_iterations = max(0, int(self._param("morph_open_iterations")))
        close_iterations = max(0, int(self._param("morph_close_iterations")))
        if open_iterations:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=open_iterations)
        if close_iterations:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=close_iterations)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        objects: list[SegmentedObject] = []
        for contour in contours:
            contour_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 1, thickness=cv2.FILLED)
            class_name = self._classify_shape(contour)
            shape_filter = str(self._param("shape_filter")).lower()
            if shape_filter not in ("", "any", "all") and class_name != shape_filter:
                continue
            if profile_label:
                class_name = f"{class_name}_{profile_label}"
            obj = self._object_from_mask(
                contour_mask,
                header,
                len(objects),
                0,
                class_name,
                1.0,
                contour=contour,
                frame=frame,
            )
            if obj is not None:
                objects.append(obj)
        return objects

    def _build_opencv_mask(self, frame: np.ndarray) -> np.ndarray:
        mode = str(self._param("opencv_mode")).lower()
        mode_tokens = {token for token in mode.split("_or_") if token}
        use_hsv = "hsv" in mode_tokens
        use_rgb = "rgb" in mode_tokens
        use_bright = "bright" in mode_tokens
        use_edge = "edge" in mode_tokens
        masks: list[np.ndarray] = []

        if use_hsv:
            lower = np.array([int(v) for v in self._param_list("hsv_lower")], dtype=np.uint8)
            upper = np.array([int(v) for v in self._param_list("hsv_upper")], dtype=np.uint8)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            masks.append(cv2.inRange(hsv, lower, upper))

        if use_rgb:
            lower = np.array([int(v) for v in self._param_list("rgb_lower")], dtype=np.uint8)
            upper = np.array([int(v) for v in self._param_list("rgb_upper")], dtype=np.uint8)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            masks.append(cv2.inRange(rgb, lower, upper))

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur_kernel = max(1, int(self._param("blur_kernel")))
        if blur_kernel % 2 == 0:
            blur_kernel += 1
        if blur_kernel > 1:
            gray = cv2.GaussianBlur(gray, (blur_kernel, blur_kernel), 0)

        if use_bright:
            threshold = int(self._param("brightness_threshold"))
            threshold_type = cv2.THRESH_BINARY_INV if bool(self._param("brightness_invert")) else cv2.THRESH_BINARY
            _unused, bright = cv2.threshold(gray, threshold, 255, threshold_type)
            masks.append(bright)

        if use_edge:
            low = int(self._param("canny_low"))
            high = int(self._param("canny_high"))
            aperture = int(self._param("edge_aperture_size"))
            if aperture not in (3, 5, 7):
                aperture = 3
            edge = cv2.Canny(gray, low, high, apertureSize=aperture)
            edge_kernel = np.ones((3, 3), np.uint8)
            dilate_iterations = max(0, int(self._param("edge_dilate_iterations")))
            erode_iterations = max(0, int(self._param("edge_erode_iterations")))
            if dilate_iterations:
                edge = cv2.dilate(edge, edge_kernel, iterations=dilate_iterations)
            if erode_iterations:
                edge = cv2.erode(edge, edge_kernel, iterations=erode_iterations)
            masks.append(edge)

        if not masks:
            self.get_logger().warning(
                f"unknown opencv_mode '{mode}', falling back to hsv",
                throttle_duration_sec=5.0,
            )
            lower = np.array([int(v) for v in self._param_list("hsv_lower")], dtype=np.uint8)
            upper = np.array([int(v) for v in self._param_list("hsv_upper")], dtype=np.uint8)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            masks.append(cv2.inRange(hsv, lower, upper))

        mask = masks[0]
        for extra in masks[1:]:
            mask = cv2.bitwise_or(mask, extra)

        margin = max(0, int(self._param("roi_margin_px")))
        if margin:
            mask[:margin, :] = 0
            mask[-margin:, :] = 0
            mask[:, :margin] = 0
            mask[:, -margin:] = 0
        workspace_mask = self._workspace_mask(mask.shape[1], mask.shape[0])
        if workspace_mask is not None:
            mask = cv2.bitwise_and(mask, workspace_mask)
        return mask

    def _workspace_mask(self, width: int, height: int) -> np.ndarray | None:
        if not bool(self._param("use_workspace_mask")):
            return None
        shape = str(self._param("workspace_limit_shape")).lower()
        margin = max(0, int(round(float(self._param("workspace_mask_margin_px")))))
        mask = np.zeros((height, width), dtype=np.uint8)
        if shape == "circle":
            circle = [float(value) for value in self._param_list("workspace_circle_pixel")]
            if len(circle) < 3:
                return None
            cx, cy, radius = circle[:3]
            radius = max(1, int(round(radius - margin)))
            cv2.circle(mask, (int(round(cx)), int(round(cy))), radius, 255, thickness=cv2.FILLED)
            return mask

        points = self._workspace_limit_points()
        if len(points) < 3:
            return None
        cv2.fillPoly(mask, [np.array(points, dtype=np.int32)], 255)
        if margin:
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=margin)
        return mask

    def _object_from_mask(
        self,
        mask: np.ndarray,
        header,
        index: int,
        class_id: int,
        class_name: str,
        confidence: float,
        contour: Any | None = None,
        frame: np.ndarray | None = None,
    ) -> SegmentedObject | None:
        area = float(cv2.contourArea(contour)) if contour is not None else float(np.count_nonzero(mask))
        min_area = float(self._param("min_area_px"))
        max_area = float(self._param("max_area_px"))
        if max_area <= 0.0:
            max_area = float(mask.shape[0] * mask.shape[1]) * float(
                self._param("max_area_ratio")
            )
        if area < min_area or area > max_area:
            return None

        if contour is None:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None
            contour = max(contours, key=cv2.contourArea)

        rect = cv2.minAreaRect(contour)
        width, height = rect[1]
        if not self._bbox_within_limits(contour, width, height):
            return None
        center = self._detect_center(contour, rect, mask)
        if center is None:
            return None
        u, v = center
        axis_yaw = self._detect_direction(contour, rect)
        yaw, direction_sign, direction_start, direction_end = self._detect_sense(
            contour,
            rect,
            (u, v),
            axis_yaw,
            frame,
        )
        pick_width = float(min(width, height))
        aspect = self._aspect_ratio(width, height)
        object_label = class_name
        if aspect >= float(self._param("lying_aspect_ratio_threshold")):
            object_label = f"{object_label}_lying"

        obj = SegmentedObject()
        obj.header = header
        obj.object_id = stable_id("seg", index, class_name, u, v)
        obj.class_id = int(class_id)
        obj.class_name = object_label
        obj.confidence = float(confidence)
        obj.score = float(confidence)
        obj.reachable = False
        obj.reject_reason = "not_projected"
        obj.center_pixel = Point(x=u, y=v, z=0.0)
        obj.area_px = float(area)
        obj.yaw = float(yaw)
        obj.axis_yaw = float(axis_yaw)
        obj.gripper_yaw = float(yaw)
        obj.direction_sign = int(direction_sign)
        obj.direction_sense = str(self._param("direction_sense_method"))
        obj.direction_start_pixel = Point(x=float(direction_start[0]), y=float(direction_start[1]), z=0.0)
        obj.direction_end_pixel = Point(x=float(direction_end[0]), y=float(direction_end[1]), z=0.0)
        obj.pick_width = pick_width
        obj.bounding_box_pixels = self._box_points(rect)
        obj.contour_pixels = self._sample_contour(contour)

        orientation = self._classify_orientation(
            axis_yaw,
            float(width),
            float(height),
            tolerance_deg=20.0,
        )
        obj.orientation = orientation
        grasp_zone, grasp_inset_zone, preferred, inset_ratio = self._grasp_zones(rect, orientation)
        obj.grasp_zone_pixels = grasp_zone
        obj.grasp_inset_zone_pixels = grasp_inset_zone
        obj.preferred_grasp_pixel = preferred
        obj.grasp_inset_ratio = inset_ratio
        obj.gripper_width_estimate = float(self.get_parameter("gripper_width_px").value)
        pickable, reason = self._evaluate_pickable(obj, grasp_inset_zone, orientation)
        obj.pickable = pickable
        obj.pickable_reason = reason

        if not self._object_inside_workspace(obj):
            return None
        return obj

    def _bbox_within_limits(self, contour, width: float, height: float) -> bool:
        _, _, axis_w, axis_h = cv2.boundingRect(contour)
        long_side = max(abs(float(width)), abs(float(height)))
        short_side = min(abs(float(width)), abs(float(height)))
        aspect = self._aspect_ratio(width, height)
        max_checks = [
            ("max_bbox_long_side_px", long_side),
            ("max_bbox_short_side_px", short_side),
            ("max_aspect_ratio", aspect),
        ]
        for parameter, measured in max_checks:
            limit = float(self._param(parameter))
            if limit > 0.0 and measured > limit:
                return False
        min_checks = [
            ("min_bbox_long_side_px", long_side),
            ("min_bbox_short_side_px", short_side),
        ]
        for parameter, measured in min_checks:
            limit = float(self._param(parameter))
            if limit > 0.0 and measured < limit:
                return False
        min_aspect = float(self._param("min_aspect_ratio"))
        if min_aspect > 0.0 and aspect < min_aspect:
            return False
        axis_area = max(float(axis_w * axis_h), 1e-6)
        contour_area = max(float(cv2.contourArea(contour)), 0.0)
        hull = cv2.convexHull(contour)
        hull_area = max(float(cv2.contourArea(hull)), 1e-6)
        extent = contour_area / axis_area
        solidity = contour_area / hull_area
        min_extent = float(self._param("min_extent"))
        max_extent = float(self._param("max_extent"))
        min_solidity = float(self._param("min_solidity"))
        if min_extent > 0.0 and extent < min_extent:
            return False
        if max_extent > 0.0 and extent > max_extent:
            return False
        if min_solidity > 0.0 and solidity < min_solidity:
            return False
        return True

    def _finalize_objects(self, objects: list[SegmentedObject], width: int, height: int) -> None:
        objects.sort(key=lambda item: (item.center_pixel.y, item.center_pixel.x))
        boxes = [self._oriented_box(obj) for obj in objects]
        for index, obj in enumerate(objects, start=1):
            obj.object_number = index
            left, right, up, down = self._oriented_clearances(index - 1, boxes, float(width), float(height))
            obj.clearance_left_px = float(left)
            obj.clearance_right_px = float(right)
            obj.clearance_up_px = float(up)
            obj.clearance_down_px = float(down)

    def _oriented_clearances(
        self,
        index: int,
        boxes: list[OrientedBox],
        image_width: float,
        image_height: float,
    ) -> tuple[float, float, float, float]:
        box = boxes[index]
        left, right, up, down = self._boundary_clearances(box, image_width, image_height)
        for other_index, other in enumerate(boxes):
            if other_index == index:
                continue
            other_min_long, other_max_long = self._project_points(other.corners, box.long_axis)
            other_min_short, other_max_short = self._project_points(other.corners, box.short_axis)
            long_overlap = min(box.max_long, other_max_long) - max(box.min_long, other_min_long)
            short_overlap = min(box.max_short, other_max_short) - max(box.min_short, other_min_short)
            if long_overlap > 0.0:
                if other_max_short <= box.min_short:
                    left = min(left, box.min_short - other_max_short)
                if other_min_short >= box.max_short:
                    right = min(right, other_min_short - box.max_short)
            if short_overlap > 0.0:
                if other_max_long <= box.min_long:
                    up = min(up, box.min_long - other_max_long)
                if other_min_long >= box.max_long:
                    down = min(down, other_min_long - box.max_long)
        return max(left, 0.0), max(right, 0.0), max(up, 0.0), max(down, 0.0)

    def _object_inside_workspace(self, obj: SegmentedObject) -> bool:
        if not bool(self._param("use_workspace_mask")):
            return True
        center = obj.center_pixel
        points = [(float(center.x), float(center.y))]
        points.extend((float(point.x), float(point.y)) for point in obj.bounding_box_pixels)
        return all(self._point_inside_workspace(point) for point in points)

    def _boundary_clearances(
        self,
        box: OrientedBox,
        image_width: float,
        image_height: float,
    ) -> tuple[float, float, float, float]:
        if bool(self._param("use_workspace_mask")):
            return (
                self._workspace_clearance_along(box, (-box.short_axis[0], -box.short_axis[1]), box.half_short, box.long_axis, box.half_long),
                self._workspace_clearance_along(box, box.short_axis, box.half_short, box.long_axis, box.half_long),
                self._workspace_clearance_along(box, (-box.long_axis[0], -box.long_axis[1]), box.half_long, box.short_axis, box.half_short),
                self._workspace_clearance_along(box, box.long_axis, box.half_long, box.short_axis, box.half_short),
            )
        image_corners = (
            (0.0, 0.0),
            (max(image_width - 1.0, 0.0), 0.0),
            (max(image_width - 1.0, 0.0), max(image_height - 1.0, 0.0)),
            (0.0, max(image_height - 1.0, 0.0)),
        )
        image_min_long, image_max_long = self._project_points(image_corners, box.long_axis)
        image_min_short, image_max_short = self._project_points(image_corners, box.short_axis)
        return (
            box.min_short - image_min_short,
            image_max_short - box.max_short,
            box.min_long - image_min_long,
            image_max_long - box.max_long,
        )

    def _workspace_clearance_along(
        self,
        box: OrientedBox,
        direction: tuple[float, float],
        edge_half: float,
        span_axis: tuple[float, float],
        span_half: float,
    ) -> float:
        start = (box.center_x + direction[0] * edge_half, box.center_y + direction[1] * edge_half)
        span = (span_axis[0] * span_half, span_axis[1] * span_half)
        max_distance = 4096.0
        low = 0.0
        high = 1.0
        while high < max_distance and self._workspace_edge_inside(start, span, direction, high):
            low = high
            high *= 2.0
        high = min(high, max_distance)
        for _iteration in range(14):
            mid = (low + high) * 0.5
            if self._workspace_edge_inside(start, span, direction, mid):
                low = mid
            else:
                high = mid
        return low

    def _workspace_edge_inside(
        self,
        start: tuple[float, float],
        span: tuple[float, float],
        direction: tuple[float, float],
        distance: float,
    ) -> bool:
        center = (start[0] + direction[0] * distance, start[1] + direction[1] * distance)
        samples = (
            center,
            (center[0] + span[0], center[1] + span[1]),
            (center[0] - span[0], center[1] - span[1]),
        )
        return all(self._point_inside_workspace(point) for point in samples)

    def _point_inside_workspace(self, point: tuple[float, float]) -> bool:
        if not bool(self._param("use_workspace_mask")):
            return True
        shape = str(self._param("workspace_limit_shape")).lower()
        margin = max(0.0, float(self._param("workspace_mask_margin_px")))
        if shape == "circle":
            circle = [float(value) for value in self._param_list("workspace_circle_pixel")]
            if len(circle) < 3:
                return True
            cx, cy, radius = circle[:3]
            return math.hypot(point[0] - cx, point[1] - cy) <= max(radius - margin, 0.0)
        polygon = self._workspace_limit_points()
        if len(polygon) < 3:
            return True
        distance = cv2.pointPolygonTest(np.array(polygon, dtype=np.float32), point, True)
        return distance >= margin

    @staticmethod
    def _oriented_box(obj: SegmentedObject) -> OrientedBox:
        points = obj.bounding_box_pixels or obj.contour_pixels
        if len(points) >= 4:
            corners = tuple((float(point.x), float(point.y)) for point in points[:4])
        else:
            u = float(obj.center_pixel.x)
            v = float(obj.center_pixel.y)
            corners = ((u, v), (u, v), (u, v), (u, v))
        center_x = sum(point[0] for point in corners) / len(corners)
        center_y = sum(point[1] for point in corners) / len(corners)
        edges: list[tuple[float, float, float]] = []
        for point_index in range(len(corners)):
            x0, y0 = corners[point_index]
            x1, y1 = corners[(point_index + 1) % len(corners)]
            dx = x1 - x0
            dy = y1 - y0
            edges.append((math.hypot(dx, dy), dx, dy))
        long_index, long_edge = max(enumerate(edges), key=lambda item: item[1][0])
        short_edge = edges[(long_index + 1) % len(edges)]
        if long_edge[0] <= 1e-6:
            yaw = float(obj.axis_yaw)
            long_axis = (math.cos(yaw), math.sin(yaw))
            short_axis = (-math.sin(yaw), math.cos(yaw))
        else:
            long_axis = (long_edge[1] / long_edge[0], long_edge[2] / long_edge[0])
            if short_edge[0] <= 1e-6:
                short_axis = (-long_axis[1], long_axis[0])
            else:
                short_axis = (short_edge[1] / short_edge[0], short_edge[2] / short_edge[0])
        min_long, max_long = SegmentationNode._project_points(corners, long_axis)
        min_short, max_short = SegmentationNode._project_points(corners, short_axis)
        return OrientedBox(
            center_x=center_x,
            center_y=center_y,
            long_axis=long_axis,
            short_axis=short_axis,
            half_long=(max_long - min_long) * 0.5,
            half_short=(max_short - min_short) * 0.5,
            min_long=min_long,
            max_long=max_long,
            min_short=min_short,
            max_short=max_short,
            corners=corners,
        )

    @staticmethod
    def _project_points(points: tuple[tuple[float, float], ...], axis: tuple[float, float]) -> tuple[float, float]:
        values = [point[0] * axis[0] + point[1] * axis[1] for point in points]
        return min(values), max(values)

    @staticmethod
    def _box_points(rect) -> list[Point]:
        points = cv2.boxPoints(rect)
        return [Point(x=float(x), y=float(y), z=0.0) for x, y in points]

    def _detect_center(
        self,
        contour,
        rect: tuple[tuple[float, float], tuple[float, float], float],
        mask: np.ndarray,
    ) -> tuple[float, float] | None:
        method = str(self._param("center_method")).lower()
        if method in ("rect", "min_area_rect"):
            (center, _size, _angle) = rect
            return float(center[0]), float(center[1])
        if method in ("bbox", "bounding_box"):
            x, y, w, h = cv2.boundingRect(contour)
            return float(x + (w * 0.5)), float(y + (h * 0.5))
        if method in ("contour_mean", "mean"):
            points = contour.reshape((-1, 2))
            return float(np.mean(points[:, 0])), float(np.mean(points[:, 1]))

        moments = cv2.moments(contour)
        if abs(moments["m00"]) < 1e-9:
            ys, xs = np.nonzero(mask)
            if len(xs) == 0:
                return None
            return float(np.mean(xs)), float(np.mean(ys))
        return float(moments["m10"] / moments["m00"]), float(moments["m01"] / moments["m00"])

    def _detect_direction(
        self,
        contour,
        rect: tuple[tuple[float, float], tuple[float, float], float],
    ) -> float:
        method = str(self._param("direction_method")).lower()
        if method == "none":
            return 0.0
        if method == "pca":
            points = contour.reshape((-1, 2)).astype(np.float32)
            if len(points) >= 2:
                _, eigenvectors = cv2.PCACompute(points, mean=None, maxComponents=2)
                vx, vy = eigenvectors[0]
                return math.atan2(float(vy), float(vx))
        (_center, (width, height), angle_deg) = rect
        angle = float(angle_deg)
        if width < height:
            angle += 90.0
        return math.radians(angle)

    def _detect_sense(
        self,
        contour,
        rect: tuple[tuple[float, float], tuple[float, float], float],
        center: tuple[float, float],
        axis_yaw: float,
        frame: np.ndarray | None,
    ) -> tuple[float, int, tuple[float, float], tuple[float, float]]:
        method = str(self._param("direction_sense_method")).lower()
        _rect_center, (width, height), _angle = rect
        half_len = max(float(width), float(height), 1.0) * 0.55
        ux = math.cos(axis_yaw)
        uy = math.sin(axis_yaw)
        sign = 1

        if method in ("positive_x", "right"):
            sign = 1 if ux >= 0.0 else -1
        elif method in ("negative_x", "left"):
            sign = 1 if ux <= 0.0 else -1
        elif method in ("positive_y", "down"):
            sign = 1 if uy >= 0.0 else -1
        elif method in ("negative_y", "up"):
            sign = 1 if uy <= 0.0 else -1
        elif method in ("bright_end", "dark_end") and frame is not None:
            plus = (center[0] + ux * half_len, center[1] + uy * half_len)
            minus = (center[0] - ux * half_len, center[1] - uy * half_len)
            plus_score = self._endpoint_intensity(frame, plus)
            minus_score = self._endpoint_intensity(frame, minus)
            if method == "bright_end":
                sign = 1 if plus_score >= minus_score else -1
            else:
                sign = 1 if plus_score <= minus_score else -1

        yaw = axis_yaw if sign > 0 else axis_yaw + math.pi
        dux = math.cos(yaw)
        duy = math.sin(yaw)
        start = (center[0] - dux * half_len, center[1] - duy * half_len)
        end = (center[0] + dux * half_len, center[1] + duy * half_len)
        return self._normalize_angle(yaw), sign, start, end

    @staticmethod
    def _endpoint_intensity(frame: np.ndarray, point: tuple[float, float]) -> float:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cx = int(round(point[0]))
        cy = int(round(point[1]))
        radius = 12
        x0 = max(0, cx - radius)
        x1 = min(gray.shape[1], cx + radius + 1)
        y0 = max(0, cy - radius)
        y1 = min(gray.shape[0], cy + radius + 1)
        if x0 >= x1 or y0 >= y1:
            return 0.0
        patch = gray[y0:y1, x0:x1]
        return float(np.mean(patch))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        return float(angle)

    def _classify_shape(self, contour) -> str:
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 1e-6:
            return "unknown"
        epsilon_ratio = float(self._param("shape_epsilon_ratio"))
        approx = cv2.approxPolyDP(contour, epsilon_ratio * perimeter, True)
        vertices = len(approx)
        area = max(float(cv2.contourArea(contour)), 1e-6)
        circularity = (4.0 * math.pi * area) / (perimeter * perimeter)

        if vertices == 3:
            return "triangle"
        if vertices == 4:
            (_center, (width, height), _angle) = cv2.minAreaRect(contour)
            aspect = self._aspect_ratio(width, height)
            tolerance = float(self._param("square_aspect_tolerance"))
            return "square" if aspect <= (1.0 + tolerance) else "rectangle"
        if circularity >= float(self._param("circle_min_circularity")):
            return "circle"
        return "object"

    @staticmethod
    def _aspect_ratio(width: float, height: float) -> float:
        major = max(abs(float(width)), abs(float(height)))
        minor = max(min(abs(float(width)), abs(float(height))), 1e-6)
        return major / minor

    def _sample_contour(self, contour) -> list[Point]:
        max_points = max(4, int(self._param("max_contour_points")))
        contour = contour.reshape((-1, 2))
        if len(contour) > max_points:
            step = max(1, len(contour) // max_points)
            contour = contour[::step][:max_points]
        return [Point(x=float(x), y=float(y), z=0.0) for x, y in contour]

    def _draw_object(self, frame: np.ndarray, obj: SegmentedObject, color: tuple[int, int, int]) -> None:
        points = np.array([[p.x, p.y] for p in obj.contour_pixels], dtype=np.int32)
        if len(points) >= 3:
            cv2.polylines(frame, [points], True, color, 2)
        box = np.array([[p.x, p.y] for p in obj.bounding_box_pixels], dtype=np.int32)
        if len(box) == 4:
            cv2.polylines(frame, [box], True, (80, 240, 255), 2)
            if bool(self._param("draw_clearance")):
                self._draw_clearance(frame, obj)
        center = (int(round(obj.center_pixel.x)), int(round(obj.center_pixel.y)))
        cv2.circle(frame, center, 5, (0, 80, 255), -1)
        start = (
            int(round(obj.direction_start_pixel.x)),
            int(round(obj.direction_start_pixel.y)),
        )
        end = (
            int(round(obj.direction_end_pixel.x)),
            int(round(obj.direction_end_pixel.y)),
        )
        cv2.arrowedLine(frame, start, end, (255, 180, 40), 2, tipLength=0.25)
        label = f"{obj.object_number}: {obj.class_name} ({obj.orientation[:3]})"
        cv2.putText(frame, label, (center[0] + 8, center[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(frame, label, (center[0] + 8, center[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
        clearances = (
            f"S-{obj.clearance_left_px:.0f} S+{obj.clearance_right_px:.0f} "
            f"L-{obj.clearance_up_px:.0f} L+{obj.clearance_down_px:.0f}"
        )
        cv2.putText(frame, clearances, (center[0] + 8, center[1] + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (230, 230, 230), 1)
        if obj.pickable:
            cv2.circle(frame, center, 3, (0, 255, 0), -1)
        SegmentationNode._draw_grasp_zone(frame, obj)

    @staticmethod
    def _draw_grasp_zone(frame: np.ndarray, obj: SegmentedObject) -> None:
        def to_ints(points):
            pts = []
            for p in points:
                pts.append([int(round(p.x)), int(round(p.y))])
            return np.array(pts, dtype=np.int32)

        if len(obj.grasp_zone_pixels) >= 3:
            cv2.polylines(frame, [to_ints(obj.grasp_zone_pixels)], True, (0, 165, 255), 1)
        if len(obj.grasp_inset_zone_pixels) >= 3:
            cv2.polylines(frame, [to_ints(obj.grasp_inset_zone_pixels)], True, (0, 255, 0), 1)
        if obj.grasp_inset_zone_pixels:
            p = obj.preferred_grasp_pixel
            cv2.circle(frame, (int(round(p.x)), int(round(p.y))), 3, (255, 0, 255), -1)

    @staticmethod
    def _draw_clearance(frame: np.ndarray, obj: SegmentedObject) -> None:
        box = SegmentationNode._oriented_box(obj)
        color = (80, 255, 180)
        thickness = 1
        max_draw = 80.0
        spans = [
            (box.short_axis, -1.0, box.half_short, min(float(obj.clearance_left_px), max_draw)),
            (box.short_axis, 1.0, box.half_short, min(float(obj.clearance_right_px), max_draw)),
            (box.long_axis, -1.0, box.half_long, min(float(obj.clearance_up_px), max_draw)),
            (box.long_axis, 1.0, box.half_long, min(float(obj.clearance_down_px), max_draw)),
        ]
        for axis, sign, half_size, clearance in spans:
            start = (
                int(round(box.center_x + axis[0] * sign * half_size)),
                int(round(box.center_y + axis[1] * sign * half_size)),
            )
            end = (
                int(round(box.center_x + axis[0] * sign * (half_size + clearance))),
                int(round(box.center_y + axis[1] * sign * (half_size + clearance))),
            )
            cv2.line(frame, start, end, color, thickness)

    def _draw_workspace_limit(self, frame: np.ndarray) -> None:
        if not bool(self.get_parameter("draw_workspace_limit").value):
            return
        shape = str(self.get_parameter("workspace_limit_shape").value).lower()
        color = (80, 120, 255)
        margin_color = (120, 255, 120)
        margin = max(0, int(round(float(self.get_parameter("workspace_mask_margin_px").value))))
        if shape == "circle":
            circle = [float(value) for value in self._parameter_list("workspace_circle_pixel")]
            if len(circle) >= 3:
                center = (int(round(circle[0])), int(round(circle[1])))
                radius = max(1, int(round(circle[2])))
                cv2.circle(frame, center, radius, color, 2)
                inner_radius = int(round(radius - margin))
                if margin > 0 and inner_radius > 1:
                    cv2.circle(frame, center, inner_radius, margin_color, 2)
            return
        points = self._workspace_limit_points()
        if len(points) >= 2:
            cv2.polylines(frame, [np.array(points, dtype=np.int32)], len(points) >= 3, color, 2)
        if margin <= 0 or len(points) < 3:
            return
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [np.array(points, dtype=np.int32)], 255)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=margin)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, margin_color, 2)

    def _workspace_limit_points(self) -> list[tuple[int, int]]:
        values = [float(value) for value in self._param_list("workspace_limit_pixels")]
        if len(values) < 4:
            return []
        if len(values) % 2 == 1:
            values = values[:-1]
        return [
            (int(round(values[index])), int(round(values[index + 1])))
            for index in range(0, len(values), 2)
        ]

    def _parameter_list(self, name: str) -> list:
        value = self.get_parameter(name).value
        if value is None:
            return []
        return list(value)

    def _param(self, name: str) -> Any:
        if self._profile_overrides is not None and name in self._profile_overrides:
            return self._profile_overrides[name]
        return self.get_parameter(name).value

    def _param_list(self, name: str) -> list:
        value = self._param(name)
        if value is None:
            return []
        if isinstance(value, str):
            try:
                parsed = json.loads(value)
                if isinstance(parsed, list):
                    return parsed
            except Exception:
                return []
        return list(value)

    def _orientation_profiles(self) -> list[tuple[str, dict[str, Any]]]:
        profiles: list[tuple[str, dict[str, Any]]] = []

        # Prefer a full calibrated profile if available.
        calibrated = self._calibrated_profile()
        if calibrated is not None:
            width = height = None
            if self._latest_frame is not None:
                height, width = self._latest_frame.shape[:2]
            for label in ("vertical", "horizontal"):
                profile = calibrated.profiles.get(label)
                if isinstance(profile, CalibratedProfile):
                    data = profile_to_segmentation_params(profile, width, height)
                    profiles.append((label, data))
            if profiles:
                return profiles

        if not bool(self.get_parameter("use_orientation_profiles").value):
            return []
        for label, parameter in (
            ("vertical", "vertical_profile_json"),
            ("horizontal", "horizontal_profile_json"),
        ):
            raw = str(self.get_parameter(parameter).value or "{}")
            try:
                data = json.loads(raw)
            except Exception as exc:
                self.get_logger().warning(f"perfil {label} inválido: {exc}", throttle_duration_sec=5.0)
                continue
            if isinstance(data, dict) and data:
                data = dict(data)
                morph_operation = int(float(data.get("morph_operation", 0)))
                data.setdefault("morph_open_iterations", abs(morph_operation) if morph_operation < 0 else 0)
                data.setdefault("morph_close_iterations", morph_operation if morph_operation > 0 else 0)
                edge_operation = int(float(data.get("edge_operation", 0)))
                data.setdefault("edge_erode_iterations", abs(edge_operation) if edge_operation < 0 else 0)
                data.setdefault("edge_dilate_iterations", edge_operation if edge_operation > 0 else 0)
                profiles.append((label, data))
        return profiles

    def _calibrated_profile(self) -> VisionProfile | None:
        raw = str(self.get_parameter("calibrated_profile_json").value or "{}")
        if not raw or raw == "{}":
            return None
        try:
            return VisionProfile.from_json(raw)
        except Exception as exc:
            self.get_logger().warning(f"calibrated_profile_json inválido: {exc}", throttle_duration_sec=5.0)
            return None

    def _calibrated_profile_for_label(self, label: str) -> CalibratedProfile | None:
        profile = self._calibrated_profile()
        if profile is None:
            return None
        candidate = profile.profiles.get(label)
        if not isinstance(candidate, CalibratedProfile):
            return None
        return candidate

    @staticmethod
    @staticmethod
    def _classify_orientation(
        axis_yaw: float,
        width: float,
        height: float,
        tolerance_deg: float = 20.0,
    ) -> str:
        """Classify the object's predominant axis as horizontal, vertical or ambiguous."""
        long_angle = float(axis_yaw)
        if width < height:
            long_angle += math.pi / 2.0
        long_angle = normalize_angle(long_angle)
        tolerance_rad = math.radians(max(0.0, tolerance_deg))
        sin_val = abs(math.sin(long_angle))
        cos_val = abs(math.cos(long_angle))
        if sin_val <= math.sin(tolerance_rad):
            return "horizontal"
        if cos_val <= math.sin(tolerance_rad):
            return "vertical"
        return "ambiguous"

    def _grasp_zones(
        self,
        rect: tuple[tuple[float, float], tuple[float, float], float],
        orientation: str,
    ) -> tuple[list[Point], list[Point], Point, float]:
        """Return (grasp_zone_pixels, grasp_inset_zone_pixels, preferred_point, inset_ratio)."""
        (center, (width, height), angle_deg) = rect
        cx, cy = float(center[0]), float(center[1])
        half_long = max(abs(float(width)), abs(float(height))) * 0.5
        half_short = min(abs(float(width)), abs(float(height))) * 0.5
        angle_rad = math.radians(float(angle_deg))
        if abs(float(width)) < abs(float(height)):
            angle_rad += math.pi / 2.0
        long_axis, short_axis = self._bbox_axes(angle_rad)

        profile = self._calibrated_profile()
        inset_ratio = float(self.get_parameter("grasp_inset_ratio").value)
        local_zone: list[tuple[float, float]] = []

        # Try to use a reference object template matching class/orientation.
        if profile is not None and profile.reference_objects:
            for ref in profile.reference_objects:
                if ref.orientation == orientation and ref.grasp_zone:
                    local_zone = [tuple(p) for p in ref.grasp_zone]
                    inset_ratio = ref.grasp_inset_ratio
                    break

        if len(local_zone) < 3:
            # Default: a rectangle inset from the BBox edges.
            inset = clamp(inset_ratio, 0.0, 0.5)
            local_zone = [
                (-1.0 + inset, -1.0 + inset),
                (1.0 - inset, -1.0 + inset),
                (1.0 - inset, 1.0 - inset),
                (-1.0 + inset, 1.0 - inset),
            ]

        grasp_zone = [
            Point(
                x=local_to_image_point((cx, cy), long_axis, short_axis, half_long, half_short, p)[0],
                y=local_to_image_point((cx, cy), long_axis, short_axis, half_long, half_short, p)[1],
                z=0.0,
            )
            for p in local_zone
        ]

        inset_zone_local = shrink_polygon(
            local_zone,
            inset_ratio * 2.0,
            inset_ratio * 2.0,
            (0.0, 0.0),
            (1.0, 0.0),
            (0.0, 1.0),
        )
        grasp_inset_zone = [
            Point(
                x=local_to_image_point((cx, cy), long_axis, short_axis, half_long, half_short, p)[0],
                y=local_to_image_point((cx, cy), long_axis, short_axis, half_long, half_short, p)[1],
                z=0.0,
            )
            for p in inset_zone_local
        ]

        preferred = Point(x=cx, y=cy, z=0.0)
        if grasp_inset_zone:
            preferred = grasp_inset_zone[0]

        return grasp_zone, grasp_inset_zone, preferred, inset_ratio

    @staticmethod
    def _bbox_axes(angle_rad: float) -> tuple[tuple[float, float], tuple[float, float]]:
        long_axis = (math.cos(angle_rad), math.sin(angle_rad))
        short_axis = (-math.sin(angle_rad), math.cos(angle_rad))
        return long_axis, short_axis

    def _evaluate_pickable(
        self,
        obj: SegmentedObject,
        inset_zone: list[Point],
        orientation: str,
    ) -> tuple[bool, str]:
        if orientation == "ambiguous":
            return False, "orientation_ambiguous"
        if len(inset_zone) < 3:
            return False, "grasp_inset_zone_too_small"
        zone_points = [(float(p.x), float(p.y)) for p in inset_zone]
        area = polygon_area(zone_points)
        gripper_width = float(self.get_parameter("gripper_width_px").value)
        if gripper_width > 0.0 and area < gripper_width * gripper_width:
            return False, "gripper_footprint_too_large"
        if area < 4.0:
            return False, "grasp_inset_area_too_small"
        return True, ""

    def _deduplicate_objects(self, objects: list[SegmentedObject]) -> list[SegmentedObject]:
        profile = self._calibrated_profile()
        merge = profile.merge if profile is not None else None
        iou_threshold = float(merge.rotated_iou_threshold) if merge is not None else 0.45
        center_threshold = float(merge.center_distance_threshold) if merge is not None else -1.0

        kept: list[SegmentedObject] = []
        for obj in objects:
            duplicate = False
            for other in kept:
                du = float(obj.center_pixel.x) - float(other.center_pixel.x)
                dv = float(obj.center_pixel.y) - float(other.center_pixel.y)
                center_dist = math.hypot(du, dv)
                min_width = max(4.0, min(float(obj.pick_width), float(other.pick_width)))
                if center_dist <= min_width * 0.5:
                    duplicate = True
                    break
                if center_threshold >= 0.0 and center_dist <= center_threshold:
                    duplicate = True
                    break
                if iou_threshold > 0.0 and self._objects_rotated_iou(obj, other) >= iou_threshold:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(obj)
        return kept

    def _objects_rotated_iou(self, a: SegmentedObject, b: SegmentedObject) -> float:
        box_a = self._object_rotated_box(a)
        box_b = self._object_rotated_box(b)
        if box_a is None or box_b is None:
            return 0.0
        return rotated_iou(box_a, box_b)

    def _object_rotated_box(
        self, obj: SegmentedObject
    ) -> tuple[tuple[float, float], tuple[float, float], float, float, float] | None:
        points = obj.bounding_box_pixels
        if len(points) != 4:
            return None
        box = self._oriented_box(obj)
        angle = math.atan2(box.long_axis[1], box.long_axis[0])
        return (
            (box.center_x, box.center_y),
            box.long_axis,
            box.half_long,
            box.half_short,
            angle,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
