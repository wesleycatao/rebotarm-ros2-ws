from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rebotarm_msgs.msg import SegmentedObject, SegmentedObjectArray


class PickSchedulerNode(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_pick_scheduler")
        self.declare_parameter("objects_topic", "/rebotarm/vision/objects")
        self.declare_parameter("queue_topic", "/rebotarm/vision/pick_queue")
        self.declare_parameter("selected_topic", "/rebotarm/vision/selected_object")
        self.declare_parameter("min_score", 0.35)
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("stable_frames", 2)
        self.declare_parameter("max_objects", 8)
        self.declare_parameter("strategy", "score")
        self.declare_parameter("allowed_orientations", ["horizontal", "vertical", "ambiguous"])
        self.declare_parameter("min_clearance_px", 0.0)
        self.declare_parameter("min_clearance_horizontal_px", 0.0)
        self.declare_parameter("min_clearance_vertical_px", 0.0)
        self.declare_parameter("auxiliary_attribute", "")
        self.declare_parameter("route_origin_xy", [0.0, 0.0])

        objects_topic = str(self.get_parameter("objects_topic").value)
        self._queue_pub = self.create_publisher(
            SegmentedObjectArray,
            str(self.get_parameter("queue_topic").value),
            10,
        )
        self._selected_pub = self.create_publisher(
            SegmentedObject,
            str(self.get_parameter("selected_topic").value),
            10,
        )
        self._seen_counts: dict[str, int] = {}
        self._last_log = 0.0
        self.create_subscription(SegmentedObjectArray, objects_topic, self._on_objects, 10)
        self.get_logger().info(f"scheduling pick candidates from {objects_topic}")

    def _on_objects(self, message: SegmentedObjectArray) -> None:
        candidates = [
            obj
            for obj in message.objects
            if obj.reachable
            and obj.score >= float(self.get_parameter("min_score").value)
            and obj.confidence >= float(self.get_parameter("min_confidence").value)
            and self._orientation_allowed(obj)
            and self._clearance_allowed(obj)
        ]

        current_ids = {obj.object_id for obj in candidates}
        for object_id in list(self._seen_counts):
            if object_id not in current_ids:
                self._seen_counts.pop(object_id, None)
        for obj in candidates:
            self._seen_counts[obj.object_id] = self._seen_counts.get(obj.object_id, 0) + 1

        stable_frames = int(self.get_parameter("stable_frames").value)
        stable = [obj for obj in candidates if self._seen_counts.get(obj.object_id, 0) >= stable_frames]
        queue = self._sort_queue(stable)
        max_objects = int(self.get_parameter("max_objects").value)
        if max_objects > 0:
            queue = queue[:max_objects]

        output = SegmentedObjectArray()
        output.header = message.header
        output.image_width = float(getattr(message, "image_width", 0.0))
        output.image_height = float(getattr(message, "image_height", 0.0))
        output.objects = queue
        self._queue_pub.publish(output)

        if queue:
            self._selected_pub.publish(queue[0])
            now = time.monotonic()
            if now - self._last_log > 2.0:
                obj = queue[0]
                p = obj.pose.position
                self.get_logger().info(
                    "selected "
                    f"{obj.object_id} score={obj.score:.2f} "
                    f"xyz=[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]"
                )
                self._last_log = now

    def _sort_queue(self, objects: list[SegmentedObject]) -> list[SegmentedObject]:
        strategy = str(self.get_parameter("strategy").value).lower()
        if strategy == "front_to_back":
            return sorted(objects, key=lambda obj: (-obj.pose.position.x, -obj.score))
        if strategy == "back_to_front":
            return sorted(objects, key=lambda obj: (obj.pose.position.x, -obj.score))
        if strategy == "left_to_right":
            return sorted(objects, key=lambda obj: (obj.pose.position.y, -obj.score))
        if strategy == "right_to_left":
            return sorted(objects, key=lambda obj: (-obj.pose.position.y, -obj.score))
        if strategy == "nearest_neighbor":
            return self._nearest_neighbor_route(objects)
        return sorted(objects, key=lambda obj: obj.score, reverse=True)

    def _orientation_allowed(self, obj: SegmentedObject) -> bool:
        allowed = {
            str(value).strip().lower()
            for value in self.get_parameter("allowed_orientations").value
            if str(value).strip()
        }
        return not allowed or str(obj.orientation).lower() in allowed

    def _clearance_allowed(self, obj: SegmentedObject) -> bool:
        orientation = str(obj.orientation).lower()
        required = max(0.0, float(self.get_parameter("min_clearance_px").value))
        if orientation == "horizontal":
            required = max(
                required,
                float(self.get_parameter("min_clearance_horizontal_px").value),
            )
            available = min(float(obj.clearance_left_px), float(obj.clearance_right_px))
        elif orientation == "vertical":
            required = max(
                required,
                float(self.get_parameter("min_clearance_vertical_px").value),
            )
            # A vertical object may be approached from any of its four sides;
            # the detailed gripper planner validates the chosen corridor.
            available = max(
                float(obj.clearance_left_px),
                float(obj.clearance_right_px),
                float(obj.clearance_up_px),
                float(obj.clearance_down_px),
            )
        else:
            available = max(
                float(obj.clearance_left_px),
                float(obj.clearance_right_px),
                float(obj.clearance_up_px),
                float(obj.clearance_down_px),
            )
        return available >= required

    def _nearest_neighbor_route(
        self,
        objects: list[SegmentedObject],
    ) -> list[SegmentedObject]:
        origin = list(self.get_parameter("route_origin_xy").value)
        current = (
            float(origin[0]) if len(origin) >= 1 else 0.0,
            float(origin[1]) if len(origin) >= 2 else 0.0,
        )
        remaining = list(objects)
        route: list[SegmentedObject] = []
        while remaining:
            closest = min(
                remaining,
                key=lambda obj: (
                    math.hypot(
                        float(obj.pose.position.x) - current[0],
                        float(obj.pose.position.y) - current[1],
                    ),
                    -float(obj.score),
                ),
            )
            route.append(closest)
            remaining.remove(closest)
            current = (float(closest.pose.position.x), float(closest.pose.position.y))
        return route


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickSchedulerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
