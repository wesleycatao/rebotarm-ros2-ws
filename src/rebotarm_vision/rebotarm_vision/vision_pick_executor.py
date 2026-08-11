from __future__ import annotations

import threading
from typing import Callable

import rclpy
from geometry_msgs.msg import Point, Pose
from rclpy.action import ActionClient
from rclpy.node import Node
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.msg import SegmentedObject
from rebotarm_msgs.srv import GripperCommand

from .geometry import yaw_to_quaternion


class VisionPickExecutor(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_vision_pick_executor")
        self.declare_parameter("selected_topic", "/rebotarm/vision/selected_object")
        self.declare_parameter("auto_execute", False)
        self.declare_parameter("move_action", "/rebotarm/move_to_pose")
        self.declare_parameter("open_gripper_service", "/rebotarm/gripper/open")
        self.declare_parameter("close_gripper_service", "/rebotarm/gripper/close")
        self.declare_parameter("approach_height", 0.080)
        self.declare_parameter("pick_depth_offset", 0.010)
        self.declare_parameter("lift_height", 0.100)
        self.declare_parameter("motion_duration", 2.5)
        self.declare_parameter("gripper_timeout", 3.0)
        self.declare_parameter("open_gripper_position", 0.0)
        self.declare_parameter("close_gripper_position", 0.0)
        self.declare_parameter("place_pose", [0.28, -0.16, 0.10, 0.0, 1.5708, 0.0])

        self._move = ActionClient(
            self,
            MoveToPose,
            str(self.get_parameter("move_action").value),
        )
        self._open = self.create_client(
            GripperCommand,
            str(self.get_parameter("open_gripper_service").value),
        )
        self._close = self.create_client(
            GripperCommand,
            str(self.get_parameter("close_gripper_service").value),
        )
        self._active = False
        self._lock = threading.Lock()
        self.create_subscription(
            SegmentedObject,
            str(self.get_parameter("selected_topic").value),
            self._on_selected,
            10,
        )
        self.get_logger().info("vision pick executor ready; auto_execute is disabled by default")

    def _on_selected(self, obj: SegmentedObject) -> None:
        if not bool(self.get_parameter("auto_execute").value):
            return
        if not obj.reachable:
            return
        with self._lock:
            if self._active:
                return
            self._active = True
        thread = threading.Thread(target=self._execute_pick, args=(obj,), daemon=True)
        thread.start()

    def _execute_pick(self, obj: SegmentedObject) -> None:
        try:
            p = obj.pose.position
            self.get_logger().info(
                f"executing vision pick for {obj.object_id} at "
                f"[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]"
            )
            if not self._wait_for_interfaces():
                return

            approach_height = float(self.get_parameter("approach_height").value)
            pick_offset = float(self.get_parameter("pick_depth_offset").value)
            lift_height = float(self.get_parameter("lift_height").value)

            if not self._command_gripper(self._open, "open", "open_gripper_position"):
                return
            if not self._move_to(self._pose_at(obj.pose, p.z + approach_height), "approach"):
                return
            if not self._move_to(self._pose_at(obj.pose, p.z + pick_offset), "pick"):
                return
            if not self._command_gripper(self._close, "close", "close_gripper_position"):
                return
            if not self._move_to(self._pose_at(obj.pose, p.z + lift_height), "lift"):
                return
            if not self._move_to(self._place_pose(), "place"):
                return
            self._command_gripper(self._open, "open", "open_gripper_position")
        finally:
            with self._lock:
                self._active = False

    def _wait_for_interfaces(self) -> bool:
        if not self._move.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning("move action is not available")
            return False
        if not self._open.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("open gripper service is not available")
            return False
        if not self._close.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("close gripper service is not available")
            return False
        return True

    def _command_gripper(self, client, label: str, position_param: str) -> bool:
        request = GripperCommand.Request()
        request.position = float(self.get_parameter(position_param).value)
        request.timeout = float(self.get_parameter("gripper_timeout").value)
        return self._wait_future(
            client.call_async(request),
            lambda result: bool(result.success),
            f"gripper {label}",
            request.timeout + 1.0,
        )

    def _move_to(self, pose: Pose, label: str) -> bool:
        goal = MoveToPose.Goal()
        goal.target_pose = pose
        goal.duration = float(self.get_parameter("motion_duration").value)

        send_future = self._move.send_goal_async(goal)
        if not self._wait_future(send_future, lambda handle: handle is not None and handle.accepted, f"send {label}", 5.0):
            return False
        goal_handle = send_future.result()
        result_future = goal_handle.get_result_async()
        return self._wait_future(
            result_future,
            lambda wrapped: bool(wrapped.result.success),
            f"move {label}",
            goal.duration + 5.0,
        )

    def _wait_future(
        self,
        future,
        predicate: Callable[[object], bool],
        label: str,
        timeout: float,
    ) -> bool:
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(timeout=max(timeout, 0.1)):
            self.get_logger().warning(f"timed out waiting for {label}")
            return False
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warning(f"{label} failed: {exc}")
            return False
        ok = predicate(result)
        if not ok:
            self.get_logger().warning(f"{label} was rejected or failed")
        return ok

    @staticmethod
    def _pose_at(source: Pose, z: float) -> Pose:
        pose = Pose()
        pose.position = Point(
            x=float(source.position.x),
            y=float(source.position.y),
            z=float(z),
        )
        pose.orientation = source.orientation
        return pose

    def _place_pose(self) -> Pose:
        values = [float(value) for value in self.get_parameter("place_pose").value]
        pose = Pose()
        pose.position = Point(x=values[0], y=values[1], z=values[2])
        pose.orientation = yaw_to_quaternion(values[5], values[3], values[4])
        return pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionPickExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
