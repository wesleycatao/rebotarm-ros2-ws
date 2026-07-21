from __future__ import annotations

import threading

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .hardware_manager import HardwareManager
from .motor_passthrough import MotorPassthrough
from .ros_actions import ArmActions
from .ros_publishers import JointStatePublisher
from .ros_services import ArmServices
from .safety_event_logger import SafetyEventLogger
from .torque_profile import (
    SEVERITY_FAIL,
    SEVERITY_WARN,
    TorqueProfile,
    TorqueProfileMonitor,
    normalize_mode,
)


class reBotArmController(Node):
    def __init__(self) -> None:
        super().__init__("reBotArmController")

        self.reentrant_group = ReentrantCallbackGroup()
        self.slow_group = MutuallyExclusiveCallbackGroup()
        self.sensor_qos = qos_profile_sensor_data

        self.declare_parameter("hardware_config", "")
        self.declare_parameter("model", "")
        self.declare_parameter("channel", "")
        self.declare_parameter("joint_state_rate", 100.0)
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("cmd_arbitration", "reject")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ee_frame_id", "end_link")
        self.declare_parameter("disable_after_safe_home", True)
        self.declare_parameter("torque_profile_path", "")
        self.declare_parameter("torque_profile_mode", "off")
        self.declare_parameter("torque_profile_violation_samples", 3)
        self.declare_parameter("torque_profile_step_id", "default")
        self.declare_parameter("torque_profile_derivative_limit", 0.0)

        hardware_config = self.get_parameter("hardware_config").value or None
        model = str(self.get_parameter("model").value or "")
        channel = str(self.get_parameter("channel").value or "")
        self.arm_namespace = str(self.get_parameter("arm_namespace").value or "rebotarm").strip("/")
        joint_state_rate = float(self.get_parameter("joint_state_rate").value)
        cmd_arbitration = str(self.get_parameter("cmd_arbitration").value or "reject")
        self.disable_after_safe_home = bool(
            self.get_parameter("disable_after_safe_home").value
        )
        torque_profile_path = str(
            self.get_parameter("torque_profile_path").value or ""
        ).strip()
        torque_profile_mode = normalize_mode(
            self.get_parameter("torque_profile_mode").value or "off"
        )
        torque_profile_violation_samples = int(
            self.get_parameter("torque_profile_violation_samples").value
        )
        torque_profile_step_id = str(
            self.get_parameter("torque_profile_step_id").value or "default"
        ).strip()
        torque_profile_derivative_limit = float(
            self.get_parameter("torque_profile_derivative_limit").value or 0.0
        )
        if cmd_arbitration not in ("reject", "preempt"):
            self.get_logger().warn(
                f"unsupported cmd_arbitration={cmd_arbitration!r}; using 'reject'"
            )
            cmd_arbitration = "reject"

        self.hardware = HardwareManager(
            hardware_config=hardware_config,
            model=model,
            channel=channel,
        )
        self.hardware.connect()
        self._last_torque_profile_warning = 0.0
        self._safety_event_logger = SafetyEventLogger()
        self._torque_profile_path = torque_profile_path
        torque_profile = self._load_torque_profile(torque_profile_path)
        self.torque_profile_monitor = TorqueProfileMonitor(
            torque_profile,
            mode=torque_profile_mode,
            violation_samples=torque_profile_violation_samples,
            active_step_id=torque_profile_step_id,
            derivative_limit=torque_profile_derivative_limit,
        )

        self.joint_state_publisher = JointStatePublisher(
            self,
            self.hardware,
            self.arm_namespace,
            joint_state_rate,
        )
        self.arm_services = ArmServices(self, self.hardware, self.arm_namespace)
        self.arm_actions = ArmActions(self, self.hardware, self.arm_namespace)
        self.motor_passthrough = MotorPassthrough(
            self,
            self.hardware,
            self.arm_namespace,
            cmd_arbitration,
        )

        # Dedicated safety monitor timer (100 Hz), independent of action loops.
        self._safety_lock = threading.Lock()
        self._safety_active = False
        self._safety_motion_type = "none"  # "move_to_pose" | "follow_joint_trajectory" | "none"
        self._safety_trajectory: list[np.ndarray] = []
        self._safety_start_q: np.ndarray | None = None
        self._safety_target_q: np.ndarray | None = None
        self._safety_timer = self.create_timer(
            0.01, self._safety_timer_callback, callback_group=self.reentrant_group
        )

        self.get_logger().info(
            f"reBotArmController started: namespace=/{self.arm_namespace}, "
            f"joints={self.hardware.joint_names}"
        )

    def publish_arm_status(self, *, read_hardware: bool = True) -> None:
        self.joint_state_publisher.publish_status(read_hardware=read_hardware)

    def reset_torque_profile_monitor(self) -> None:
        self.torque_profile_monitor.reset()

    def set_torque_profile(
        self,
        path: str,
        mode: str,
        violation_samples: int,
        active_step_id: str = "",
        derivative_limit: float = 0.0,
    ) -> str:
        profile = self._load_torque_profile(path) if path else None
        if path and profile is None:
            raise RuntimeError(f"failed to load torque profile: {path}")
        self._torque_profile_path = path
        self.torque_profile_monitor.configure(
            profile,
            mode=mode,
            violation_samples=violation_samples,
            active_step_id=active_step_id,
            derivative_limit=derivative_limit,
        )
        if profile is None:
            return f"torque profile disabled (mode={self.torque_profile_monitor.mode})"
        return (
            f"loaded {path} mode={self.torque_profile_monitor.mode} "
            f"steps={profile.step_ids()}"
        )

    def set_torque_profile_step(self, step_id: str) -> None:
        self.torque_profile_monitor.set_step(step_id)

    def reset_safety_fault(self) -> None:
        self.hardware.reset_protective_stop()
        self.torque_profile_monitor.reset()
        self._safety_event_logger.clear()

    def start_safety_monitor(
        self,
        motion_type: str,
        *,
        trajectory: list[np.ndarray] | None = None,
        start_q: np.ndarray | None = None,
        target_q: np.ndarray | None = None,
    ) -> None:
        """Register motion context so the safety timer can compute phase."""
        with self._safety_lock:
            self._safety_active = True
            self._safety_motion_type = motion_type
            self._safety_trajectory = [
                np.array(p, dtype=np.float64, copy=True)
                for p in (trajectory or [])
            ]
            self._safety_start_q = (
                np.array(start_q, dtype=np.float64, copy=True)
                if start_q is not None
                else None
            )
            self._safety_target_q = (
                np.array(target_q, dtype=np.float64, copy=True)
                if target_q is not None
                else None
            )
            self.torque_profile_monitor.reset()

    def stop_safety_monitor(self) -> None:
        """Clear motion context used by the safety timer."""
        with self._safety_lock:
            self._safety_active = False
            self._safety_motion_type = "none"
            self._safety_trajectory = []
            self._safety_start_q = None
            self._safety_target_q = None

    def _compute_safety_phase(self) -> float:
        """Compute phase based on current position and registered motion context."""
        from .ros_actions import compute_phase_by_position

        current_q = self.hardware.get_joint_positions()
        with self._safety_lock:
            trajectory = self._safety_trajectory
            start_q = self._safety_start_q
            target_q = self._safety_target_q
            motion_type = self._safety_motion_type

        if trajectory:
            return compute_phase_by_position(current_q, trajectory)

        if motion_type == "move_to_pose" and start_q is not None and target_q is not None:
            total_dist = float(np.linalg.norm(target_q - start_q)) or 1e-9
            traveled = float(np.linalg.norm(current_q - start_q))
            return max(0.0, min(1.0, traveled / total_dist))

        return 0.0

    def _safety_timer_callback(self) -> None:
        """100 Hz independent safety monitor."""
        with self._safety_lock:
            active = self._safety_active and self.torque_profile_monitor.enabled
            step_id = self.torque_profile_monitor.active_step_id
        if not active:
            return

        try:
            phase = self._compute_safety_phase()
        except Exception as exc:
            self.get_logger().warn(f"safety phase computation failed: {exc}")
            return

        try:
            pos, vel, effort = self.hardware.get_joint_state()
            self._safety_event_logger.record_sample(
                phase=phase,
                step_id=step_id,
                joint_names=self.hardware.joint_names,
                positions=pos,
                velocities=vel,
                torques=effort,
            )
            should_stop, severity, message = self.check_torque_profile(phase)
            if severity in (SEVERITY_WARN, SEVERITY_FAIL):
                self._safety_event_logger.save_event(
                    severity, message, phase, step_id
                )
        except Exception as exc:
            self.get_logger().error(f"safety timer check failed: {exc}")

    def check_torque_profile(self, phase: float, step_id: str = "") -> tuple[bool, str, str]:
        if not self.torque_profile_monitor.enabled:
            return False, "OK", ""
        try:
            _, _, effort = self.hardware.get_joint_state()
        except Exception as exc:
            self.get_logger().warn(f"torque profile check skipped: {exc}")
            return False, "OK", ""

        should_stop, severity, results = self.torque_profile_monitor.check(
            phase,
            self.hardware.joint_names,
            [float(v) for v in effort],
            step_id=step_id,
        )
        if severity == "OK":
            return False, severity, ""

        issues = [result for result in results if result.status != "OK"]
        message = "; ".join(v.message() for v in issues[:3])
        now = self.get_clock().now().nanoseconds * 1e-9
        if should_stop:
            self.get_logger().error(f"torque profile stop: {message}")
            self.hardware.enter_protective_stop("TORQUE_PROFILE_FAIL")
            self.publish_arm_status()
        elif severity == SEVERITY_FAIL:
            self.get_logger().error(f"torque profile fail: {message}")
        elif severity == SEVERITY_WARN and now - self._last_torque_profile_warning > 1.0:
            self.get_logger().warn(f"torque profile warning: {message}")
            self._last_torque_profile_warning = now
        return should_stop, severity, message

    def evaluate_current_torque(self, phase: float, step_id: str = ""):
        if self.torque_profile_monitor.profile is None:
            return "OK", []
        _, _, effort = self.hardware.get_joint_state()
        return self.torque_profile_monitor.profile.check(
            step_id or self.torque_profile_monitor.active_step_id,
            phase,
            self.hardware.joint_names,
            [float(v) for v in effort],
        )

    def _load_torque_profile(self, path: str) -> TorqueProfile | None:
        if not path:
            return None
        try:
            profile = TorqueProfile.load(path)
        except Exception as exc:
            self.get_logger().error(f"failed to load torque profile {path!r}: {exc}")
            return None
        self.get_logger().info(f"loaded torque profile: {path}")
        return profile

    def shutdown(self) -> None:
        self.hardware.shutdown(
            disable_after_safe_home=self.disable_after_safe_home,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = reBotArmController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
