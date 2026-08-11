from __future__ import annotations

import json
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rebotarm_msgs.msg import TimingStatus
from std_msgs.msg import String

from .authorization import AuthorizationGuard, CAP_SPEED_OVERRIDE
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
        self.declare_parameter("torque_feedback_timeout", 0.10)
        self.declare_parameter("safety_reset_max_velocity", 0.03)
        self.declare_parameter("safety_reset_quiet_samples", 5)
        self.declare_parameter("torque_payload_tolerance_kg", 0.10)
        self.declare_parameter("deadline_watchdog_enabled", True)
        self.declare_parameter("control_loop_deadline_ms", 6.0)
        self.declare_parameter("control_loop_deadline_violations", 3)
        self.declare_parameter("safety_monitor_deadline_ms", 30.0)
        self.declare_parameter("timing_diagnostics_rate", 1.0)
        self.declare_parameter("authorization_required", False)
        self.declare_parameter("authorization_secret_path", "")
        self.declare_parameter("speed_override_minimum", 0.05)
        self.declare_parameter("speed_override_increase_rate", 0.50)
        self.declare_parameter("speed_override_decrease_rate", 2.00)

        hardware_config = self.get_parameter("hardware_config").value or None
        model = str(self.get_parameter("model").value or "")
        channel = str(self.get_parameter("channel").value or "")
        self.arm_namespace = str(self.get_parameter("arm_namespace").value or "rebotarm").strip("/")
        self.authorization = AuthorizationGuard(
            required=bool(self.get_parameter("authorization_required").value),
            secret_path=(
                str(self.get_parameter("authorization_secret_path").value).strip()
                or None
            ),
        )
        self._authorization_subscription = self.create_subscription(
            String,
            f"/{self.arm_namespace}/authorization/lease",
            self._authorization_lease_cb,
            10,
            callback_group=self.reentrant_group,
        )
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
        self._torque_feedback_timeout = max(
            0.02, float(self.get_parameter("torque_feedback_timeout").value)
        )
        self._safety_reset_max_velocity = max(
            0.0, float(self.get_parameter("safety_reset_max_velocity").value)
        )
        self._safety_reset_quiet_samples = max(
            1, int(self.get_parameter("safety_reset_quiet_samples").value)
        )
        self._torque_payload_tolerance_kg = max(
            0.0,
            float(self.get_parameter("torque_payload_tolerance_kg").value),
        )
        self._deadline_watchdog_enabled = bool(
            self.get_parameter("deadline_watchdog_enabled").value
        )
        self._control_loop_deadline_sec = max(
            0.0,
            float(self.get_parameter("control_loop_deadline_ms").value) * 1e-3,
        )
        self._control_loop_deadline_violations = max(
            1,
            int(self.get_parameter("control_loop_deadline_violations").value),
        )
        self._safety_monitor_period_sec = 0.01
        self._safety_monitor_deadline_sec = (
            float(self.get_parameter("safety_monitor_deadline_ms").value)
            * 1e-3
        )
        if self._deadline_watchdog_enabled:
            if self._control_loop_deadline_sec <= 0.0:
                raise ValueError("control_loop_deadline_ms must be positive")
            if self._safety_monitor_deadline_sec < self._safety_monitor_period_sec:
                raise ValueError(
                    "safety_monitor_deadline_ms must be at least 10 ms"
                )
        timing_diagnostics_rate = max(
            0.1,
            float(self.get_parameter("timing_diagnostics_rate").value),
        )
        speed_override_minimum = float(
            self.get_parameter("speed_override_minimum").value
        )
        speed_override_increase_rate = float(
            self.get_parameter("speed_override_increase_rate").value
        )
        speed_override_decrease_rate = float(
            self.get_parameter("speed_override_decrease_rate").value
        )
        if cmd_arbitration not in ("reject", "preempt"):
            self.get_logger().warning(
                f"unsupported cmd_arbitration={cmd_arbitration!r}; using 'reject'"
            )
            cmd_arbitration = "reject"

        self.hardware = HardwareManager(
            hardware_config=hardware_config,
            model=model,
            channel=channel,
            speed_override_minimum=speed_override_minimum,
            speed_override_increase_rate=speed_override_increase_rate,
            speed_override_decrease_rate=speed_override_decrease_rate,
        )
        limits_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._trajectory_limits_publisher = self.create_publisher(
            String,
            f"/{self.arm_namespace}/trajectory_limits",
            limits_qos,
        )
        self._trajectory_limits_publisher.publish(
            String(
                data=json.dumps(
                    self.hardware.trajectory_limits_snapshot(),
                    sort_keys=True,
                    separators=(",", ":"),
                )
            )
        )
        self._speed_command_session = ""
        self._speed_command_sequence = -1
        self._speed_command_message = "aguardando comando"
        self._speed_override_subscription = self.create_subscription(
            String,
            f"/{self.arm_namespace}/speed_override/command",
            self._speed_override_command_cb,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
            callback_group=self.reentrant_group,
        )
        self._speed_override_publisher = self.create_publisher(
            String,
            f"/{self.arm_namespace}/speed_override/status",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self._speed_override_timer = self.create_timer(
            0.1,
            self._publish_speed_override_status,
            callback_group=self.reentrant_group,
        )
        self._publish_speed_override_status()
        control_nominal_period = (
            self.hardware.control_loop_timing_snapshot().nominal_period_sec
        )
        if (
            self._deadline_watchdog_enabled
            and self._control_loop_deadline_sec < control_nominal_period
        ):
            raise ValueError(
                "control_loop_deadline_ms must be at least the configured "
                f"control period ({control_nominal_period * 1e3:.3f} ms)"
            )
        # The SDK path is resolved by HardwareManager. Reuse its bounded,
        # thread-safe timing tracker for both the actuator and ROS safety loops.
        from reBotArm_control_py.timing import LoopTimingMonitor

        self._deadline_fault_lock = threading.Lock()
        self._deadline_fault_event = threading.Event()
        self._deadline_shutdown = threading.Event()
        self._pending_deadline_fault: tuple[str, object] | None = None
        self._safety_stop_lock = threading.Lock()
        control_deadline = (
            self._control_loop_deadline_sec
            if self._deadline_watchdog_enabled
            else 0.0
        )
        self.hardware.configure_control_loop_watchdog(
            deadline_sec=control_deadline,
            violation_limit=self._control_loop_deadline_violations,
            on_fault=lambda fault: self._queue_deadline_fault(
                "control", fault
            ),
        )
        safety_deadline = (
            self._safety_monitor_deadline_sec
            if self._deadline_watchdog_enabled
            else 0.0
        )
        self._safety_timing = LoopTimingMonitor(
            self._safety_monitor_period_sec,
            deadline_sec=safety_deadline,
            violation_limit=1,
            on_fault=self._on_safety_deadline_fault,
        )
        self._last_torque_profile_warning = 0.0
        self._safety_event_logger = SafetyEventLogger()
        self._torque_profile_path = torque_profile_path
        torque_profile = self._load_torque_profile(torque_profile_path)
        if torque_profile_path and torque_profile is None:
            raise RuntimeError(
                f"configured torque profile is invalid: {torque_profile_path}"
            )
        if torque_profile_mode == "stop_on_fail" and torque_profile is None:
            raise RuntimeError(
                "torque_profile_mode=stop_on_fail requires a valid profile"
            )
        if (
            torque_profile is not None
            and not torque_profile.has_step(torque_profile_step_id)
        ):
            raise RuntimeError(
                f"torque profile does not cover step {torque_profile_step_id!r}"
            )
        self.torque_profile_monitor = TorqueProfileMonitor(
            torque_profile,
            mode=torque_profile_mode,
            violation_samples=torque_profile_violation_samples,
            active_step_id=torque_profile_step_id,
            derivative_limit=torque_profile_derivative_limit,
        )
        self._update_torque_monitor_status()

        # Validate every safety input before enabling drive output. A malformed
        # production profile must never leave the arm energized without its
        # configured collision monitor.
        self.hardware.connect()

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
        self._safety_check_lock = threading.Lock()
        self._safety_active = False
        self._safety_motion_type = "none"  # "move_to_pose" | "follow_joint_trajectory" | "none"
        self._safety_trajectory: list[np.ndarray] = []
        self._safety_start_q: np.ndarray | None = None
        self._safety_target_q: np.ndarray | None = None
        self._safety_last_valid_time = time.monotonic()
        self._safety_fault_phase = 0.0
        self._safety_fault_step = torque_profile_step_id or "default"
        self._safety_fault_message = ""
        self._safety_event_saved = False
        self._safety_timer_heartbeat = time.monotonic()
        self._safety_timer = self.create_timer(
            self._safety_monitor_period_sec,
            self._safety_timer_callback,
            callback_group=self.reentrant_group,
        )
        self._timing_publisher = self.create_publisher(
            TimingStatus,
            f"/{self.arm_namespace}/timing_status",
            10,
        )
        self._timing_timer = self.create_timer(
            1.0 / timing_diagnostics_rate,
            self._publish_timing_status,
            callback_group=self.reentrant_group,
        )
        self._deadline_supervisor = threading.Thread(
            target=self._deadline_supervisor_loop,
            name="rebotarm-deadline-supervisor",
            daemon=True,
        )
        self._deadline_supervisor.start()

        self.get_logger().info(
            f"reBotArmController started: namespace=/{self.arm_namespace}, "
            f"joints={self.hardware.joint_names}"
        )

    def _authorization_lease_cb(self, message: String) -> None:
        valid, detail = self.authorization.update(message.data)
        if valid:
            self.get_logger().info(f"authorization lease accepted: {detail}")
        else:
            self.get_logger().warning(f"authorization lease rejected: {detail}")

    def _speed_override_command_cb(self, message: String) -> None:
        """Apply a versioned global override command.

        Slowdowns are always accepted.  Raising the requested speed requires
        the controller-side capability lease once authorization is enabled.
        Invalid payloads fail slow to the configured minimum.
        """

        current = self.hardware.dynamic_speed_override_snapshot()
        try:
            payload = json.loads(str(message.data))
            if int(payload.get("version", 0)) != 1:
                raise ValueError("versão de comando não suportada")
            value = float(payload["value"])
            session = str(payload["session"]).strip()
            sequence = int(payload["sequence"])
            if not session or sequence < 0:
                raise ValueError("sessão/sequência inválida")
            if (
                session == self._speed_command_session
                and sequence <= self._speed_command_sequence
            ):
                self._speed_command_message = "comando obsoleto ignorado"
                self._publish_speed_override_status()
                return
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            valid, _, detail = self.hardware.set_dynamic_speed_override(float("nan"))
            del valid
            self._speed_command_message = f"comando inválido: {exc}; {detail}"
            self.get_logger().error(self._speed_command_message)
            self._publish_speed_override_status()
            return

        if (
            value > current.requested + 1e-9
            and not self.authorization.permits(CAP_SPEED_OVERRIDE)
        ):
            self._speed_command_message = (
                "aumento de velocidade rejeitado: autorização ausente"
            )
            self.get_logger().warning(self._speed_command_message)
            self._publish_speed_override_status()
            return

        valid, requested, detail = self.hardware.set_dynamic_speed_override(value)
        self._speed_command_session = session
        self._speed_command_sequence = sequence
        self._speed_command_message = detail or (
            f"override solicitado {requested * 100.0:.1f}%"
        )
        if valid:
            self.get_logger().info(self._speed_command_message)
        else:
            self.get_logger().error(self._speed_command_message)
        self._publish_speed_override_status()

    def _publish_speed_override_status(self) -> None:
        snapshot = self.hardware.dynamic_speed_override_snapshot()
        payload = {
            "version": 1,
            "supported": True,
            "requested": snapshot.requested,
            "applied": snapshot.applied,
            "ramping": snapshot.ramping,
            "minimum": snapshot.minimum,
            "maximum": snapshot.maximum,
            "increase_rate_per_s": snapshot.increase_rate_per_s,
            "decrease_rate_per_s": snapshot.decrease_rate_per_s,
            "motion_active": self.hardware.state_machine == "TRAJ_RUNNING",
            "session": self._speed_command_session,
            "sequence": self._speed_command_sequence,
            "message": snapshot.last_error or self._speed_command_message,
            "stamp": time.time(),
        }
        self._speed_override_publisher.publish(
            String(
                data=json.dumps(
                    payload,
                    sort_keys=True,
                    separators=(",", ":"),
                )
            )
        )

    def publish_arm_status(self, *, read_hardware: bool = True) -> None:
        self.joint_state_publisher.publish_status(read_hardware=read_hardware)

    def reset_torque_profile_monitor(self) -> None:
        self.torque_profile_monitor.reset()

    def _queue_deadline_fault(self, source: str, fault) -> None:
        if not self._deadline_watchdog_enabled or self._deadline_shutdown.is_set():
            return
        with self._deadline_fault_lock:
            if self._pending_deadline_fault is None:
                self._pending_deadline_fault = (str(source), fault)
        self._deadline_fault_event.set()

    def _on_safety_deadline_fault(self, fault) -> None:
        with self._safety_lock:
            active = self._safety_active
        if active:
            self._queue_deadline_fault("safety", fault)

    def _deadline_supervisor_loop(self) -> None:
        """Independent watchdog for faults and a starved ROS executor."""
        poll_period = min(
            0.005,
            max(0.001, self._safety_monitor_deadline_sec / 4.0),
        )
        while not self._deadline_shutdown.is_set():
            self._deadline_fault_event.wait(timeout=poll_period)
            self._deadline_fault_event.clear()
            if self._deadline_shutdown.is_set():
                return

            if self._deadline_watchdog_enabled:
                with self._safety_lock:
                    safety_active = self._safety_active
                    heartbeat = self._safety_timer_heartbeat
                heartbeat_age = time.monotonic() - heartbeat
                if (
                    safety_active
                    and heartbeat_age > self._safety_monitor_deadline_sec
                ):
                    self._safety_timing.report_stall(heartbeat_age)

            with self._deadline_fault_lock:
                pending = self._pending_deadline_fault
                self._pending_deadline_fault = None
            if pending is None or self.hardware.protective_stop_active:
                continue

            source, fault = pending
            snapshot = fault.snapshot
            if source == "control":
                reason = "SAFETY_CONTROL_DEADLINE"
                label = "500 Hz control loop"
            else:
                reason = "SAFETY_MONITOR_DEADLINE"
                label = "100 Hz safety monitor"
            message = (
                f"{label} missed deadline: observed="
                f"{fault.observed_sec * 1e3:.3f} ms, limit="
                f"{snapshot.deadline_sec * 1e3:.3f} ms, "
                f"misses={snapshot.deadline_misses}, "
                f"lost_cycles={snapshot.lost_cycles}"
            )
            step_id = self.torque_profile_monitor.active_step_id or "default"
            self._trigger_safety_stop(reason, message, 0.0, step_id)

    @staticmethod
    def _milliseconds(value: float) -> float:
        return float(value) * 1e3

    def _publish_timing_status(self) -> None:
        control = self.hardware.control_loop_timing_snapshot()
        safety = self._safety_timing.snapshot()
        msg = TimingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.watchdog_enabled = self._deadline_watchdog_enabled

        msg.control_nominal_rate_hz = 1.0 / control.nominal_period_sec
        msg.control_last_period_ms = self._milliseconds(control.last_period_sec)
        msg.control_last_execution_ms = self._milliseconds(
            control.last_execution_sec
        )
        msg.control_max_jitter_ms = self._milliseconds(control.max_jitter_sec)
        msg.control_p95_jitter_ms = self._milliseconds(control.p95_jitter_sec)
        msg.control_p99_jitter_ms = self._milliseconds(control.p99_jitter_sec)
        msg.control_deadline_misses = control.deadline_misses
        msg.control_lost_cycles = control.lost_cycles

        msg.safety_nominal_rate_hz = 1.0 / safety.nominal_period_sec
        msg.safety_last_period_ms = self._milliseconds(safety.last_period_sec)
        msg.safety_last_execution_ms = self._milliseconds(
            safety.last_execution_sec
        )
        msg.safety_max_jitter_ms = self._milliseconds(safety.max_jitter_sec)
        msg.safety_p95_jitter_ms = self._milliseconds(safety.p95_jitter_sec)
        msg.safety_p99_jitter_ms = self._milliseconds(safety.p99_jitter_sec)
        msg.safety_deadline_misses = safety.deadline_misses
        msg.safety_lost_cycles = safety.lost_cycles
        self._timing_publisher.publish(msg)

    def _update_torque_monitor_status(self) -> None:
        disabled = not self.torque_profile_monitor.enabled
        self.hardware.set_error_condition("TORQUE_MONITOR_DISABLED", disabled)
        warn_only = self.torque_profile_monitor.mode == "warn_only"
        self.hardware.set_error_condition("TORQUE_MONITOR_WARN_ONLY", warn_only)
        if disabled:
            self.get_logger().warning(
                "torque collision monitor is disabled; use a validated profile "
                "with mode=stop_on_fail for production motion"
            )
        elif warn_only:
            self.get_logger().warning(
                "torque collision monitor is in warn_only mode and will not stop motion"
            )

    def set_torque_profile(
        self,
        path: str,
        mode: str,
        violation_samples: int,
        active_step_id: str = "",
        derivative_limit: float = 0.0,
    ) -> str:
        if self.hardware.state_machine not in ("IDLE", "PROTECTIVE_STOP"):
            raise RuntimeError("torque profile can only be changed while motion is idle")
        if self.hardware.protective_stop_active:
            raise RuntimeError("reset safety fault before changing the torque profile")
        profile = self._load_torque_profile(path) if path else None
        if path and profile is None:
            raise RuntimeError(f"failed to load torque profile: {path}")
        normalized_mode = normalize_mode(mode)
        if normalized_mode == "stop_on_fail" and profile is None:
            raise RuntimeError("stop_on_fail requires a valid torque profile")
        requested_step = active_step_id or self.torque_profile_monitor.active_step_id
        if profile is not None and not profile.has_step(requested_step):
            raise RuntimeError(
                f"torque profile does not cover step {requested_step!r}"
            )
        self._torque_profile_path = path
        self.torque_profile_monitor.configure(
            profile,
            mode=mode,
            violation_samples=violation_samples,
            active_step_id=active_step_id,
            derivative_limit=derivative_limit,
        )
        self._update_torque_monitor_status()
        if profile is None:
            return f"torque profile disabled (mode={self.torque_profile_monitor.mode})"
        return (
            f"loaded {path} mode={self.torque_profile_monitor.mode} "
            f"steps={profile.step_ids()}"
        )

    def set_torque_profile_step(
        self, step_id: str, payload_kg: float = 0.0
    ) -> None:
        with self._safety_lock:
            if self._safety_active:
                raise RuntimeError(
                    "safety step cannot be changed while motion is active"
                )
        requested = step_id or "default"
        profile = self.torque_profile_monitor.profile
        if profile is not None and not profile.has_step(requested):
            raise RuntimeError(
                f"torque profile does not cover step {requested!r}"
            )
        requested_payload = float(payload_kg)
        expected_payload = profile.payload_mass_kg if profile is not None else None
        # payload_kg=0 keeps backward-compatible "unknown/not supplied"
        # semantics. When vision or the program provides a positive estimate,
        # reject a profile calibrated for a materially different load.
        if (
            requested_payload > 0.0
            and expected_payload is not None
            and abs(requested_payload - expected_payload)
            > self._torque_payload_tolerance_kg
        ):
            raise RuntimeError(
                "payload does not match torque profile: "
                f"requested={requested_payload:.3f} kg "
                f"profile={expected_payload:.3f} kg"
            )
        self.torque_profile_monitor.set_step(step_id)

    def reset_safety_fault(self) -> None:
        if not self.hardware.protective_stop_active:
            self.torque_profile_monitor.reset()
            return

        phase = self._safety_fault_phase
        step_id = self._safety_fault_step
        for _ in range(self._safety_reset_quiet_samples):
            positions, velocities, effort = self.hardware.get_joint_state()
            arrays = (positions, velocities, effort)
            if any(not np.all(np.isfinite(values)) for values in arrays):
                raise RuntimeError("safety reset rejected: non-finite motor feedback")
            max_velocity = float(np.max(np.abs(velocities))) if len(velocities) else 0.0
            if max_velocity > self._safety_reset_max_velocity:
                raise RuntimeError(
                    "safety reset rejected: robot is still moving "
                    f"({max_velocity:.4f} rad/s)"
                )
            status_codes = self.hardware.get_joint_status_codes(strict=True)
            if any(code != 0 for code in status_codes):
                raise RuntimeError(
                    "safety reset rejected: actuator fault is still active"
                )
            profile = self.torque_profile_monitor.profile
            if profile is not None:
                severity, _ = profile.check(
                    step_id,
                    phase,
                    self.hardware.joint_names,
                    [float(value) for value in effort],
                )
                if severity == SEVERITY_FAIL:
                    raise RuntimeError(
                        "safety reset rejected: torque remains outside FAIL limits"
                    )
            time.sleep(0.02)

        self.hardware.reset_protective_stop()
        self.hardware.reset_control_loop_timing_runtime()
        self._safety_timing.reset_runtime()
        with self._deadline_fault_lock:
            self._pending_deadline_fault = None
        self.torque_profile_monitor.reset()
        self._update_torque_monitor_status()
        self._safety_event_logger.reset_buffer()
        self._safety_fault_message = ""
        self._safety_event_saved = False

    def start_safety_monitor(
        self,
        motion_type: str,
        *,
        trajectory: list[np.ndarray] | None = None,
        start_q: np.ndarray | None = None,
        target_q: np.ndarray | None = None,
    ) -> None:
        """Register motion context so the safety timer can compute phase."""
        profile = self.torque_profile_monitor.profile
        active_step = self.torque_profile_monitor.active_step_id
        if (
            self.torque_profile_monitor.enabled
            and profile is not None
            and not profile.has_step(active_step)
        ):
            raise RuntimeError(
                f"torque profile does not cover active step {active_step!r}"
            )
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
            self._safety_last_valid_time = time.monotonic()
            self._safety_event_saved = False
            self._safety_timer_heartbeat = time.monotonic()
            self._safety_timing.reset_runtime()

    def update_safety_monitor_context(
        self,
        *,
        trajectory: list[np.ndarray] | None = None,
        target_q: np.ndarray | None = None,
    ) -> None:
        """Update a running monitor without resetting its sample history."""
        with self._safety_lock:
            if trajectory is not None:
                self._safety_trajectory = [
                    np.array(point, dtype=np.float64, copy=True)
                    for point in trajectory
                ]
            if target_q is not None:
                self._safety_target_q = np.array(
                    target_q, dtype=np.float64, copy=True
                )

    def stop_safety_monitor(self) -> None:
        """Clear motion context used by the safety timer."""
        with self._safety_lock:
            self._safety_active = False
            self._safety_motion_type = "none"
            self._safety_trajectory = []
            self._safety_start_q = None
            self._safety_target_q = None
            self._safety_timing.reset_runtime()

    @property
    def safety_fault_message(self) -> str:
        return self._safety_fault_message

    def _compute_safety_phase(self, current_q: np.ndarray | None = None) -> float:
        """Compute phase based on current position and registered motion context."""
        from .ros_actions import compute_phase_by_position

        if current_q is None:
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
        """Measure and execute one 100 Hz independent safety-monitor cycle."""
        started = time.perf_counter()
        with self._safety_lock:
            self._safety_timer_heartbeat = time.monotonic()
        try:
            self._run_safety_timer_callback()
        finally:
            self._safety_timing.observe_cycle(
                started,
                time.perf_counter() - started,
            )

    def _run_safety_timer_callback(self) -> None:
        """Read one atomic feedback sample and evaluate collision limits."""
        with self._safety_lock:
            active = (
                self._safety_active
                and self.torque_profile_monitor.enabled
                and not self.hardware.protective_stop_active
            )
            step_id = self.torque_profile_monitor.active_step_id
        if not active:
            return

        try:
            pos, vel, effort = self.hardware.get_joint_state()
            joint_count = len(self.hardware.joint_names)
            if any(len(values) != joint_count for values in (pos, vel, effort)):
                raise RuntimeError("incomplete arm feedback")
            if any(
                not np.all(np.isfinite(values))
                for values in (pos, vel, effort)
            ):
                raise RuntimeError("non-finite arm feedback")
            phase = self._compute_safety_phase(pos)
        except Exception as exc:
            self._handle_safety_monitor_error(exc, step_id)
            return

        try:
            self._safety_event_logger.record_sample(
                phase=phase,
                step_id=step_id,
                joint_names=self.hardware.joint_names,
                positions=pos,
                velocities=vel,
                torques=effort,
            )
            self.check_torque_profile(phase, effort=effort)
            self._safety_last_valid_time = time.monotonic()
        except Exception as exc:
            self._handle_safety_monitor_error(exc, step_id, phase)

    def _handle_safety_monitor_error(
        self,
        exc: Exception,
        step_id: str,
        phase: float = 0.0,
    ) -> None:
        message = f"torque safety monitor unavailable: {exc}"
        self.get_logger().error(message)
        if (
            self.torque_profile_monitor.mode == "stop_on_fail"
            and time.monotonic() - self._safety_last_valid_time
            >= self._torque_feedback_timeout
        ):
            self._trigger_safety_stop(
                "TORQUE_MONITOR_FAULT", message, phase, step_id
            )

    def _trigger_safety_stop(
        self,
        reason: str,
        message: str,
        phase: float,
        step_id: str,
    ) -> None:
        with self._safety_stop_lock:
            if self.hardware.protective_stop_active:
                return
            self._safety_fault_phase = float(phase)
            self._safety_fault_step = step_id or "default"
            self._safety_fault_message = str(message)
            self.get_logger().error(f"protective stop: {message}")
            self.hardware.enter_protective_stop(reason)
            with self._safety_lock:
                self._safety_active = False
            if not self._safety_event_saved:
                self._safety_event_logger.save_event(
                    SEVERITY_FAIL, message, phase, step_id
                )
                self._safety_event_saved = True
            self.publish_arm_status()

    def check_torque_profile(
        self,
        phase: float,
        step_id: str = "",
        *,
        effort: np.ndarray | list[float] | None = None,
    ) -> tuple[bool, str, str]:
        if not self.torque_profile_monitor.enabled:
            return False, "OK", ""
        if effort is None:
            _, _, effort = self.hardware.get_joint_state()

        with self._safety_check_lock:
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
        if self.torque_profile_monitor.last_derivative > (
            self.torque_profile_monitor.derivative_limit
        ):
            derivative_message = (
                f"{self.torque_profile_monitor.last_derivative_joint}: "
                "torque derivative="
                f"{self.torque_profile_monitor.last_derivative:.3f} Nm/s"
            )
            message = "; ".join(filter(None, (message, derivative_message)))
        now = self.get_clock().now().nanoseconds * 1e-9
        if should_stop:
            reason = (
                "TORQUE_DERIVATIVE_FAIL"
                if self.torque_profile_monitor.critical_derivative_hit
                else "TORQUE_PROFILE_FAIL"
            )
            self._trigger_safety_stop(
                reason,
                message or "torque profile violation",
                phase,
                step_id or self.torque_profile_monitor.active_step_id,
            )
        elif (
            severity in (SEVERITY_WARN, SEVERITY_FAIL)
            and now - self._last_torque_profile_warning > 1.0
        ):
            log = (
                self.get_logger().error
                if severity == SEVERITY_FAIL
                else self.get_logger().warn
            )
            log(f"torque profile {severity.lower()}: {message}")
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
            profile.validate_joint_coverage(self.hardware.joint_names)
        except Exception as exc:
            self.get_logger().error(f"failed to load torque profile {path!r}: {exc}")
            return None
        self.get_logger().info(f"loaded torque profile: {path}")
        return profile

    def shutdown(self) -> None:
        self._deadline_shutdown.set()
        self._deadline_fault_event.set()
        supervisor = getattr(self, "_deadline_supervisor", None)
        if supervisor is not None and supervisor.is_alive():
            supervisor.join(timeout=1.0)
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
