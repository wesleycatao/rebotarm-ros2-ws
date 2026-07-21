"""ROS 2 bridge for the reBotArm GUI.

This module runs a dedicated ``rclpy`` node inside a ``QThread``. All ROS
subscriptions, service clients and action clients live here, and results are
delivered to the Qt UI layer via signals (thread-safe).

The bridge is a pure client: it never touches hardware directly, only the
interfaces exposed by the ``reBotArmController`` node under ``/rebotarm``.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, Pose as GeometryPose, Quaternion
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from rclpy.task import Future
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from PySide6.QtCore import QObject, QThread, Signal
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.msg import (
    ArmStatus,
    DigitalIO,
    DigitalIOState,
    JointMitCmd,
    JointMotorState,
    JointPosVelCmd,
    SegmentedObjectArray,
)
from rebotarm_msgs.srv import CalibrateVision, CheckTorqueProfile
from rebotarm_msgs.srv import GripperCommand as GripperCommandSrv
from rebotarm_msgs.srv import MoveToPoseIK, SetDigitalIOConfig, SetDigitalOutput, SetGripper, SetZero
from rebotarm_msgs.srv import SetSafetyStep, SetTorqueProfile
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from . import kinematics


def duration_msg(seconds: float) -> Duration:
    """Convert seconds into a ``builtin_interfaces/Duration``."""
    sec = int(seconds)
    nanosec = int((float(seconds) - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def make_pose(
    x: float,
    y: float,
    z: float,
    qx: float = 0.0,
    qy: float = 0.0,
    qz: float = 0.0,
    qw: float = 1.0,
) -> Pose:
    """Build a :class:`geometry_msgs/Pose` from position + quaternion."""
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation = Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))
    return pose


def _parameter_value(value: object) -> ParameterValue:
    parameter_value = ParameterValue()
    if isinstance(value, bool):
        parameter_value.type = ParameterType.PARAMETER_BOOL
        parameter_value.bool_value = bool(value)
    elif isinstance(value, int):
        parameter_value.type = ParameterType.PARAMETER_INTEGER
        parameter_value.integer_value = int(value)
    elif isinstance(value, float):
        parameter_value.type = ParameterType.PARAMETER_DOUBLE
        parameter_value.double_value = float(value)
    elif isinstance(value, str):
        parameter_value.type = ParameterType.PARAMETER_STRING
        parameter_value.string_value = str(value)
    elif isinstance(value, (list, tuple)):
        if all(isinstance(item, int) and not isinstance(item, bool) for item in value):
            parameter_value.type = ParameterType.PARAMETER_INTEGER_ARRAY
            parameter_value.integer_array_value = [int(item) for item in value]
        elif all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in value):
            parameter_value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            parameter_value.double_array_value = [float(item) for item in value]
        else:
            parameter_value.type = ParameterType.PARAMETER_STRING_ARRAY
            parameter_value.string_array_value = [str(item) for item in value]
    else:
        parameter_value.type = ParameterType.PARAMETER_STRING
        parameter_value.string_value = str(value)
    return parameter_value


class _BridgeNode(Node):
    """Internal node owning every subscription / client."""

    def __init__(self, namespace: str = "rebotarm") -> None:
        super().__init__("rebotarm_gui")
        self._ns = namespace.strip().strip("/")
        self._cb_reentrant = ReentrantCallbackGroup()
        self._diagnostic_lock = threading.RLock()
        self._topic_metrics: dict[str, dict[str, object]] = {}
        self._telemetry_errors: deque[str] = deque(maxlen=20)
        self._last_telemetry_error: tuple[str, float] | None = None

        # ---- Latest state cache (read from the UI thread) ----
        self.joint_state: Optional[JointState] = None
        self.arm_status: Optional[ArmStatus] = None
        self.last_arm_status_stamp: Optional[float] = None  # monotonic; heartbeat
        self.robot_description: Optional[str] = None
        self.gripper_state: Optional[JointMotorState] = None
        self.joint_detail: dict[str, JointMotorState] = {}
        self.dio_ios: dict[str, DigitalIO] = {}  # id -> latest DigitalIO snapshot
        self.vision_detected: bool = False
        self.vision_pose: Optional[GeometryPose] = None
        self.vision_objects: Optional[SegmentedObjectArray] = None
        self.vision_debug_image = None
        self.vision_raw_image = None
        self._vision_objects_dirty = False
        self._vision_debug_dirty = False
        self._vision_raw_dirty = False
        self._vision_calibration_progress_message: Optional[str] = None
        self._vision_calibration_progress_dirty = False

        # ---- Subscriptions ----
        self.create_subscription(
            JointState,
            f"/{self._ns}/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            ArmStatus,
            f"/{self._ns}/arm_status",
            self._arm_status_cb,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )

        self.create_subscription(
            JointMotorState,
            f"/{self._ns}/gripper/state",
            self._gripper_state_cb,
            qos_profile_sensor_data,
        )

        # URDF published by robot_state_publisher (latched). Used to derive
        # joint limits and forward kinematics for the UI. Subscribed both at
        # the global scope (real bringup) and inside the arm namespace (the
        # mock_visualize launch namespaces its robot_state_publisher).
        for urdf_topic in ("/robot_description", f"/{self._ns}/robot_description"):
            self.create_subscription(
                String,
                urdf_topic,
                lambda msg, topic=urdf_topic: self._robot_description_cb(msg, topic),
                QoSProfile(
                    depth=1,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                ),
            )

        self.create_subscription(
            DigitalIOState,
            f"/{self._ns}/dio/state",
            self._digital_io_cb,
            qos_profile_sensor_data,
        )

        # Computer-vision signals (published by an external webcam/vision node
        # on the Raspberry Pi). Subscribed defensively: the topics may not
        # exist yet, in which case the cache keeps its defaults.
        self.create_subscription(
            Bool,
            f"/{self._ns}/vision/detected",
            self._vision_detected_cb,
            10,
        )
        self.create_subscription(
            GeometryPose,
            f"/{self._ns}/vision/part_pose",
            self._vision_pose_cb,
            10,
        )
        self.create_subscription(
            SegmentedObjectArray,
            f"/{self._ns}/vision/objects",
            lambda msg: self._vision_objects_cb(msg, f"/{self._ns}/vision/objects"),
            10,
        )
        self.create_subscription(
            SegmentedObjectArray,
            f"/{self._ns}/vision/objects_2d",
            lambda msg: self._vision_objects_cb(msg, f"/{self._ns}/vision/objects_2d"),
            10,
        )
        self.create_subscription(
            Image,
            f"/{self._ns}/vision/debug_image",
            self._vision_debug_image_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/image_raw",
            self._vision_raw_image_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/rebotarm_vision_calibration/progress",
            self._vision_calibration_progress_cb,
            10,
        )

        # Joint detail topics are created lazily once joint names are known.
        self._joint_detail_created: set[str] = set()

        # ---- Service clients ----
        self.cli_enable = self.create_client(Trigger, f"/{self._ns}/enable")
        self.cli_disable = self.create_client(Trigger, f"/{self._ns}/disable")
        self.cli_safe_home = self.create_client(Trigger, f"/{self._ns}/safe_home")
        self.cli_grav_start = self.create_client(
            Trigger, f"/{self._ns}/gravity_compensation/start"
        )
        self.cli_grav_stop = self.create_client(
            Trigger, f"/{self._ns}/gravity_compensation/stop"
        )
        self.cli_set_zero = self.create_client(SetZero, f"/{self._ns}/set_zero")
        self.cli_move_to_pose_ik = self.create_client(
            MoveToPoseIK, f"/{self._ns}/move_to_pose_ik", callback_group=self._cb_reentrant
        )
        self.cli_gripper_set = self.create_client(
            SetGripper, f"/{self._ns}/gripper/set", callback_group=self._cb_reentrant
        )
        self.cli_gripper_open = self.create_client(
            GripperCommandSrv, f"/{self._ns}/gripper/open", callback_group=self._cb_reentrant
        )
        self.cli_gripper_close = self.create_client(
            GripperCommandSrv, f"/{self._ns}/gripper/close", callback_group=self._cb_reentrant
        )
        self.cli_safety_set_profile = self.create_client(
            SetTorqueProfile,
            f"/{self._ns}/safety/set_torque_profile",
            callback_group=self._cb_reentrant,
        )
        self.cli_safety_set_step = self.create_client(
            SetSafetyStep,
            f"/{self._ns}/safety/set_step",
            callback_group=self._cb_reentrant,
        )
        self.cli_safety_check = self.create_client(
            CheckTorqueProfile,
            f"/{self._ns}/safety/check_torque_profile",
            callback_group=self._cb_reentrant,
        )
        self.cli_safety_reset_fault = self.create_client(
            Trigger,
            f"/{self._ns}/safety/reset_fault",
            callback_group=self._cb_reentrant,
        )
        self.cli_vision_set_parameters = self.create_client(
            SetParameters,
            "/rebotarm_segmentation/set_parameters",
            callback_group=self._cb_reentrant,
        )
        self.cli_vision_pose_set_parameters = self.create_client(
            SetParameters,
            "/rebotarm_object_pose/set_parameters",
            callback_group=self._cb_reentrant,
        )
        self.cli_pick_scheduler_set_parameters = self.create_client(
            SetParameters,
            "/rebotarm_pick_scheduler/set_parameters",
            callback_group=self._cb_reentrant,
        )
        self.cli_vision_process_frame = self.create_client(
            Trigger,
            "/rebotarm_segmentation/process_frame",
            callback_group=self._cb_reentrant,
        )
        self.cli_vision_calibrate = self.create_client(
            CalibrateVision,
            "/rebotarm_vision_calibration/calibrate",
            callback_group=self._cb_reentrant,
        )
        self.cli_dio_set_output = self.create_client(
            SetDigitalOutput,
            f"/{self._ns}/dio/set_output",
            callback_group=self._cb_reentrant,
        )
        self.cli_dio_set_config = self.create_client(
            SetDigitalIOConfig,
            f"/{self._ns}/dio/set_config",
            callback_group=self._cb_reentrant,
        )

        # ---- Action clients ----
        self.ac_move_to_pose = ActionClient(
            self, MoveToPose, f"/{self._ns}/move_to_pose", callback_group=self._cb_reentrant
        )
        self.ac_follow_joint_trajectory = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._ns}/follow_joint_trajectory",
            callback_group=self._cb_reentrant,
        )
        self.ac_gripper_command = ActionClient(
            self,
            GripperCommand,
            f"/{self._ns}/gripper/command",
            callback_group=self._cb_reentrant,
        )

        # ---- Raw command publishers (created lazily) ----
        self._pubs_mit: dict[str, object] = {}
        self._pubs_posvel: dict[str, object] = {}
        self._pub_vision_test_image = self.create_publisher(Image, "/camera/image_raw", 10)

        # Stable names are kept alongside the clients so the GUI can expose
        # discovery state without invoking command-line ROS tools.
        self._diagnostic_services = {
            f"/{self._ns}/enable": self.cli_enable,
            f"/{self._ns}/disable": self.cli_disable,
            f"/{self._ns}/safe_home": self.cli_safe_home,
            f"/{self._ns}/gravity_compensation/start": self.cli_grav_start,
            f"/{self._ns}/gravity_compensation/stop": self.cli_grav_stop,
            f"/{self._ns}/set_zero": self.cli_set_zero,
            f"/{self._ns}/move_to_pose_ik": self.cli_move_to_pose_ik,
            f"/{self._ns}/gripper/set": self.cli_gripper_set,
            f"/{self._ns}/gripper/open": self.cli_gripper_open,
            f"/{self._ns}/gripper/close": self.cli_gripper_close,
            f"/{self._ns}/safety/set_torque_profile": self.cli_safety_set_profile,
            f"/{self._ns}/safety/set_step": self.cli_safety_set_step,
            f"/{self._ns}/safety/check_torque_profile": self.cli_safety_check,
            f"/{self._ns}/safety/reset_fault": self.cli_safety_reset_fault,
            "/rebotarm_segmentation/set_parameters": self.cli_vision_set_parameters,
            "/rebotarm_object_pose/set_parameters": self.cli_vision_pose_set_parameters,
            "/rebotarm_pick_scheduler/set_parameters": self.cli_pick_scheduler_set_parameters,
            "/rebotarm_segmentation/process_frame": self.cli_vision_process_frame,
            "/rebotarm_vision_calibration/calibrate": self.cli_vision_calibrate,
            f"/{self._ns}/dio/set_output": self.cli_dio_set_output,
            f"/{self._ns}/dio/set_config": self.cli_dio_set_config,
        }
        self._diagnostic_actions = {
            f"/{self._ns}/move_to_pose": self.ac_move_to_pose,
            f"/{self._ns}/follow_joint_trajectory": self.ac_follow_joint_trajectory,
            f"/{self._ns}/gripper/command": self.ac_gripper_command,
        }

        for topic in (
            f"/{self._ns}/joint_states",
            f"/{self._ns}/arm_status",
            f"/{self._ns}/gripper/state",
            "/robot_description",
            f"/{self._ns}/robot_description",
            f"/{self._ns}/dio/state",
            f"/{self._ns}/vision/detected",
            f"/{self._ns}/vision/part_pose",
            f"/{self._ns}/vision/objects",
            f"/{self._ns}/vision/objects_2d",
            f"/{self._ns}/vision/debug_image",
            "/camera/image_raw",
            "/rebotarm_vision_calibration/progress",
        ):
            self._ensure_topic_metric(topic)

    # ------------------------------------------------------------------
    # namespace helpers
    # ------------------------------------------------------------------
    @property
    def namespace(self) -> str:
        return self._ns

    def arm_joint_names(self) -> list[str]:
        """Return only the arm joint names (excluding ``gripper*``)."""
        if self.joint_state is None:
            return []
        return [n for n in self.joint_state.name if not n.startswith("gripper")]

    def all_joint_names(self) -> list[str]:
        return list(self.joint_state.name) if self.joint_state else []

    # ------------------------------------------------------------------
    # diagnostics
    # ------------------------------------------------------------------
    def _ensure_topic_metric(self, topic: str) -> dict[str, object]:
        with self._diagnostic_lock:
            return self._topic_metrics.setdefault(
                str(topic),
                {
                    "samples": deque(maxlen=500),
                    "last_seen": None,
                    "latency_ms": None,
                },
            )

    def _record_topic(self, topic: str, msg: object) -> None:
        """Record receipt time and header latency for one subscribed topic."""
        now = time.monotonic()
        latency_ms: float | None = None
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None)
        sec = int(getattr(stamp, "sec", 0)) if stamp is not None else 0
        nanosec = int(getattr(stamp, "nanosec", 0)) if stamp is not None else 0
        if sec or nanosec:
            message_ns = sec * 1_000_000_000 + nanosec
            latency_ms = (self.get_clock().now().nanoseconds - message_ns) / 1_000_000.0
            # A different clock domain (or a system clock jump) makes transport
            # latency meaningless. Preserve the topic rate but report N/D.
            if abs(latency_ms) > 86_400_000.0:
                latency_ms = None
        metric = self._ensure_topic_metric(topic)
        with self._diagnostic_lock:
            samples = metric["samples"]
            if isinstance(samples, deque):
                samples.append(now)
            metric["last_seen"] = now
            metric["latency_ms"] = latency_ms

    def diagnostic_snapshot(self, heartbeat_timeout_s: float = 1.0) -> dict[str, object]:
        """Return a non-blocking snapshot of topics and endpoint discovery."""
        now = time.monotonic()
        topics: list[dict[str, object]] = []
        with self._diagnostic_lock:
            metrics = list(self._topic_metrics.items())
            for topic, metric in metrics:
                samples = metric["samples"]
                if isinstance(samples, deque):
                    while samples and now - samples[0] > 5.0:
                        samples.popleft()
                    hz = (
                        (len(samples) - 1) / (samples[-1] - samples[0])
                        if len(samples) >= 2 and samples[-1] > samples[0]
                        else None
                    )
                else:  # pragma: no cover - defensive
                    hz = None
                last_seen = metric["last_seen"]
                age_s = now - float(last_seen) if last_seen is not None else None
                topics.append(
                    {
                        "name": topic,
                        "hz": hz,
                        "age_ms": age_s * 1000.0 if age_s is not None else None,
                        "latency_ms": metric["latency_ms"],
                        "received": last_seen is not None,
                    }
                )

        services = [
            {"name": name, "ready": self._client_ready(client, action=False)}
            for name, client in self._diagnostic_services.items()
        ]
        actions = [
            {"name": name, "ready": self._client_ready(client, action=True)}
            for name, client in self._diagnostic_actions.items()
        ]
        heartbeat = next(
            (row for row in topics if row["name"] == f"/{self._ns}/arm_status"),
            {"hz": None, "age_ms": None, "latency_ms": None, "received": False},
        )
        heartbeat_age = heartbeat.get("age_ms")
        heartbeat["alive"] = bool(
            heartbeat_age is not None
            and float(heartbeat_age) < max(0.0, float(heartbeat_timeout_s)) * 1000.0
        )
        heartbeat["timeout_ms"] = float(heartbeat_timeout_s) * 1000.0
        return {
            "connected": True,
            "heartbeat": heartbeat,
            "topics": sorted(topics, key=lambda item: str(item["name"])),
            "services": services,
            "actions": actions,
            "captured_at": time.time(),
        }

    @staticmethod
    def _client_ready(client: object, *, action: bool) -> bool:
        try:
            if action:
                return bool(client.server_is_ready())
            return bool(client.service_is_ready())
        except Exception:
            return False

    # ------------------------------------------------------------------
    # subscription callbacks
    # ------------------------------------------------------------------
    @staticmethod
    def _validate_joint_state(msg: JointState) -> str | None:
        """Return a safety-oriented validation error, or ``None`` when usable."""
        names = list(msg.name)
        if not names:
            return "lista de juntas vazia"
        if any(not str(name).strip() for name in names):
            return "nome de junta vazio"
        if len(set(names)) != len(names):
            return "nomes de juntas duplicados"
        if len(msg.position) != len(names):
            return f"position possui {len(msg.position)} valor(es) para {len(names)} junta(s)"
        for field_name in ("velocity", "effort"):
            values = getattr(msg, field_name)
            if values and len(values) != len(names):
                return f"{field_name} possui {len(values)} valor(es) para {len(names)} junta(s)"
        numeric_values = list(msg.position) + list(msg.velocity) + list(msg.effort)
        if any(not math.isfinite(float(value)) for value in numeric_values):
            return "valor NaN ou infinito"
        return None

    def _queue_telemetry_error(self, topic: str, reason: str) -> None:
        now = time.monotonic()
        key = f"{topic}: {reason}"
        with self._diagnostic_lock:
            previous = self._last_telemetry_error
            if previous is not None and previous[0] == key and now - previous[1] < 1.0:
                return
            self._last_telemetry_error = (key, now)
            self._telemetry_errors.append(key)

    def take_telemetry_errors(self) -> list[str]:
        """Drain validation errors for delivery on the Qt thread boundary."""
        with self._diagnostic_lock:
            errors = list(self._telemetry_errors)
            self._telemetry_errors.clear()
        return errors

    def _joint_state_cb(self, msg: JointState) -> None:
        topic = f"/{self._ns}/joint_states"
        self._record_topic(topic, msg)
        validation_error = self._validate_joint_state(msg)
        if validation_error is not None:
            self._queue_telemetry_error(topic, validation_error)
            return
        self.joint_state = msg
        self._ensure_joint_detail_subs()

    def _arm_status_cb(self, msg: ArmStatus) -> None:
        self._record_topic(f"/{self._ns}/arm_status", msg)
        self.arm_status = msg
        self.last_arm_status_stamp = time.monotonic()

    def _robot_description_cb(self, msg: String, topic: str = "/robot_description") -> None:
        self._record_topic(topic, msg)
        if msg.data:
            self.robot_description = str(msg.data)

    def _gripper_state_cb(self, msg: JointMotorState) -> None:
        self._record_topic(f"/{self._ns}/gripper/state", msg)
        self.gripper_state = msg

    def _digital_io_cb(self, msg: DigitalIOState) -> None:
        self._record_topic(f"/{self._ns}/dio/state", msg)
        ios = getattr(msg, "ios", None)
        if not ios:
            return
        dio_ios: dict[str, DigitalIO] = {}
        for io in ios:
            dio_ios[str(io.id)] = io
        self.dio_ios = dio_ios

    def _vision_detected_cb(self, msg: Bool) -> None:
        self._record_topic(f"/{self._ns}/vision/detected", msg)
        self.vision_detected = bool(msg.data)

    def _vision_pose_cb(self, msg: GeometryPose) -> None:
        self._record_topic(f"/{self._ns}/vision/part_pose", msg)
        self.vision_pose = msg

    def _vision_objects_cb(self, msg: SegmentedObjectArray, topic: str) -> None:
        self._record_topic(topic, msg)
        self.vision_objects = msg
        self._vision_objects_dirty = True
        self.vision_detected = bool(msg.objects)
        if msg.objects:
            self.vision_pose = msg.objects[0].pose

    def _vision_debug_image_cb(self, msg: Image) -> None:
        self._record_topic(f"/{self._ns}/vision/debug_image", msg)
        self.vision_debug_image = msg
        self._vision_debug_dirty = True

    def _vision_raw_image_cb(self, msg: Image) -> None:
        self._record_topic("/camera/image_raw", msg)
        self.vision_raw_image = msg
        self._vision_raw_dirty = True

    def _vision_calibration_progress_cb(self, msg: String) -> None:
        self._record_topic("/rebotarm_vision_calibration/progress", msg)
        self._vision_calibration_progress_message = str(msg.data)
        self._vision_calibration_progress_dirty = True

    def _joint_detail_cb_factory(self, joint_name: str):
        def _cb(msg: JointMotorState) -> None:
            self._record_topic(f"/{self._ns}/joints/{joint_name}/state", msg)
            self.joint_detail[joint_name] = msg

        return _cb

    def _ensure_joint_detail_subs(self) -> None:
        if self.joint_state is None:
            return
        for name in self.joint_state.name:
            if name in self._joint_detail_created:
                continue
            self.create_subscription(
                JointMotorState,
                f"/{self._ns}/joints/{name}/state",
                self._joint_detail_cb_factory(name),
                qos_profile_sensor_data,
            )
            self._ensure_topic_metric(f"/{self._ns}/joints/{name}/state")
            self._joint_detail_created.add(name)

    def ensure_raw_publishers(self) -> None:
        """Create raw command publishers for every known joint."""
        if self.joint_state is None:
            return
        for name in self.joint_state.name:
            if name not in self._pubs_mit:
                self._pubs_mit[name] = self.create_publisher(
                    JointMitCmd, f"/{self._ns}/joints/{name}/cmd/mit", 10
                )
            if name not in self._pubs_posvel:
                self._pubs_posvel[name] = self.create_publisher(
                    JointPosVelCmd, f"/{self._ns}/joints/{name}/cmd/pos_vel", 10
                )

    # ------------------------------------------------------------------
    # publish helpers
    # ------------------------------------------------------------------
    def publish_mit(
        self, joint_name: str, pos: float, vel: float, kp: float, kd: float, tau: float
    ) -> bool:
        pub = self._pubs_mit.get(joint_name)
        if pub is None:
            return False
        msg = JointMitCmd(
            pos=float(pos), vel=float(vel), kp=float(kp), kd=float(kd), tau=float(tau)
        )
        msg.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)
        return True

    def publish_posvel(self, joint_name: str, pos: float, vlim: float) -> bool:
        pub = self._pubs_posvel.get(joint_name)
        if pub is None:
            return False
        msg = JointPosVelCmd(pos=float(pos), vlim=float(vlim))
        msg.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)
        return True

    # ------------------------------------------------------------------
    # service-call helpers (non-blocking, return Future)
    # ------------------------------------------------------------------
    def _call_service(self, client, request) -> Optional[Future]:
        if not client.wait_for_service(timeout_sec=0.01):
            return None
        return client.call_async(request)

    def call_enable(self) -> Optional[Future]:
        return self._call_service(self.cli_enable, Trigger.Request())

    def call_disable(self) -> Optional[Future]:
        return self._call_service(self.cli_disable, Trigger.Request())

    def call_safe_home(self) -> Optional[Future]:
        return self._call_service(self.cli_safe_home, Trigger.Request())

    def call_gravity_start(self) -> Optional[Future]:
        return self._call_service(self.cli_grav_start, Trigger.Request())

    def call_gravity_stop(self) -> Optional[Future]:
        return self._call_service(self.cli_grav_stop, Trigger.Request())

    def call_set_zero(self, joint_name: str = "") -> Optional[Future]:
        req = SetZero.Request()
        req.joint_name = joint_name
        return self._call_service(self.cli_set_zero, req)

    def call_move_to_pose_ik(self, pose: Pose) -> Optional[Future]:
        req = MoveToPoseIK.Request()
        req.target_pose = pose
        return self._call_service(self.cli_move_to_pose_ik, req)

    def call_gripper_set(self, position: float) -> Optional[Future]:
        req = SetGripper.Request()
        req.position = float(position)
        return self._call_service(self.cli_gripper_set, req)

    def call_gripper_open(self, position: float = 0.0, timeout: float = 3.0) -> Optional[Future]:
        req = GripperCommandSrv.Request()
        req.position = float(position)
        req.timeout = float(timeout)
        return self._call_service(self.cli_gripper_open, req)

    def call_gripper_close(self, position: float = 0.0, timeout: float = 3.0) -> Optional[Future]:
        req = GripperCommandSrv.Request()
        req.position = float(position)
        req.timeout = float(timeout)
        return self._call_service(self.cli_gripper_close, req)

    def call_safety_set_profile(
        self,
        profile_path: str,
        mode: str,
        violation_samples: int,
        active_step_id: str,
        derivative_limit: float = 0.0,
    ) -> Optional[Future]:
        req = SetTorqueProfile.Request()
        req.profile_path = str(profile_path)
        req.mode = str(mode)
        req.violation_samples = int(violation_samples)
        req.active_step_id = str(active_step_id)
        req.derivative_limit = float(derivative_limit)
        return self._call_service(self.cli_safety_set_profile, req)

    def call_safety_set_step(self, step_id: str, payload_kg: float = 0.0) -> Optional[Future]:
        req = SetSafetyStep.Request()
        req.step_id = str(step_id)
        req.payload_kg = float(payload_kg)
        return self._call_service(self.cli_safety_set_step, req)

    def call_safety_check(self, step_id: str, phase: float) -> Optional[Future]:
        req = CheckTorqueProfile.Request()
        req.step_id = str(step_id)
        req.phase = float(phase)
        return self._call_service(self.cli_safety_check, req)

    def call_safety_reset_fault(self) -> Optional[Future]:
        return self._call_service(self.cli_safety_reset_fault, Trigger.Request())

    def call_vision_set_parameters(self, values: dict[str, object]) -> Optional[Future]:
        req = SetParameters.Request()
        req.parameters = [self._make_parameter(name, value) for name, value in values.items()]
        return self._call_service(self.cli_vision_set_parameters, req)

    def call_vision_pose_set_parameters(self, values: dict[str, object]) -> Optional[Future]:
        req = SetParameters.Request()
        req.parameters = [self._make_parameter(name, value) for name, value in values.items()]
        return self._call_service(self.cli_vision_pose_set_parameters, req)

    def call_pick_scheduler_set_parameters(self, values: dict[str, object]) -> Optional[Future]:
        req = SetParameters.Request()
        req.parameters = [self._make_parameter(name, value) for name, value in values.items()]
        return self._call_service(self.cli_pick_scheduler_set_parameters, req)

    def call_vision_process_frame(self) -> Optional[Future]:
        return self._call_service(self.cli_vision_process_frame, Trigger.Request())

    def call_vision_calibrate(
        self,
        image: Image,
        prompt: str,
        api_key: str,
        max_iterations: int,
        max_time_s: float,
    ) -> Optional[Future]:
        req = CalibrateVision.Request()
        req.image = image
        req.prompt = str(prompt)
        req.api_key = str(api_key)
        req.max_iterations = int(max_iterations)
        req.max_time_s = float(max_time_s)
        return self._call_service(self.cli_vision_calibrate, req)

    def call_dio_set_output(self, io_id: str, value: bool) -> Optional[Future]:
        req = SetDigitalOutput.Request()
        req.id = str(io_id)
        req.value = bool(value)
        return self._call_service(self.cli_dio_set_output, req)

    def call_dio_set_config(self, io_id: str, tag: str, enabled: bool) -> Optional[Future]:
        req = SetDigitalIOConfig.Request()
        req.id = str(io_id)
        req.tag = str(tag)
        req.enabled = bool(enabled)
        return self._call_service(self.cli_dio_set_config, req)

    def publish_vision_test_image(
        self,
        width: int,
        height: int,
        rgb_data: bytes,
        step: int | None = None,
        frame_id: str = "camera_link",
    ) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(frame_id)
        msg.height = int(height)
        msg.width = int(width)
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = int(step) if step is not None else int(width) * 3
        msg.data = bytes(rgb_data)
        self._pub_vision_test_image.publish(msg)

    @staticmethod
    def _make_parameter(name: str, value: object) -> Parameter:
        parameter = Parameter()
        parameter.name = str(name)
        parameter.value = _parameter_value(value)
        return parameter

    # ------------------------------------------------------------------
    # action goal helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _action_ready(client: ActionClient) -> bool:
        """Return action-server discovery state without blocking the Qt thread."""
        try:
            return bool(client.server_is_ready())
        except Exception:
            return False

    def action_server_ready(self, name: str) -> bool:
        clients = {
            "move_to_pose": self.ac_move_to_pose,
            "follow_joint_trajectory": self.ac_follow_joint_trajectory,
            "gripper_command": self.ac_gripper_command,
        }
        client = clients.get(str(name))
        return self._action_ready(client) if client is not None else False

    def send_follow_joint_trajectory(
        self, joint_names: list[str], positions: list[float], duration: float
    ) -> Optional[Future]:
        if not self._action_ready(self.ac_follow_joint_trajectory):
            return None
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = duration_msg(duration)
        traj.points = [point]
        goal.trajectory = traj
        return self.ac_follow_joint_trajectory.send_goal_async(goal)

    def send_move_to_pose(
        self, pose: Pose, duration: float, feedback_callback=None
    ) -> Optional[Future]:
        if not self._action_ready(self.ac_move_to_pose):
            return None
        goal = MoveToPose.Goal()
        goal.target_pose = pose
        goal.duration = float(duration)
        return self.ac_move_to_pose.send_goal_async(
            goal, feedback_callback=feedback_callback
        )

    def send_gripper_command(self, position: float, max_effort: float = 0.0) -> Optional[Future]:
        if not self._action_ready(self.ac_gripper_command):
            return None
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        return self.ac_gripper_command.send_goal_async(goal)


class RosBridge(QObject):
    """Qt-facing bridge.

    Runs a dedicated ROS executor inside a ``QThread`` and forwards cached
    state to the UI via Qt signals.
    """

    # ---- Signals (emitted from the ROS thread, consumed in the UI thread) ----
    joint_state_changed = Signal(object)
    arm_status_changed = Signal(object)
    gripper_state_changed = Signal(object)
    joint_detail_changed = Signal(str, object)
    dio_ios_changed = Signal(object)  # dict[str, DigitalIO] id -> snapshot
    service_result = Signal(str, bool, str)  # (label, success, message)
    action_result = Signal(str, bool, str)  # (label, success, message)
    pose_action_result = Signal(str, bool, str)  # (tag, success, message) - tagged MoveToPose
    safety_check_result = Signal(object)
    # (key, success, message, q_solution) — IK reachability probe result.
    ik_pose_checked = Signal(str, bool, str, object)
    vision_objects_changed = Signal(object)
    vision_debug_image_changed = Signal(object)
    vision_raw_image_changed = Signal(object)
    vision_calibration_progress = Signal(str)
    vision_calibration_result = Signal(str, str)  # (profile_json, report)
    log_message = Signal(str, str)  # (level, text)
    connected = Signal(bool)
    arm_alive = Signal(bool)  # heartbeat of /{ns}/arm_status (False = stale)
    move_feedback = Signal(float, object)  # MoveToPose feedback (progress, current_pose)
    diagnostics_changed = Signal(object)  # read-only ROS health snapshot

    def __init__(self, namespace: str = "rebotarm") -> None:
        super().__init__()
        self._namespace = namespace
        self._node: Optional[_BridgeNode] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._thread: Optional[QThread] = None
        self._stopping = threading.Event()
        self._timer_interval = 0.08  # seconds
        self._diagnostic_interval = 1.0
        self._action_lock = threading.RLock()
        self._action_seq = 0
        self._action_requests: dict[int, dict[str, object]] = {}
        self.action_accept_timeout_s = 3.0
        self.action_result_grace_s = 5.0
        self.last_reached_pose: Optional[Pose] = None  # TCP pose captured from action results
        self._arm_alive_state = False
        self.arm_status_timeout = 1.0  # seconds without arm_status -> not alive

    @property
    def node(self) -> Optional[_BridgeNode]:
        return self._node

    @property
    def namespace(self) -> str:
        return self._namespace.strip().strip("/")

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def start(self) -> None:
        import rclpy

        self._stopping.clear()
        rclpy.init()
        self._node = _BridgeNode(self._namespace)
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)

        self._thread = QThread()
        self._thread.run = self._spin_loop
        self._thread.start()
        self.connected.emit(True)
        self._emit_diagnostics()

    def shutdown(self) -> None:
        import rclpy

        self._stopping.set()
        self._clear_action_requests(cancel_handles=False)
        if self._executor is not None:
            try:
                self._executor.wake()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()

        if self._thread is not None:
            self._thread.quit()
            if not self._thread.wait(3000):
                self._thread.terminate()
                self._thread.wait(1000)
            self._thread = None

        if self._executor is not None:
            self._executor.shutdown()
            self._executor = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        self._arm_alive_state = False
        self.connected.emit(False)
        self.diagnostics_changed.emit(self.diagnostic_snapshot())

    def _spin_loop(self) -> None:
        """Spin the ROS executor and periodically emit Qt signals."""
        import rclpy
        from datetime import datetime

        lock = threading.Event()
        last_diagnostic_emit = 0.0
        while not self._stopping.is_set() and rclpy.ok() and self._executor is not None:
            try:
                self._executor.spin_once(timeout_sec=self._timer_interval)
            except Exception as exc:  # pragma: no cover - defensive
                ts = datetime.now().strftime("%H:%M:%S")
                self.log_message.emit("error", f"[{ts}] spin error: {exc}")
                lock.wait(0.2)
                continue

            node = self._node
            if node is None:
                break

            if node.joint_state is not None:
                self.joint_state_changed.emit(node.joint_state)
                node.ensure_raw_publishers()

            stamp = node.last_arm_status_stamp
            alive = stamp is not None and (time.monotonic() - stamp) < self.arm_status_timeout
            if alive != self._arm_alive_state:
                self._arm_alive_state = alive
                if not alive:
                    cancelled = self.cancel_active_actions()
                    if cancelled:
                        self.log_message.emit(
                            "warn",
                            "heartbeat perdido: actions ativas foram canceladas",
                        )
                self.arm_alive.emit(alive)
            if alive and node.arm_status is not None:
                self.arm_status_changed.emit(node.arm_status)

            if node.gripper_state is not None:
                self.gripper_state_changed.emit(node.gripper_state)

            if node.dio_ios:
                self.dio_ios_changed.emit(node.dio_ios)

            if node.vision_objects is not None and node._vision_objects_dirty:
                self.vision_objects_changed.emit(node.vision_objects)
                node._vision_objects_dirty = False

            if node.vision_debug_image is not None and node._vision_debug_dirty:
                self.vision_debug_image_changed.emit(node.vision_debug_image)
                node._vision_debug_dirty = False

            if node.vision_raw_image is not None and node._vision_raw_dirty:
                self.vision_raw_image_changed.emit(node.vision_raw_image)
                node._vision_raw_dirty = False

            if node._vision_calibration_progress_dirty:
                self.vision_calibration_progress.emit(
                    node._vision_calibration_progress_message or ""
                )
                node._vision_calibration_progress_dirty = False

            for jn, detail in list(node.joint_detail.items()):
                self.joint_detail_changed.emit(jn, detail)

            for error in node.take_telemetry_errors():
                self.log_message.emit("error", f"telemetria inválida em {error}")

            now = time.monotonic()
            if now - last_diagnostic_emit >= self._diagnostic_interval:
                self._emit_diagnostics()
                last_diagnostic_emit = now

    def diagnostic_snapshot(self) -> dict[str, object]:
        """Return current ROS health data without network calls or waiting."""
        node = self._node
        if node is None:
            return {
                "connected": False,
                "heartbeat": {
                    "alive": False,
                    "received": False,
                    "hz": None,
                    "age_ms": None,
                    "latency_ms": None,
                    "timeout_ms": self.arm_status_timeout * 1000.0,
                },
                "topics": [],
                "services": [],
                "actions": [],
                "captured_at": time.time(),
            }
        return node.diagnostic_snapshot(self.arm_status_timeout)

    def _emit_diagnostics(self) -> None:
        try:
            self.diagnostics_changed.emit(self.diagnostic_snapshot())
        except Exception as exc:  # pragma: no cover - defensive
            self.log_message.emit("error", f"falha ao coletar diagnóstico ROS: {exc}")

    # ------------------------------------------------------------------
    # call wrappers (called from the UI thread)
    # ------------------------------------------------------------------
    def _future_done(self, label: str, future: Future, success_key: str = "success") -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.service_result.emit(label, False, str(exc))
            return
        success = bool(getattr(result, success_key, False))
        message = getattr(result, "message", "") or getattr(result, "error_string", "")
        self.service_result.emit(label, success, str(message))

    def _call(self, label: str, future: Optional[Future], success_key: str = "success") -> bool:
        if future is None:
            self.log_message.emit("warn", f"{label}: serviço indisponível")
            self.service_result.emit(label, False, "serviço indisponível")
            return False
        future.add_done_callback(lambda f: self._future_done(label, f, success_key))
        self.log_message.emit("info", f"chamando serviço {label} ...")
        return True

    # Services ---------------------------------------------------------
    def enable(self) -> bool:
        node = self._node
        return self._call("enable", node.call_enable() if node else None)

    def disable(self) -> bool:
        node = self._node
        return self._call("disable", node.call_disable() if node else None)

    def safe_home(self) -> bool:
        node = self._node
        return self._call("safe_home", node.call_safe_home() if node else None)

    def gravity_start(self) -> bool:
        node = self._node
        return self._call("gravity_start", node.call_gravity_start() if node else None)

    def gravity_stop(self) -> bool:
        node = self._node
        return self._call("gravity_stop", node.call_gravity_stop() if node else None)

    def set_zero(self, joint_name: str = "") -> bool:
        label = f"set_zero({joint_name})" if joint_name else "set_zero(all)"
        node = self._node
        return self._call(label, node.call_set_zero(joint_name) if node else None)

    def move_to_pose_ik(self, pose: Pose) -> bool:
        node = self._node
        return self._call("move_to_pose_ik", node.call_move_to_pose_ik(pose) if node else None)

    def check_pose_reachable(self, key: str, pose_dict: dict) -> bool:
        """Probe IK reachability for one pose; emit ``ik_pose_checked``.

        ``pose_dict`` uses the GUI convention (x, y, z, roll, pitch, yaw in
        metres / radians). The full service response (including ``q_solution``)
        is forwarded via :attr:`ik_pose_checked` so callers can additionally
        inspect joint limits. Returns False (and emits nothing) when the
        service is not available, so the caller can count it as skipped.
        """
        node = self._node
        if node is None:
            return False
        try:
            from .motion.trajectory import rpy_to_quat
            qx, qy, qz, qw = rpy_to_quat(
                float(pose_dict.get("roll", 0.0)),
                float(pose_dict.get("pitch", 0.0)),
                float(pose_dict.get("yaw", 0.0)),
            )
        except Exception:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        pose = make_pose(
            float(pose_dict.get("x", 0.0)),
            float(pose_dict.get("y", 0.0)),
            float(pose_dict.get("z", 0.0)),
            qx, qy, qz, qw,
        )
        future = node.call_move_to_pose_ik(pose)
        if future is None:
            return False
        future.add_done_callback(lambda f, k=str(key): self._on_ik_done(k, f))
        return True

    def _on_ik_done(self, key: str, future: Future) -> None:
        """Forward the full IK response (with q_solution) to the UI thread."""
        try:
            result = future.result()
        except Exception as exc:
            self.ik_pose_checked.emit(key, False, str(exc), [])
            return
        success = bool(getattr(result, "success", False))
        message = str(getattr(result, "message", "") or "")
        q_solution = list(getattr(result, "q_solution", []) or [])
        self.ik_pose_checked.emit(key, success, message, q_solution)

    def set_dio_output(self, io_id: str, value: bool) -> bool:
        label = f"dio_set_output({io_id}={'ON' if value else 'OFF'})"
        return self._call(label, self._node.call_dio_set_output(io_id, value))

    def set_dio_config(self, io_id: str, tag: str, enabled: bool) -> bool:
        label = f"dio_set_config({io_id})"
        return self._call(label, self._node.call_dio_set_config(io_id, tag, enabled))

    def gripper_set(self, position: float) -> bool:
        return self._call(
            "gripper_set", self._node.call_gripper_set(position), success_key="success"
        )

    def gripper_open(self, position: float = 0.0, timeout: float = 3.0) -> bool:
        return self._call(
            "gripper_open",
            self._node.call_gripper_open(position, timeout),
            success_key="success",
        )

    def gripper_close(self, position: float = 0.0, timeout: float = 3.0) -> bool:
        return self._call(
            "gripper_close",
            self._node.call_gripper_close(position, timeout),
            success_key="success",
        )

    def safety_set_profile(
        self,
        profile_path: str,
        mode: str,
        violation_samples: int,
        active_step_id: str = "",
        derivative_limit: float = 0.0,
    ) -> bool:
        return self._call(
            "safety_set_profile",
            self._node.call_safety_set_profile(
                profile_path,
                mode,
                violation_samples,
                active_step_id,
                derivative_limit,
            ),
        )

    def safety_set_step(self, step_id: str, payload_kg: float = 0.0) -> bool:
        return self._call(
            f"safety_set_step({step_id or 'default'})",
            self._node.call_safety_set_step(step_id, payload_kg),
        )

    def safety_reset_fault(self) -> bool:
        return self._call(
            "safety_reset_fault",
            self._node.call_safety_reset_fault(),
        )

    def safety_check(self, step_id: str, phase: float) -> bool:
        future = self._node.call_safety_check(step_id, phase)
        if future is None:
            self.log_message.emit("warn", "safety_check: serviço indisponível")
            self.service_result.emit("safety_check", False, "serviço indisponível")
            return False
        future.add_done_callback(self._safety_check_done)
        self.log_message.emit("info", "verificando perfil de torque ...")
        return True

    def _safety_check_done(self, future: Future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.service_result.emit("safety_check", False, str(exc))
            return
        self.safety_check_result.emit(result)

    def vision_set_parameters(self, values: dict[str, object]) -> bool:
        if self._node is None:
            return False
        future = self._node.call_vision_set_parameters(values)
        if future is None:
            self.log_message.emit("warn", "vision: nó rebotarm_segmentation indisponível")
            self.service_result.emit("vision_set_parameters", False, "nó indisponível")
            return False
        future.add_done_callback(self._vision_params_done)
        self.log_message.emit("info", "aplicando parâmetros de visão ...")
        return True

    def vision_pose_set_parameters(self, values: dict[str, object]) -> bool:
        if self._node is None:
            return False
        future = self._node.call_vision_pose_set_parameters(values)
        if future is None:
            self.log_message.emit("warn", "vision: nó rebotarm_object_pose indisponível")
            self.service_result.emit("vision_pose_set_parameters", False, "nó indisponível")
            return False
        future.add_done_callback(self._vision_pose_params_done)
        self.log_message.emit("info", "aplicando correção XY da mesa ...")
        return True

    def pick_scheduler_set_parameters(self, values: dict[str, object]) -> bool:
        if self._node is None:
            return False
        future = self._node.call_pick_scheduler_set_parameters(values)
        if future is None:
            self.log_message.emit("warn", "visão: nó rebotarm_pick_scheduler indisponível")
            self.service_result.emit("pick_scheduler_set_parameters", False, "nó indisponível")
            return False
        future.add_done_callback(self._pick_scheduler_params_done)
        self.log_message.emit("info", "aplicando planejamento da rota de pega ...")
        return True

    def vision_process_frame(self) -> bool:
        if self._node is None:
            return False
        future = self._node.call_vision_process_frame()
        if future is None:
            self.log_message.emit("warn", "vision: serviço process_frame indisponível")
            self.service_result.emit("vision_process_frame", False, "serviço indisponível")
            return False
        future.add_done_callback(self._vision_process_done)
        self.log_message.emit("info", "processando um frame de visão ...")
        return True

    def vision_calibrate(
        self,
        image: Image,
        prompt: str,
        api_key: str = "",
        max_iterations: int = 5,
        max_time_s: float = 300.0,
    ) -> bool:
        if self._node is None:
            return False
        future = self._node.call_vision_calibrate(
            image,
            prompt,
            api_key,
            max_iterations,
            max_time_s,
        )
        if future is None:
            self.log_message.emit("warn", "vision: serviço de calibração indisponível")
            self.service_result.emit("vision_calibrate", False, "serviço indisponível")
            return False
        future.add_done_callback(self._vision_calibrate_done)
        self.log_message.emit("info", "iniciando calibração de visão por IA ...")
        return True

    def _vision_params_done(self, future: Future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.service_result.emit("vision_set_parameters", False, str(exc))
            return
        results = list(getattr(response, "results", []))
        success = bool(results) and all(bool(getattr(item, "successful", False)) for item in results)
        reasons = [str(getattr(item, "reason", "")) for item in results if getattr(item, "reason", "")]
        message = "parâmetros aplicados" if success else "; ".join(reasons) or "falha ao aplicar"
        self.service_result.emit("vision_set_parameters", success, message)

    def _vision_pose_params_done(self, future: Future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.service_result.emit("vision_pose_set_parameters", False, str(exc))
            return
        results = list(getattr(response, "results", []))
        success = bool(results) and all(bool(getattr(item, "successful", False)) for item in results)
        reasons = [str(getattr(item, "reason", "")) for item in results if getattr(item, "reason", "")]
        message = "correção XY aplicada" if success else "; ".join(reasons) or "falha ao aplicar"
        self.service_result.emit("vision_pose_set_parameters", success, message)

    def _pick_scheduler_params_done(self, future: Future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.service_result.emit("pick_scheduler_set_parameters", False, str(exc))
            return
        results = list(getattr(response, "results", []))
        success = bool(results) and all(bool(getattr(item, "successful", False)) for item in results)
        reasons = [str(getattr(item, "reason", "")) for item in results if getattr(item, "reason", "")]
        message = "rota de pega aplicada" if success else "; ".join(reasons) or "falha ao aplicar"
        self.service_result.emit("pick_scheduler_set_parameters", success, message)

    def _vision_process_done(self, future: Future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.service_result.emit("vision_process_frame", False, str(exc))
            return
        success = bool(getattr(response, "success", False))
        message = str(getattr(response, "message", ""))
        self.service_result.emit("vision_process_frame", success, message)

    def _vision_calibrate_done(self, future: Future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.service_result.emit("vision_calibrate", False, str(exc))
            return
        success = bool(getattr(response, "success", False))
        message = str(getattr(response, "message", ""))
        profile_json = str(getattr(response, "profile_json", "{}"))
        report = str(getattr(response, "report", ""))
        self.service_result.emit("vision_calibrate", success, message)
        if success:
            self.vision_calibration_result.emit(profile_json, report)

    def publish_vision_test_image(
        self,
        width: int,
        height: int,
        rgb_data: bytes,
        step: int | None = None,
        frame_id: str = "camera_link",
    ) -> bool:
        if self._node is None:
            return False
        self._node.publish_vision_test_image(width, height, rgb_data, step, frame_id)
        self.log_message.emit("info", f"imagem de teste publicada em /camera/image_raw ({width}x{height})")
        return True

    def current_tcp_pose(self) -> Optional[Pose]:
        return self.last_reached_pose

    # Actions ----------------------------------------------------------
    def _emit_action_result(
        self, request: dict[str, object], success: bool, message: str
    ) -> None:
        """Route a result to the regular or tagged Qt signal."""
        tag = request.get("tag")
        if tag is None:
            self.action_result.emit(str(request["label"]), success, str(message))
        else:
            self.pose_action_result.emit(str(tag), success, str(message))

    def _new_action_request(
        self,
        label: str,
        future: Future,
        *,
        result_timeout_s: float,
        tag: str | None = None,
    ) -> int:
        """Track goal acceptance and result deadlines for one action request."""
        with self._action_lock:
            self._action_seq += 1
            request_id = self._action_seq
            request: dict[str, object] = {
                "label": str(label),
                "tag": tag,
                "future": future,
                "handle": None,
                "result_timeout_s": max(0.1, float(result_timeout_s)),
                "accept_timer": None,
                "result_timer": None,
            }
            timer = threading.Timer(
                self.action_accept_timeout_s,
                self._on_action_accept_timeout,
                args=(request_id,),
            )
            timer.daemon = True
            request["accept_timer"] = timer
            self._action_requests[request_id] = request
            timer.start()
        future.add_done_callback(lambda f, rid=request_id: self._action_goal_done(rid, f))
        return request_id

    @staticmethod
    def _cancel_timer(timer: object) -> None:
        if isinstance(timer, threading.Timer):
            timer.cancel()

    def _pop_action_request(self, request_id: int) -> dict[str, object] | None:
        with self._action_lock:
            request = self._action_requests.pop(request_id, None)
        if request is not None:
            self._cancel_timer(request.get("accept_timer"))
            self._cancel_timer(request.get("result_timer"))
        return request

    def _on_action_accept_timeout(self, request_id: int) -> None:
        request = self._pop_action_request(request_id)
        if request is None or self._stopping.is_set():
            return
        label = str(request["label"])
        self.log_message.emit("warn", f"{label}: timeout aguardando aceitação do goal")
        self._emit_action_result(request, False, "timeout aguardando aceitação do goal")

    def _on_action_result_timeout(self, request_id: int) -> None:
        request = self._pop_action_request(request_id)
        if request is None or self._stopping.is_set():
            return
        handle = request.get("handle")
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # pragma: no cover - defensive
                self.log_message.emit("warn", f"cancel após timeout falhou: {exc}")
        label = str(request["label"])
        self.log_message.emit("warn", f"{label}: timeout aguardando resultado")
        self._emit_action_result(request, False, "timeout aguardando resultado da action")

    def _action_goal_done(self, request_id: int, future: Future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            request = self._pop_action_request(request_id)
            if request is not None:
                self._emit_action_result(request, False, str(exc))
            return

        with self._action_lock:
            request = self._action_requests.get(request_id)
            if request is not None:
                self._cancel_timer(request.get("accept_timer"))
                request["accept_timer"] = None

        # The request may have timed out or been cancelled while discovery was
        # still pending. If the server accepts it later, cancel it immediately.
        if request is None:
            if goal_handle is not None and getattr(goal_handle, "accepted", False):
                try:
                    goal_handle.cancel_goal_async()
                    self.log_message.emit(
                        "info",
                        "goal aceito após parada; cancelamento imediato solicitado",
                    )
                except Exception:  # pragma: no cover - defensive
                    pass
            return
        if goal_handle is None or not goal_handle.accepted:
            self._pop_action_request(request_id)
            self._emit_action_result(request, False, "goal rejeitado")
            return

        timer = threading.Timer(
            float(request["result_timeout_s"]),
            self._on_action_result_timeout,
            args=(request_id,),
        )
        timer.daemon = True
        with self._action_lock:
            current = self._action_requests.get(request_id)
            if current is not request:
                current = None
            else:
                request["handle"] = goal_handle
                request["result_timer"] = timer
                timer.start()
        if current is None:
            try:
                goal_handle.cancel_goal_async()
            except Exception:  # pragma: no cover - defensive
                pass
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, rid=request_id: self._action_result_done(rid, f)
        )

    @staticmethod
    def _parse_action_result(label: str, wrapped_result) -> tuple[bool, str, object | None]:
        result = wrapped_result.result
        status = int(getattr(wrapped_result, "status", GoalStatus.STATUS_UNKNOWN))
        if status != GoalStatus.STATUS_SUCCEEDED:
            status_names = {
                GoalStatus.STATUS_CANCELED: "goal cancelado",
                GoalStatus.STATUS_ABORTED: "goal abortado",
            }
            return False, status_names.get(status, f"action terminou com status={status}"), None

        error_code = getattr(result, "error_code", None)
        if error_code is not None:
            success = int(error_code) == 0
            message = getattr(result, "error_string", "") or f"error_code={error_code}"
        elif hasattr(result, "reached_goal"):
            success = bool(result.reached_goal)
            if success:
                message = "objetivo do gripper atingido"
            elif bool(getattr(result, "stalled", False)):
                message = "gripper bloqueado antes de atingir o objetivo"
            else:
                message = "gripper não atingiu o objetivo"
        else:
            success = bool(getattr(result, "success", True))
            message = getattr(result, "message", "")
        final_pose = getattr(result, "final_pose", None) if success else None
        return success, str(message), final_pose

    def _action_result_done(self, request_id: int, future: Future) -> None:
        request = self._pop_action_request(request_id)
        if request is None:
            return
        try:
            wrapped_result = future.result()
            success, message, final_pose = self._parse_action_result(
                str(request["label"]), wrapped_result
            )
        except Exception as exc:
            self._emit_action_result(request, False, str(exc))
            return
        if success and final_pose is not None and hasattr(final_pose, "position"):
            self.last_reached_pose = final_pose
        self._emit_action_result(request, success, message)

    def _send_action(
        self,
        label: str,
        future: Optional[Future],
        *,
        result_timeout_s: float,
        tag: str | None = None,
    ) -> bool:
        if future is None:
            self.log_message.emit("warn", f"{label}: action server indisponível")
            request = {"label": label, "tag": tag}
            self._emit_action_result(request, False, "action server indisponível")
            return False
        self._new_action_request(
            label,
            future,
            result_timeout_s=result_timeout_s,
            tag=tag,
        )
        self.log_message.emit("info", f"enviando goal {label} ...")
        return True

    def follow_joint_trajectory(
        self, joint_names: list[str], positions: list[float], duration: float
    ) -> bool:
        node = self._node
        return self._send_action(
            "follow_joint_trajectory",
            node.send_follow_joint_trajectory(joint_names, positions, duration) if node else None,
            result_timeout_s=max(0.1, float(duration)) + self.action_result_grace_s,
        )

    def move_to_pose(self, pose: Pose, duration: float) -> bool:
        node = self._node
        return self._send_action(
            "move_to_pose",
            node.send_move_to_pose(pose, duration, self._on_move_feedback) if node else None,
            result_timeout_s=max(0.1, float(duration)) + self.action_result_grace_s,
        )

    def _on_move_feedback(self, feedback_msg) -> None:
        feedback = getattr(feedback_msg, "feedback", None)
        if feedback is None:
            return
        progress = float(getattr(feedback, "progress", 0.0))
        current_pose = getattr(feedback, "current_pose", None)
        self.move_feedback.emit(progress, current_pose)

    # ---- Tagged MoveToPose (for sequential motion programs) ----
    def move_to_pose_tagged(self, tag: str, pose: Pose, duration: float) -> bool:
        """Send a MoveToPose goal whose result is reported via ``pose_action_result``.

        The ``tag`` is passed back unchanged so a caller running a sequence can
        correlate the result with the goal it sent. Only the most recent goal
        handle is kept for cancellation.
        """
        node = self._node
        future = (
            node.send_move_to_pose(pose, duration, self._on_move_feedback)
            if node is not None
            else None
        )
        return self._send_action(
            "move_to_pose",
            future,
            result_timeout_s=max(0.1, float(duration)) + self.action_result_grace_s,
            tag=tag,
        )

    def cancel_move_to_pose(self) -> bool:
        """Cancel every pending or active MoveToPose goal."""
        return self.cancel_active_actions({"move_to_pose"}) > 0

    def _clear_action_requests(self, *, cancel_handles: bool) -> int:
        """Drop tracked actions and optionally request cancellation for accepted goals."""
        with self._action_lock:
            requests = list(self._action_requests.values())
            self._action_requests.clear()
        for request in requests:
            self._cancel_timer(request.get("accept_timer"))
            self._cancel_timer(request.get("result_timer"))
            handle = request.get("handle")
            if cancel_handles and handle is not None:
                try:
                    future = handle.cancel_goal_async()
                    future.add_done_callback(self._on_move_cancel_done)
                except Exception as exc:  # pragma: no cover - defensive
                    self.log_message.emit("warn", f"cancel action falhou: {exc}")
        return len(requests)

    def cancel_active_actions(self, labels: set[str] | None = None) -> int:
        """Cancel pending and accepted actions, optionally filtered by interface label.

        Pending goal requests are removed immediately. If a server accepts one
        after this call, its completion callback notices that it is no longer
        tracked and cancels the late goal before it can continue unattended.
        """
        wanted = {str(label) for label in labels} if labels is not None else None
        with self._action_lock:
            selected = {
                request_id: request
                for request_id, request in self._action_requests.items()
                if wanted is None or str(request["label"]) in wanted
            }
            for request_id in selected:
                self._action_requests.pop(request_id, None)
        for request in selected.values():
            self._cancel_timer(request.get("accept_timer"))
            self._cancel_timer(request.get("result_timer"))
            handle = request.get("handle")
            if handle is not None:
                try:
                    future = handle.cancel_goal_async()
                    future.add_done_callback(self._on_move_cancel_done)
                except Exception as exc:  # pragma: no cover - defensive
                    self.log_message.emit("warn", f"cancel action falhou: {exc}")
        if selected:
            labels_text = ", ".join(sorted({str(r["label"]) for r in selected.values()}))
            self.log_message.emit(
                "info", f"cancelamento solicitado para {len(selected)} action(s): {labels_text}"
            )
        return len(selected)

    def _on_move_cancel_done(self, future: Future) -> None:
        try:
            response = future.result()
            goals_canceling = getattr(response, "goals_canceling", None) or []
            if goals_canceling:
                self.log_message.emit("info", "cancelamento de action aceito pelo servidor")
            else:
                self.log_message.emit("warn", "servidor não confirmou o cancelamento da action")
        except Exception as exc:  # pragma: no cover - defensive
            self.log_message.emit("warn", f"cancel action callback: {exc}")

    def gripper_command(self, position: float, max_effort: float = 0.0) -> bool:
        node = self._node
        return self._send_action(
            "gripper_command",
            node.send_gripper_command(position, max_effort) if node else None,
            result_timeout_s=10.0,
        )

    # Raw commands -----------------------------------------------------
    def publish_mit(
        self, joint_name: str, pos: float, vel: float, kp: float, kd: float, tau: float
    ) -> bool:
        if self._node is None:
            return False
        self._node.ensure_raw_publishers()
        return self._node.publish_mit(joint_name, pos, vel, kp, kd, tau)

    def publish_posvel(self, joint_name: str, pos: float, vlim: float) -> bool:
        if self._node is None:
            return False
        self._node.ensure_raw_publishers()
        return self._node.publish_posvel(joint_name, pos, vlim)

    # ------------------------------------------------------------------
    # live-state accessors (ConditionContext for the motion-program VM)
    # ------------------------------------------------------------------
    @property
    def state_machine(self) -> str:
        node = self._node
        if self._arm_alive_state and node is not None and node.arm_status is not None:
            return str(node.arm_status.state_machine or "")
        return ""

    @property
    def is_arm_enabled(self) -> bool:
        node = self._node
        if self._arm_alive_state and node is not None and node.arm_status is not None:
            return bool(node.arm_status.enabled)
        return False

    @property
    def is_connected(self) -> bool:
        return self._node is not None

    def action_server_ready(self, name: str) -> bool:
        """Return discovery state for a named action server without blocking."""
        node = self._node
        return node.action_server_ready(name) if node is not None else False

    def ik_service_ready(self, timeout_sec: float = 0.05) -> bool:
        """Non-blocking discovery check for the ``move_to_pose_ik`` service."""
        node = self._node
        client = getattr(node, "cli_move_to_pose_ik", None) if node is not None else None
        if client is None:
            return False
        try:
            return bool(client.wait_for_service(timeout_sec=float(timeout_sec)))
        except Exception:
            return False

    def motion_readiness(self, action_name: str = "move_to_pose") -> tuple[bool, str]:
        """Central readiness policy used by every motion entry point."""
        if self._node is None:
            return False, "Não conectado ao ROS 2."
        if not self._arm_alive_state:
            return False, "Controlador sem heartbeat recente em arm_status."
        if not self.is_arm_enabled:
            return False, "Braço não está habilitado (enable)."
        state = self.state_machine.upper()
        if state in {"PROTECTIVE_STOP", "ERROR", "FAULT", "EMERGENCY_STOP"}:
            return False, f"Braço está em estado {state}."
        if not self.action_server_ready(action_name):
            return False, f"Action server {action_name} indisponível."
        return True, ""

    def signal_by_id(self, io_id: str) -> int:
        """Current level of a digital input by its logical id (0/1)."""
        node = self._node
        if node is None:
            return 0
        io = node.dio_ios.get(str(io_id))
        if io is None or not io.enabled:
            return 0
        return 1 if bool(io.value) else 0

    def dio_ios_snapshot(self) -> dict:
        """Return a copy of the latest id -> DigitalIO map (empty if no node)."""
        node = self._node
        return dict(node.dio_ios) if node is not None else {}

    @staticmethod
    def _dio_default_ids(direction: str) -> list[str]:
        if direction == "input":
            return [f"IN{i}" for i in range(1, 6)]
        return [f"OUT{i}" for i in range(1, 6)]

    def dio_input_ids(self) -> list[str]:
        node = self._node
        if node is None:
            return self._dio_default_ids("input")
        ids = sorted(i.id for i in node.dio_ios.values()
                     if int(i.direction) == DigitalIO.DIR_INPUT)
        return ids or self._dio_default_ids("input")

    def dio_output_ids(self) -> list[str]:
        node = self._node
        if node is None:
            return self._dio_default_ids("output")
        ids = sorted(i.id for i in node.dio_ios.values()
                     if int(i.direction) == DigitalIO.DIR_OUTPUT)
        return ids or self._dio_default_ids("output")

    def joint_position(self, joint: str) -> Optional[float]:
        node = self._node
        if node is None or node.joint_state is None or not joint:
            return None
        names = node.joint_state.name
        if joint in names:
            return float(node.joint_state.position[names.index(joint)])
        return None

    @property
    def gripper_position(self) -> Optional[float]:
        node = self._node
        if node is not None and node.gripper_state is not None:
            try:
                return float(node.gripper_state.position)
            except (TypeError, ValueError):
                return None
        return None

    def vision_detected(self) -> bool:
        node = self._node
        return bool(node.vision_detected) if node is not None else False

    def vision_pose(self, axis: str) -> Optional[float]:
        node = self._node
        if node is None or node.vision_pose is None:
            return None
        p = node.vision_pose.position
        return {"x": p.x, "y": p.y, "z": p.z}.get(str(axis).lower())

    def joint_names(self) -> list[str]:
        node = self._node
        return node.arm_joint_names() if node is not None else []

    # ------------------------------------------------------------------
    # emergency stop / heartbeat / URDF helpers
    # ------------------------------------------------------------------
    def emergency_stop(self) -> None:
        """Software stop: cancel every tracked action and request disable.

        This is a software convenience only — it does NOT replace a hardware
        emergency stop, safety PLC or risk assessment of the cell.
        """
        cancelled = self.cancel_active_actions()
        self.log_message.emit(
            "warn",
            "Parada de software: "
            f"cancelamento solicitado para {cancelled} action(s); solicitando disable",
        )
        self.disable()

    @property
    def is_arm_alive(self) -> bool:
        """True while ``/{ns}/arm_status`` keeps arriving (see arm_status_timeout)."""
        return self._arm_alive_state

    def joint_limits(self) -> Dict[str, Tuple[float, float]]:
        """Per-joint (lower, upper) position limits parsed from the URDF.

        Returns an empty dict while ``/robot_description`` is unavailable.
        """
        node = self._node
        urdf = node.robot_description if node is not None else None
        if not urdf:
            return {}
        try:
            return kinematics.joint_limits(urdf)
        except Exception:  # pragma: no cover - defensive
            return {}

    def tcp_pose_fk(self) -> Optional[Pose]:
        """TCP pose from forward kinematics over the latest joint_states.

        Uses the URDF chain base_link -> gripper_link. Returns None when the
        URDF or joint states are not available yet.
        """
        node = self._node
        if node is None or node.robot_description is None or node.joint_state is None:
            return None
        try:
            chain = kinematics.load_chain(node.robot_description)
            if not chain:
                return None
            q = {
                name: float(pos)
                for name, pos in zip(node.joint_state.name, node.joint_state.position)
            }
            transform = kinematics.tcp_transform(chain, q)
            x, y, z = (float(v) for v in transform[:3, 3])
            qx, qy, qz, qw = kinematics.rotation_to_quaternion(transform[:3, :3])
            return make_pose(x, y, z, qx, qy, qz, qw)
        except Exception:  # pragma: no cover - defensive
            return None
