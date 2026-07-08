"""ROS 2 bridge for the reBotArm GUI.

This module runs a dedicated ``rclpy`` node inside a ``QThread``. All ROS
subscriptions, service clients and action clients live here, and results are
delivered to the Qt UI layer via signals (thread-safe).

The bridge is a pure client: it never touches hardware directly, only the
interfaces exposed by the ``reBotArmController`` node under ``/rebotarm``.
"""

from __future__ import annotations

import threading
from typing import Optional

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, Quaternion
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from rclpy.task import Future
from PySide6.QtCore import QObject, QThread, Signal
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.msg import ArmStatus, JointMitCmd, JointMotorState, JointPosVelCmd
from rebotarm_msgs.srv import GripperCommand as GripperCommandSrv
from rebotarm_msgs.srv import MoveToPoseIK, SetGripper, SetZero
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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


class _BridgeNode(Node):
    """Internal node owning every subscription / client."""

    def __init__(self, namespace: str = "rebotarm") -> None:
        super().__init__("rebotarm_gui")
        self._ns = namespace.strip().strip("/")
        self._cb_reentrant = ReentrantCallbackGroup()

        # ---- Latest state cache (read from the UI thread) ----
        self.joint_state: Optional[JointState] = None
        self.arm_status: Optional[ArmStatus] = None
        self.gripper_state: Optional[JointMotorState] = None
        self.joint_detail: dict[str, JointMotorState] = {}

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
    # subscription callbacks
    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState) -> None:
        self.joint_state = msg
        self._ensure_joint_detail_subs()

    def _arm_status_cb(self, msg: ArmStatus) -> None:
        self.arm_status = msg

    def _gripper_state_cb(self, msg: JointMotorState) -> None:
        self.gripper_state = msg

    def _joint_detail_cb_factory(self, joint_name: str):
        def _cb(msg: JointMotorState) -> None:
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

    # ------------------------------------------------------------------
    # action goal helpers
    # ------------------------------------------------------------------
    def send_follow_joint_trajectory(
        self, joint_names: list[str], positions: list[float], duration: float
    ) -> Optional[Future]:
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = duration_msg(duration)
        traj.points = [point]
        goal.trajectory = traj
        return self.ac_follow_joint_trajectory.send_goal_async(goal)

    def send_move_to_pose(self, pose: Pose, duration: float) -> Optional[Future]:
        goal = MoveToPose.Goal()
        goal.target_pose = pose
        goal.duration = float(duration)
        return self.ac_move_to_pose.send_goal_async(goal)

    def send_gripper_command(self, position: float, max_effort: float = 0.0) -> Optional[Future]:
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
    service_result = Signal(str, bool, str)  # (label, success, message)
    action_result = Signal(str, bool, str)  # (label, success, message)
    log_message = Signal(str, str)  # (level, text)
    connected = Signal(bool)

    def __init__(self, namespace: str = "rebotarm") -> None:
        super().__init__()
        self._namespace = namespace
        self._node: Optional[_BridgeNode] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._thread: Optional[QThread] = None
        self._stopping = threading.Event()
        self._timer_interval = 0.08  # seconds

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

    def shutdown(self) -> None:
        import rclpy

        self._stopping.set()
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

    def _spin_loop(self) -> None:
        """Spin the ROS executor and periodically emit Qt signals."""
        import rclpy
        from datetime import datetime

        lock = threading.Event()
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

            if node.arm_status is not None:
                self.arm_status_changed.emit(node.arm_status)

            if node.gripper_state is not None:
                self.gripper_state_changed.emit(node.gripper_state)

            for jn, detail in list(node.joint_detail.items()):
                self.joint_detail_changed.emit(jn, detail)

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
        return self._call("enable", self._node.call_enable())

    def disable(self) -> bool:
        return self._call("disable", self._node.call_disable())

    def safe_home(self) -> bool:
        return self._call("safe_home", self._node.call_safe_home())

    def gravity_start(self) -> bool:
        return self._call("gravity_start", self._node.call_gravity_start())

    def gravity_stop(self) -> bool:
        return self._call("gravity_stop", self._node.call_gravity_stop())

    def set_zero(self, joint_name: str = "") -> bool:
        label = f"set_zero({joint_name})" if joint_name else "set_zero(all)"
        return self._call(label, self._node.call_set_zero(joint_name))

    def move_to_pose_ik(self, pose: Pose) -> bool:
        return self._call("move_to_pose_ik", self._node.call_move_to_pose_ik(pose))

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

    # Actions ----------------------------------------------------------
    def _action_goal_done(self, label: str, future: Future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.action_result.emit(label, False, str(exc))
            return
        if goal_handle is None or not goal_handle.accepted:
            self.action_result.emit(label, False, "goal rejeitado")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._action_result_done(label, f))

    def _action_result_done(self, label: str, future: Future) -> None:
        try:
            res = future.result().result
        except Exception as exc:
            self.action_result.emit(label, False, str(exc))
            return
        # FollowJointTrajectory uses error_code/error_string; others use success/message.
        error_code = getattr(res, "error_code", None)
        if error_code is not None:
            success = error_code == 0
            message = getattr(res, "error_string", "") or f"error_code={error_code}"
        else:
            success = bool(getattr(res, "success", True))
            message = getattr(res, "message", "")
        self.action_result.emit(label, success, str(message))

    def _send_action(self, label: str, future: Optional[Future]) -> bool:
        if future is None:
            self.log_message.emit("warn", f"{label}: action server indisponível")
            self.action_result.emit(label, False, "action server indisponível")
            return False
        future.add_done_callback(lambda f: self._action_goal_done(label, f))
        self.log_message.emit("info", f"enviando goal {label} ...")
        return True

    def follow_joint_trajectory(
        self, joint_names: list[str], positions: list[float], duration: float
    ) -> bool:
        return self._send_action(
            "follow_joint_trajectory",
            self._node.send_follow_joint_trajectory(joint_names, positions, duration),
        )

    def move_to_pose(self, pose: Pose, duration: float) -> bool:
        return self._send_action("move_to_pose", self._node.send_move_to_pose(pose, duration))

    def gripper_command(self, position: float, max_effort: float = 0.0) -> bool:
        return self._send_action(
            "gripper_command", self._node.send_gripper_command(position, max_effort)
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
