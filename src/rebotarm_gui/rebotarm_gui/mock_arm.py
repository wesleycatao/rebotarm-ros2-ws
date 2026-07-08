"""Mock reBotArm node for GUI testing without real hardware.

Publica todas as interfaces que a GUI consome e responde a todos os
serviços/ações, simulando um braço de 6 DOF + gripper.

Uso:
    ros2 run rebotarm_gui mock_arm
    # ou
    python3 -m rebotarm_gui.mock_arm
"""

from __future__ import annotations

import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose, Quaternion
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.msg import ArmStatus, JointMitCmd, JointMotorState, JointPosVelCmd
from rebotarm_msgs.srv import GripperCommand as GripperCommandSrv
from rebotarm_msgs.srv import MoveToPoseIK, SetGripper, SetZero
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]
GRIPPER_NAME = "gripper"

INITIAL_POSITIONS = [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]


def _duration(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int((float(seconds) - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class MockArmNode(Node):
    """Simulador do braço reBotArm."""

    def __init__(self, namespace: str = "rebotarm") -> None:
        super().__init__("mock_rebotarm", namespace=namespace)
        self._ns = namespace.strip().strip("/")
        self._cb = ReentrantCallbackGroup()
        self.get_logger().info(f"Mock arm iniciado — namespace /{self._ns}")

        # Estado interno
        self._positions: dict[str, float] = {}
        for name, pos in zip(JOINT_NAMES, INITIAL_POSITIONS):
            self._positions[name] = pos
        self._positions[GRIPPER_NAME] = 0.0
        self._enabled = True
        self._gravity_on = False
        self._mode = "mit"
        self._target_positions = dict(self._positions)

        # ---- Publishers ----
        self._pub_js = self.create_publisher(JointState, f"/{self._ns}/joint_states", 10)

        qos_latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._pub_status = self.create_publisher(
            ArmStatus, f"/{self._ns}/arm_status", qos_latched
        )

        self._pub_gripper = self.create_publisher(
            JointMotorState, f"/{self._ns}/gripper/state", 10
        )

        self._pub_detail: dict[str, object] = {}
        for name in JOINT_NAMES + [GRIPPER_NAME]:
            self._pub_detail[name] = self.create_publisher(
                JointMotorState, f"/{self._ns}/joints/{name}/state", 10
            )

        # ---- Subscriptions (raw commands) ----
        for name in JOINT_NAMES + [GRIPPER_NAME]:
            self.create_subscription(
                JointMitCmd,
                f"/{self._ns}/joints/{name}/cmd/mit",
                lambda msg, jn=name: self._on_mit_cmd(jn, msg),
                10,
            )
            self.create_subscription(
                JointPosVelCmd,
                f"/{self._ns}/joints/{name}/cmd/pos_vel",
                lambda msg, jn=name: self._on_posvel_cmd(jn, msg),
                10,
            )

        # ---- Services ----
        self.create_service(Trigger, f"/{self._ns}/enable", self._srv_enable)
        self.create_service(Trigger, f"/{self._ns}/disable", self._srv_disable)
        self.create_service(Trigger, f"/{self._ns}/safe_home", self._srv_safe_home)
        self.create_service(
            Trigger,
            f"/{self._ns}/gravity_compensation/start",
            self._srv_grav_start,
        )
        self.create_service(
            Trigger,
            f"/{self._ns}/gravity_compensation/stop",
            self._srv_grav_stop,
        )
        self.create_service(SetZero, f"/{self._ns}/set_zero", self._srv_set_zero)
        self.create_service(
            MoveToPoseIK,
            f"/{self._ns}/move_to_pose_ik",
            self._srv_move_ik,
            callback_group=self._cb,
        )
        self.create_service(
            SetGripper,
            f"/{self._ns}/gripper/set",
            self._srv_gripper_set,
            callback_group=self._cb,
        )
        self.create_service(
            GripperCommandSrv,
            f"/{self._ns}/gripper/open",
            self._srv_gripper_open,
            callback_group=self._cb,
        )
        self.create_service(
            GripperCommandSrv,
            f"/{self._ns}/gripper/close",
            self._srv_gripper_close,
            callback_group=self._cb,
        )

        # ---- Action servers ----
        self._as_move_pose = ActionServer(
            self,
            MoveToPose,
            f"/{self._ns}/move_to_pose",
            execute_callback=self._exec_move_pose,
            goal_callback=lambda gh: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=self._cb,
        )
        self._as_follow = ActionServer(
            self,
            FollowJointTrajectory,
            f"/{self._ns}/follow_joint_trajectory",
            execute_callback=self._exec_follow,
            goal_callback=lambda gh: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=self._cb,
        )
        self._as_gripper_cmd = ActionServer(
            self,
            GripperCommand,
            f"/{self._ns}/gripper/command",
            execute_callback=self._exec_gripper_cmd,
            goal_callback=lambda gh: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=self._cb,
        )

        # ---- Timer de publicação (20 Hz) ----
        self._t = 0.0
        self._timer = self.create_timer(0.05, self._tick)

    # ==================================================================
    # Tick — publica estado a cada 50ms
    # ==================================================================
    def _tick(self) -> None:
        self._t += 0.05
        now = self.get_clock().now().to_msg()

        # Interpolação simples em direção ao alvo (movimento suave)
        for name, target in self._target_positions.items():
            current = self._positions[name]
            self._positions[name] = current + (target - current) * 0.1

        # ---- JointState ----
        js = JointState()
        js.header.stamp = now
        js.name = list(self._positions.keys())
        js.position = [self._positions[n] for n in js.name]
        js.velocity = [0.0] * len(js.name)
        js.effort = [0.0] * len(js.name)
        self._pub_js.publish(js)

        # ---- ArmStatus (latched) ----
        st = ArmStatus()
        st.header.stamp = now
        st.mode = self._mode
        st.enabled = self._enabled
        st.control_loop_active = self._enabled
        st.state_machine = "OPERATIONAL" if self._enabled else "DISABLED"
        st.joint_names = list(JOINT_NAMES)
        st.per_joint_status_code = [0] * len(JOINT_NAMES)
        st.error_codes = []
        self._pub_status.publish(st)

        # ---- Gripper state ----
        gs = JointMotorState()
        gs.header.stamp = now
        gs.joint_name = GRIPPER_NAME
        gs.position = self._positions[GRIPPER_NAME]
        gs.velocity = 0.0
        gs.torque = 0.0
        gs.status_code = 0
        self._pub_gripper.publish(gs)

        # ---- Per-joint detail ----
        for name in JOINT_NAMES + [GRIPPER_NAME]:
            d = JointMotorState()
            d.header.stamp = now
            d.joint_name = name
            d.position = self._positions[name]
            # Velocidade simulada
            diff = abs(self._target_positions[name] - self._positions[name])
            d.velocity = min(diff * 5.0, 2.0)
            # Torque simulado com leve oscilação
            d.torque = 0.1 * math.sin(self._t + hash(name) % 10)
            d.status_code = 0
            self._pub_detail[name].publish(d)

    # ==================================================================
    # Raw command callbacks
    # ==================================================================
    def _on_mit_cmd(self, joint_name: str, msg: JointMitCmd) -> None:
        if msg.kp > 0.0:
            self._target_positions[joint_name] = msg.pos
        self.get_logger().debug(
            f"MIT cmd {joint_name}: pos={msg.pos:.3f} kp={msg.kp:.1f} kd={msg.kd:.1f} tau={msg.tau:.2f}"
        )

    def _on_posvel_cmd(self, joint_name: str, msg: JointPosVelCmd) -> None:
        self._target_positions[joint_name] = msg.pos
        self.get_logger().debug(f"PosVel cmd {joint_name}: pos={msg.pos:.3f} vlim={msg.vlim:.2f}")

    # ==================================================================
    # Services
    # ==================================================================
    def _ok_response(self, message: str = "OK"):
        resp = Trigger.Response()
        resp.success = True
        resp.message = message
        return resp

    def _srv_enable(self, req, resp):
        self._enabled = True
        self.get_logger().info("Enable")
        return self._ok_response("Arm enabled (mock)")

    def _srv_disable(self, req, resp):
        self._enabled = False
        self.get_logger().info("Disable")
        return self._ok_response("Arm disabled (mock)")

    def _srv_safe_home(self, req, resp):
        for name, pos in zip(JOINT_NAMES, INITIAL_POSITIONS):
            self._target_positions[name] = pos
        self.get_logger().info("Safe home")
        return self._ok_response("Moving to safe home (mock)")

    def _srv_grav_start(self, req, resp):
        self._gravity_on = True
        self.get_logger().info("Gravity compensation ON")
        return self._ok_response("Gravity compensation started (mock)")

    def _srv_grav_stop(self, req, resp):
        self._gravity_on = False
        self.get_logger().info("Gravity compensation OFF")
        return self._ok_response("Gravity compensation stopped (mock)")

    def _srv_set_zero(self, req: SetZero.Request, resp: SetZero.Response):
        jn = req.joint_name or "all"
        if req.joint_name and req.joint_name in self._positions:
            self._positions[req.joint_name] = 0.0
            self._target_positions[req.joint_name] = 0.0
        elif not req.joint_name:
            for k in self._positions:
                self._positions[k] = 0.0
                self._target_positions[k] = 0.0
        self.get_logger().info(f"Set zero: {jn}")
        resp.success = True
        resp.message = f"Zeroed: {jn} (mock)"
        return resp

    def _srv_move_ik(self, req: MoveToPoseIK.Request, resp: MoveToPoseIK.Response):
        p = req.target_pose.position
        self.get_logger().info(
            f"MoveToPoseIK: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
        )
        resp.success = True
        resp.message = "IK solved (mock)"
        if hasattr(resp, "joint_positions"):
            resp.joint_positions = [self._positions[n] for n in JOINT_NAMES]
        return resp

    def _srv_gripper_set(self, req: SetGripper.Request, resp: SetGripper.Response):
        self._target_positions[GRIPPER_NAME] = max(0.0, min(1.0, req.position))
        self.get_logger().info(f"Gripper set: {req.position:.2f}")
        resp.success = True
        resp.message = f"Gripper moved to {req.position:.2f} (mock)"
        return resp

    def _srv_gripper_open(self, req: GripperCommandSrv.Request, resp: GripperCommandSrv.Response):
        self._target_positions[GRIPPER_NAME] = 0.0
        self.get_logger().info("Gripper open")
        resp.success = True
        resp.message = "Gripper open (mock)"
        return resp

    def _srv_gripper_close(self, req: GripperCommandSrv.Request, resp: GripperCommandSrv.Response):
        self._target_positions[GRIPPER_NAME] = 1.0
        self.get_logger().info("Gripper close")
        resp.success = True
        resp.message = "Gripper closed (mock)"
        return resp

    # ==================================================================
    # Actions
    # ==================================================================
    def _exec_move_pose(self, goal_handle):
        goal: MoveToPose.Goal = goal_handle.request
        p = goal.target_pose.position
        self.get_logger().info(
            f"Action MoveToPose: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} dur={goal.duration:.1f}s"
        )
        steps = max(int(goal.duration / 0.05), 1)
        for i in range(steps + 1):
            fb = MoveToPose.Feedback()
            fb.progress = float(i) / steps
            fb.time_elapsed = float(i) * 0.05
            fb.current_pose = goal.target_pose
            goal_handle.publish_feedback(fb)
            time.sleep(0.05)
        goal_handle.succeed()
        result = MoveToPose.Result()
        result.success = True
        result.message = "Reached pose (mock)"
        result.final_pose = goal.target_pose
        return result

    def _exec_follow(self, goal_handle):
        goal: FollowJointTrajectory.Goal = goal_handle.request
        traj = goal.trajectory
        self.get_logger().info(
            f"Action FollowJointTrajectory: joints={traj.joint_names} "
            f"points={len(traj.points)}"
        )
        if traj.points:
            pt = traj.points[-1]
            for jn, pos in zip(traj.joint_names, pt.positions):
                if jn in self._target_positions:
                    self._target_positions[jn] = pos
            dur = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            time.sleep(max(dur, 0.1))
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        result.error_string = "success (mock)"
        return result

    def _exec_gripper_cmd(self, goal_handle):
        goal: GripperCommand.Goal = goal_handle.request
        pos = goal.command.position
        self.get_logger().info(f"Action GripperCommand: position={pos:.2f}")
        self._target_positions[GRIPPER_NAME] = max(0.0, min(1.0, pos))
        time.sleep(0.3)
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = pos
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True
        return result


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = MockArmNode(namespace="rebotarm")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())