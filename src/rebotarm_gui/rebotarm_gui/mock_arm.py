"""Mock reBotArm node for GUI testing without real hardware.

Publica todas as interfaces que a GUI consome e responde a todos os
serviços/ações, simulando um braço de 6 DOF + gripper.

Uso:
    ros2 run rebotarm_gui mock_arm
    # ou
    python3 -m rebotarm_gui.mock_arm
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.msg import (
    ArmStatus,
    DigitalIO,
    DigitalIOState,
    JointMitCmd,
    JointMotorState,
    JointPosVelCmd,
    TimingStatus,
)
from rebotarm_msgs.srv import CheckTorqueProfile
from rebotarm_msgs.srv import GripperCommand as GripperCommandSrv
from rebotarm_msgs.srv import (
    MoveToPoseIK,
    SetDigitalIOConfig,
    SetDigitalOutput,
    SetGripper,
    SetSafetyStep,
    SetTorqueProfile,
    SetZero,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
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

ACTION_NAMES = ("move_to_pose", "follow_joint_trajectory", "gripper_command")


@dataclass(frozen=True)
class FaultProfile:
    """Deterministic failures used by integration tests and bench validation."""

    heartbeat_stop_after: float | None = None
    hang_action: str | None = None
    reject_action: str | None = None
    invalid_telemetry_after: float | None = None
    late_accept_action: str | None = None
    accept_delay: float = 0.0


def _duration(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int((float(seconds) - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


class MockArmNode(Node):
    """Simulador do braço reBotArm."""

    def __init__(
        self,
        namespace: str = "rebotarm",
        fault_profile: FaultProfile | None = None,
    ) -> None:
        super().__init__("mock_rebotarm", namespace=namespace)
        self._ns = namespace.strip().strip("/")
        self._cb = ReentrantCallbackGroup()
        self._faults = fault_profile or FaultProfile()
        self._started_at = time.monotonic()
        self.get_logger().info(f"Mock arm iniciado — namespace /{self._ns}")
        if self._faults != FaultProfile():
            self.get_logger().warning(f"Injeção de falhas ativa: {self._faults}")

        # Estado interno
        self._positions: dict[str, float] = {}
        for name, pos in zip(JOINT_NAMES, INITIAL_POSITIONS):
            self._positions[name] = pos
        self._positions[GRIPPER_NAME] = 0.0
        self._enabled = True
        self._gravity_on = False
        self._mode = "mit"
        self._target_positions = dict(self._positions)
        self._moving = False  # drives state_machine TRAJ_RUNNING vs IDLE
        self._speed_override_requested = 1.0
        self._speed_override_applied = 1.0
        self._speed_command_session = ""
        self._speed_command_sequence = -1
        self._speed_command_message = "mock pronto"
        # Simulated digital I/O (Raspberry Pi GPIO): 5 inputs + 5 outputs.
        # IN1 toggles ~every 3 s so WAIT_FOR_SIGNAL can be tested.
        self._dio_inputs = {f"IN{i}": p for i, p in enumerate([17, 18, 27, 22, 23], 1)}
        self._dio_outputs = {f"OUT{i}": p for i, p in enumerate([5, 6, 12, 16, 26], 1)}
        self._dio_tags = {iid: "" for iid in list(self._dio_inputs) + list(self._dio_outputs)}
        self._dio_enabled = {iid: True for iid in list(self._dio_inputs) + list(self._dio_outputs)}
        self._dio_value = {iid: False for iid in list(self._dio_inputs) + list(self._dio_outputs)}
        self._dio_count = {iid: 0 for iid in list(self._dio_inputs) + list(self._dio_outputs)}

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
        self._pub_trajectory_limits = self.create_publisher(
            String, f"/{self._ns}/trajectory_limits", qos_latched
        )
        self._pub_trajectory_limits.publish(
            String(
                data=json.dumps(
                    {
                        "joint_names": JOINT_NAMES,
                        "max_velocity": [1.0] * len(JOINT_NAMES),
                        "max_acceleration": [1.0] * len(JOINT_NAMES),
                        "safety_factor": 0.9,
                        "dynamic_speed_override": {
                            "supported": True,
                            "minimum": 0.05,
                            "maximum": 1.0,
                            "increase_rate_per_s": 0.5,
                            "decrease_rate_per_s": 2.0,
                        },
                    },
                    sort_keys=True,
                )
            )
        )
        self._pub_timing = self.create_publisher(
            TimingStatus, f"/{self._ns}/timing_status", 10
        )
        self._pub_speed_override = self.create_publisher(
            String, f"/{self._ns}/speed_override/status", qos_latched
        )
        self.create_subscription(
            String,
            f"/{self._ns}/speed_override/command",
            self._on_speed_override_command,
            qos_latched,
            callback_group=self._cb,
        )

        self._pub_gripper = self.create_publisher(
            JointMotorState, f"/{self._ns}/gripper/state", 10
        )

        self._pub_dio = self.create_publisher(
            DigitalIOState, f"/{self._ns}/dio/state", 10
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
        self.create_service(
            SetDigitalOutput,
            f"/{self._ns}/dio/set_output",
            self._srv_dio_set_output,
            callback_group=self._cb,
        )
        self.create_service(
            SetDigitalIOConfig,
            f"/{self._ns}/dio/set_config",
            self._srv_dio_set_config,
            callback_group=self._cb,
        )
        self.create_service(
            SetTorqueProfile,
            f"/{self._ns}/safety/set_torque_profile",
            self._srv_safety_set_profile,
            callback_group=self._cb,
        )
        self.create_service(
            SetSafetyStep,
            f"/{self._ns}/safety/set_step",
            self._srv_safety_set_step,
            callback_group=self._cb,
        )
        self.create_service(
            CheckTorqueProfile,
            f"/{self._ns}/safety/check_torque_profile",
            self._srv_safety_check,
            callback_group=self._cb,
        )
        self.create_service(
            Trigger,
            f"/{self._ns}/safety/reset_fault",
            self._srv_safety_reset_fault,
            callback_group=self._cb,
        )

        # ---- Action servers ----
        self._as_move_pose = ActionServer(
            self,
            MoveToPose,
            f"/{self._ns}/move_to_pose",
            execute_callback=self._exec_move_pose,
            goal_callback=lambda _goal: self._goal_response("move_to_pose"),
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=self._cb,
        )
        self._as_follow = ActionServer(
            self,
            FollowJointTrajectory,
            f"/{self._ns}/follow_joint_trajectory",
            execute_callback=self._exec_follow,
            goal_callback=lambda _goal: self._goal_response("follow_joint_trajectory"),
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=self._cb,
        )
        self._as_gripper_cmd = ActionServer(
            self,
            GripperCommand,
            f"/{self._ns}/gripper/command",
            execute_callback=self._exec_gripper_cmd,
            goal_callback=lambda _goal: self._goal_response("gripper_command"),
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
        delta = (
            self._speed_override_requested - self._speed_override_applied
        )
        if abs(delta) > 1e-9:
            rate = 0.5 if delta > 0.0 else 2.0
            step = min(abs(delta), rate * 0.05)
            self._speed_override_applied += math.copysign(step, delta)

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
        if self._fault_due(self._faults.invalid_telemetry_after):
            # A mensagem continua válida no wire format, mas é semanticamente
            # insegura: vetor truncado e valor não finito.
            js.position = [float("nan")]
        self._pub_js.publish(js)

        # ---- ArmStatus (latched) ----
        st = ArmStatus()
        st.header.stamp = now
        st.mode = self._mode
        st.enabled = self._enabled
        st.control_loop_active = self._enabled
        # Only valid state-machine values (see hardware_manager.set_state_machine).
        st.state_machine = "TRAJ_RUNNING" if self._moving else "IDLE"
        st.joint_names = list(JOINT_NAMES)
        st.per_joint_status_code = [0] * len(JOINT_NAMES)
        st.error_codes = []
        if not self._fault_due(self._faults.heartbeat_stop_after):
            self._pub_status.publish(st)

        timing = TimingStatus()
        timing.header.stamp = now
        timing.watchdog_enabled = True
        timing.control_nominal_rate_hz = 500.0
        timing.control_last_period_ms = 2.0
        timing.safety_nominal_rate_hz = 100.0
        timing.safety_last_period_ms = 10.0
        self._pub_timing.publish(timing)
        self._publish_speed_override_status()

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

        # ---- Digital I/O (simulated Raspberry Pi GPIO) ----
        # IN1 toggles ~every 3 s so WAIT_FOR_SIGNAL can be tested.
        new_in1 = (math.floor(self._t / 3.0) % 2) == 0
        if new_in1 and not self._dio_value["IN1"]:
            self._dio_count["IN1"] += 1
        self._dio_value["IN1"] = new_in1
        dio = DigitalIOState()
        dio.header.stamp = now
        for iid, pin in self._dio_inputs.items():
            dio.ios.append(self._mk_dio(
                iid, DigitalIO.DIR_INPUT, pin,
                self._dio_value[iid] if self._dio_enabled[iid] else False))
        for iid, pin in self._dio_outputs.items():
            val = self._dio_value[iid] if self._dio_enabled[iid] else False
            dio.ios.append(self._mk_dio(iid, DigitalIO.DIR_OUTPUT, pin, val))
        self._pub_dio.publish(dio)

    def _on_speed_override_command(self, message: String) -> None:
        try:
            payload = json.loads(str(message.data))
            if int(payload.get("version", 0)) != 1:
                raise ValueError("versão inválida")
            value = float(payload["value"])
            session = str(payload["session"]).strip()
            sequence = int(payload["sequence"])
            if not math.isfinite(value) or not session or sequence < 0:
                raise ValueError("conteúdo inválido")
            if (
                session == self._speed_command_session
                and sequence <= self._speed_command_sequence
            ):
                return
            self._speed_override_requested = max(0.05, min(1.0, value))
            self._speed_command_session = session
            self._speed_command_sequence = sequence
            self._speed_command_message = "override aceito pelo mock"
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._speed_override_requested = 0.05
            self._speed_command_message = (
                f"comando inválido; mínimo seguro: {exc}"
            )
        self._publish_speed_override_status()

    def _publish_speed_override_status(self) -> None:
        self._pub_speed_override.publish(
            String(
                data=json.dumps(
                    {
                        "version": 1,
                        "supported": True,
                        "requested": self._speed_override_requested,
                        "applied": self._speed_override_applied,
                        "ramping": abs(
                            self._speed_override_requested
                            - self._speed_override_applied
                        )
                        > 1e-6,
                        "minimum": 0.05,
                        "maximum": 1.0,
                        "increase_rate_per_s": 0.5,
                        "decrease_rate_per_s": 2.0,
                        "motion_active": self._moving,
                        "session": self._speed_command_session,
                        "sequence": self._speed_command_sequence,
                        "message": self._speed_command_message,
                        "stamp": time.time(),
                    },
                    sort_keys=True,
                    separators=(",", ":"),
                )
            )
        )

    def _mk_dio(self, iid, direction, pin, value):
        io = DigitalIO()
        io.direction = direction
        io.id = iid
        io.tag = self._dio_tags.get(iid, "")
        io.pin = int(pin)
        io.enabled = bool(self._dio_enabled.get(iid, True))
        io.value = bool(value)
        io.activation_count = int(self._dio_count.get(iid, 0))
        return io

    def _srv_dio_set_output(self, req, resp):
        iid = str(req.id).strip().upper()
        if iid not in self._dio_outputs:
            resp.success = False
            resp.message = f"saída desconhecida: {req.id}"
            return resp
        if not self._dio_enabled.get(iid, True):
            resp.success = False
            resp.message = f"{iid} desabilitada"
            return resp
        value = bool(req.value)
        self._dio_value[iid] = value
        if value:
            self._dio_count[iid] += 1
        resp.success = True
        resp.message = f"{iid}={'ON' if value else 'OFF'}"
        return resp

    def _srv_dio_set_config(self, req, resp):
        iid = str(req.id).strip().upper()
        if iid not in self._dio_inputs and iid not in self._dio_outputs:
            resp.success = False
            resp.message = f"I/O desconhecido: {req.id}"
            return resp
        self._dio_tags[iid] = str(req.tag)
        self._dio_enabled[iid] = bool(req.enabled)
        if not req.enabled and iid in self._dio_outputs:
            self._dio_value[iid] = False
        resp.success = True
        resp.message = f"{iid} atualizado"
        return resp

    def _srv_safety_set_profile(self, req: SetTorqueProfile.Request, resp: SetTorqueProfile.Response):
        self.get_logger().info(
            f"Safety set profile: path={req.profile_path} mode={req.mode} "
            f"step={req.active_step_id}"
        )
        resp.success = True
        resp.message = "Profile applied (mock)"
        return resp

    def _srv_safety_set_step(self, req: SetSafetyStep.Request, resp: SetSafetyStep.Response):
        self.get_logger().info(f"Safety set step: id={req.step_id} payload={req.payload_kg:.2f}kg")
        resp.success = True
        resp.message = "Safety step armed (mock)"
        return resp

    def _srv_safety_check(self, req: CheckTorqueProfile.Request, resp: CheckTorqueProfile.Response):
        self.get_logger().info(f"Safety check: step={req.step_id} phase={req.phase:.2f}")
        resp.success = True
        resp.severity = "OK"
        resp.message = "Within limits (mock)"
        resp.joint_names = list(JOINT_NAMES)
        resp.torque = [0.1 * math.sin(self._t + i) for i in range(len(JOINT_NAMES))]
        resp.warn_min = [-1.0] * len(JOINT_NAMES)
        resp.warn_max = [1.0] * len(JOINT_NAMES)
        resp.fail_min = [-2.0] * len(JOINT_NAMES)
        resp.fail_max = [2.0] * len(JOINT_NAMES)
        resp.status = ["OK"] * len(JOINT_NAMES)
        return resp

    def _srv_safety_reset_fault(self, req, resp):
        self.get_logger().info("Safety reset fault")
        return self._ok_response("Fault reset (mock)")

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
        resp.q_solution = [
            math.atan2(float(p.y), max(1e-6, float(p.x))),
            float(p.z) - 0.5,
            float(p.x) - 0.3,
            0.0,
            0.5,
            0.0,
        ]
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
    def _fault_due(self, after: float | None) -> bool:
        return after is not None and time.monotonic() - self._started_at >= after

    def _goal_response(self, action_name: str) -> GoalResponse:
        if self._faults.reject_action == action_name:
            self.get_logger().warning(f"Goal rejeitado por injeção: {action_name}")
            return GoalResponse.REJECT
        if self._faults.late_accept_action == action_name:
            delay = max(0.0, self._faults.accept_delay)
            self.get_logger().warning(
                f"Aceitação atrasada por injeção: {action_name} ({delay:.2f}s)"
            )
            time.sleep(delay)
        return GoalResponse.ACCEPT

    def _hang_if_requested(self, action_name: str) -> None:
        if self._faults.hang_action != action_name:
            return
        self.get_logger().warning(f"Action travada por injeção: {action_name}")
        while rclpy.ok():
            time.sleep(0.1)

    def _exec_move_pose(self, goal_handle):
        goal: MoveToPose.Goal = goal_handle.request
        p = goal.target_pose.position
        self.get_logger().info(
            f"Action MoveToPose: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} dur={goal.duration:.1f}s"
        )
        self._moving = True
        self._hang_if_requested("move_to_pose")
        nominal_duration = max(float(goal.duration), 0.05)
        virtual_time = 0.0
        previous = time.monotonic()
        while virtual_time < nominal_duration:
            if goal_handle.is_cancel_requested:
                self._moving = False
                goal_handle.canceled()
                result = MoveToPose.Result()
                result.success = False
                result.message = "Goal cancelado (mock)"
                result.final_pose = goal.target_pose
                self.get_logger().warning("MoveToPose cancelado durante execução")
                return result
            fb = MoveToPose.Feedback()
            now = time.monotonic()
            elapsed = max(0.0, now - previous)
            previous = now
            virtual_time = min(
                nominal_duration,
                virtual_time + elapsed * self._speed_override_applied,
            )
            fb.progress = virtual_time / nominal_duration
            fb.time_elapsed = virtual_time
            fb.current_pose = goal.target_pose
            goal_handle.publish_feedback(fb)
            time.sleep(0.01)
        self._moving = False
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
        self._hang_if_requested("follow_joint_trajectory")
        if traj.points:
            pt = traj.points[-1]
            for jn, pos in zip(traj.joint_names, pt.positions):
                if jn in self._target_positions:
                    self._target_positions[jn] = pos
            dur = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            nominal_duration = max(dur, 0.1)
            virtual_time = 0.0
            previous = time.monotonic()
            self._moving = True
            while virtual_time < nominal_duration:
                if goal_handle.is_cancel_requested:
                    self._moving = False
                    goal_handle.canceled()
                    result = FollowJointTrajectory.Result()
                    result.error_code = 0
                    result.error_string = "cancelado (mock)"
                    return result
                now = time.monotonic()
                elapsed = max(0.0, now - previous)
                previous = now
                virtual_time += elapsed * self._speed_override_applied
                time.sleep(0.01)
            self._moving = False
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        result.error_string = "success (mock)"
        return result

    def _exec_gripper_cmd(self, goal_handle):
        goal: GripperCommand.Goal = goal_handle.request
        pos = goal.command.position
        self.get_logger().info(f"Action GripperCommand: position={pos:.2f}")
        self._hang_if_requested("gripper_command")
        self._target_positions[GRIPPER_NAME] = max(0.0, min(1.0, pos))
        time.sleep(0.3)
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = pos
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True
        return result


def _parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="Mock ROS 2 do reBotArm")
    parser.add_argument("--namespace", default="rebotarm")
    parser.add_argument("--heartbeat-stop-after", type=float)
    parser.add_argument("--hang-action", choices=ACTION_NAMES)
    parser.add_argument("--reject-action", choices=ACTION_NAMES)
    parser.add_argument("--invalid-telemetry-after", type=float)
    parser.add_argument("--late-accept-action", choices=ACTION_NAMES)
    parser.add_argument("--accept-delay", type=float, default=0.0)
    return parser.parse_known_args(argv)


def main(argv: list[str] | None = None) -> int:
    args, ros_args = _parse_args(list(sys.argv[1:] if argv is None else argv))
    rclpy.init(args=ros_args)
    faults = FaultProfile(
        heartbeat_stop_after=args.heartbeat_stop_after,
        hang_action=args.hang_action,
        reject_action=args.reject_action,
        invalid_telemetry_after=args.invalid_telemetry_after,
        late_accept_action=args.late_accept_action,
        accept_delay=args.accept_delay,
    )
    node = MockArmNode(namespace=args.namespace, fault_profile=faults)
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
