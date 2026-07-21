from __future__ import annotations

from rebotarm_msgs.srv import (
    CheckTorqueProfile,
    GripperCommand,
    MoveToPoseIK,
    SetSafetyStep,
    SetGripper,
    SetTorqueProfile,
    SetZero,
)
from std_srvs.srv import Trigger

from .conversions import pose_to_xyz_rpy


class ArmServices:
    def __init__(self, node, hardware, namespace: str) -> None:
        self._node = node
        self._hardware = hardware

        services = (
            (Trigger, "enable", self.enable, node.slow_group),
            (Trigger, "disable", self.disable, node.slow_group),
            (Trigger, "safe_home", self.safe_home, node.slow_group),
            (
                Trigger,
                "gravity_compensation/start",
                self.start_gravity_compensation,
                node.slow_group,
            ),
            (
                Trigger,
                "gravity_compensation/stop",
                self.stop_gravity_compensation,
                node.slow_group,
            ),
            (SetZero, "set_zero", self.set_zero, node.slow_group),
            (MoveToPoseIK, "move_to_pose_ik", self.move_to_pose_ik, node.reentrant_group),
            (SetGripper, "gripper/set", self.set_gripper, node.reentrant_group),
            (GripperCommand, "gripper/open", self.open_gripper, node.slow_group),
            (GripperCommand, "gripper/close", self.close_gripper, node.slow_group),
            (SetTorqueProfile, "safety/set_torque_profile", self.set_torque_profile, node.slow_group),
            (SetSafetyStep, "safety/set_step", self.set_safety_step, node.reentrant_group),
            (CheckTorqueProfile, "safety/check_torque_profile", self.check_torque_profile, node.reentrant_group),
            (Trigger, "safety/reset_fault", self.reset_safety_fault, node.slow_group),
        )
        for srv_type, name, handler, group in services:
            node.create_service(
                srv_type,
                f"/{namespace}/{name}",
                handler,
                callback_group=group,
            )

    def _run(self, response, action, success_message: str, *, read_hardware=True):
        try:
            action()
            response.success = True
            response.message = success_message
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status(read_hardware=read_hardware)
        return response

    def enable(self, _request, response):
        return self._run(response, self._hardware.enable, "enabled")

    def disable(self, _request, response):
        def action():
            self._hardware.stop_gravity_compensation()
            self._hardware.disable()

        return self._run(response, action, "disabled", read_hardware=False)

    def safe_home(self, _request, response):
        return self._run(response, self._hardware.safe_home, "safe_home complete")

    def start_gravity_compensation(self, _request, response):
        return self._run(
            response,
            self._hardware.start_gravity_compensation,
            "gravity compensation started",
        )

    def stop_gravity_compensation(self, _request, response):
        return self._run(
            response,
            self._hardware.stop_gravity_compensation,
            "gravity compensation stopped",
        )

    def reset_safety_fault(self, _request, response):
        return self._run(
            response,
            self._node.reset_safety_fault,
            "safety fault reset",
        )

    def set_torque_profile(self, request, response):
        try:
            response.message = self._node.set_torque_profile(
                request.profile_path,
                request.mode,
                int(request.violation_samples or 3),
                request.active_step_id,
                float(getattr(request, "derivative_limit", 0.0) or 0.0),
            )
            response.success = True
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def set_safety_step(self, request, response):
        try:
            self._node.set_torque_profile_step(request.step_id)
            response.success = True
            response.message = f"active safety step={request.step_id or 'default'}"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def check_torque_profile(self, request, response):
        try:
            severity, results = self._node.evaluate_current_torque(
                request.phase,
                request.step_id,
            )
            response.success = True
            response.severity = severity
            issues = [item.message() for item in results if item.status != "OK"]
            response.message = "; ".join(issues[:3]) if issues else "OK"
            response.joint_names = [item.joint_name for item in results]
            response.torque = [float(item.torque) for item in results]
            response.warn_min = [float(item.warn_min) for item in results]
            response.warn_max = [float(item.warn_max) for item in results]
            response.fail_min = [float(item.fail_min) for item in results]
            response.fail_max = [float(item.fail_max) for item in results]
            response.status = [item.status for item in results]
        except Exception as exc:
            response.success = False
            response.severity = "FAIL"
            response.message = str(exc)
        return response

    def set_zero(self, request, response):
        def action():
            self._hardware.stop_gravity_compensation()
            if not self._hardware.set_zero(request.joint_name):
                raise RuntimeError("set_zero failed")

        return self._run(response, action, "set_zero complete")

    def move_to_pose_ik(self, request, response):
        try:
            self._hardware.stop_gravity_compensation()
            x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(request.target_pose)
            ok, q_solution = self._hardware.move_to_pose_ik(x, y, z, roll, pitch, yaw)
            response.success = ok
            response.message = "IK target accepted" if ok else "IK failed"
            response.q_solution = q_solution
        except Exception as exc:
            self._hardware.hold_current_position()
            response.success = False
            response.message = str(exc)
            response.q_solution = []
        self._node.publish_arm_status()
        return response

    def set_gripper(self, request, response):
        try:
            reached, reached_position = self._hardware.set_gripper_position(
                request.position,
            )
            response.success = bool(reached)
            response.reached_position = float(reached_position)
        except Exception as exc:
            response.success = False
            response.reached_position = 0.0
            self._node.get_logger().error(f"gripper set failed: {exc}")
        self._node.publish_arm_status()
        return response

    def open_gripper(self, request, response):
        return self._move_gripper(
            request, response, self._hardware.gripper_open_position, "open"
        )

    def close_gripper(self, request, response):
        return self._move_gripper(
            request, response, self._hardware.gripper_close_position, "close"
        )

    def _move_gripper(self, request, response, default_target: float, label: str):
        try:
            target = (
                default_target if request.position == 0.0 else float(request.position)
            )
            success, position = self._hardware.set_gripper_position(
                target,
                timeout=request.timeout if request.timeout > 0.0 else 3.0,
            )
            response.success = bool(success)
            response.reached_position = float(position)
            response.message = (
                f"gripper {label} complete" if success else f"gripper {label} timeout"
            )
        except Exception as exc:
            response.success = False
            response.reached_position = 0.0
            response.message = str(exc)
        self._node.publish_arm_status()
        return response
