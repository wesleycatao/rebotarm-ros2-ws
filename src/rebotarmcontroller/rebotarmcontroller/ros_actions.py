from __future__ import annotations

import time

from control_msgs.action import FollowJointTrajectory, GripperCommand
import numpy as np
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rebotarm_msgs.action import MoveToPose
from .authorization import CAP_EXECUTE_RELEASED
from .conversions import pose_to_xyz_rpy


def compute_phase_by_position(
    current_positions: np.ndarray,
    trajectory_positions: list[np.ndarray],
) -> float:
    """Compute motion phase [0.0, 1.0] from current joint positions.

    The phase is derived from the distance travelled along the planned joint
    trajectory, making it robust to temporal variations (delays, speed changes,
    pauses) that would distort a time-based phase.

    Args:
        current_positions: Current joint positions.
        trajectory_positions: Planned joint positions (one array per point).

    Returns:
        Phase in [0.0, 1.0]. Returns 0.0 if the trajectory is empty or invalid.
    """
    if not trajectory_positions:
        return 0.0

    targets = [np.asarray(t, dtype=np.float64).reshape(-1) for t in trajectory_positions]
    n = len(current_positions)
    if any(len(t) != n for t in targets):
        return 0.0

    if len(targets) == 1:
        return 0.0

    # Compute cumulative arc-length along the trajectory in joint space.
    segment_lengths = [
        float(np.linalg.norm(targets[i] - targets[i - 1]))
        for i in range(1, len(targets))
    ]
    total_length = sum(segment_lengths)
    if total_length < 1e-9:
        return 0.0

    # Find closest projection on any segment.
    best_phase = 0.0
    min_dist_sq = float("inf")
    accumulated = 0.0

    for i in range(len(targets) - 1):
        p0 = targets[i]
        p1 = targets[i + 1]
        segment = p1 - p0
        seg_len_sq = float(np.dot(segment, segment))

        if seg_len_sq < 1e-18:
            # Degenerate segment; use distance to the single point.
            dist_sq = float(np.sum((current_positions - p0) ** 2))
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                best_phase = accumulated / total_length
            continue

        # Project current position onto the segment, clamped to [0, 1].
        t = max(0.0, min(1.0, np.dot(current_positions - p0, segment) / seg_len_sq))
        projection = p0 + t * segment
        dist_sq = float(np.sum((current_positions - projection) ** 2))

        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            best_phase = (accumulated + t * segment_lengths[i]) / total_length

        accumulated += segment_lengths[i]

    return float(max(0.0, min(1.0, best_phase)))


def sample_minimum_jerk_segment(
    start: np.ndarray,
    target: np.ndarray,
    duration: float,
    sample_period: float = 0.01,
) -> tuple[list[np.ndarray], list[float]]:
    """Sample a synchronized minimum-jerk move between two joint targets."""

    if not np.isfinite(duration) or duration <= 0.0:
        raise ValueError("single-point trajectory duration must be > 0")
    q0 = np.asarray(start, dtype=np.float64).reshape(-1)
    q1 = np.asarray(target, dtype=np.float64).reshape(-1)
    if q0.shape != q1.shape or not np.all(np.isfinite(q0)) or not np.all(np.isfinite(q1)):
        raise ValueError("minimum-jerk endpoints must be finite and have equal size")
    point_count = max(11, int(np.ceil(duration / max(sample_period, 1e-4))) + 1)
    times = np.linspace(0.0, duration, point_count)
    normalized_time = times / duration
    progress = (
        10.0 * normalized_time**3
        - 15.0 * normalized_time**4
        + 6.0 * normalized_time**5
    )
    positions = q0 + progress[:, np.newaxis] * (q1 - q0)
    return [row.copy() for row in positions], [float(value) for value in times]


class ArmActions:
    def __init__(self, node, hardware, namespace: str) -> None:
        self._node = node
        self._hardware = hardware
        self._namespace = namespace
        self._move_to_pose_server = ActionServer(
            node,
            MoveToPose,
            f"/{namespace}/move_to_pose",
            execute_callback=self.execute_move_to_pose,
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )
        self._follow_joint_trajectory_server = ActionServer(
            node,
            FollowJointTrajectory,
            f"/{namespace}/follow_joint_trajectory",
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )
        self._gripper_command_server = ActionServer(
            node,
            GripperCommand,
            f"/{namespace}/gripper/command",
            execute_callback=self.execute_gripper_command,
            goal_callback=self.gripper_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=node.reentrant_group,
        )

    def arm_goal_callback(self, _goal_request):
        if not self._node.authorization.permits(CAP_EXECUTE_RELEASED):
            self._node.get_logger().warning("rejecting arm motion: authorization denied")
            return GoalResponse.REJECT
        return self._gate_goal(
            (
                "TRAJ_RUNNING",
                "GRAVITY_COMP",
                "SAFE_HOMING",
                "PROTECTIVE_STOP",
            ),
            "arm motion",
        )

    def gripper_goal_callback(self, _goal_request):
        if not self._node.authorization.permits(CAP_EXECUTE_RELEASED):
            self._node.get_logger().warning("rejecting gripper: authorization denied")
            return GoalResponse.REJECT
        return self._gate_goal(
            ("GRAVITY_COMP", "SAFE_HOMING", "PROTECTIVE_STOP"),
            "gripper",
        )

    def _gate_goal(self, blocked, label):
        state = self._hardware.state_machine
        if state in blocked:
            self._node.get_logger().warning(f"rejecting {label} goal in state {state}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _fail_move_to_pose(self, goal_handle, result, message, *, canceled=False):
        protective_stop = self._hardware.protective_stop_active
        if self._hardware.state_machine not in ("SAFE_HOMING", "PROTECTIVE_STOP"):
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()
        if canceled:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        result.success = False
        if protective_stop and self._node.safety_fault_message:
            message = f"protective stop: {self._node.safety_fault_message}"
        result.message = message
        result.final_pose = self._hardware.current_pose()
        return result

    def execute_move_to_pose(self, goal_handle):
        goal = goal_handle.request
        result = MoveToPose.Result()
        self._node.reset_torque_profile_monitor()

        try:
            start_q = self._hardware.get_joint_positions().copy()
            self._node.start_safety_monitor(
                "move_to_pose",
                start_q=start_q,
            )
            x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(goal.target_pose)
            ok = self._hardware.move_to_pose_traj(
                x, y, z, roll, pitch, yaw, float(goal.duration)
            )
        except Exception as exc:
            self._node.stop_safety_monitor()
            if not self._hardware.protective_stop_active:
                self._hardware.hold_current_position()
            return self._fail_move_to_pose(goal_handle, result, str(exc))

        if not ok:
            self._node.stop_safety_monitor()
            return self._fail_move_to_pose(
                goal_handle, result, "trajectory planning failed"
            )
        self._node.publish_arm_status()

        # Capture planned trajectory and start position for position-based phase.
        planned_trajectory = self._hardware.get_planned_trajectory()
        target_q = None
        if planned_trajectory:
            target_q = np.array(planned_trajectory[-1], dtype=np.float64, copy=True)
        else:
            # Fallback: use the controller's end-pose target for straight-line distance.
            target_q = self._hardware.get_endpos_target()

        self._node.update_safety_monitor_context(
            trajectory=planned_trajectory,
            target_q=target_q,
        )

        monitor_start = time.monotonic()
        planned_duration = max(self._hardware.get_planned_duration(), 0.001)
        duration = planned_duration
        # The online common-clock governor may extend wall time while a joint
        # catches up.  A persistent stall still ends in a controlled hold.
        minimum_override = max(
            0.01, self._hardware.minimum_dynamic_speed_override
        )
        deadline = monitor_start + max(
            duration / minimum_override + 5.0,
            duration * 3.0,
        )
        try:
            while self._hardware.motion_active():
                if self._hardware.protective_stop_active:
                    return self._fail_move_to_pose(
                        goal_handle,
                        result,
                        "motion interrupted by protective stop",
                    )
                if self._hardware.state_machine == "SAFE_HOMING":
                    self._hardware.stop_motion()
                    break
                if goal_handle.is_cancel_requested:
                    self._hardware.stop_motion()
                    self._hardware.hold_current_position()
                    return self._fail_move_to_pose(
                        goal_handle, result, "move_to_pose canceled", canceled=True
                    )
                if time.monotonic() > deadline:
                    self._hardware.stop_motion()
                    self._hardware.hold_current_position()
                    return self._fail_move_to_pose(
                        goal_handle, result, "move_to_pose timeout"
                    )
                time.sleep(0.01)
        finally:
            self._node.stop_safety_monitor()

        if self._hardware.state_machine == "SAFE_HOMING":
            return self._fail_move_to_pose(
                goal_handle, result, "move_to_pose preempted by safe_home"
            )
        if self._hardware.protective_stop_active:
            return self._fail_move_to_pose(
                goal_handle, result, "motion interrupted by protective stop"
            )

        motion_error = self._hardware.get_motion_error()
        if motion_error:
            if not self._hardware.protective_stop_active:
                self._hardware.hold_current_position()
            return self._fail_move_to_pose(goal_handle, result, motion_error)

        positions = self._hardware.get_joint_positions()
        velocities = self._hardware.get_joint_velocities()
        result.success = True
        retime_scale, limiting_joint = self._hardware.get_retime_summary()
        limiting_label = (
            self._hardware.joint_names[limiting_joint]
            if limiting_joint is not None
            else "none"
        )
        result.message = (
            "move_to_traj completed "
            f"duration={planned_duration:.3f}s retime_scale={retime_scale:.3f} "
            f"limiting_joint={limiting_label} "
            f"positions={[float(v) for v in positions]} "
            f"velocities={[float(v) for v in velocities]}"
        )
        result.final_pose = self._hardware.current_pose()
        self._hardware.set_state_machine("IDLE")
        self._node.publish_arm_status()
        goal_handle.succeed()
        return result

    def execute_follow_joint_trajectory(self, goal_handle):
        goal = goal_handle.request
        result = FollowJointTrajectory.Result()
        trajectory = goal.trajectory
        joint_names = list(trajectory.joint_names)

        if not joint_names or not trajectory.points:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory must include joint_names and points"
            return result

        if joint_names != self._hardware.joint_names:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = (
                f"trajectory joint_names must be {self._hardware.joint_names}"
            )
            return result

        targets = [np.array(point.positions, dtype=np.float64) for point in trajectory.points]
        if any(len(target) != len(self._hardware.joint_names) for target in targets):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "point positions length must match joint_names"
            return result

        if any(not np.all(np.isfinite(target)) for target in targets):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "point positions must be finite"
            return result

        point_times = [
            float(point.time_from_start.sec)
            + float(point.time_from_start.nanosec) * 1e-9
            for point in trajectory.points
        ]
        if any(not np.isfinite(value) or value < 0.0 for value in point_times):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory times must be finite and non-negative"
            return result

        current_start = self._hardware.get_joint_positions().copy()
        if point_times[0] > 0.0:
            targets.insert(0, current_start)
            point_times.insert(0, 0.0)
        elif not np.allclose(targets[0], current_start, atol=0.05, rtol=0.0):
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = (
                "trajectory point at t=0 must match current joint positions"
            )
            return result

        # GUI joint moves normally contain only one future target.  Give that
        # segment a bounded minimum-jerk profile before applying joint limits.
        if len(targets) == 2:
            targets, point_times = sample_minimum_jerk_segment(
                targets[0], targets[1], point_times[1]
            )

        try:
            retimed = self._hardware.retime_joint_trajectory(targets, point_times)
        except ValueError as exc:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"invalid trajectory timing: {exc}"
            return result
        point_times = [float(value) for value in retimed.times]
        if retimed.scale_factor > 1.0 + 1e-6:
            limiting_label = (
                self._hardware.joint_names[retimed.limiting_joint]
                if retimed.limiting_joint is not None
                else "unknown"
            )
            self._node.get_logger().warning(
                "synchronized trajectory retiming: "
                f"scale={retimed.scale_factor:.3f} "
                f"limit={limiting_label}/{retimed.limiting_quantity} "
                f"duration={retimed.duration:.3f}s"
            )

        try:
            self._node.reset_torque_profile_monitor()
            self._hardware.begin_trajectory_stream()
            # Register full trajectory (including inserted start point) for the
            # independent safety monitor.
            self._node.start_safety_monitor(
                "follow_joint_trajectory", trajectory=targets
            )
            self._node.publish_arm_status()
            start = time.monotonic()
            last_update = start
            virtual_time = 0.0
            commanded_target = np.array(targets[0], dtype=np.float64, copy=True)
            self._hardware.set_joint_position_target(commanded_target)
            execution_deadline = start + max(
                retimed.duration / max(
                    0.01, self._hardware.minimum_dynamic_speed_override
                ) + 5.0,
                retimed.duration * 3.0,
            )

            for index in range(1, len(targets)):
                q0 = targets[index - 1]
                q1 = targets[index]
                t0 = point_times[index - 1]
                t1 = max(point_times[index], t0)
                desired_velocities = (
                    np.zeros_like(q1) if t1 <= t0 else (q1 - q0) / (t1 - t0)
                )

                while True:
                    if self._hardware.protective_stop_active:
                        goal_handle.abort()
                        result.error_code = (
                            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        )
                        result.error_string = (
                            "protective stop: "
                            + (
                                self._node.safety_fault_message
                                or "torque safety fault"
                            )
                        )
                        return result
                    if self._hardware.state_machine == "SAFE_HOMING":
                        goal_handle.abort()
                        result.error_code = (
                            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        )
                        result.error_string = (
                            "follow_joint_trajectory preempted by safe_home"
                        )
                        return result
                    wall_now = time.monotonic()
                    if wall_now > execution_deadline:
                        raise TimeoutError(
                            "tracking governor timeout; joint did not converge"
                        )
                    elapsed = max(0.0, wall_now - last_update)
                    last_update = wall_now
                    current_positions = self._hardware.get_joint_positions()
                    tracking_scale = self._hardware.trajectory_speed_scale(
                        commanded_target, current_positions
                    )
                    path_accelerations = retimed.accelerations[
                        min(index, len(retimed.accelerations) - 1)
                    ]
                    external_scale = self._hardware.advance_dynamic_speed_override(
                        elapsed,
                        desired_velocities,
                        path_accelerations,
                        tracking_scale,
                    )
                    speed_scale = tracking_scale * external_scale
                    virtual_time = min(t1, virtual_time + elapsed * speed_scale)
                    ratio = 1.0 if t1 <= t0 else max(
                        0.0, min(1.0, (virtual_time - t0) / (t1 - t0))
                    )
                    target = q0 + (q1 - q0) * ratio
                    self._hardware.set_joint_position_target(target)
                    commanded_target = target

                    positions = self._hardware.get_joint_positions()
                    velocities = self._hardware.get_joint_velocities()
                    feedback = FollowJointTrajectory.Feedback()
                    feedback.header.stamp = self._node.get_clock().now().to_msg()
                    feedback.joint_names = self._hardware.joint_names
                    feedback.desired.positions = [float(v) for v in target]
                    feedback.desired.velocities = [
                        float(v) for v in desired_velocities * speed_scale
                    ]
                    feedback.actual.positions = [float(v) for v in positions]
                    feedback.actual.velocities = [float(v) for v in velocities]
                    feedback.error.positions = [float(v) for v in target - positions]
                    feedback.error.velocities = [
                        float(v) for v in desired_velocities - velocities
                    ]
                    goal_handle.publish_feedback(feedback)

                    if goal_handle.is_cancel_requested:
                        self._hardware.hold_current_position()
                        goal_handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "follow_joint_trajectory canceled"
                        return result

                    if virtual_time >= t1:
                        final_segment = index == len(targets) - 1
                        final_error = float(np.max(np.abs(q1 - positions)))
                        if not final_segment or final_error <= (
                            self._hardware.trajectory_tracking_tolerance
                        ):
                            break
                    time.sleep(0.01)

        except Exception as exc:
            if not self._hardware.protective_stop_active:
                self._hardware.hold_current_position()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = f"execution failed: {exc}"
            return result
        finally:
            self._node.stop_safety_monitor()
            if self._hardware.state_machine not in (
                "SAFE_HOMING",
                "PROTECTIVE_STOP",
            ):
                self._hardware.end_trajectory_stream()
                self._node.publish_arm_status()

        if self._hardware.protective_stop_active:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = "motion interrupted by protective stop"
            return result
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        positions = self._hardware.get_joint_positions()
        velocities = self._hardware.get_joint_velocities()
        result.error_string = (
            "joint trajectory completed "
            f"duration={retimed.duration:.3f}s "
            f"retime_scale={retimed.scale_factor:.3f} "
            f"positions={[float(v) for v in positions]} "
            f"velocities={[float(v) for v in velocities]}"
        )
        return result

    def execute_gripper_command(self, goal_handle):
        goal = goal_handle.request.command
        result = GripperCommand.Result()
        feedback = GripperCommand.Feedback()

        try:
            self._hardware.set_gripper_target(goal.position)
        except Exception:
            goal_handle.abort()
            result.position = 0.0
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
            return result

        start = time.monotonic()
        last_pos = self._hardware.get_gripper_state()[0]
        stalled = False
        while time.monotonic() - start < 5.0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                pos, _, effort, _ = self._hardware.get_gripper_state()
                result.position = pos
                result.effort = effort
                result.stalled = stalled
                result.reached_goal = False
                return result

            pos, _, effort, _ = self._hardware.get_gripper_state()
            reached = self._hardware.gripper_reached_target()
            stalled = abs(pos - last_pos) < 1e-4 and abs(effort) >= float(goal.max_effort)
            feedback.position = pos
            feedback.effort = effort
            feedback.stalled = stalled
            feedback.reached_goal = reached
            goal_handle.publish_feedback(feedback)
            if reached:
                break
            last_pos = pos
            time.sleep(0.05)

        pos, _, effort, _ = self._hardware.get_gripper_state()
        result.position = pos
        result.effort = effort
        result.stalled = stalled
        result.reached_goal = self._hardware.gripper_reached_target()
        goal_handle.succeed()
        return result
