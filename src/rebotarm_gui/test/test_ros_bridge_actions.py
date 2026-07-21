"""Unit coverage for action readiness, deadlines and global cancellation."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

from action_msgs.msg import GoalStatus

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

from rebotarm_gui.ros_bridge import RosBridge  # noqa: E402


class _ImmediateFuture:
    def add_done_callback(self, callback) -> None:
        callback(self)

    @staticmethod
    def result():
        return SimpleNamespace(goals_canceling=[object()])


class _FakeGoalHandle:
    def __init__(self) -> None:
        self.cancel_calls = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return _ImmediateFuture()


class TestRosBridgeActions(unittest.TestCase):
    def test_readiness_requires_fresh_heartbeat_and_action_server(self) -> None:
        bridge = RosBridge()
        status = SimpleNamespace(enabled=True, state_machine="READY")
        bridge._node = SimpleNamespace(
            arm_status=status,
            action_server_ready=lambda name: name == "move_to_pose",
        )

        bridge._arm_alive_state = False
        ok, reason = bridge.motion_readiness()
        self.assertFalse(ok)
        self.assertIn("heartbeat", reason)

        bridge._arm_alive_state = True
        ok, reason = bridge.motion_readiness()
        self.assertTrue(ok, reason)

        ok, reason = bridge.motion_readiness("follow_joint_trajectory")
        self.assertFalse(ok)
        self.assertIn("indisponível", reason)

    def test_cancel_active_actions_covers_pending_and_accepted_goals(self) -> None:
        bridge = RosBridge()
        handle = _FakeGoalHandle()
        bridge._action_requests = {
            1: {
                "label": "move_to_pose",
                "tag": None,
                "handle": handle,
                "accept_timer": None,
                "result_timer": None,
            },
            2: {
                "label": "follow_joint_trajectory",
                "tag": None,
                "handle": None,
                "accept_timer": None,
                "result_timer": None,
            },
            3: {
                "label": "gripper_command",
                "tag": None,
                "handle": None,
                "accept_timer": None,
                "result_timer": None,
            },
        }

        count = bridge.cancel_active_actions()

        self.assertEqual(count, 3)
        self.assertEqual(handle.cancel_calls, 1)
        self.assertEqual(bridge._action_requests, {})

    def test_gripper_result_does_not_default_to_success(self) -> None:
        wrapped = SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(reached_goal=False, stalled=True),
        )
        success, message, _pose = RosBridge._parse_action_result(
            "gripper_command", wrapped
        )
        self.assertFalse(success)
        self.assertIn("bloqueado", message)

    def test_acceptance_timeout_emits_failure_and_forgets_request(self) -> None:
        bridge = RosBridge()
        results = []
        bridge.action_result.connect(lambda name, ok, msg: results.append((name, ok, msg)))
        bridge._action_requests = {
            1: {
                "label": "move_to_pose",
                "tag": None,
                "handle": None,
                "accept_timer": None,
                "result_timer": None,
            }
        }

        bridge._on_action_accept_timeout(1)

        self.assertEqual(bridge._action_requests, {})
        self.assertEqual(results[0][0:2], ("move_to_pose", False))
        self.assertIn("aceitação", results[0][2])

    def test_result_timeout_requests_cancel_and_emits_failure(self) -> None:
        bridge = RosBridge()
        results = []
        handle = _FakeGoalHandle()
        bridge.action_result.connect(lambda name, ok, msg: results.append((name, ok, msg)))
        bridge._action_requests = {
            1: {
                "label": "follow_joint_trajectory",
                "tag": None,
                "handle": handle,
                "accept_timer": None,
                "result_timer": None,
            }
        }

        bridge._on_action_result_timeout(1)

        self.assertEqual(handle.cancel_calls, 1)
        self.assertEqual(results[0][0:2], ("follow_joint_trajectory", False))
        self.assertIn("resultado", results[0][2])

    def test_non_success_goal_status_is_reported_as_failure(self) -> None:
        wrapped = SimpleNamespace(
            status=GoalStatus.STATUS_ABORTED,
            result=SimpleNamespace(success=True, message=""),
        )
        success, message, _pose = RosBridge._parse_action_result(
            "move_to_pose", wrapped
        )
        self.assertFalse(success)
        self.assertIn("abortado", message)


if __name__ == "__main__":
    unittest.main()
