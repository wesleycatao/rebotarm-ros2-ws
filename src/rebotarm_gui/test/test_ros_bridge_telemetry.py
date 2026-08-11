"""Unit tests for fail-closed telemetry validation."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

from sensor_msgs.msg import JointState

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

from rebotarm_gui.ros_bridge import (  # noqa: E402
    _BridgeNode,
    parse_trajectory_limits_payload,
)


class TestJointStateValidation(unittest.TestCase):
    def test_valid_joint_state_is_accepted(self) -> None:
        msg = JointState()
        msg.name = ["joint1", "joint2"]
        msg.position = [0.1, -0.2]
        msg.velocity = []
        msg.effort = [0.0, 0.0]

        self.assertIsNone(_BridgeNode._validate_joint_state(msg))

    def test_mismatched_vector_is_rejected(self) -> None:
        msg = JointState()
        msg.name = ["joint1", "joint2"]
        msg.position = [0.1]

        reason = _BridgeNode._validate_joint_state(msg)
        self.assertIsNotNone(reason)
        self.assertIn("position", str(reason))

    def test_non_finite_value_is_rejected(self) -> None:
        msg = JointState()
        msg.name = ["joint1"]
        msg.position = [float("nan")]

        reason = _BridgeNode._validate_joint_state(msg)
        self.assertIsNotNone(reason)
        self.assertIn("NaN", str(reason))

    def test_duplicate_joint_name_is_rejected(self) -> None:
        msg = JointState()
        msg.name = ["joint1", "joint1"]
        msg.position = [0.1, 0.2]

        reason = _BridgeNode._validate_joint_state(msg)
        self.assertIsNotNone(reason)
        self.assertIn("duplicados", str(reason))

    def test_dynamic_limits_payload_is_validated(self) -> None:
        parsed = parse_trajectory_limits_payload(
            '{"joint_names":["joint1"],"max_velocity":[1.0],'
            '"max_acceleration":[2.0],"safety_factor":0.9}'
        )
        self.assertEqual(parsed["joint_names"], ["joint1"])
        self.assertEqual(parsed["max_acceleration"], [2.0])

        with self.assertRaises(ValueError):
            parse_trajectory_limits_payload(
                '{"joint_names":["joint1"],"max_velocity":[0.0],'
                '"max_acceleration":[2.0],"safety_factor":0.9}'
            )


if __name__ == "__main__":
    unittest.main()
