from __future__ import annotations

import json
from pathlib import Path
import sys
import threading

import pytest


CONTROLLER_ROOT = Path(__file__).resolve().parents[1]
if str(CONTROLLER_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROLLER_ROOT))

from rebotarmcontroller.hardware_manager import HardwareManager  # noqa: E402
from rebotarmcontroller.safety_event_logger import SafetyEventLogger  # noqa: E402
from rebotarmcontroller.torque_profile import (  # noqa: E402
    SEVERITY_FAIL,
    TorqueProfile,
    TorqueProfileMonitor,
    normalize_mode,
)


def _profile_data(*, include_joint2: bool = True):
    joints = {
        "joint1": {
            "phase": [0.0, 1.0],
            "warn_min": [-1.0, -1.0],
            "warn_max": [1.0, 1.0],
            "fail_min": [-2.0, -2.0],
            "fail_max": [2.0, 2.0],
        }
    }
    if include_joint2:
        joints["joint2"] = {
            "phase": [0.0, 1.0],
            "warn_min": [-1.0, -1.0],
            "warn_max": [1.0, 1.0],
            "fail_min": [-2.0, -2.0],
            "fail_max": [2.0, 2.0],
        }
    return {"version": 2, "steps": {"pick": {"joints": joints}}}


def test_profile_rejects_missing_joint_coverage():
    profile = TorqueProfile(_profile_data(include_joint2=False))

    with pytest.raises(ValueError, match="missing joints: joint2"):
        profile.validate_joint_coverage(["joint1", "joint2"])


def test_unknown_step_does_not_silently_use_first_profile():
    profile = TorqueProfile(_profile_data())

    with pytest.raises(KeyError, match="no step"):
        profile.check("place", 0.5, ["joint1", "joint2"], [0.0, 0.0])


def test_invalid_mode_and_incomplete_feedback_are_rejected():
    profile = TorqueProfile(_profile_data())

    with pytest.raises(ValueError, match="unsupported torque profile mode"):
        normalize_mode("stpo_on_fail")
    with pytest.raises(ValueError, match="length must match"):
        profile.check("pick", 0.5, ["joint1", "joint2"], [0.0])


def test_static_violation_requires_configured_consecutive_samples():
    monitor = TorqueProfileMonitor(
        TorqueProfile(_profile_data()),
        mode="stop_on_fail",
        active_step_id="pick",
        violation_samples=3,
    )

    assert monitor.check(0.5, ["joint1", "joint2"], [3.0, 0.0])[0] is False
    assert monitor.check(0.5, ["joint1", "joint2"], [3.0, 0.0])[0] is False
    should_stop, severity, _ = monitor.check(
        0.5, ["joint1", "joint2"], [3.0, 0.0]
    )

    assert should_stop is True
    assert severity == SEVERITY_FAIL


def test_critical_torque_derivative_stops_immediately(monkeypatch):
    times = iter([10.0, 10.01])
    monkeypatch.setattr(
        "rebotarmcontroller.torque_profile.time.monotonic",
        lambda: next(times),
    )
    monitor = TorqueProfileMonitor(
        TorqueProfile(_profile_data()),
        mode="stop_on_fail",
        active_step_id="pick",
        violation_samples=10,
        derivative_limit=100.0,
    )

    monitor.check(0.5, ["joint1", "joint2"], [0.0, 0.0])
    should_stop, severity, _ = monitor.check(
        0.5, ["joint1", "joint2"], [2.1, 0.0]
    )

    assert should_stop is True
    assert severity == SEVERITY_FAIL
    assert monitor.critical_derivative_hit is True


def test_protective_stop_latch_cannot_be_cleared_by_normal_state_change():
    hardware = HardwareManager.__new__(HardwareManager)
    hardware._cmd_lock = threading.RLock()
    hardware._protective_stop_latched = True
    hardware._state_machine = "PROTECTIVE_STOP"

    hardware.set_state_machine("IDLE")

    assert hardware.state_machine == "PROTECTIVE_STOP"


def test_reset_buffer_preserves_persisted_fault(tmp_path):
    path = tmp_path / "last_safety_event.json"
    logger = SafetyEventLogger(path)
    logger.record_sample(
        phase=0.4,
        step_id="pick",
        joint_names=["joint1"],
        positions=[0.1],
        velocities=[0.0],
        torques=[2.5],
    )
    logger.save_event("FAIL", "collision", 0.4, "pick")

    logger.reset_buffer()

    assert json.loads(path.read_text(encoding="utf-8"))["message"] == "collision"
