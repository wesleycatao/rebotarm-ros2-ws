from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "third_party"
    / "reBotArm_control_py"
    / "reBotArm_control_py"
    / "trajectory"
    / "retiming.py"
)
SPEC = importlib.util.spec_from_file_location("rebotarm_trajectory_retiming", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
RETIMING = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = RETIMING
SPEC.loader.exec_module(RETIMING)
retime_joint_trajectory = RETIMING.retime_joint_trajectory
tracking_speed_scale = RETIMING.tracking_speed_scale

CONTROLLER_ROOT = Path(__file__).resolve().parents[1]
if str(CONTROLLER_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROLLER_ROOT))

from rebotarmcontroller.hardware_config import _add_runtime_config  # noqa: E402
from rebotarmcontroller.hardware_manager import HardwareManager  # noqa: E402
from rebotarmcontroller.ros_actions import sample_minimum_jerk_segment  # noqa: E402
from rebotarmcontroller.speed_override import DynamicSpeedOverride  # noqa: E402


def test_velocity_bottleneck_stretches_every_joint_without_changing_path():
    positions = np.array([[0.0, 0.0], [2.0, 0.5], [4.0, 1.0]])
    result = retime_joint_trajectory(
        positions,
        [0.0, 1.0, 2.0],
        max_velocities=[1.0, 1.0],
        max_accelerations=[10.0, 10.0],
    )

    assert result.scale_factor == pytest.approx(2.0)
    assert result.times.tolist() == pytest.approx([0.0, 2.0, 4.0])
    assert result.limiting_joint == 0
    assert result.limiting_quantity == "velocity"
    assert np.max(np.abs(result.velocities), axis=0).tolist() == pytest.approx(
        [1.0, 0.25]
    )
    # Retiming returns only timing/derivatives; geometric samples are untouched.
    assert positions.tolist() == [[0.0, 0.0], [2.0, 0.5], [4.0, 1.0]]


def test_acceleration_bottleneck_uses_square_root_time_scaling():
    positions = np.array([[0.0], [1.0], [0.0]])
    result = retime_joint_trajectory(
        positions,
        [0.0, 1.0, 2.0],
        max_velocities=[10.0],
        max_accelerations=[0.5],
    )

    assert result.scale_factor == pytest.approx(2.0)
    assert result.duration == pytest.approx(4.0)
    assert result.limiting_joint == 0
    assert result.limiting_quantity == "acceleration"
    assert np.max(np.abs(result.accelerations)) <= 0.5 + 1e-12


def test_velocity_check_uses_segments_even_when_point_average_cancels():
    positions = np.array([[0.0], [0.0], [2.0], [0.0], [0.0]])
    result = retime_joint_trajectory(
        positions,
        [0.0, 1.0, 2.0, 3.0, 4.0],
        max_velocities=[1.0],
        max_accelerations=[100.0],
    )

    assert result.scale_factor == pytest.approx(2.0)


def test_safety_factor_reduces_effective_joint_ceiling():
    result = retime_joint_trajectory(
        [[0.0], [1.0], [2.0]],
        [0.0, 1.0, 2.0],
        max_velocities=[1.0],
        max_accelerations=[10.0],
        safety_factor=0.8,
    )

    assert result.scale_factor == pytest.approx(1.25)
    assert np.max(np.abs(result.velocities)) <= 0.8 + 1e-12


def test_single_future_target_gets_bounded_minimum_jerk_before_retiming():
    positions, times = sample_minimum_jerk_segment(
        np.array([0.0, 0.0]),
        np.array([1.0, 0.25]),
        duration=0.1,
    )
    result = retime_joint_trajectory(
        positions,
        times,
        max_velocities=[1.0, 1.0],
        max_accelerations=[1.0, 1.0],
        safety_factor=0.9,
    )

    assert len(positions) >= 11
    assert positions[0].tolist() == pytest.approx([0.0, 0.0])
    assert positions[-1].tolist() == pytest.approx([1.0, 0.25])
    assert result.scale_factor > 1.0
    segment_velocities = np.diff(np.asarray(positions), axis=0) / np.diff(
        result.times
    )[:, np.newaxis]
    assert np.max(np.abs(segment_velocities), axis=0)[0] <= 0.9 + 1e-12
    assert np.max(np.abs(result.accelerations), axis=0)[0] <= 0.9 + 1e-12


@pytest.mark.parametrize(
    ("error", "expected"),
    [(0.02, 1.0), (0.05, 1.0), (0.10, 0.5), (0.15, 0.0), (0.30, 0.0)],
)
def test_tracking_governor_scales_one_common_virtual_clock(error, expected):
    scale = tracking_speed_scale([error, 0.0], [0.0, 0.0], 0.05, 0.15)
    assert scale == pytest.approx(expected)


def test_invalid_or_non_monotonic_trajectory_is_rejected():
    with pytest.raises(ValueError, match="strictly increasing"):
        retime_joint_trajectory(
            [[0.0], [1.0], [2.0]],
            [0.0, 1.0, 1.0],
            [1.0],
            [1.0],
        )


def _hardware_data():
    return {
        'groups': {'arm': {'joints': ['joint1', 'joint2']}},
        'joints': [
            {'name': 'joint1', 'POS_VEL': {'vlim': 5.0}, 'MIT': {}},
            {'name': 'joint2', 'POS_VEL': {'vlim': 3.0}, 'MIT': {}},
        ],
        'trajectory_limits': {
            'max_velocity': [1.0, 0.8],
            'max_acceleration': [2.0, 1.5],
            'safety_factor': 0.9,
            'tracking_error_soft': 0.04,
            'tracking_error_hard': 0.12,
        },
    }


def test_hardware_config_exposes_one_authoritative_runtime_limit_set():
    data = _hardware_data()
    _add_runtime_config(data)

    assert data['_runtime']['trajectory'] == {
        'max_velocity': [1.0, 0.8],
        'max_acceleration': [2.0, 1.5],
        'safety_factor': 0.9,
        'tracking_error_soft': 0.04,
        'tracking_error_hard': 0.12,
    }
    assert data['_runtime']['safety'] == {'protective_stop_mode': 'hold'}


def test_operational_velocity_cannot_exceed_driver_ceiling():
    data = _hardware_data()
    data['trajectory_limits']['max_velocity'] = [6.0, 0.8]

    with pytest.raises(ValueError, match='cannot exceed actuator'):
        _add_runtime_config(data)


def test_controller_exposes_authoritative_trajectory_limits():
    hardware = HardwareManager.__new__(HardwareManager)
    hardware._arm_group = SimpleNamespace(joint_names=["joint1", "joint2"])
    hardware._trajectory_max_velocities = np.asarray([1.0, 0.8])
    hardware._trajectory_max_accelerations = np.asarray([2.0, 1.5])
    hardware._trajectory_safety_factor = 0.9
    hardware._speed_override = DynamicSpeedOverride(
        hardware._trajectory_max_accelerations,
        safety_factor=hardware._trajectory_safety_factor,
    )

    assert hardware.trajectory_limits_snapshot() == {
        "joint_names": ["joint1", "joint2"],
        "max_velocity": [1.0, 0.8],
        "max_acceleration": [2.0, 1.5],
        "safety_factor": 0.9,
        "dynamic_speed_override": {
            "supported": True,
            "minimum": 0.05,
            "maximum": 1.0,
            "increase_rate_per_s": 0.5,
            "decrease_rate_per_s": 2.0,
        },
    }
