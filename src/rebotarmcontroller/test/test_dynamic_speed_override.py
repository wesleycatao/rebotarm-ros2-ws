from __future__ import annotations

import math

import numpy as np
import pytest

from rebotarmcontroller.speed_override import DynamicSpeedOverride


def _governor(**kwargs) -> DynamicSpeedOverride:
    return DynamicSpeedOverride(
        [1.0, 2.0],
        safety_factor=0.9,
        minimum=0.05,
        increase_rate_per_s=0.5,
        decrease_rate_per_s=2.0,
        **kwargs,
    )


def test_idle_request_settles_without_a_motion_discontinuity():
    governor = _governor()
    valid, requested, message = governor.request(0.35)

    assert valid
    assert message == ""
    assert requested == pytest.approx(0.35)
    assert governor.settle_while_idle() == pytest.approx(0.35)
    assert not governor.snapshot().ramping


def test_invalid_request_fails_slow_to_minimum():
    governor = _governor()

    valid, requested, message = governor.request(math.nan)

    assert not valid
    assert requested == pytest.approx(0.05)
    assert "mínimo seguro" in message


def test_decrease_is_ramped_and_never_changes_individual_joint_scales():
    governor = _governor()
    governor.request(0.05)

    applied = governor.advance(
        0.1,
        path_velocities=np.array([0.2, 0.8]),
        path_accelerations=np.zeros(2),
        tracking_scale=1.0,
    )

    # One scalar is returned for the common trajectory clock. The configured
    # 2/s ramp would reach 0.8, and both joints therefore share that value.
    assert applied == pytest.approx(0.8)
    assert governor.snapshot().ramping


def test_acceleration_interval_limits_a_speed_increase():
    governor = _governor()
    governor.request(0.5)
    governor.settle_while_idle()
    governor.request(1.0)

    # Joint 1 already consumes the complete effective acceleration ceiling.
    # Raising the common clock here would add q_dot * scale_dot and is held
    # until a later trajectory sample has margin.
    applied = governor.advance(
        0.1,
        path_velocities=np.array([1.0, 0.0]),
        path_accelerations=np.array([3.6, 0.0]),
        tracking_scale=1.0,
    )

    assert applied == pytest.approx(0.5)


def test_acceleration_interval_uses_available_margin_for_braking():
    governor = _governor()
    governor.request(0.05)

    applied = governor.advance(
        0.1,
        path_velocities=np.array([1.0, 0.0]),
        path_accelerations=np.array([0.9, 0.0]),
        tracking_scale=1.0,
    )

    # base=+0.9 and decreasing scale contributes -rate. The admissible
    # interval reaches rate=1.8/s, below the configured 2/s cap.
    assert applied == pytest.approx(0.82)


def test_bad_runtime_derivatives_freeze_ramp_instead_of_guessing():
    governor = _governor()
    governor.request(0.2)

    applied = governor.advance(
        0.1,
        path_velocities=[float("nan"), 0.0],
        path_accelerations=[0.0, 0.0],
    )

    assert applied == pytest.approx(1.0)
    assert "congelada" in governor.snapshot().last_error
