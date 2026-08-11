"""Regression tests for robust vision-position envelope sampling."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

from rebotarm_gui.motion import MoveStep, Program, pose  # noqa: E402
from rebotarm_gui.motion.vision_envelope import (  # noqa: E402
    bind_calibration_error,
    envelope_spec,
    envelope_variants,
)


def _program(shape: str = "polygon") -> Program:
    envelope: dict[str, object] = {
        "version": 1,
        "anchor_m": [0.30, 0.00, 0.05],
        "affected_step_labels": ["aproximar", "coletar"],
        "shape": shape,
        "z_bounds_m": [0.05, 0.05],
        "uncertainty_m": [0.005, 0.006, 0.002],
        "calibration_error_m": 0.004,
        "calibration_integrity_hash": "calibration-a",
        "production_ready": shape != "point",
    }
    if shape == "polygon":
        envelope["polygon_m"] = [
            [0.20, -0.10],
            [0.40, -0.10],
            [0.40, 0.10],
            [0.20, 0.10],
        ]
    elif shape == "circle":
        envelope["circle_m"] = [0.30, 0.00, 0.10]
    else:
        envelope["point_m"] = [0.30, 0.00]
    return Program(
        name="pick",
        parameters={
            "_vision_guided": True,
            "_vision_envelope": envelope,
        },
        steps=[
            MoveStep(label="home", pose=pose(0.10, 0.0, 0.30)),
            MoveStep(label="aproximar", pose=pose(0.30, 0.0, 0.20)),
            MoveStep(label="coletar", pose=pose(0.30, 0.0, 0.05)),
        ],
    )


def test_polygon_variants_translate_only_declared_steps() -> None:
    program = _program()
    variants = envelope_variants(program)
    assert len(variants) > 4
    for _label, variant in variants:
        assert variant.steps[0].pose == program.steps[0].pose
    translated = next(
        variant
        for _label, variant in variants
        if variant.steps[1].pose != program.steps[1].pose
    )
    assert translated.steps[1].pose != program.steps[1].pose
    assert translated.steps[2].pose != program.steps[2].pose


def test_effective_uncertainty_includes_calibration_residual() -> None:
    program = _program("circle")
    spec = envelope_spec(program)
    assert spec is not None
    assert spec.effective_uncertainty_m == pytest.approx((0.009, 0.010, 0.002))
    bind_calibration_error(
        program,
        {
            "max_error_m": 0.012,
            "integrity_hash": "calibration-a",
        },
    )
    rebound = envelope_spec(program)
    assert rebound is not None
    assert rebound.effective_uncertainty_m == pytest.approx((0.017, 0.018, 0.002))


def test_point_only_observation_is_not_production_envelope() -> None:
    spec = envelope_spec(_program("point"))
    assert spec is not None
    with pytest.raises(ValueError, match="posição observada"):
        spec.validate_for_production()


def test_operational_area_bound_to_other_calibration_fails_closed() -> None:
    program = _program()
    with pytest.raises(ValueError, match="outra calibração"):
        bind_calibration_error(
            program,
            {
                "max_error_m": 0.004,
                "integrity_hash": "calibration-b",
            },
        )


def test_missing_relative_step_fails_closed() -> None:
    program = _program()
    raw = dict(program.parameters["_vision_envelope"])
    raw["affected_step_labels"] = ["etapa removida"]
    program.parameters["_vision_envelope"] = raw
    with pytest.raises(ValueError, match="não encontradas"):
        envelope_variants(program)


def test_duplicate_relative_labels_fail_closed() -> None:
    program = _program()
    program.steps.append(MoveStep(label="coletar", pose=pose(0.3, 0.0, 0.05)))
    with pytest.raises(ValueError, match="únicos"):
        envelope_variants(program)


def test_guided_execution_subset_may_omit_already_completed_steps() -> None:
    program = _program()
    program.steps = [program.steps[0]]
    program.parameters["_execution_subset_start_index"] = 2
    variants = envelope_variants(program)
    assert len(variants) == 1
    assert variants[0][0].startswith("subconjunto")


def test_envelope_is_covered_by_recipe_integrity_hash() -> None:
    program = _program()
    program.release("supervisor")
    assert program.verify_release()[0]
    raw = dict(program.parameters["_vision_envelope"])
    raw["uncertainty_m"] = [0.020, 0.020, 0.005]
    program.parameters["_vision_envelope"] = raw
    valid, reason = program.verify_release()
    assert not valid
    assert "alterado" in reason
