"""Tests for structural program validation before saving."""

from __future__ import annotations

import os
import sys
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from rebotarm_gui.motion.program import (  # noqa: E402
    ArcStep,
    IfStep,
    MoveStep,
    OutputStep,
    Program,
    WaitStep,
    pose,
)
from rebotarm_gui.motion.program_validator import (  # noqa: E402
    validate_edit,
    validate_operational,
    validate_program,
)


def _valid_program() -> Program:
    p = Program(name="teste")
    p.steps.append(MoveStep(pose=pose(0.3, 0.0, 0.3), duration=2.0))
    return p


def test_empty_program_is_blocked() -> None:
    p = Program(name="vazio")
    r = validate_edit(p)
    assert not r.can_save
    assert any("vazio" in i.message for i in r.structural_errors)


def test_valid_program_saves() -> None:
    p = _valid_program()
    r = validate_edit(p)
    assert r.can_save
    assert r.can_release


def test_nan_speed_scale_blocks_save() -> None:
    p = _valid_program()
    p.speed_scale = float("nan")
    r = validate_edit(p)
    assert not r.can_save


def test_zero_speed_scale_blocks_save() -> None:
    p = _valid_program()
    p.speed_scale = 0.0
    r = validate_edit(p)
    assert not r.can_save


def test_missing_name_blocks_save() -> None:
    p = Program(name="")
    p.steps.append(MoveStep())
    r = validate_edit(p)
    assert not r.can_save


def test_duplicate_ids_not_required_for_save() -> None:
    """Duplicate labels are not structural errors; preflight handles them."""
    p = Program(name="dup")
    p.steps.append(MoveStep(label="a", duration=1.0))
    p.steps.append(MoveStep(label="a", duration=1.0))
    r = validate_edit(p)
    assert r.can_save


def test_invalid_pose_nan_blocks_save() -> None:
    p = Program(name="nan")
    step = MoveStep()
    step.pose["x"] = float("nan")
    p.steps.append(step)
    r = validate_edit(p)
    assert not r.can_save


def test_invalid_arc_samples_blocks_save() -> None:
    p = Program(name="arc")
    step = ArcStep()
    step.samples = 1
    p.steps.append(step)
    r = validate_edit(p)
    assert not r.can_save


def test_negative_wait_time_blocks_save() -> None:
    p = Program(name="wait")
    step = WaitStep(time=-1.0)
    p.steps.append(step)
    r = validate_edit(p)
    assert not r.can_save


def test_missing_io_id_blocks_save() -> None:
    p = Program(name="io")
    step = OutputStep(io_id="")
    p.steps.append(step)
    r = validate_edit(p)
    assert not r.can_save


def test_invalid_output_action_blocks_save() -> None:
    p = Program(name="out")
    step = OutputStep(action="pulse")
    p.steps.append(step)
    r = validate_edit(p)
    assert not r.can_save


def test_empty_if_condition_warns() -> None:
    p = Program(name="if")
    step = IfStep()
    step.then_steps.append(MoveStep(duration=1.0))
    p.steps.append(step)
    r = validate_edit(p)
    assert r.can_save
    assert any("vazia" in i.message for i in r.operational_warnings or r.infos)


def test_operational_without_bridge_warns() -> None:
    p = _valid_program()
    r = validate_operational(p)
    assert any("ROS" in i.message for i in r.operational_warnings)


def test_program_merge_includes_operational_warnings() -> None:
    p = _valid_program()
    r = validate_program(p, bridge=None)
    assert r.can_save
    assert not r.can_release
    assert any("ROS" in i.message for i in r.operational_warnings)


if __name__ == "__main__":
    import pytest

    pytest.main([__file__, "-v"])
