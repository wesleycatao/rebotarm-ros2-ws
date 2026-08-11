"""Joint-demand analysis regression tests."""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

from PySide6.QtCore import QObject, Signal

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/rebotarm_gui_test_mpl")

from rebotarm_gui.motion import (  # noqa: E402
    Condition,
    ConditionExpr,
    IfStep,
    LoopStep,
    MoveStep,
    Program,
    SRC_SIGNAL,
    pose,
)
from rebotarm_gui.motion.dynamics import (  # noqa: E402
    MINIMUM_JERK_PEAK_ACCELERATION,
    MINIMUM_JERK_PEAK_VELOCITY,
    DynamicsProbe,
    analyze_joint_dynamics,
    build_motion_paths,
    limits_signature,
)
from rebotarm_gui.widgets.trajectory_preview_dialog import (  # noqa: E402
    TrajectoryPreviewDialog,
)
from PySide6.QtWidgets import QApplication  # noqa: E402


class _SynchronousBridge(QObject):
    ik_pose_checked = Signal(str, bool, str, object)

    def trajectory_limits(self):
        return {
            "joint_names": ["joint1"],
            "max_velocity": [1.0],
            "max_acceleration": [1.0],
            "safety_factor": 0.9,
        }

    def joint_names(self):
        return ["joint1"]

    def current_arm_joint_positions(self):
        return [0.0]

    def ik_service_ready(self):
        return True

    def check_pose_reachable(self, key, pose_dict):
        self.ik_pose_checked.emit(
            key, True, "", [float(pose_dict["x"])]
        )
        return True


def _solutions(paths, values):
    return {
        target.key(): list(values[index])
        for path in paths
        for index, target in enumerate(path)
    }


def test_motion_paths_unroll_loop_and_keep_if_branches_separate():
    conditional = IfStep(
        condition=ConditionExpr(
            items=[
                Condition(
                    source=SRC_SIGNAL,
                    params={"io_id": "IN1"},
                    op="==",
                    value=1,
                )
            ]
        ),
        then_steps=[MoveStep(pose=pose(0.2, 0.0, 0.3), duration=2.0)],
        else_steps=[MoveStep(pose=pose(0.4, 0.0, 0.3), duration=2.0)],
    )
    program = Program(
        steps=[
            LoopStep(
                count=2,
                steps=[MoveStep(pose=pose(0.1, 0.0, 0.3), duration=1.0)],
            ),
            conditional,
        ]
    )

    paths, notes = build_motion_paths(program)

    assert len(paths) == 2
    assert {len(path) for path in paths} == {3}
    assert paths[0][0].key() == paths[0][1].key()
    assert notes == []


def test_minimum_jerk_peaks_and_safe_override_are_calculated():
    program = Program(
        steps=[MoveStep(pose=pose(0.2, 0.0, 0.3), duration=2.0)]
    )
    paths, _notes = build_motion_paths(program)
    solution_map = {paths[0][0].key(): [1.0, 0.2]}

    report = analyze_joint_dynamics(
        paths,
        solution_map,
        joint_names=["joint1", "joint2"],
        current_positions=[0.0, 0.0],
        max_velocities=[1.0, 1.0],
        max_accelerations=[1.0, 1.0],
        safety_factor=1.0,
        analyzed_override=1.0,
    )

    joint1 = report.metrics[0]
    assert joint1.peak_velocity == (
        MINIMUM_JERK_PEAK_VELOCITY / 2.0
    )
    assert joint1.peak_acceleration == (
        MINIMUM_JERK_PEAK_ACCELERATION / 4.0
    )
    assert report.verified
    assert not report.within_limits
    assert report.limiting_joint == "joint1"
    assert report.limiting_quantity == "aceleração"
    assert report.required_time_scale == math.sqrt(
        MINIMUM_JERK_PEAK_ACCELERATION / 4.0
    )
    assert report.max_safe_override == 1.0 / report.required_time_scale


def test_safety_factor_is_applied_to_displayed_limits():
    program = Program(
        steps=[MoveStep(pose=pose(0.2, 0.0, 0.3), duration=4.0)]
    )
    paths, _notes = build_motion_paths(program)
    report = analyze_joint_dynamics(
        paths,
        {paths[0][0].key(): [0.5]},
        joint_names=["joint1"],
        current_positions=[0.0],
        max_velocities=[1.0],
        max_accelerations=[2.0],
        safety_factor=0.8,
        analyzed_override=1.0,
    )

    assert report.ok
    assert report.metrics[0].velocity_limit == 0.8
    assert report.metrics[0].acceleration_limit == 1.6
    assert report.max_safe_override == 1.0


def test_missing_solution_fails_closed():
    program = Program(
        steps=[MoveStep(pose=pose(0.2, 0.0, 0.3), duration=2.0)]
    )
    paths, _notes = build_motion_paths(program)

    report = analyze_joint_dynamics(
        paths,
        {},
        joint_names=["joint1"],
        current_positions=[0.0],
        max_velocities=[1.0],
        max_accelerations=[1.0],
        safety_factor=0.9,
        analyzed_override=1.0,
    )

    assert not report.verified
    assert not report.ok
    assert "IK ausente" in report.summary()


def test_limit_signature_is_stable_and_sensitive():
    limits = {
        "joint_names": ["joint1"],
        "max_velocity": [1.0],
        "max_acceleration": [2.0],
        "safety_factor": 0.9,
    }

    first = limits_signature(limits)
    second = limits_signature(dict(reversed(list(limits.items()))))
    changed = limits_signature({**limits, "safety_factor": 0.8})

    assert len(first) == 64
    assert first == second
    assert first != changed


def test_probe_handles_synchronous_ik_responses_without_finishing_early():
    program = Program(
        steps=[
            MoveStep(pose=pose(0.1, 0.0, 0.3), duration=4.0),
            MoveStep(pose=pose(0.2, 0.0, 0.3), duration=4.0),
        ]
    )
    received = []
    probe = DynamicsProbe(program, _SynchronousBridge())
    probe.finished.connect(received.append)

    probe.start()

    assert len(received) == 1
    assert received[0].verified
    assert received[0].target_count == 2


def test_preview_populates_joint_dynamics_table():
    app = QApplication.instance() or QApplication(sys.argv)
    dialog = TrajectoryPreviewDialog(
        Program(
            steps=[
                MoveStep(
                    label="aproximar",
                    pose=pose(0.1, 0.0, 0.3),
                    duration=4.0,
                )
            ]
        ),
        bridge=_SynchronousBridge(),
    )
    try:
        app.processEvents()
        assert dialog.dynamics_report is not None
        assert dialog.dynamics_report.verified
        assert dialog._dynamics_table.rowCount() == 1
        assert "APROVADO" in dialog._dynamics_status.text()
        assert (
            dialog._dynamics_table.item(0, 8).text()
            == "aproximar"
        )
    finally:
        dialog.close()
