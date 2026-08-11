"""Tests for productive motion editing and trajectory preview workflows."""

from __future__ import annotations

import os
import sys
import time
import unittest
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/rebotarm_gui_test_mpl")

from PySide6.QtCore import QModelIndex  # noqa: E402
from PySide6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from rebotarm_gui.motion import (  # noqa: E402
    GripperStep,
    LineStep,
    LoopStep,
    MoveStep,
    Program,
    pose,
)
from rebotarm_gui.ros_bridge import RosBridge  # noqa: E402
from rebotarm_gui.tabs.motion_tab import MotionTab  # noqa: E402
from rebotarm_gui.widgets.trajectory_preview_dialog import (  # noqa: E402
    TrajectoryPreviewDialog,
    build_trajectory_preview,
)


def _three_moves() -> Program:
    return Program(
        name="produtividade",
        steps=[
            MoveStep(label="A", pose=pose(0.1, 0.0, 0.2), duration=1.0),
            MoveStep(label="B", pose=pose(0.2, 0.0, 0.2), duration=1.0),
            MoveStep(label="C", pose=pose(0.3, 0.0, 0.2), duration=1.0),
        ],
    )


class TestMotionEditorProductivity(unittest.TestCase):
    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self) -> None:
        self.bridge = RosBridge()
        self.tab = MotionTab(self.bridge)
        self.tab.load_program(_three_moves())
        self.tab.show()
        self._app.processEvents()

    def tearDown(self) -> None:
        self.tab.close()
        self.tab.deleteLater()
        self._app.processEvents()

    def test_internal_drag_reorders_model_and_can_be_undone(self) -> None:
        editor = self.tab._editor
        moved = editor._list.model().moveRow(
            QModelIndex(), 0, QModelIndex(), editor._list.count()
        )

        self.assertTrue(moved)
        self.assertEqual([step.label for step in self.tab._program.steps], ["B", "C", "A"])
        self.assertTrue(self.tab._is_modified)
        self.tab._undo()
        self.assertEqual([step.label for step in self.tab._program.steps], ["A", "B", "C"])

    def test_duplicate_and_remove_shortcuts_mutate_selected_step(self) -> None:
        editor = self.tab._editor
        editor._list.setCurrentRow(1)
        editor._shortcut_duplicate.activated.emit()

        self.assertEqual([step.label for step in self.tab._program.steps], ["A", "B", "B", "C"])
        self.assertIsNot(self.tab._program.steps[1], self.tab._program.steps[2])
        editor._shortcut_remove.activated.emit()
        self.assertEqual([step.label for step in self.tab._program.steps], ["A", "B", "C"])

    def test_nested_drag_participates_in_parent_undo(self) -> None:
        self.tab.load_program(
            Program(
                name="loop",
                steps=[
                    LoopStep(
                        label="repetir",
                        count=2,
                        steps=[MoveStep(label="A"), MoveStep(label="B")],
                    )
                ],
            )
        )
        nested = self.tab._editor._stack._loop_editor
        self.assertIsNotNone(nested)

        nested._list.model().moveRow(QModelIndex(), 0, QModelIndex(), 2)
        loop = self.tab._program.steps[0]
        self.assertEqual([step.label for step in loop.steps], ["B", "A"])
        self.tab._undo()
        loop = self.tab._program.steps[0]
        self.assertEqual([step.label for step in loop.steps], ["A", "B"])

    def test_execution_suffix_preserves_original_highlight_indexes(self) -> None:
        run_program, ops, meta, _labels = self.tab._compile_execution(1)

        self.assertEqual([step.label for step in run_program.steps], ["B", "C"])
        self.assertEqual(len(ops), 2)
        self.assertEqual(set(meta["pc_to_root_index"].values()), {1, 2})
        self.assertEqual(meta["action_count"], 2)

    def test_run_from_selected_forwards_current_top_level_index(self) -> None:
        requested: list[int] = []
        self.tab._editor._list.setCurrentRow(2)
        self.tab._start_run = requested.append

        self.tab._run_from_selected()

        self.assertEqual(requested, [2])

    def test_production_blocks_recipe_released_with_other_calibration(self) -> None:
        program = _three_moves()
        program.parameters["_calibration_profile"] = {
            "integrity_hash": "old-calibration"
        }
        program.release("supervisor")
        self.tab.load_program(program)
        self.tab._production_mode = True
        self.tab._dry_run = False
        self.tab._production_prerequisite_provider = lambda: (
            True,
            "perfil ativo",
            {"integrity_hash": "new-calibration"},
        )
        messages: list[str] = []
        original_critical = QMessageBox.critical
        QMessageBox.critical = (
            lambda _parent, _title, message, *args, **kwargs: messages.append(
                str(message)
            )
        )
        try:
            self.tab._start_run(0)
        finally:
            QMessageBox.critical = original_critical

        self.assertFalse(self.tab._running)
        self.assertTrue(messages)
        self.assertIn("não foi liberada", messages[-1])

    def test_selected_suffix_runs_to_completion_in_simulation(self) -> None:
        for step in self.tab._program.steps:
            step.duration = 0.1
        self.tab._dry_run = True
        self.tab._chk_dry_run.setChecked(True)
        self.tab._preflight_gate = lambda _program=None: True
        self.tab._check_execution_preconditions = lambda: (True, "")
        self.bridge.safety_set_step = lambda *_args, **_kwargs: True
        markers: list[str] = []
        self.tab._record_step_marker = markers.append
        self.tab._editor._list.setCurrentRow(1)

        original_question = QMessageBox.question
        QMessageBox.question = lambda *args, **kwargs: QMessageBox.Yes
        try:
            self.tab._run_from_selected()
            deadline = time.monotonic() + 3.0
            while self.tab._running and time.monotonic() < deadline:
                self._app.processEvents()
                time.sleep(0.02)
        finally:
            QMessageBox.question = original_question

        self.assertFalse(self.tab._running)
        self.assertEqual(markers, ["B_001", "C_002"])
        self.assertEqual(self.tab._lbl_status.text(), "Programa concluído.")

    def test_preview_compiles_generated_waypoints_and_non_motion_actions(self) -> None:
        program = Program(
            name="preview",
            speed_scale=2.0,
            steps=[
                MoveStep(label="ignorado", pose=pose(0.0, 0.0, 0.2), duration=2.0),
                LineStep(
                    label="linha",
                    start=pose(0.0, 0.0, 0.2),
                    end=pose(0.1, 0.0, 0.2),
                    speed=0.1,
                ),
                GripperStep(label="fecha", position_mm=10.0),
            ],
        )

        data = build_trajectory_preview(program, start_index=1)
        slowed = build_trajectory_preview(
            program, start_index=1, speed_override=0.5
        )

        self.assertEqual(data.start_index, 1)
        self.assertGreater(len(data.points), 1)
        self.assertTrue(all(point.root_index == 1 for point in data.points))
        self.assertAlmostEqual(data.distance_m, 0.08, places=6)
        self.assertEqual(data.non_motion_action_count, 1)
        self.assertGreater(data.motion_duration_s, 0.0)
        self.assertAlmostEqual(
            slowed.motion_duration_s, data.motion_duration_s * 2.0
        )

    def test_preview_dialog_handles_program_without_cartesian_moves(self) -> None:
        dialog = TrajectoryPreviewDialog(
            Program(name="gripper", steps=[GripperStep(position_mm=5.0)])
        )
        try:
            self.assertEqual(dialog.data.points, ())
            self.assertIn("Nenhum waypoint", dialog._figure.axes[0].texts[0].get_text())
        finally:
            dialog.close()


if __name__ == "__main__":
    unittest.main()
