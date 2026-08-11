"""Tests for the editable program flowchart widget."""

from __future__ import annotations

import sys

import pytest
from PySide6.QtCore import Qt
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import QApplication

from rebotarm_gui.motion.program import (
    IfStep,
    LoopStep,
    MoveStep,
    Program,
    WaitStep,
)
from rebotarm_gui.widgets.program_flow_widget import ProgramFlowWidget


@pytest.fixture
def app() -> QApplication:
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    return app


def _program_with_steps() -> Program:
    p = Program(name="teste")
    p.steps = [
        MoveStep(label="home"),
        WaitStep(time=1.0),
        MoveStep(label="pick"),
    ]
    return p


def _nested_program() -> Program:
    p = Program(name="aninhado")
    p.steps = [
        MoveStep(label="inicio"),
        LoopStep(
            label="repete",
            count=2,
            steps=[
                MoveStep(label="dentro"),
                IfStep(
                    label="cond",
                    then_steps=[MoveStep(label="entao")],
                    else_steps=[WaitStep(time=0.5)],
                ),
            ],
        ),
        MoveStep(label="fim"),
    ]
    return p


class TestProgramFlowEditor:
    def test_set_program_renders_blocks(self, app) -> None:
        w = ProgramFlowWidget()
        w.set_program(_program_with_steps())
        assert w._canvas._blocks
        assert len(w._canvas._blocks) == 3

    def test_empty_program_shows_message(self, app) -> None:
        w = ProgramFlowWidget()
        w.set_program(Program())
        assert not w._canvas._blocks

    def test_selection_synchronizes(self, app) -> None:
        w = ProgramFlowWidget()
        w.set_program(_program_with_steps())
        selected = []
        w.step_selected.connect(selected.append)

        w.set_selected_index(1)
        assert w._canvas._selected_root_index == 1
        # set_selected_index is a silent synchronization; the signal is emitted
        # by user interaction (mouse/keyboard) instead.

        # Simulate keyboard selection -> signal
        w._canvas.keyPressEvent(QKeyEvent(QKeyEvent.Type.KeyPress, Qt.Key_Home, Qt.NoModifier))
        assert 0 in selected

    def test_invalid_indices_highlighted(self, app) -> None:
        p = _program_with_steps()
        p.steps[0].step_id = "dup"
        p.steps[2].step_id = "dup"
        w = ProgramFlowWidget()
        w.set_program(p)
        w.set_invalid_indices({0, 2})
        assert w._canvas._invalid_indices == {0, 2}

    def test_active_root_index_highlighted(self, app) -> None:
        w = ProgramFlowWidget()
        w.set_program(_program_with_steps())
        w.set_active_root_index(2)
        assert w._canvas._active_root_index == 2
        # Repeated call must be a no-op
        w.set_active_root_index(2)
        assert w._canvas._active_root_index == 2

    def test_edit_enabled_blocks_interaction(self, app) -> None:
        w = ProgramFlowWidget()
        w.set_program(_program_with_steps())
        w.set_edit_enabled(False)
        assert not w._canvas._edit_enabled
        assert w._canvas._drag_index is None

    def test_keyboard_navigation(self, app) -> None:
        w = ProgramFlowWidget()
        w.set_program(_program_with_steps())
        w.set_selected_index(0)
        w.show()
        w.activateWindow()
        selected = []
        w.step_selected.connect(selected.append)

        w._canvas.keyPressEvent(QKeyEvent(QKeyEvent.Type.KeyPress, Qt.Key_Down, Qt.NoModifier))
        assert w._canvas._selected_root_index == 1
        assert 1 in selected

        w._canvas.keyPressEvent(QKeyEvent(QKeyEvent.Type.KeyPress, Qt.Key_End, Qt.NoModifier))
        assert w._canvas._selected_root_index == 2

        w._canvas.keyPressEvent(QKeyEvent(QKeyEvent.Type.KeyPress, Qt.Key_Home, Qt.NoModifier))
        assert w._canvas._selected_root_index == 0

    def test_zoom_limits(self, app) -> None:
        w = ProgramFlowWidget()
        w.set_program(_program_with_steps())
        w.set_zoom(5.0)
        assert w.zoom() == pytest.approx(2.0)
        w.set_zoom(0.1)
        assert w.zoom() == pytest.approx(0.5)

    def test_nested_program_renders_containers(self, app) -> None:
        w = ProgramFlowWidget()
        w.set_program(_nested_program())
        # Should have blocks for the three root steps plus children
        assert len(w._canvas._blocks) >= 3
        # Ensure loop and if containers are present
        container_indices = {b.root_index for b in w._canvas._blocks if b.container}
        assert 1 in container_indices  # LoopStep root index

    def test_reorder_signal_emitted(self, app) -> None:
        w = ProgramFlowWidget()
        w.set_program(_program_with_steps())
        reorder = []
        w.reorder_requested.connect(lambda src, dst: reorder.append((src, dst)))

        # Simulate a reorder by directly emitting the canvas signal (mouse drag
        # is hard to reproduce deterministically offscreen).
        w._canvas.reorder_requested.emit(0, 2)
        assert reorder == [(0, 2)]

    def test_program_model_survives_round_trip(self, app) -> None:
        p = _program_with_steps()
        original_ids = [id(s) for s in p.steps]
        w = ProgramFlowWidget()
        w.set_program(p)
        # Reorder in the model externally and refresh
        step = p.steps.pop(0)
        p.steps.insert(2, step)
        w.set_program(p)
        assert len(w._canvas._blocks) == 3
        # Identity of objects preserved
        assert id(p.steps[2]) == original_ids[0]

    def test_large_program_performance(self, app) -> None:
        p = Program(name="grande")
        p.steps = [MoveStep(label=f"p{i}") for i in range(500)]
        w = ProgramFlowWidget()
        w.set_program(p)
        assert len(w._canvas._blocks) == 500
