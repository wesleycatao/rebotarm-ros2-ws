"""Reusable step editor for the motion-planning tab.

This module factors out two widgets that used to live inside ``MotionTab``:

* :class:`StepEditorStack` - the per-kind parameter editors (one page per
  ``KIND_*``), including the nested step editors for ``LOOP`` and ``IF-ELSE``.
* :class:`StepListEditor` - a list of steps with add/remove/move/duplicate
  controls plus an embedded :class:`StepEditorStack`.

Both are reused at the top level (the program steps) and inside the ``LOOP``
and ``IF-ELSE`` editor pages (their child step lists), so editing nested
control flow works exactly like editing the top-level program.
"""

from __future__ import annotations

import copy
from typing import List, Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QKeySequence, QShortcut
from rebotarm_gui.widgets.combo_box import RetractingComboBox
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMenu,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from .layout_utils import fit_button_text
from .pose_editor import PoseEditor
from ..motion import (
    COMBINER_ALL,
    COMBINER_LABELS_PT,
    EDGE_LABELS_PT,
    KIND_ARC,
    KIND_GRIPPER,
    KIND_IF,
    KIND_LABELS_PT,
    KIND_LINE,
    KIND_LOOP,
    KIND_MENU_ORDER,
    KIND_MOVE,
    KIND_OUTPUT,
    KIND_PALLET,
    KIND_WAIT,
    KIND_WAIT_SIG,
    KIND_WAIT_SM,
    OUTPUT_ACTION_LABELS_PT,
    SRC_SIGNAL,
    SRC_STATE_MACHINE,
    SRC_VISION_DETECTED,
    SOURCE_LABELS_PT,
    VALID_STATE_MACHINES,
    Condition,
    ConditionExpr,
    Step,
    make_step,
    quat_to_rpy,
    rpy_to_quat,
)
from ..motion.conditions import (
    OPS_NUMERIC,
    OPS_STRING,
    SOURCE_NEEDS_AXIS,
    SOURCE_NEEDS_IO_ID,
    SOURCE_NEEDS_JOINT,
)
from ..ros_bridge import make_pose




# ----------------------------------------------------------------------
# small helpers
# ----------------------------------------------------------------------
def _dict_to_pose(d: dict):
    qx, qy, qz, qw = rpy_to_quat(float(d["roll"]), float(d["pitch"]), float(d["yaw"]))
    return make_pose(d["x"], d["y"], d["z"], qx, qy, qz, qw)


def _pose_msg_to_dict(msg) -> Optional[dict]:
    if msg is None:
        return None
    try:
        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        qw = float(msg.orientation.w)
        roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)
        return {
            "x": float(msg.position.x),
            "y": float(msg.position.y),
            "z": float(msg.position.z),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }
    except Exception:
        return None


def _wrap_scroll(content: QWidget) -> QScrollArea:
    scroll = QScrollArea()
    scroll.setWidgetResizable(True)
    scroll.setFrameShape(QScrollArea.NoFrame)
    scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
    scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
    scroll.setWidget(content)
    return scroll


def _dspin(minimum, maximum, value, suffix="", step=0.005, width=120) -> QDoubleSpinBox:
    sp = QDoubleSpinBox()
    sp.setRange(minimum, maximum)
    sp.setDecimals(3)
    sp.setSingleStep(step)
    sp.setValue(value)
    if suffix:
        sp.setSuffix(suffix)
    sp.setAlignment(Qt.AlignRight)
    sp.setMinimumWidth(width)
    return sp


def _ispin(minimum, maximum, value, width=100) -> QSpinBox:
    sp = QSpinBox()
    sp.setRange(minimum, maximum)
    sp.setValue(value)
    sp.setAlignment(Qt.AlignRight)
    sp.setMinimumWidth(width)
    return sp


# ======================================================================
# Condition editor
# ======================================================================
class ConditionRow(QWidget):
    """A single ``<source> <op> <value>`` row."""

    removed = Signal(object)

    def __init__(self, cond: Condition, joint_names_cb, input_ids_cb=None,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._joint_names_cb = joint_names_cb
        self._input_ids_cb = input_ids_cb or (lambda: [f"IN{i}" for i in range(1, 6)])
        self._loading = True

        self.src = RetractingComboBox()
        for key, label in SOURCE_LABELS_PT.items():
            self.src.addItem(label, key)
        self.param = QWidget()  # container swapped per source
        self.param_lay = QHBoxLayout(self.param)
        self.param_lay.setContentsMargins(0, 0, 0, 0)
        self.op = RetractingComboBox()
        self.value_w = QWidget()
        self.value_lay = QHBoxLayout(self.value_w)
        self.value_lay.setContentsMargins(0, 0, 0, 0)

        self.btn_remove = QPushButton("×")
        self.btn_remove.setFixedWidth(28)
        self.btn_remove.setToolTip("Remover condição")
        self.btn_remove.clicked.connect(lambda: self.removed.emit(self))

        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        lay.addWidget(self.src)
        lay.addWidget(self.param, 1)
        lay.addWidget(self.op)
        lay.addWidget(self.value_w, 1)
        lay.addWidget(self.btn_remove)

        self.src.currentIndexChanged.connect(self._on_source_changed)
        self.op.currentIndexChanged.connect(self._emit_changed)
        self.load(cond)
        self._loading = False
        self._on_source_changed()  # sync param/value widgets

    # ---- public ----
    def load(self, cond: Condition) -> None:
        self._loading = True
        try:
            idx = self.src.findData(cond.source)
            if idx >= 0:
                self.src.setCurrentIndex(idx)
            self._rebuild_param(cond)
            self._rebuild_op(cond)
            self._rebuild_value(cond)
        finally:
            self._loading = False

    def to_condition(self) -> Condition:
        source = self.src.currentData()
        params = self._collect_params(source)
        op = self.op.currentData() or "=="
        value, vtype = self._collect_value(source)
        return Condition(source=source, params=params, op=op, value=value, value_type=vtype)

    changed = Signal()

    # ---- internals ----
    def _clear(self, layout: QHBoxLayout) -> None:
        while layout.count():
            item = layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()

    def _on_source_changed(self, *_args) -> None:
        if self._loading:
            return
        cond = self.to_condition()
        self._rebuild_param(cond)
        self._rebuild_op(cond)
        self._rebuild_value(cond)
        self._emit_changed()

    def _rebuild_param(self, cond: Condition) -> None:
        self._clear(self.param_lay)
        source = self.src.currentData()
        if source in SOURCE_NEEDS_IO_ID:
            cb = RetractingComboBox()
            cb.setEditable(True)
            for n in (self._input_ids_cb() or []):
                cb.addItem(n)
            cur = str(cond.params.get("io_id", ""))
            if cur:
                cb.setCurrentText(cur)
            cb.currentTextChanged.connect(self._emit_changed)
            self.param_lay.addWidget(QLabel("Entrada"))
            self.param_lay.addWidget(cb)
            self._param_io = cb
        elif source in SOURCE_NEEDS_JOINT:
            cb = RetractingComboBox()
            cb.setEditable(True)
            for n in (self._joint_names_cb() or []):
                cb.addItem(n)
            cur = str(cond.params.get("joint", ""))
            if cur:
                cb.setCurrentText(cur)
            cb.currentTextChanged.connect(self._emit_changed)
            self.param_lay.addWidget(QLabel("Junta"))
            self.param_lay.addWidget(cb)
            self._param_joint = cb
        elif source in SOURCE_NEEDS_AXIS:
            cb = RetractingComboBox()
            cb.addItems(["x", "y", "z"])
            cb.setCurrentText(str(cond.params.get("axis", "x")))
            cb.currentTextChanged.connect(self._emit_changed)
            self.param_lay.addWidget(QLabel("Eixo"))
            self.param_lay.addWidget(cb)
            self._param_axis = cb

    def _rebuild_op(self, cond: Condition) -> None:
        self.op.blockSignals(True)
        self.op.clear()
        source = self.src.currentData()
        allowed = OPS_STRING if source in (SRC_STATE_MACHINE,) else OPS_NUMERIC
        for o in allowed:
            self.op.addItem(o, o)
        idx = self.op.findData(cond.op)
        self.op.setCurrentIndex(idx if idx >= 0 else 0)
        self.op.blockSignals(False)

    def _rebuild_value(self, cond: Condition) -> None:
        self._clear(self.value_lay)
        source = self.src.currentData()
        if source == SRC_STATE_MACHINE:
            cb = RetractingComboBox()
            for s in VALID_STATE_MACHINES:
                cb.addItem(s, s)
            idx = cb.findData(str(cond.value))
            cb.setCurrentIndex(idx if idx >= 0 else 0)
            cb.currentIndexChanged.connect(self._emit_changed)
            self.value_lay.addWidget(cb)
            self._value_combo = cb
        elif source in (SRC_SIGNAL, SRC_VISION_DETECTED):
            cb = RetractingComboBox()
            cb.addItem("0 (falso)", 0)
            cb.addItem("1 (verdadeiro)", 1)
            cb.setCurrentIndex(1 if int(bool(cond.value)) else 0)
            cb.currentIndexChanged.connect(self._emit_changed)
            self.value_lay.addWidget(cb)
            self._value_combo = cb
        else:
            sp = _dspin(-1000.0, 1000.0, float(cond.value or 0.0), "", step=0.01)
            sp.valueChanged.connect(self._emit_changed)
            self.value_lay.addWidget(sp)
            self._value_spin = sp

    def _collect_params(self, source: str) -> dict:
        if source in SOURCE_NEEDS_IO_ID:
            return {"io_id": self._param_io.currentText().strip()}
        if source in SOURCE_NEEDS_JOINT:
            return {"joint": self._param_joint.currentText().strip()}
        if source in SOURCE_NEEDS_AXIS:
            return {"axis": self._param_axis.currentText().strip()}
        return {}

    def _collect_value(self, source: str):
        if source == SRC_STATE_MACHINE:
            vtype = "string"
            return self._value_combo.currentData(), vtype
        if source in (SRC_SIGNAL, SRC_VISION_DETECTED):
            return int(self._value_combo.currentData()), "bool"
        return float(self._value_spin.value()), "number"

    def _emit_changed(self, *_args) -> None:
        if not self._loading:
            self.changed.emit()


class ConditionExprEditor(QWidget):
    """Edit a :class:`ConditionExpr` (combiner + list of rows)."""

    changed = Signal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._rows: List[ConditionRow] = []

        self.combiner = RetractingComboBox()
        for key, label in COMBINER_LABELS_PT.items():
            self.combiner.addItem(label, key)
        self.combiner.currentIndexChanged.connect(self._emit_changed)

        self.btn_add = QPushButton("+ condição")
        self.btn_add.clicked.connect(lambda: self.add_condition(Condition()))

        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        top.addWidget(QLabel("Combinar:"))
        top.addWidget(self.combiner)
        top.addWidget(self.btn_add)
        top.addStretch(1)

        self.rows_box = QVBoxLayout()
        self.rows_box.setContentsMargins(0, 0, 0, 0)
        self.rows_box.setSpacing(4)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        lay.addLayout(top)
        lay.addLayout(self.rows_box)
        lay.addStretch(1)

    def joint_names_cb(self):
        return self._joint_names_cb if hasattr(self, "_joint_names_cb") else (lambda: [])

    def set_joint_names_cb(self, cb) -> None:
        self._joint_names_cb = cb

    def input_ids_cb(self):
        return self._input_ids_cb if hasattr(self, "_input_ids_cb") else (lambda: [])

    def set_input_ids_cb(self, cb) -> None:
        self._input_ids_cb = cb

    def load(self, expr: ConditionExpr) -> None:
        for r in list(self._rows):
            r.setParent(None)
            r.deleteLater()
        self._rows = []
        self.combiner.setCurrentIndex(0 if expr.combiner == COMBINER_ALL else 1)
        for c in expr.items:
            self.add_condition(c, emit=False)

    def add_condition(self, cond: Condition, emit: bool = True) -> ConditionRow:
        row = ConditionRow(cond, self.joint_names_cb, self.input_ids_cb())
        row.changed.connect(self._emit_changed)
        row.removed.connect(self._remove_row)
        self._rows.append(row)
        self.rows_box.addWidget(row)
        if emit:
            self._emit_changed()
        return row

    def _remove_row(self, row: ConditionRow) -> None:
        if row in self._rows:
            self._rows.remove(row)
            row.setParent(None)
            row.deleteLater()
            self._emit_changed()

    def to_expr(self) -> ConditionExpr:
        return ConditionExpr(
            items=[r.to_condition() for r in self._rows],
            combiner=self.combiner.currentData() or COMBINER_ALL,
        )

    def _emit_changed(self, *_args) -> None:
        self.changed.emit()


# ======================================================================
# Step editor stack (one page per kind)
# ======================================================================
class StepEditorStack(QWidget):
    """Holds a stacked widget with a parameter page for every step kind."""

    changed = Signal()
    about_to_change = Signal()
    request_current_pose_for = Signal(object)  # emits the PoseEditor to fill

    def __init__(self, bridge, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._loading = False
        self._current: Optional[Step] = None
        self._joint_names_cb = lambda: []
        self._input_ids_cb = lambda: list(getattr(bridge, "dio_input_ids", lambda: [])())

        self._lbl_kind = QLabel("")
        self._lbl_kind.setObjectName("FieldLabel")
        self._label_edit = QLineEdit()
        self._label_edit.setPlaceholderText("Rótulo (opcional)")
        self._label_edit.textChanged.connect(self._on_label_changed)

        self._stack = QStackedWidget()
        self._pages: dict[str, int] = {}
        for builder, kind in (
            (self._build_move_page, KIND_MOVE),
            (self._build_gripper_page, KIND_GRIPPER),
            (self._build_wait_page, KIND_WAIT),
            (self._build_wait_sm_page, KIND_WAIT_SM),
            (self._build_wait_sig_page, KIND_WAIT_SIG),
            (self._build_output_page, KIND_OUTPUT),
            (self._build_line_page, KIND_LINE),
            (self._build_arc_page, KIND_ARC),
            (self._build_pallet_page, KIND_PALLET),
            (self._build_loop_page, KIND_LOOP),
            (self._build_if_page, KIND_IF),
        ):
            self._pages[kind] = self._stack.addWidget(_wrap_scroll(builder()))
        top = QFormLayout()
        top.setHorizontalSpacing(10)
        top.setVerticalSpacing(8)
        top.addRow("Tipo:", self._lbl_kind)
        top.addRow("Rótulo:", self._label_edit)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(10)
        lay.addLayout(top)
        lay.addWidget(self._stack, 1)

        # Keep the I/O id dropdowns current as the DioBridge reports points.
        if hasattr(bridge, "dio_ios_changed"):
            bridge.dio_ios_changed.connect(lambda _ios: self._refresh_io_combos())

    def _refresh_io_combos(self) -> None:
        inputs = list(self._input_ids_cb() or [])
        outputs = list(getattr(self._bridge, "dio_output_ids", lambda: [])())
        for combo, ids in (
            (getattr(self, "_wait_sig_id", None), inputs),
            (getattr(self, "_output_id", None), outputs),
        ):
            if combo is None:
                continue
            try:
                cur = combo.currentText()
            except RuntimeError:
                # Underlying C++ object may already be destroyed during teardown.
                continue
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(ids)
            if cur:
                combo.setCurrentText(cur)
            elif ids:
                combo.setCurrentIndex(0)
            combo.blockSignals(False)

    # ---- public API ----
    def set_joint_names_cb(self, cb) -> None:
        self._joint_names_cb = cb
        self._if_cond_editor.set_joint_names_cb(cb)
        for ed in (self._loop_editor, self._if_then_editor, self._if_else_editor):
            if ed is not None:
                ed.set_joint_names_cb(cb)

    def set_input_ids_cb(self, cb) -> None:
        self._input_ids_cb = cb
        self._if_cond_editor.set_input_ids_cb(cb)
        for ed in (self._loop_editor, self._if_then_editor, self._if_else_editor):
            if ed is not None:
                ed.set_input_ids_cb(cb)
        self._refresh_io_combos()

    def load_from(self, step: Optional[Step]) -> None:
        self._current = step
        enabled = step is not None
        self._stack.setEnabled(enabled)
        self._label_edit.setEnabled(enabled)
        if step is None:
            self._lbl_kind.setText("--")
            self._label_edit.blockSignals(True)
            self._label_edit.clear()
            self._label_edit.blockSignals(False)
            return

        self._loading = True
        try:
            self._lbl_kind.setText(KIND_LABELS_PT.get(step.kind, step.kind))
            self._label_edit.blockSignals(True)
            self._label_edit.setText(step.label or "")
            self._label_edit.blockSignals(False)
            self._stack.setCurrentIndex(self._pages.get(step.kind, 0))
            self._load_page_from_step(step)
        finally:
            self._loading = False

    def apply_to(self, step: Step) -> None:
        """Write the current widget values back into ``step``."""
        if self._loading or step is None:
            return
        k = step.kind
        if k == KIND_MOVE:
            step.pose = self._move_pose.values()
            step.duration = float(self._move_duration.value())
        elif k == KIND_GRIPPER:
            step.position_mm = float(self._gripper_position.value())
            step.max_effort = float(self._gripper_effort.value())
            step.timeout = float(self._gripper_timeout.value())
        elif k == KIND_WAIT:
            step.time = float(self._wait_time.value())
        elif k == KIND_WAIT_SM:
            step.state = self._wait_sm_state.currentText()
            step.timeout = float(self._wait_sm_timeout.value())
        elif k == KIND_WAIT_SIG:
            step.io_id = self._wait_sig_id.currentText().strip()
            step.value = int(self._wait_sig_value.currentData())
            step.edge = self._wait_sig_edge.currentData()
            step.timeout = float(self._wait_sig_timeout.value())
        elif k == KIND_OUTPUT:
            step.io_id = self._output_id.currentText().strip()
            step.action = self._output_action.currentData() or "on"
        elif k == KIND_LINE:
            step.start = self._line_start.values()
            step.end = self._line_end.values()
            step.speed = float(self._line_speed.value())
            step.orientation_from = "start" if self._line_ori.currentIndex() == 0 else "end"
        elif k == KIND_ARC:
            step.p1 = self._arc_p1.values()
            step.p2 = self._arc_p2.values()
            step.p3 = self._arc_p3.values()
            step.speed = float(self._arc_speed.value())
            step.samples = int(self._arc_samples.value())
            step.point_to_center = self._arc_center.isChecked()
        elif k == KIND_PALLET:
            step.corner = self._pal_corner.values()
            step.nx = int(self._pal_nx.value())
            step.ny = int(self._pal_ny.value())
            step.nz = int(self._pal_nz.value())
            step.dx = float(self._pal_dx.value())
            step.dy = float(self._pal_dy.value())
            step.dz = float(self._pal_dz.value())
            step.speed = float(self._pal_speed.value())
            step.approach_height = float(self._pal_approach.value())
            step.snake = self._pal_snake.isChecked()
        elif k == KIND_LOOP:
            step.count = -1 if self._loop_forever.isChecked() else int(self._loop_count.value())
            # child steps are edited in place by the nested StepListEditor
        elif k == KIND_IF:
            step.condition = self._if_cond_editor.to_expr()
            # then/else steps edited in place by nested StepListEditors

    def set_enabled(self, enabled: bool) -> None:
        self._stack.setEnabled(enabled and self._current is not None)
        self._label_edit.setEnabled(enabled and self._current is not None)

    # ---- per-kind page builders ----
    def _pose_editor(self, title: str, columns: int = 2) -> PoseEditor:
        ed = PoseEditor(title, show_load_current=True, columns=columns)
        ed.changed.connect(self._on_widget_changed)
        ed.request_current_pose.connect(self._load_current_pose_into_active)
        return ed

    def _build_move_page(self) -> QWidget:
        page = QWidget()
        self._move_pose = self._pose_editor("Pose alvo")
        self._move_duration = _dspin(0.1, 120.0, 2.0, " s", step=0.1)
        self._move_duration.valueChanged.connect(self._on_widget_changed)
        form = QFormLayout()
        form.addRow("Duração", self._move_duration)
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        lay.addWidget(self._move_pose)
        lay.addLayout(form)
        lay.addStretch(1)
        return page

    def _build_gripper_page(self) -> QWidget:
        page = QWidget()
        self._gripper_position = _dspin(-1000.0, 1000.0, 0.0, " mm", step=0.1)
        self._gripper_effort = _dspin(0.0, 100.0, 0.0, "", step=0.5)
        self._gripper_timeout = _dspin(0.0, 30.0, 3.0, " s", step=0.1)
        self._gripper_position.valueChanged.connect(self._on_widget_changed)
        self._gripper_effort.valueChanged.connect(self._on_widget_changed)
        self._gripper_timeout.valueChanged.connect(self._on_widget_changed)
        form = QFormLayout()
        form.addRow("Abertura/posição", self._gripper_position)
        form.addRow("Esforço máximo", self._gripper_effort)
        form.addRow("Timeout", self._gripper_timeout)
        hint = QLabel("Usa a action de comando do gripper. A unidade esperada pelo driver deve ser compatível com mm.")
        hint.setObjectName("MutedLabel")
        hint.setWordWrap(True)
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        lay.addLayout(form)
        lay.addWidget(hint)
        lay.addStretch(1)
        return page

    def _build_wait_page(self) -> QWidget:
        page = QWidget()
        self._wait_time = _dspin(0.0, 600.0, 1.0, " s", step=0.1)
        self._wait_time.valueChanged.connect(self._on_widget_changed)
        form = QFormLayout()
        form.addRow("Tempo", self._wait_time)
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        lay.addLayout(form)
        lay.addStretch(1)
        return page

    def _build_wait_sm_page(self) -> QWidget:
        page = QWidget()
        self._wait_sm_state = RetractingComboBox()
        for s in VALID_STATE_MACHINES:
            self._wait_sm_state.addItem(s)
        self._wait_sm_state.setCurrentText("IDLE")
        self._wait_sm_timeout = _dspin(0.0, 3600.0, 0.0, " s", step=1.0)
        self._wait_sm_state.currentIndexChanged.connect(self._on_widget_changed)
        self._wait_sm_timeout.valueChanged.connect(self._on_widget_changed)
        form = QFormLayout()
        form.addRow("Estado alvo", self._wait_sm_state)
        form.addRow("Timeout (0 = infinito)", self._wait_sm_timeout)
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        lay.addLayout(form)
        lay.addStretch(1)
        return page

    def _build_wait_sig_page(self) -> QWidget:
        page = QWidget()
        self._wait_sig_id = RetractingComboBox()
        self._wait_sig_id.setEditable(True)
        self._wait_sig_id.addItems([f"IN{i}" for i in range(1, 6)])
        self._wait_sig_id.setCurrentText("IN1")
        self._wait_sig_value = RetractingComboBox()
        self._wait_sig_value.addItem("0 (baixo)", 0)
        self._wait_sig_value.addItem("1 (alto)", 1)
        self._wait_sig_value.setCurrentIndex(1)
        self._wait_sig_edge = RetractingComboBox()
        for key, label in EDGE_LABELS_PT.items():
            self._wait_sig_edge.addItem(label, key)
        self._wait_sig_timeout = _dspin(0.0, 3600.0, 0.0, " s", step=1.0)
        self._wait_sig_id.currentTextChanged.connect(self._on_widget_changed)
        self._wait_sig_timeout.valueChanged.connect(self._on_widget_changed)
        self._wait_sig_value.currentIndexChanged.connect(self._on_widget_changed)
        self._wait_sig_edge.currentIndexChanged.connect(self._on_widget_changed)
        form = QFormLayout()
        form.addRow("Entrada digital", self._wait_sig_id)
        form.addRow("Valor esperado", self._wait_sig_value)
        form.addRow("Tipo de espera", self._wait_sig_edge)
        form.addRow("Timeout (0 = infinito)", self._wait_sig_timeout)
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        lay.addLayout(form)
        lay.addStretch(1)
        return page

    def _build_output_page(self) -> QWidget:
        page = QWidget()
        self._output_id = RetractingComboBox()
        self._output_id.setEditable(True)
        self._output_id.addItems([f"OUT{i}" for i in range(1, 6)])
        self._output_id.setCurrentText("OUT1")
        self._output_action = RetractingComboBox()
        for key, label in OUTPUT_ACTION_LABELS_PT.items():
            self._output_action.addItem(label, key)
        self._output_id.currentTextChanged.connect(self._on_widget_changed)
        self._output_action.currentIndexChanged.connect(self._on_widget_changed)
        form = QFormLayout()
        form.addRow("Saída digital", self._output_id)
        form.addRow("Ação", self._output_action)
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        lay.addLayout(form)
        lay.addStretch(1)
        return page

    def _build_line_page(self) -> QWidget:
        page = QWidget()
        self._line_start = self._pose_editor("Pose inicial")
        self._line_end = self._pose_editor("Pose final")
        self._line_speed = _dspin(0.005, 1.0, 0.10, " m/s", step=0.005)
        self._line_ori = RetractingComboBox()
        self._line_ori.addItems(["Pose inicial", "Pose final"])
        self._line_speed.valueChanged.connect(self._on_widget_changed)
        self._line_ori.currentIndexChanged.connect(self._on_widget_changed)
        form = QFormLayout()
        form.setHorizontalSpacing(14)
        form.addRow("Velocidade", self._line_speed)
        form.addRow("Orientação", self._line_ori)
        lays = QHBoxLayout()
        lays.setSpacing(10)
        lays.addWidget(self._line_start, 1)
        lays.addWidget(self._line_end, 1)
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        lay.addLayout(lays)
        lay.addLayout(form)
        lay.addStretch(1)
        return page

    def _build_arc_page(self) -> QWidget:
        page = QWidget()
        self._arc_p1 = self._pose_editor("P1", columns=1)
        self._arc_p2 = self._pose_editor("P2", columns=1)
        self._arc_p3 = self._pose_editor("P3", columns=1)
        self._arc_speed = _dspin(0.005, 1.0, 0.08, " m/s", step=0.005)
        self._arc_samples = _ispin(2, 400, 24)
        self._arc_center = QCheckBox("Orientação apontando para o centro (soldagem)")
        for w in (self._arc_speed, self._arc_samples):
            w.valueChanged.connect(self._on_widget_changed)
        self._arc_center.toggled.connect(self._on_widget_changed)
        form = QFormLayout()
        form.setHorizontalSpacing(14)
        form.addRow("Velocidade", self._arc_speed)
        form.addRow("Amostras", self._arc_samples)
        lays = QHBoxLayout()
        lays.setSpacing(10)
        lays.addWidget(self._arc_p1, 1)
        lays.addWidget(self._arc_p2, 1)
        lays.addWidget(self._arc_p3, 1)
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        lay.addLayout(lays)
        lay.addLayout(form)
        lay.addWidget(self._arc_center)
        lay.addStretch(1)
        return page

    def _build_pallet_page(self) -> QWidget:
        page = QWidget()
        self._pal_corner = self._pose_editor("Canto de origem")
        self._pal_nx = _ispin(1, 50, 3)
        self._pal_ny = _ispin(1, 50, 3)
        self._pal_nz = _ispin(1, 20, 1)
        self._pal_dx = _dspin(0.0, 0.5, 0.05, " m")
        self._pal_dy = _dspin(0.0, 0.5, 0.05, " m")
        self._pal_dz = _dspin(0.0, 0.5, 0.04, " m")
        self._pal_speed = _dspin(0.005, 1.0, 0.12, " m/s", step=0.005)
        self._pal_approach = _dspin(0.0, 0.3, 0.05, " m")
        self._pal_snake = QCheckBox("Padrão serpenteado (snake)")
        self._pal_snake.setChecked(True)
        self._pal_corner.changed.connect(self._on_widget_changed)
        for w in (self._pal_nx, self._pal_ny, self._pal_nz,
                  self._pal_dx, self._pal_dy, self._pal_dz,
                  self._pal_speed, self._pal_approach):
            w.valueChanged.connect(self._on_widget_changed)
        self._pal_snake.toggled.connect(self._on_widget_changed)
        form = QFormLayout()
        form.setHorizontalSpacing(14)
        form.addRow("Colunas (nx)", self._pal_nx)
        form.addRow("Linhas (ny)", self._pal_ny)
        form.addRow("Camadas (nz)", self._pal_nz)
        form.addRow("Espaço X (dx)", self._pal_dx)
        form.addRow("Espaço Y (dy)", self._pal_dy)
        form.addRow("Espaço Z (dz)", self._pal_dz)
        form.addRow("Velocidade", self._pal_speed)
        form.addRow("Altura de aproximação", self._pal_approach)
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        lay.addWidget(self._pal_corner)
        lay.addLayout(form)
        lay.addWidget(self._pal_snake)
        lay.addStretch(1)
        return page

    def _build_loop_page(self) -> QWidget:
        page = QWidget()
        self._loop_count = _ispin(1, 100000, 3)
        self._loop_forever = QCheckBox("Repetir para sempre")
        self._loop_forever.toggled.connect(self._on_loop_forever_toggled)
        self._loop_count.valueChanged.connect(self._on_widget_changed)
        # The nested step editor is created lazily (see _ensure_loop_editor) to
        # avoid infinite recursion: a StepListEditor contains a StepEditorStack
        # which would otherwise build a LOOP/IF page -> another StepListEditor.
        self._loop_container = QWidget()
        QVBoxLayout(self._loop_container).setContentsMargins(0, 0, 0, 0)
        self._loop_editor: Optional[StepListEditor] = None

        top = QFormLayout()
        top.addRow("Repetições", self._loop_count)
        top.addRow("", self._loop_forever)

        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        lay.addLayout(top)
        lay.addWidget(self._loop_container, 1)
        return page

    def _ensure_loop_editor(self) -> "StepListEditor":
        if self._loop_editor is None:
            self._loop_editor = StepListEditor(self._bridge, compact=True)
            self._loop_editor.set_joint_names_cb(self._joint_names_cb)
            self._loop_editor.about_to_change.connect(self.about_to_change.emit)
            self._loop_editor.changed.connect(self._on_widget_changed)
            self._loop_container.layout().addWidget(self._loop_editor)
        return self._loop_editor

    def _build_if_page(self) -> QWidget:
        page = QWidget()
        self._if_cond_editor = ConditionExprEditor()
        self._if_cond_editor.changed.connect(self._on_widget_changed)
        self._if_then_container = QWidget()
        QVBoxLayout(self._if_then_container).setContentsMargins(0, 0, 0, 0)
        self._if_else_container = QWidget()
        QVBoxLayout(self._if_else_container).setContentsMargins(0, 0, 0, 0)
        self._if_then_editor: Optional[StepListEditor] = None
        self._if_else_editor: Optional[StepListEditor] = None

        self._if_empty_warning = QLabel(
            "⚠ Condição vazia: este bloco IF será sempre executado."
        )
        self._if_empty_warning.setObjectName("WarningLabel")
        self._if_empty_warning.setWordWrap(True)
        self._if_empty_warning.setStyleSheet("color: #f0a500;")

        cond_box = QGroupBox("Condição")
        cond_lay = QVBoxLayout(cond_box)
        cond_lay.addWidget(self._if_cond_editor)
        cond_lay.addWidget(self._if_empty_warning)
        then_box = QGroupBox("Então (THEN)")
        QVBoxLayout(then_box).addWidget(self._if_then_container)
        else_box = QGroupBox("Senão (ELSE)")
        QVBoxLayout(else_box).addWidget(self._if_else_container)

        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        lay.addWidget(cond_box)
        lay.addWidget(then_box, 1)
        lay.addWidget(else_box, 1)
        return page

    def _ensure_then_editor(self) -> "StepListEditor":
        if self._if_then_editor is None:
            self._if_then_editor = StepListEditor(self._bridge, compact=True)
            self._if_then_editor.set_joint_names_cb(self._joint_names_cb)
            self._if_then_editor.about_to_change.connect(self.about_to_change.emit)
            self._if_then_editor.changed.connect(self._on_widget_changed)
            self._if_then_container.layout().addWidget(self._if_then_editor)
        return self._if_then_editor

    def _ensure_else_editor(self) -> "StepListEditor":
        if self._if_else_editor is None:
            self._if_else_editor = StepListEditor(self._bridge, compact=True)
            self._if_else_editor.set_joint_names_cb(self._joint_names_cb)
            self._if_else_editor.about_to_change.connect(self.about_to_change.emit)
            self._if_else_editor.changed.connect(self._on_widget_changed)
            self._if_else_container.layout().addWidget(self._if_else_editor)
        return self._if_else_editor

    # ---- load page values from step ----
    def _load_page_from_step(self, step: Step) -> None:
        k = step.kind
        if k == KIND_MOVE:
            self._move_pose.set_values(step.pose)
            self._move_duration.setValue(step.duration)
        elif k == KIND_GRIPPER:
            self._gripper_position.setValue(float(getattr(step, "position_mm", 0.0)))
            self._gripper_effort.setValue(float(getattr(step, "max_effort", 0.0)))
            self._gripper_timeout.setValue(float(getattr(step, "timeout", 3.0)))
        elif k == KIND_WAIT:
            self._wait_time.setValue(step.time)
        elif k == KIND_WAIT_SM:
            self._wait_sm_state.setCurrentText(step.state)
            self._wait_sm_timeout.setValue(step.timeout)
        elif k == KIND_WAIT_SIG:
            self._wait_sig_id.setCurrentText(str(getattr(step, "io_id", "IN1") or "IN1"))
            idx = self._wait_sig_value.findData(int(bool(step.value)))
            self._wait_sig_value.setCurrentIndex(idx if idx >= 0 else 1)
            eidx = self._wait_sig_edge.findData(step.edge)
            self._wait_sig_edge.setCurrentIndex(eidx if eidx >= 0 else 0)
            self._wait_sig_timeout.setValue(step.timeout)
        elif k == KIND_OUTPUT:
            self._output_id.setCurrentText(str(getattr(step, "io_id", "OUT1") or "OUT1"))
            aidx = self._output_action.findData(getattr(step, "action", "on"))
            self._output_action.setCurrentIndex(aidx if aidx >= 0 else 0)
        elif k == KIND_LINE:
            self._line_start.set_values(step.start)
            self._line_end.set_values(step.end)
            self._line_speed.setValue(step.speed)
            self._line_ori.setCurrentIndex(0 if step.orientation_from == "start" else 1)
        elif k == KIND_ARC:
            self._arc_p1.set_values(step.p1)
            self._arc_p2.set_values(step.p2)
            self._arc_p3.set_values(step.p3)
            self._arc_speed.setValue(step.speed)
            self._arc_samples.setValue(step.samples)
            self._arc_center.setChecked(step.point_to_center)
        elif k == KIND_PALLET:
            self._pal_corner.set_values(step.corner)
            self._pal_nx.setValue(step.nx)
            self._pal_ny.setValue(step.ny)
            self._pal_nz.setValue(step.nz)
            self._pal_dx.setValue(step.dx)
            self._pal_dy.setValue(step.dy)
            self._pal_dz.setValue(step.dz)
            self._pal_speed.setValue(step.speed)
            self._pal_approach.setValue(step.approach_height)
            self._pal_snake.setChecked(step.snake)
        elif k == KIND_LOOP:
            forever = step.count <= 0
            self._loop_forever.setChecked(forever)
            self._loop_count.setValue(0 if forever else step.count)
            self._loop_count.setEnabled(not forever)
            self._ensure_loop_editor().set_steps(step.steps)
        elif k == KIND_IF:
            self._if_cond_editor.load(step.condition)
            self._ensure_then_editor().set_steps(step.then_steps)
            self._ensure_else_editor().set_steps(step.else_steps)
            self._update_if_empty_warning()

    # ---- change handlers ----
    def _on_widget_changed(self, *_args) -> None:
        if self._loading or self._current is None:
            return
        self.apply_to(self._current)
        if self._current is not None and self._current.kind == KIND_IF:
            self._update_if_empty_warning()
        self.changed.emit()

    def _on_label_changed(self, text: str) -> None:
        if self._loading or self._current is None:
            return
        self._current.label = text
        self.changed.emit()

    def _on_loop_forever_toggled(self, checked: bool) -> None:
        self._loop_count.setEnabled(not checked)
        self._on_widget_changed()

    def _update_if_empty_warning(self) -> None:
        if self._if_empty_warning is None:
            return
        empty = self._if_cond_editor.to_expr().is_empty()
        self._if_empty_warning.setVisible(empty)

    # ---- current-pose capture ----
    def _load_current_pose_into_active(self) -> None:
        msg = getattr(self._bridge, "last_reached_pose", None)
        d = _pose_msg_to_dict(msg)
        if d is None:
            return
        sender = self.sender()
        for ed in self._pose_editors():
            if ed is sender:
                ed.set_values(d, block_signals=False)
                return

    def _pose_editors(self):
        return (
            self._move_pose, self._line_start, self._line_end,
            self._arc_p1, self._arc_p2, self._arc_p3, self._pal_corner,
        )


# ======================================================================
# Step list editor (list + buttons + embedded stack)
# ======================================================================
class StepListEditor(QWidget):
    """Editable ordered list of steps with a parameter editor."""

    changed = Signal()           # the list contents changed
    step_changed = Signal()      # the currently selected step's params changed
    about_to_change = Signal()   # emitted before discrete list mutations (add/remove/move/duplicate)
    selection_changed = Signal(int)

    def __init__(self, bridge, parent: Optional[QWidget] = None, compact: bool = False) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._steps: List[Step] = []
        self._compact = compact

        self._list = QListWidget()
        self._list.setMinimumWidth(180 if compact else 220)
        self._list.setMaximumWidth(360 if compact else 320)
        self._list.setDragDropMode(QAbstractItemView.InternalMove)
        self._list.setDefaultDropAction(Qt.MoveAction)
        self._list.setDragEnabled(True)
        self._list.setAcceptDrops(True)
        self._list.setDropIndicatorShown(True)
        self._list.setToolTip(
            "Arraste para reordenar. Ctrl+D duplica; Delete ou Backspace remove."
        )
        self._list.currentRowChanged.connect(self._on_selection_changed)
        model = self._list.model()
        model.rowsAboutToBeMoved.connect(lambda *_args: self.about_to_change.emit())
        model.rowsMoved.connect(self._on_rows_moved)

        self._shortcut_duplicate = QShortcut(QKeySequence("Ctrl+D"), self._list)
        self._shortcut_duplicate.setContext(Qt.WidgetWithChildrenShortcut)
        self._shortcut_duplicate.activated.connect(self._duplicate_step)
        self._shortcut_remove = QShortcut(QKeySequence("Delete"), self._list)
        self._shortcut_remove.setContext(Qt.WidgetWithChildrenShortcut)
        self._shortcut_remove.activated.connect(self._remove_step)
        self._shortcut_remove_alt = QShortcut(QKeySequence("Backspace"), self._list)
        self._shortcut_remove_alt.setContext(Qt.WidgetWithChildrenShortcut)
        self._shortcut_remove_alt.activated.connect(self._remove_step)

        self._highlight_brush = None
        self._highlighted_row: Optional[int] = None

        self._btn_add = QPushButton("Adicionar")
        self._add_menu = QMenu(self)
        for kind in KIND_MENU_ORDER:
            self._add_menu.addAction(
                KIND_LABELS_PT[kind], lambda _=False, k=kind: self._add_step(k)
            )
        self._btn_add.setMenu(self._add_menu)

        self._btn_remove = QPushButton("Remover")
        self._btn_remove.setToolTip("Remove a etapa selecionada (Delete ou Backspace).")
        self._btn_remove.clicked.connect(self._remove_step)
        self._btn_up = QPushButton("▲")
        self._btn_up.setToolTip("Mover para cima")
        self._btn_up.clicked.connect(lambda: self._move_step(-1))
        self._btn_down = QPushButton("▼")
        self._btn_down.setToolTip("Mover para baixo")
        self._btn_down.clicked.connect(lambda: self._move_step(1))
        self._btn_dup = QPushButton("Duplicar")
        self._btn_dup.setToolTip("Duplica a etapa selecionada (Ctrl+D).")
        self._btn_dup.clicked.connect(self._duplicate_step)
        for b in (self._btn_add, self._btn_dup, self._btn_remove):
            b.setMinimumWidth(90 if compact else 96)
            fit_button_text(b)

        up_down = QHBoxLayout()
        up_down.setSpacing(6)
        up_down.addWidget(self._btn_up)
        up_down.addWidget(self._btn_down)

        btn_col = QVBoxLayout()
        btn_col.setSpacing(6)
        btn_col.addWidget(self._btn_add)
        btn_col.addWidget(self._btn_dup)
        btn_col.addWidget(self._btn_remove)
        btn_col.addLayout(up_down)
        btn_col.addStretch(1)

        list_row = QHBoxLayout()
        list_row.setSpacing(8)
        list_row.addWidget(self._list, 1)
        list_row.addLayout(btn_col, 0)

        self._stack = StepEditorStack(self._bridge)
        self._stack.about_to_change.connect(self.about_to_change.emit)
        self._stack.changed.connect(self._on_step_edited)

        left_box = QGroupBox("Etapas")
        left_box.setLayout(self._box_layout(list_row))
        left_box.setMaximumWidth(420 if compact else 440)

        editor_box = QGroupBox("Parâmetros da etapa")
        er = QVBoxLayout(editor_box)
        er.setContentsMargins(10, 14, 10, 10)
        er.addWidget(self._stack)

        main = QHBoxLayout(self)
        main.setContentsMargins(0, 0, 0, 0)
        main.setSpacing(10)
        main.addWidget(left_box, 0)
        main.addWidget(editor_box, 1)

    def _box_layout(self, list_row) -> QVBoxLayout:
        lay = QVBoxLayout()
        lay.setContentsMargins(10, 14, 10, 10)
        lay.setSpacing(8)
        lay.addLayout(list_row, 1)
        hint = QLabel(
            "Arraste para reordenar · Ctrl+D duplica · Delete remove"
            if not self._compact
            else "Arraste · Ctrl+D · Delete"
        )
        hint.setObjectName("MutedLabel")
        hint.setWordWrap(True)
        lay.addWidget(hint)
        return lay

    # ---- public API ----
    def set_steps(self, steps: List[Step]) -> None:
        self._steps = steps
        self._refresh()

    def steps(self) -> List[Step]:
        return self._steps

    def selected_index(self) -> int:
        row = self._list.currentRow()
        return row if 0 <= row < len(self._steps) else -1

    def set_joint_names_cb(self, cb) -> None:
        self._stack.set_joint_names_cb(cb)

    def set_input_ids_cb(self, cb) -> None:
        self._stack.set_input_ids_cb(cb)

    def set_edit_enabled(self, enabled: bool) -> None:
        for b in (self._btn_add, self._btn_remove, self._btn_dup,
                  self._btn_up, self._btn_down, self._list):
            b.setEnabled(enabled)
        self._stack.set_enabled(enabled)

    def refresh(self) -> None:
        self._refresh()

    def highlight_row(self, index: int | None) -> None:
        """Highlight a row without changing the current selection/editor."""
        self.clear_highlight()
        if index is None:
            return
        row = int(index)
        if not (0 <= row < self._list.count()):
            return
        item = self._list.item(row)
        if item is None:
            return
        self._highlighted_row = row
        self._highlight_brush = item.background()
        item.setBackground(Qt.yellow)
        item.setForeground(Qt.black)
        self._list.scrollToItem(item)

    def clear_highlight(self) -> None:
        """Remove any active row highlight."""
        if self._highlighted_row is None:
            return
        if 0 <= self._highlighted_row < self._list.count():
            item = self._list.item(self._highlighted_row)
            if item is not None and self._highlight_brush is not None:
                item.setBackground(self._highlight_brush)
                item.setForeground(self._list.palette().text())
        self._highlighted_row = None
        self._highlight_brush = None

    # ---- internal ----
    def _refresh(self, keep_row: Optional[int] = None) -> None:
        self._list.blockSignals(True)
        self._list.clear()
        for idx, step in enumerate(self._steps):
            self._list.addItem(QListWidgetItem(f"{idx + 1}. {step.short_description()}"))
        row = keep_row if keep_row is not None else (self._list.count() - 1)
        if 0 <= row < self._list.count():
            self._list.setCurrentRow(row)
        self._list.blockSignals(False)
        self._on_selection_changed(self._list.currentRow())

    def _current_step(self) -> Optional[Step]:
        row = self._list.currentRow()
        if 0 <= row < len(self._steps):
            return self._steps[row]
        return None

    def _on_selection_changed(self, row: int) -> None:
        self._stack.load_from(self._current_step())
        self.selection_changed.emit(self.selected_index())

    def _on_rows_moved(
        self,
        _source_parent,
        source_start: int,
        source_end: int,
        _destination_parent,
        destination_row: int,
    ) -> None:
        """Mirror an internal QListWidget drag into the program data model."""
        count = source_end - source_start + 1
        moved = self._steps[source_start : source_end + 1]
        del self._steps[source_start : source_end + 1]
        insert_at = destination_row - count if destination_row > source_end else destination_row
        insert_at = max(0, min(insert_at, len(self._steps)))
        self._steps[insert_at:insert_at] = moved
        for row, step in enumerate(self._steps):
            item = self._list.item(row)
            if item is not None:
                item.setText(f"{row + 1}. {step.short_description()}")
        self._on_selection_changed(self._list.currentRow())
        self.changed.emit()

    def _on_step_edited(self) -> None:
        row = self._list.currentRow()
        if 0 <= row < len(self._steps):
            step = self._steps[row]
            self._list.item(row).setText(f"{row + 1}. {step.short_description()}")
        self.step_changed.emit()
        self.changed.emit()

    def _add_step(self, kind: str) -> None:
        self.about_to_change.emit()
        step = make_step(kind)
        self._steps.append(step)
        self._refresh(keep_row=len(self._steps) - 1)
        self.changed.emit()

    def _remove_step(self) -> None:
        row = self._list.currentRow()
        if row < 0:
            return
        self.about_to_change.emit()
        self._steps.pop(row)
        self._refresh(keep_row=min(row, max(0, len(self._steps) - 1)))
        self.changed.emit()

    def _move_step(self, delta: int) -> None:
        row = self._list.currentRow()
        if row < 0:
            return
        new_row = row + delta
        if not (0 <= new_row < len(self._steps)):
            return
        self.about_to_change.emit()
        step = self._steps.pop(row)
        self._steps.insert(new_row, step)
        self._refresh(keep_row=new_row)
        self.changed.emit()

    def _duplicate_step(self) -> None:
        step = self._current_step()
        if step is None:
            return
        row = self._list.currentRow()
        self.about_to_change.emit()
        clone = copy.deepcopy(step)
        self._steps.insert(row + 1, clone)
        self._refresh(keep_row=row + 1)
        self.changed.emit()
