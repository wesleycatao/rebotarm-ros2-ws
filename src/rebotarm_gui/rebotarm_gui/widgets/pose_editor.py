"""Compact pose editor (XYZ + RPY) reused by the motion-planning tab."""

from __future__ import annotations

from typing import Dict

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from .layout_utils import shrink_spinbox


class PoseEditor(QGroupBox):
    """Group box editing a pose dict ``{x, y, z, roll, pitch, yaw}``.

    Layout is a two-column form (Posição | Orientação) so the editor stays
    short and the spinboxes can expand. Orientation values are in radians
    (consistent with :class:`PoseInput`).
    """

    changed = Signal()

    def __init__(
        self,
        title: str = "Pose",
        parent: QWidget | None = None,
        show_load_current: bool = False,
        columns: int = 2,
    ) -> None:
        super().__init__(title, parent)
        self._show_load_current = show_load_current
        self._columns = max(1, columns)

        self._x = self._spin(-1.5, 1.5, 0.30, " m")
        self._y = self._spin(-1.5, 1.5, 0.00, " m")
        self._z = self._spin(-1.5, 1.5, 0.30, " m")
        self._roll = self._spin(-3.1416, 3.1416, 0.0, " rad")
        self._pitch = self._spin(-3.1416, 3.1416, 0.0, " rad")
        self._yaw = self._spin(-3.1416, 3.1416, 0.0, " rad")
        for sp in (self._x, self._y, self._z, self._roll, self._pitch, self._yaw):
            # signal-to-signal: valueChanged(double) -> changed() (extra arg dropped)
            sp.valueChanged.connect(self.changed)

        # Build the form(s). With one column every field gets the full editor
        # width (wider spinboxes when several editors share a row, e.g. the arc
        # page). With two columns the editor stays compact (3 rows tall).
        def _form(rows):
            f = QFormLayout()
            f.setHorizontalSpacing(8)
            f.setVerticalSpacing(6)
            f.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
            for label, widget in rows:
                f.addRow(label, widget)
            return f

        cols = QHBoxLayout()
        cols.setSpacing(14)
        if self._columns >= 2:
            cols.addLayout(_form([("X", self._x), ("Y", self._y), ("Z", self._z)]), 1)
            cols.addLayout(
                _form([("Roll", self._roll), ("Pitch", self._pitch), ("Yaw", self._yaw)]), 1
            )
        else:
            cols.addLayout(
                _form([
                    ("X", self._x), ("Y", self._y), ("Z", self._z),
                    ("Roll", self._roll), ("Pitch", self._pitch), ("Yaw", self._yaw),
                ]),
                1,
            )

        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 14, 10, 10)
        lay.setSpacing(6)
        lay.addLayout(cols)
        if show_load_current:
            self._btn_current = QPushButton("Usar pose atual")
            self._btn_current.setToolTip("Preenche com o TCP atual (quando disponível)")
            self._btn_current.clicked.connect(self._emit_load_current)
            lay.addWidget(self._btn_current)

        # Minimum width so layouts never squeeze the spinboxes into overlap;
        # if the parent is narrower than this, a scroll area handles overflow.
        self.setMinimumWidth(230 if self._columns == 1 else 330)

    # ---- public API ----
    def _spin(self, minimum: float, maximum: float, value: float, suffix: str) -> QDoubleSpinBox:
        sp = QDoubleSpinBox()
        sp.setRange(minimum, maximum)
        sp.setDecimals(3)
        sp.setSingleStep(0.01)
        sp.setValue(value)
        sp.setSuffix(suffix)
        sp.setAlignment(Qt.AlignRight)
        sp.setMinimumWidth(90)
        sp.setMaximumWidth(160)
        shrink_spinbox(sp)
        return sp

    def values(self) -> Dict[str, float]:
        return {
            "x": float(self._x.value()),
            "y": float(self._y.value()),
            "z": float(self._z.value()),
            "roll": float(self._roll.value()),
            "pitch": float(self._pitch.value()),
            "yaw": float(self._yaw.value()),
        }

    def set_values(self, p: Dict[str, float], *, block_signals: bool = True) -> None:
        prev = self.blockSignals(block_signals)
        try:
            self._x.setValue(float(p.get("x", 0.0)))
            self._y.setValue(float(p.get("y", 0.0)))
            self._z.setValue(float(p.get("z", 0.0)))
            self._roll.setValue(float(p.get("roll", 0.0)))
            self._pitch.setValue(float(p.get("pitch", 0.0)))
            self._yaw.setValue(float(p.get("yaw", 0.0)))
        finally:
            self.blockSignals(prev)
        if not block_signals:
            self.changed.emit()

    request_current_pose = Signal()

    def _emit_load_current(self) -> None:
        self.request_current_pose.emit()
