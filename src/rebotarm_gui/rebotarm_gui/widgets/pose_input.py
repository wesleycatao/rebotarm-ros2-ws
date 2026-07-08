"""Cartesian pose input widget (XYZ + RPY / quaternion)."""

from __future__ import annotations

import math
from typing import Tuple

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QButtonGroup,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QRadioButton,
    QVBoxLayout,
    QWidget,
)


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert RPY (radians) into a quaternion (qx, qy, qz, qw)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class PoseInput(QWidget):
    """Input widget returning position + orientation + duration."""

    move_requested = Signal()
    ik_requested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self._x = self._make_spin(-1.0, 1.0, 0.30, " m")
        self._y = self._make_spin(-1.0, 1.0, 0.00, " m")
        self._z = self._make_spin(-1.0, 1.0, 0.30, " m")
        self._roll = self._make_spin(-3.14, 3.14, 0.0, " rad")
        self._pitch = self._make_spin(-3.14, 3.14, 3.14, " rad")
        self._yaw = self._make_spin(-3.14, 3.14, 0.0, " rad")
        self._qx = self._make_spin(-1.0, 1.0, 0.0, "")
        self._qy = self._make_spin(-1.0, 1.0, 0.0, "")
        self._qz = self._make_spin(-1.0, 1.0, 0.0, "")
        self._qw = self._make_spin(-1.0, 1.0, 1.0, "")
        self._duration = self._make_spin(0.1, 60.0, 2.0, " s")

        # Orientation mode
        self._mode_rpy = QRadioButton("Roll / Pitch / Yaw")
        self._mode_rpy.setChecked(True)
        self._mode_quat = QRadioButton("Quaternion (qx, qy, qz, qw)")
        mode_group = QButtonGroup(self)
        mode_group.addButton(self._mode_rpy)
        mode_group.addButton(self._mode_quat)
        self._mode_rpy.toggled.connect(self._update_orientation_visibility)

        # Position box
        pos_box = QGroupBox("Posição (base_link)")
        pos_form = QFormLayout(pos_box)
        pos_form.setHorizontalSpacing(18)
        pos_form.setVerticalSpacing(12)
        pos_form.addRow("X", self._x)
        pos_form.addRow("Y", self._y)
        pos_form.addRow("Z", self._z)

        # Orientation box
        self._rpy_box = QGroupBox("Orientação (RPY)")
        rpy_form = QFormLayout(self._rpy_box)
        rpy_form.setHorizontalSpacing(18)
        rpy_form.setVerticalSpacing(12)
        rpy_form.addRow("Roll", self._roll)
        rpy_form.addRow("Pitch", self._pitch)
        rpy_form.addRow("Yaw", self._yaw)

        self._quat_box = QGroupBox("Orientação (Quaternion)")
        quat_form = QFormLayout(self._quat_box)
        quat_form.setHorizontalSpacing(18)
        quat_form.setVerticalSpacing(12)
        quat_form.addRow("qx", self._qx)
        quat_form.addRow("qy", self._qy)
        quat_form.addRow("qz", self._qz)
        quat_form.addRow("qw", self._qw)

        self._dur_box = QGroupBox("Duração")
        dur_form = QFormLayout(self._dur_box)
        dur_form.setHorizontalSpacing(18)
        dur_form.setVerticalSpacing(12)
        dur_form.addRow("tempo", self._duration)

        # Layout
        mode_lay = QHBoxLayout()
        mode_lay.setSpacing(24)
        mode_lay.addWidget(self._mode_rpy)
        mode_lay.addWidget(self._mode_quat)
        mode_lay.addStretch(1)

        cols = QHBoxLayout()
        cols.setSpacing(18)
        cols.addWidget(pos_box)
        cols.addWidget(self._rpy_box)
        cols.addWidget(self._quat_box)
        cols.addWidget(self._dur_box)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)
        lay.addLayout(mode_lay)
        lay.addLayout(cols)

        self._update_orientation_visibility()

    def _make_spin(self, minimum: float, maximum: float, value: float, suffix: str) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(3)
        spin.setSingleStep(0.05)
        spin.setValue(value)
        spin.setSuffix(suffix)
        spin.setAlignment(Qt.AlignRight)
        spin.setMinimumWidth(150)
        return spin

    def _update_orientation_visibility(self) -> None:
        rpy = self._mode_rpy.isChecked()
        self._rpy_box.setVisible(rpy)
        self._quat_box.setVisible(not rpy)

    def values(self) -> Tuple[float, float, float, float, float, float, float]:
        """Return (x, y, z, qx, qy, qz, qw, duration)."""
        x = float(self._x.value())
        y = float(self._y.value())
        z = float(self._z.value())
        if self._mode_rpy.isChecked():
            qx, qy, qz, qw = _rpy_to_quat(
                float(self._roll.value()), float(self._pitch.value()), float(self._yaw.value())
            )
        else:
            qx = float(self._qx.value())
            qy = float(self._qy.value())
            qz = float(self._qz.value())
            qw = float(self._qw.value())
        duration = float(self._duration.value())
        return x, y, z, qx, qy, qz, qw, duration

    def set_current_pose_label(self, text: str) -> None:
        """Allow external code to show the current TCP pose near the inputs."""
        setattr(self, "_pose_hint", text)
