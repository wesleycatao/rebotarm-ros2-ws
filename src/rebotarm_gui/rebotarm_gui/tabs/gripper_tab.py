"""Gripper tab: open/close/set/command."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..widgets.layout_utils import fit_button_text, shrink_spinbox
from ..widgets.status_led import StatusLed


class GripperTab(QWidget):
    """Tab to control the gripper via services and action (position in mm)."""

    def __init__(self, bridge, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._robot_ready = False

        # ---- Status ----
        self._led = StatusLed("off", 16)
        self._lbl_pos = QLabel("--")
        self._lbl_vel = QLabel("--")
        self._lbl_eff = QLabel("--")
        self._lbl_raw = QLabel("--")
        for label in (self._lbl_pos, self._lbl_vel, self._lbl_eff, self._lbl_raw):
            label.setObjectName("MetricValue")
            label.setMinimumHeight(30)

        status_box = QGroupBox("Estado do gripper")
        sform = QFormLayout(status_box)
        sform.setHorizontalSpacing(18)
        sform.setVerticalSpacing(12)
        sform.addRow("Posição", self._lbl_pos)
        sform.addRow("Velocidade", self._lbl_vel)
        sform.addRow("Esforço", self._lbl_eff)
        sform.addRow("Leitura bruta", self._lbl_raw)
        top = QHBoxLayout()
        top.setSpacing(12)
        top.addWidget(self._led)
        top.addWidget(QLabel("Gripper"))
        top.addStretch(1)

        # ---- Inputs ----
        self._position = QDoubleSpinBox()
        self._position.setRange(0.0, 85.0)
        self._position.setValue(0.0)
        self._position.setDecimals(1)
        self._position.setSingleStep(1.0)
        self._position.setSuffix(" mm")
        self._position.setAlignment(Qt.AlignRight)
        self._position.setMinimumWidth(120)
        self._position.setMaximumWidth(170)
        shrink_spinbox(self._position)

        self._timeout = QDoubleSpinBox()
        self._timeout.setRange(0.1, 30.0)
        self._timeout.setValue(3.0)
        self._timeout.setSuffix(" s")
        self._timeout.setMinimumWidth(110)
        self._timeout.setMaximumWidth(150)
        shrink_spinbox(self._timeout)

        self._max_effort = QDoubleSpinBox()
        self._max_effort.setRange(0.0, 100.0)
        self._max_effort.setValue(0.0)
        self._max_effort.setMinimumWidth(120)
        self._max_effort.setMaximumWidth(170)
        shrink_spinbox(self._max_effort)

        in_box = QGroupBox("Parâmetros")
        iform = QFormLayout(in_box)
        iform.setHorizontalSpacing(18)
        iform.setVerticalSpacing(12)
        iform.addRow("Posição alvo", self._position)
        iform.addRow("Tempo limite (abrir/fechar)", self._timeout)
        iform.addRow("Esforço máx (action)", self._max_effort)

        # ---- Buttons ----
        self._btn_open = QPushButton("Abrir")
        self._btn_close = QPushButton("Fechar")
        self._btn_set = QPushButton("Definir posição")
        self._btn_cmd = QPushButton("Comandar (action)")
        self._btn_cmd.setObjectName("PrimaryButton")
        self._btn_close.setObjectName("WarningButton")
        for b in (self._btn_open, self._btn_close, self._btn_set, self._btn_cmd):
            b.setMinimumHeight(40)
            fit_button_text(b)
            b.setEnabled(False)

        self._btn_open.clicked.connect(self._open)
        self._btn_close.clicked.connect(self._close)
        self._btn_set.clicked.connect(self._set)
        self._btn_cmd.clicked.connect(self._command)

        cmds_box = QGroupBox("Comandos")
        cgrid = QGridLayout(cmds_box)
        cgrid.setHorizontalSpacing(12)
        cgrid.setVerticalSpacing(12)
        cgrid.addWidget(self._btn_open, 0, 0)
        cgrid.addWidget(self._btn_close, 0, 1)
        cgrid.addWidget(self._btn_set, 1, 0)
        cgrid.addWidget(self._btn_cmd, 1, 1)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(14)
        lay.addLayout(top)
        lay.addWidget(status_box)
        lay.addWidget(in_box)
        lay.addWidget(cmds_box)
        lay.addStretch(1)

    # ------------------------------------------------------------------
    def update_gripper_state(self, msg) -> None:
        pos = getattr(msg, "position", None)
        vel = getattr(msg, "velocity", None)
        eff = getattr(msg, "effort", None)
        raw = getattr(msg, "raw", None)
        self._lbl_pos.setText(f"{pos:+.3f}" if pos is not None else "--")
        self._lbl_vel.setText(f"{vel:+.3f}" if vel is not None else "--")
        self._lbl_eff.setText(f"{eff:+.3f}" if eff is not None else "--")
        self._lbl_raw.setText(str(raw) if raw is not None else "--")
        self._led.set_color("green")

    # ------------------------------------------------------------------
    def _open(self) -> None:
        if not self._robot_ready:
            return
        self._bridge.gripper_open(float(self._position.value()), float(self._timeout.value()))

    def _close(self) -> None:
        if not self._robot_ready:
            return
        self._bridge.gripper_close(float(self._position.value()), float(self._timeout.value()))

    def _set(self) -> None:
        if not self._robot_ready:
            return
        self._bridge.gripper_set(float(self._position.value()))

    def _command(self) -> None:
        if not self._robot_ready:
            return
        self._bridge.gripper_command(
            float(self._position.value()), float(self._max_effort.value())
        )

    def set_robot_ready(self, ready: bool) -> None:
        """Gate gripper commands when the controller heartbeat is stale."""
        self._robot_ready = bool(ready)
        for button in (self._btn_open, self._btn_close, self._btn_set, self._btn_cmd):
            button.setEnabled(self._robot_ready)
