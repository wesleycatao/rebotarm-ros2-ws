"""System control tab: enable/disable/home/gravity/set_zero."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..widgets.status_led import StatusLed


class SystemTab(QWidget):
    """Tab with system-level commands and status summary."""

    def __init__(self, bridge, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._bridge = bridge

        # ---- Status panel ----
        self._led_state = StatusLed("off", 18)
        self._lbl_state = QLabel("desconhecido")
        self._lbl_mode = QLabel("--")
        self._lbl_model = QLabel("--")
        self._lbl_rate = QLabel("--")
        self._lbl_channel = QLabel("--")
        self._lbl_control_mode = QLabel("--")
        self._last_status_stamp: float | None = None
        self._rate_hz: float | None = None

        for label in (
            self._lbl_state,
            self._lbl_mode,
            self._lbl_model,
            self._lbl_rate,
            self._lbl_channel,
            self._lbl_control_mode,
        ):
            label.setObjectName("MetricValue")
            label.setMinimumHeight(44)
        self._lbl_state.setObjectName("StatusValue")

        status_box = QGroupBox("Status do braço")
        grid = QGridLayout(status_box)
        grid.setHorizontalSpacing(18)
        grid.setVerticalSpacing(14)
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 0)
        grid.setColumnStretch(2, 1)
        grid.addWidget(self._field_label("Estado"), 0, 0)
        grid.addWidget(self._led_state, 0, 1)
        grid.addWidget(self._lbl_state, 0, 2)
        grid.addWidget(self._field_label("Modo"), 1, 0)
        grid.addWidget(self._lbl_mode, 1, 1, 1, 2)
        grid.addWidget(self._field_label("Modelo"), 2, 0)
        grid.addWidget(self._lbl_model, 2, 1, 1, 2)
        grid.addWidget(self._field_label("Rate (Hz)"), 3, 0)
        grid.addWidget(self._lbl_rate, 3, 1, 1, 2)
        grid.addWidget(self._field_label("Canal"), 4, 0)
        grid.addWidget(self._lbl_channel, 4, 1, 1, 2)
        grid.addWidget(self._field_label("Control mode"), 5, 0)
        grid.addWidget(self._lbl_control_mode, 5, 1, 1, 2)

        # ---- Command buttons ----
        self._btn_enable = QPushButton("Enable")
        self._btn_disable = QPushButton("Disable")
        self._btn_home = QPushButton("Safe Home")
        self._btn_grav_on = QPushButton("Gravity ON")
        self._btn_grav_off = QPushButton("Gravity OFF")
        self._btn_zero_all = QPushButton("Set Zero (all)")
        self._btn_zero_sel = QPushButton("Set Zero (joint...)")
        self._btn_enable.setObjectName("PrimaryButton")
        self._btn_disable.setObjectName("DangerButton")
        self._btn_home.setObjectName("WarningButton")

        self._btn_enable.clicked.connect(self._bridge.enable)
        self._btn_disable.clicked.connect(self._bridge.disable)
        self._btn_home.clicked.connect(self._bridge.safe_home)
        self._btn_grav_on.clicked.connect(self._bridge.gravity_start)
        self._btn_grav_off.clicked.connect(self._bridge.gravity_stop)
        self._btn_zero_all.clicked.connect(self._bridge.set_zero)
        self._btn_zero_sel.clicked.connect(self._zero_selected)

        for btn in (
            self._btn_enable,
            self._btn_disable,
            self._btn_home,
            self._btn_grav_on,
            self._btn_grav_off,
            self._btn_zero_all,
            self._btn_zero_sel,
        ):
            btn.setMinimumHeight(56)

        cmds_box = QGroupBox("Comandos")
        cmds_grid = QGridLayout(cmds_box)
        cmds_grid.setHorizontalSpacing(12)
        cmds_grid.setVerticalSpacing(12)
        cmds_grid.addWidget(self._btn_enable, 0, 0)
        cmds_grid.addWidget(self._btn_disable, 0, 1)
        cmds_grid.addWidget(self._btn_home, 1, 0)
        cmds_grid.addWidget(self._btn_zero_all, 1, 1)
        cmds_grid.addWidget(self._btn_grav_on, 2, 0)
        cmds_grid.addWidget(self._btn_grav_off, 2, 1)
        cmds_grid.addWidget(self._btn_zero_sel, 3, 0, 1, 2)

        lay = QHBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(18)
        lay.addWidget(status_box, 1)
        lay.addWidget(cmds_box, 1)

    # ------------------------------------------------------------------
    def _field_label(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("FieldLabel")
        label.setMinimumHeight(44)
        return label

    def update_arm_status(self, status) -> None:
        """Refresh the status panel from ArmStatus msg."""
        enabled = bool(getattr(status, "enabled", False))
        mode = str(getattr(status, "mode", "--"))
        model = str(getattr(status, "model", "reBotArm"))
        rate = getattr(status, "rate_hz", getattr(status, "rate", None))
        channel = str(getattr(status, "channel", f"/{self._bridge.namespace}"))
        control_mode = str(getattr(status, "control_mode", mode))

        stamp = getattr(getattr(status, "header", None), "stamp", None)
        stamp_sec = None
        if stamp is not None:
            stamp_sec = float(stamp.sec) + (float(stamp.nanosec) * 1e-9)

        if stamp_sec is not None and stamp_sec != self._last_status_stamp:
            if self._last_status_stamp is not None:
                dt = stamp_sec - self._last_status_stamp
                if 0.001 <= dt <= 10.0:
                    instant_rate = 1.0 / dt
                    self._rate_hz = (
                        instant_rate
                        if self._rate_hz is None
                        else (self._rate_hz * 0.8) + (instant_rate * 0.2)
                    )
            self._last_status_stamp = stamp_sec

        self._led_state.set_color("green" if enabled else "amber")
        self._lbl_state.setText("enabled" if enabled else "disabled")
        self._lbl_mode.setText(mode)
        self._lbl_model.setText(model)
        if rate is not None:
            self._lbl_rate.setText(f"{float(rate):.1f}")
        elif self._rate_hz is not None:
            self._lbl_rate.setText(f"{self._rate_hz:.1f}")
        else:
            self._lbl_rate.setText("--")
        self._lbl_channel.setText(channel)
        self._lbl_control_mode.setText(control_mode)

    def _zero_selected(self) -> None:
        node = self._bridge.node
        if node is None:
            return
        names = node.arm_joint_names()
        if not names:
            QMessageBox.warning(self, "Set Zero", "Sem juntas disponíveis.")
            return
        from PySide6.QtWidgets import QInputDialog

        choice, ok = QInputDialog.getItem(
            self, "Set Zero", "Escolha a junta:", names, 0, False
        )
        if ok and choice:
            self._bridge.set_zero(choice)
