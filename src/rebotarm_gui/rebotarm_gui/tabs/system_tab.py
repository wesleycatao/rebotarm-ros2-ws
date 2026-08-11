"""System control tab: enable/disable/home/gravity/set_zero."""

from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QWidget,
)

from ..widgets.layout_utils import fit_button_text
from ..widgets.status_led import StatusLed


class SystemTab(QWidget):
    """Tab with system-level commands and status summary."""

    request_safe_home = Signal()
    request_disable = Signal()

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
        self._lbl_watchdog = QLabel("--")
        self._lbl_control_jitter = QLabel("--")
        self._lbl_safety_jitter = QLabel("--")
        self._lbl_lost_cycles = QLabel("--")
        self._last_status_stamp: float | None = None
        self._rate_hz: float | None = None

        for label in (
            self._lbl_state,
            self._lbl_mode,
            self._lbl_model,
            self._lbl_rate,
            self._lbl_channel,
            self._lbl_control_mode,
            self._lbl_watchdog,
            self._lbl_control_jitter,
            self._lbl_safety_jitter,
            self._lbl_lost_cycles,
        ):
            label.setObjectName("MetricValue")
            label.setMinimumHeight(30)
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
        grid.addWidget(self._field_label("Modo de controle"), 5, 0)
        grid.addWidget(self._lbl_control_mode, 5, 1, 1, 2)
        grid.addWidget(self._field_label("Watchdog temporal"), 6, 0)
        grid.addWidget(self._lbl_watchdog, 6, 1, 1, 2)
        grid.addWidget(self._field_label("Jitter controle p99/máx."), 7, 0)
        grid.addWidget(self._lbl_control_jitter, 7, 1, 1, 2)
        grid.addWidget(self._field_label("Jitter segurança p99/máx."), 8, 0)
        grid.addWidget(self._lbl_safety_jitter, 8, 1, 1, 2)
        grid.addWidget(self._field_label("Ciclos perdidos ctrl/seg."), 9, 0)
        grid.addWidget(self._lbl_lost_cycles, 9, 1, 1, 2)

        # ---- Command buttons ----
        self._btn_enable = QPushButton("Ativar")
        self._btn_disable = QPushButton("Desabilitar")
        self._btn_home = QPushButton("Home segura")
        self._btn_grav_on = QPushButton("Gravidade ON")
        self._btn_grav_off = QPushButton("Gravidade OFF")
        self._btn_zero_all = QPushButton("Zerar todas")
        self._btn_zero_sel = QPushButton("Zerar junta...")
        self._btn_enable.setObjectName("PrimaryButton")
        self._btn_disable.setObjectName("DangerButton")
        self._btn_home.setObjectName("WarningButton")

        self._btn_enable.clicked.connect(self._bridge.enable)
        self._btn_disable.clicked.connect(self.request_disable.emit)
        self._btn_home.clicked.connect(self.request_safe_home.emit)
        self._btn_grav_on.clicked.connect(self._bridge.gravity_start)
        self._btn_grav_off.clicked.connect(self._bridge.gravity_stop)
        self._btn_zero_all.clicked.connect(self._confirm_zero_all)
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
            btn.setMinimumHeight(40)
            fit_button_text(btn)
            btn.setEnabled(False)

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
        label.setMinimumHeight(30)
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
        self._lbl_state.setText("ativado" if enabled else "desativado")
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

    def set_robot_state(self, alive: bool, enabled: bool) -> None:
        """Gate every command using fresh heartbeat and enable state."""
        self._btn_enable.setEnabled(bool(alive and not enabled))
        operational = bool(alive and enabled)
        for button in (
            self._btn_disable,
            self._btn_home,
            self._btn_grav_on,
            self._btn_grav_off,
            self._btn_zero_all,
            self._btn_zero_sel,
        ):
            button.setEnabled(operational)
        if not alive:
            self._led_state.set_color("off")
            self._lbl_state.setText("sem sinal")

    def update_timing_status(self, status) -> None:
        """Display bounded-window jitter and lifetime missed-cycle counters."""
        enabled = bool(getattr(status, "watchdog_enabled", False))
        self._lbl_watchdog.setText("ativo" if enabled else "desativado")
        self._lbl_control_jitter.setText(
            f"{float(getattr(status, 'control_p99_jitter_ms', 0.0)):.3f} / "
            f"{float(getattr(status, 'control_max_jitter_ms', 0.0)):.3f} ms"
        )
        self._lbl_safety_jitter.setText(
            f"{float(getattr(status, 'safety_p99_jitter_ms', 0.0)):.3f} / "
            f"{float(getattr(status, 'safety_max_jitter_ms', 0.0)):.3f} ms"
        )
        self._lbl_lost_cycles.setText(
            f"{int(getattr(status, 'control_lost_cycles', 0))} / "
            f"{int(getattr(status, 'safety_lost_cycles', 0))}"
        )

    def _confirm_zero_all(self) -> None:
        answer = QMessageBox.question(
            self,
            "Zerar todas as juntas",
            "Confirma redefinir como zero a posição atual de todas as juntas?\n\n"
            "Esta operação altera a referência do robô e exige validação mecânica.",
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer == QMessageBox.Yes:
            self._bridge.set_zero()

    def _zero_selected(self) -> None:
        node = self._bridge.node
        if node is None:
            return
        names = node.arm_joint_names()
        if not names:
            QMessageBox.warning(self, "Zerar junta", "Sem juntas disponíveis.")
            return
        from PySide6.QtWidgets import QInputDialog

        choice, ok = QInputDialog.getItem(
            self, "Zerar junta", "Escolha a junta:", names, 0, False
        )
        if ok and choice:
            answer = QMessageBox.question(
                self,
                "Confirmar zero da junta",
                f"Confirma redefinir a posição atual de {choice} como zero?",
                QMessageBox.Yes | QMessageBox.Cancel,
                QMessageBox.Cancel,
            )
            if answer == QMessageBox.Yes:
                self._bridge.set_zero(choice)
