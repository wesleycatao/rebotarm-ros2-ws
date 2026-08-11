"""Joints tab: per-joint sliders, press-and-hold jog and trajectory execution."""

from __future__ import annotations

import math
from typing import Dict

from PySide6.QtCore import QEvent, Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from ..widgets.joint_slider import JointSlider
from ..widgets.layout_utils import fit_button_text, shrink_spinbox

_JOG_INTERVAL_MS = 120
_JOG_DURATION_S = 0.25


class JointsTab(QWidget):
    """Tab to command arm joints via FollowJointTrajectory action."""

    def __init__(self, bridge, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._sliders: Dict[str, JointSlider] = {}
        self._limits: Dict[str, tuple[float, float]] = {}
        self._robot_ready = False

        # ---- Jog state (press-and-hold) ----
        self._jog_timer = QTimer(self)
        self._jog_timer.setInterval(_JOG_INTERVAL_MS)
        self._jog_timer.timeout.connect(self._jog_step)
        self._bridge.action_result.connect(self._on_action_result)
        self._jog_joint: str | None = None
        self._jog_direction = 0
        self._jog_in_flight = False

        # ---- Keyboard jog (opt-in, tab-scoped) ----
        self._kbd_jog_enabled = False
        self._kbd_active_idx = 0
        self._kbd_held: tuple[str, int] | None = None

        # ---- Sliders container ----
        self._sliders_host = QWidget()
        self._sliders_lay = QVBoxLayout(self._sliders_host)
        self._sliders_lay.setContentsMargins(8, 8, 8, 8)
        self._sliders_lay.setSpacing(12)
        self._sliders_lay.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self._sliders_host)

        # ---- Jog step selector ----
        self._jog_step_combo = QComboBox()
        for label, deg in (("1°", 1.0), ("5°", 5.0), ("10°", 10.0)):
            self._jog_step_combo.addItem(label, deg)
        self._jog_step_combo.setCurrentIndex(1)
        self._jog_step_combo.setMinimumWidth(90)
        self._jog_step_combo.setAccessibleName("Passo do jog de junta")

        # ---- Duration + buttons ----
        self._duration = QDoubleSpinBox()
        self._duration.setRange(0.1, 60.0)
        self._duration.setValue(2.0)
        self._duration.setSuffix(" s")
        self._duration.setSingleStep(0.5)
        self._duration.setAlignment(Qt.AlignRight)
        self._duration.setMinimumWidth(110)
        self._duration.setMaximumWidth(150)
        self._duration.setAccessibleName("Duração da trajetória")
        shrink_spinbox(self._duration)

        self._btn_apply = QPushButton("Mover (trajetória)")
        self._btn_apply.setObjectName("PrimaryButton")
        self._btn_apply.setMinimumHeight(40)
        self._btn_apply.clicked.connect(self._send)
        self._btn_apply.setEnabled(False)

        self._btn_reset = QPushButton("Alvos = posição atual")
        self._btn_reset.setMinimumHeight(40)
        self._btn_reset.clicked.connect(self._reset_to_current)
        fit_button_text(self._btn_apply)
        fit_button_text(self._btn_reset)

        ctrl_box = QGroupBox("Execução")
        ctrl_lay = QHBoxLayout(ctrl_box)
        ctrl_lay.setSpacing(14)
        ctrl_lay.addWidget(QLabel("Passo de jog"))
        ctrl_lay.addWidget(self._jog_step_combo)
        ctrl_lay.addSpacing(10)
        ctrl_lay.addWidget(QLabel("Duração"))
        ctrl_lay.addWidget(self._duration)
        ctrl_lay.addWidget(self._btn_reset)
        ctrl_lay.addSpacing(10)
        self._chk_kbd_jog = QCheckBox("Jog por teclado")
        self._chk_kbd_jog.setAccessibleName("Ativar jog por teclado")
        self._chk_kbd_jog.setToolTip(
            "Ativa o jog pelas setas do teclado (apenas nesta aba):\n"
            "↑/↓ move a junta ativa · ←/→ troca a junta ativa.\n"
            "Antes de usar, dê foco a um controle que não seja campo de texto\n"
            "(ex.: clique numa das barras). Não há atalho global de movimento."
        )
        self._chk_kbd_jog.toggled.connect(self._on_kbd_jog_toggled)
        ctrl_lay.addWidget(self._chk_kbd_jog)
        self._lbl_active_joint = QLabel("--")
        self._lbl_active_joint.setObjectName("MutedLabel")
        self._lbl_active_joint.setAccessibleName("Junta ativa do jog por teclado")
        ctrl_lay.addWidget(self._lbl_active_joint)
        ctrl_lay.addStretch(1)
        ctrl_lay.addWidget(self._btn_apply)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(14)
        lay.addWidget(scroll, 1)
        lay.addWidget(ctrl_box)

        # Keyboard jog is an application-scope filter that only acts while this
        # tab is focused and the opt-in checkbox is on, so it never interferes
        # with typing or with the other tabs.
        app = QApplication.instance()
        if app is not None:
            app.installEventFilter(self)

    # ------------------------------------------------------------------
    def update_joint_state(self, msg) -> None:
        """Rebuild sliders if joint set changed and refresh current values."""
        arm_names = [n for n in msg.name if not n.startswith("gripper")]
        if not arm_names:
            return

        # Build sliders on first sight of joints
        if not self._sliders:
            self._build_sliders(arm_names, msg)

        # Update current values
        for name, pos in zip(msg.name, msg.position):
            if name in self._sliders:
                self._sliders[name].set_current(float(pos))

    def apply_joint_limits(self) -> None:
        """Apply URDF limits to existing sliders (called when they arrive)."""
        limits = self._bridge.joint_limits()
        if not limits:
            return
        self._limits.update(limits)
        for name, slider in self._sliders.items():
            pair = limits.get(name)
            if pair is not None:
                slider.set_range(pair[0], pair[1])

    def _build_sliders(self, arm_names: list[str], msg) -> None:
        positions = {n: p for n, p in zip(msg.name, msg.position)}
        limits = self._bridge.joint_limits()
        if limits:
            self._limits.update(limits)
        insert_at = self._sliders_lay.count() - 1  # before the stretch
        for idx, name in enumerate(arm_names, start=1):
            lower, upper = self._limits.get(name, (-3.14, 3.14))
            slider = JointSlider(
                joint_name=name,
                index=idx,
                minimum=lower,
                maximum=upper,
            )
            cur = positions.get(name)
            if cur is not None:
                slider.set_value(float(cur))
                slider.set_current(float(cur))
            slider.jog_pressed.connect(self._on_jog_pressed)
            slider.jog_released.connect(self._on_jog_released)
            slider.set_command_enabled(self._robot_ready)
            self._sliders[name] = slider
            self._sliders_lay.insertWidget(insert_at, slider)
            insert_at += 1
        self._kbd_active_idx = min(self._kbd_active_idx, max(0, len(self._sliders) - 1))
        self._update_kbd_active_label()

    # ------------------------------------------------------------------
    # press-and-hold jog
    # ------------------------------------------------------------------
    def _on_jog_pressed(self, joint_name: str, direction: int) -> None:
        if not self._robot_ready:
            return
        self._jog_joint = joint_name
        self._jog_direction = direction
        self._jog_in_flight = False
        self._jog_step()
        self._jog_timer.start()

    def _on_jog_released(self, joint_name: str, direction: int) -> None:
        if joint_name == self._jog_joint and direction == self._jog_direction:
            self._jog_timer.stop()
            self._jog_joint = None
            self._jog_direction = 0

    def _jog_step(self) -> None:
        if (
            not self._robot_ready
            or self._jog_joint is None
            or self._jog_direction == 0
            or self._jog_in_flight
        ):
            return
        if self._bridge.node is None:
            return
        slider = self._sliders.get(self._jog_joint)
        if slider is None:
            return
        step = math.radians(float(self._jog_step_combo.currentData()))
        target = slider.value() + self._jog_direction * step
        lower, upper = slider.limits()
        target = max(lower, min(upper, target))
        slider.set_value(target)
        self._jog_in_flight = self._bridge.follow_joint_trajectory(
            [self._jog_joint], [target], _JOG_DURATION_S
        )

    def _on_action_result(self, name: str, _success: bool, _message: str) -> None:
        if name == "follow_joint_trajectory":
            self._jog_in_flight = False

    # ------------------------------------------------------------------
    # keyboard jog (opt-in, scoped to this tab while it has focus)
    # ------------------------------------------------------------------
    def _on_kbd_jog_toggled(self, enabled: bool) -> None:
        self._kbd_jog_enabled = bool(enabled)
        if not enabled:
            self._stop_kbd_jog()

    def _kbd_active_joint(self) -> str | None:
        names = list(self._sliders)
        if not names:
            return None
        self._kbd_active_idx = max(0, min(self._kbd_active_idx, len(names) - 1))
        return names[self._kbd_active_idx]

    def _update_kbd_active_label(self) -> None:
        name = self._kbd_active_joint()
        self._lbl_active_joint.setText(f"Junta ativa: {name}" if name else "--")

    def _stop_kbd_jog(self) -> None:
        self._kbd_held = None
        if self._jog_joint is not None:
            self._on_jog_released(self._jog_joint, self._jog_direction)

    @staticmethod
    def _focus_is_editable(focus: QWidget) -> bool:
        from PySide6.QtWidgets import (
            QComboBox,
            QDoubleSpinBox,
            QLineEdit,
            QPlainTextEdit,
            QSpinBox,
            QTextEdit,
        )

        return isinstance(
            focus,
            (
                QLineEdit,
                QSpinBox,
                QDoubleSpinBox,
                QComboBox,
                QTextEdit,
                QPlainTextEdit,
            ),
        )

    def eventFilter(self, obj, event) -> bool:  # noqa: N802 (Qt naming)
        if (
            not self._kbd_jog_enabled
            or not self.isVisible()
            or event.type() not in (QEvent.KeyPress, QEvent.KeyRelease)
        ):
            return super().eventFilter(obj, event)
        focus = QApplication.focusWidget()
        if (
            focus is None
            or not self.isAncestorOf(focus)
            or self._focus_is_editable(focus)
        ):
            return super().eventFilter(obj, event)
        if int(event.key()) in (Qt.Key_Up, Qt.Key_Down, Qt.Key_Left, Qt.Key_Right):
            return self._handle_kbd_jog(event)
        return super().eventFilter(obj, event)

    def _handle_kbd_jog(self, event) -> bool:
        key = int(event.key())
        active = self._kbd_active_joint()
        if active is None:
            return False
        pressed = event.type() == QEvent.KeyPress
        if key in (Qt.Key_Left, Qt.Key_Right):
            if not pressed or event.isAutoRepeat():
                return True
            self._stop_kbd_jog()
            names = list(self._sliders)
            if names:
                step = -1 if key == Qt.Key_Left else 1
                self._kbd_active_idx = (self._kbd_active_idx + step) % len(names)
                self._update_kbd_active_label()
            return True
        direction = 1 if key == Qt.Key_Up else -1
        if pressed and not event.isAutoRepeat():
            self._kbd_held = (active, direction)
            self._on_jog_pressed(active, direction)
        elif not pressed and self._kbd_held == (active, direction):
            self._stop_kbd_jog()
        return True

    # ------------------------------------------------------------------
    def _reset_to_current(self) -> None:
        node = self._bridge.node
        if node is None or node.joint_state is None:
            return
        for name, pos in zip(node.joint_state.name, node.joint_state.position):
            if name in self._sliders:
                self._sliders[name].set_value(float(pos))

    def _send(self) -> None:
        if not self._robot_ready:
            return
        node = self._bridge.node
        if node is None:
            return
        names = node.arm_joint_names()
        if not names:
            return
        positions = []
        for name in names:
            slider = self._sliders.get(name)
            if slider is None:
                return
            positions.append(slider.value())
        self._bridge.follow_joint_trajectory(
            names, positions, float(self._duration.value())
        )

    def set_robot_ready(self, ready: bool) -> None:
        """Gate all trajectory/jog entry points using the global readiness state."""
        self._robot_ready = bool(ready)
        self._btn_apply.setEnabled(self._robot_ready)
        for slider in self._sliders.values():
            slider.set_command_enabled(self._robot_ready)
        if not self._robot_ready:
            self._jog_timer.stop()
            self._jog_joint = None
            self._jog_direction = 0
            self._jog_in_flight = False
            self._kbd_held = None
