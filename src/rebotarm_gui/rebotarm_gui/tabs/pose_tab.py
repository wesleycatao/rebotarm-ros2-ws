"""Cartesian pose tab: MoveToPose action, live TCP readout and Cartesian jog."""

from __future__ import annotations

import time

from PySide6.QtCore import QEvent, Qt
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..ros_bridge import make_pose
from ..widgets.layout_utils import fit_button_text
from ..widgets.pose_input import PoseInput

_TCP_REFRESH_S = 0.2  # throttle for the live TCP label
_JOG_DURATION_S = 0.5


class PoseTab(QWidget):
    """Tab to command the end-effector to a Cartesian pose."""

    def __init__(self, bridge, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._last_tcp_refresh = 0.0
        self._robot_ready = False
        self._move_in_flight = False
        self._kbd_jog_enabled = False

        self._pose_input = PoseInput()

        self._btn_move = QPushButton("Mover para pose (action)")
        self._btn_move.setObjectName("PrimaryButton")
        self._btn_move.setMinimumHeight(40)
        self._btn_move.clicked.connect(self._send_move)
        self._btn_move.setEnabled(False)

        self._btn_ik = QPushButton("Resolver IK (service)")
        self._btn_ik.setMinimumHeight(40)
        self._btn_ik.clicked.connect(self._send_ik)
        self._btn_ik.setEnabled(False)
        fit_button_text(self._btn_move)
        fit_button_text(self._btn_ik)

        self._lbl_current = QLabel("Pose atual do TCP: --")
        self._lbl_current.setObjectName("MutedLabel")
        self._lbl_current.setAlignment(Qt.AlignCenter)

        self._progress = QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setTextVisible(True)
        self._progress.setFormat("Movimento: %p%")
        self._progress.setVisible(False)

        buttons = QHBoxLayout()
        buttons.setSpacing(12)
        buttons.addWidget(self._btn_ik)
        buttons.addWidget(self._btn_move)

        # ---- Cartesian jog ----
        self._jog_step_combo = QComboBox()
        for label, mm in (("5 mm", 0.005), ("10 mm", 0.010), ("50 mm", 0.050)):
            self._jog_step_combo.addItem(label, mm)
        self._jog_step_combo.setCurrentIndex(1)
        self._jog_step_combo.setMinimumWidth(90)
        self._jog_step_combo.setAccessibleName("Passo do jog cartesiano")

        self._jog_buttons: list[QPushButton] = []
        jog_grid = QGridLayout()
        jog_grid.setHorizontalSpacing(10)
        jog_grid.setVerticalSpacing(10)
        for row, axis in enumerate(("x", "y", "z")):
            for col, (direction, sign) in enumerate((("−", -1.0), ("+", 1.0))):
                btn = QPushButton(f"{axis.upper()}{direction}")
                btn.setMinimumHeight(40)
                btn.setToolTip(
                    f"Jog de {axis.upper()} em {'sentido negativo' if sign < 0 else 'sentido positivo'}"
                )
                btn.clicked.connect(
                    lambda _checked=False, a=axis, s=sign: self._jog(a, s)
                )
                btn.setEnabled(False)
                self._jog_buttons.append(btn)
                jog_grid.addWidget(btn, row, col)

        jog_box = QGroupBox("Jog cartesiano")
        jog_lay = QHBoxLayout(jog_box)
        jog_lay.setSpacing(14)
        jog_lay.addWidget(QLabel("Passo"))
        jog_lay.addWidget(self._jog_step_combo)
        jog_lay.addLayout(jog_grid, 1)
        self._chk_kbd_jog = QCheckBox("Jog por teclado")
        self._chk_kbd_jog.setAccessibleName("Ativar jog por teclado")
        self._chk_kbd_jog.setToolTip(
            "Ativa o jog pelas setas do teclado (apenas nesta aba):\n"
            "←/→ eixo X · ↑/↓ eixo Y · Page Up/Down eixo Z.\n"
            "Antes de usar, dê foco a um controle que não seja campo de texto.\n"
            "Não há atalho global de movimento."
        )
        self._chk_kbd_jog.toggled.connect(self._on_kbd_jog_toggled)
        jog_lay.addWidget(self._chk_kbd_jog)

        self._bridge.move_feedback.connect(self._on_move_feedback)
        self._bridge.action_result.connect(self._on_action_result)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(14)
        lay.addWidget(self._pose_input)
        lay.addWidget(jog_box)
        lay.addStretch(1)
        lay.addWidget(self._lbl_current)
        lay.addWidget(self._progress)
        lay.addLayout(buttons)

        app = QApplication.instance()
        if app is not None:
            app.installEventFilter(self)

    # ------------------------------------------------------------------
    def update_joint_state(self, _msg) -> None:
        """Refresh the live TCP label (throttled) from FK / last reached pose."""
        now = time.monotonic()
        if now - self._last_tcp_refresh < _TCP_REFRESH_S:
            return
        self._last_tcp_refresh = now
        pose = self._bridge.tcp_pose_fk() or self._bridge.current_tcp_pose()
        self._set_tcp_label(pose)

    def _set_tcp_label(self, pose) -> None:
        if pose is None:
            self._lbl_current.setText("Pose atual do TCP: --")
            for btn in self._jog_buttons:
                btn.setEnabled(False)
            return
        p = pose.position
        self._lbl_current.setText(
            f"TCP: x={p.x:+.3f}  y={p.y:+.3f}  z={p.z:+.3f} m"
        )
        for btn in self._jog_buttons:
            btn.setEnabled(self._robot_ready and not self._move_in_flight)

    def _on_move_feedback(self, progress: float, current_pose) -> None:
        self._progress.setVisible(True)
        self._progress.setValue(int(round(progress * 100)))
        if progress >= 1.0:
            self._progress.setVisible(False)
        if current_pose is not None:
            self._set_tcp_label(current_pose)

    def _jog(self, axis: str, sign: float) -> None:
        if not self._robot_ready or self._move_in_flight:
            return
        pose = self._bridge.tcp_pose_fk() or self._bridge.current_tcp_pose()
        if pose is None:
            self._lbl_current.setText("Pose atual do TCP: indisponível (sem FK/URDF)")
            return
        step = float(self._jog_step_combo.currentData()) * sign
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        if axis == "x":
            x += step
        elif axis == "y":
            y += step
        else:
            z += step
        q = pose.orientation
        self._move_in_flight = self._bridge.move_to_pose(
            make_pose(x, y, z, q.x, q.y, q.z, q.w), _JOG_DURATION_S
        )
        self.set_robot_ready(self._robot_ready)

    # ------------------------------------------------------------------
    # keyboard jog (opt-in, scoped to this tab while it has focus)
    # ------------------------------------------------------------------
    def _on_kbd_jog_toggled(self, enabled: bool) -> None:
        self._kbd_jog_enabled = bool(enabled)

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
        key = int(event.key())
        mapping = {
            Qt.Key_Left: ("x", -1.0),
            Qt.Key_Right: ("x", 1.0),
            Qt.Key_Down: ("y", -1.0),
            Qt.Key_Up: ("y", 1.0),
            Qt.Key_PageDown: ("z", -1.0),
            Qt.Key_PageUp: ("z", 1.0),
        }
        if key not in mapping:
            return super().eventFilter(obj, event)
        if event.type() == QEvent.KeyPress:
            axis, sign = mapping[key]
            self._jog(axis, sign)
        return True

    # ------------------------------------------------------------------
    def _values(self):
        return self._pose_input.values()

    def _send_move(self) -> None:
        if not self._robot_ready or self._move_in_flight:
            return
        x, y, z, qx, qy, qz, qw, duration = self._values()
        pose = make_pose(x, y, z, qx, qy, qz, qw)
        self._progress.setValue(0)
        self._progress.setVisible(True)
        self._move_in_flight = self._bridge.move_to_pose(pose, duration)
        self.set_robot_ready(self._robot_ready)
        if not self._move_in_flight:
            self._progress.setVisible(False)

    def _send_ik(self) -> None:
        if not self._robot_ready:
            return
        x, y, z, qx, qy, qz, qw, _duration = self._values()
        pose = make_pose(x, y, z, qx, qy, qz, qw)
        self._bridge.move_to_pose_ik(pose)

    def _on_action_result(self, name: str, _success: bool, _message: str) -> None:
        if name == "move_to_pose":
            self._move_in_flight = False
            self._progress.setVisible(False)
            self.set_robot_ready(self._robot_ready)

    def set_robot_ready(self, ready: bool) -> None:
        """Gate Cartesian commands while preserving the live TCP readout."""
        self._robot_ready = bool(ready)
        self._btn_move.setEnabled(self._robot_ready and not self._move_in_flight)
        self._btn_ik.setEnabled(self._robot_ready)
        pose_available = (
            self._bridge.tcp_pose_fk() is not None
            or self._bridge.current_tcp_pose() is not None
        )
        for button in self._jog_buttons:
            button.setEnabled(
                self._robot_ready and pose_available and not self._move_in_flight
            )
        if not self._robot_ready:
            self._move_in_flight = False
            self._progress.setVisible(False)
