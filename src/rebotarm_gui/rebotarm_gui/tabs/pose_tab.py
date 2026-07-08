"""Cartesian pose tab: MoveToPose action + MoveToPoseIK service."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..ros_bridge import make_pose
from ..widgets.pose_input import PoseInput


class PoseTab(QWidget):
    """Tab to command the end-effector to a Cartesian pose."""

    def __init__(self, bridge, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._bridge = bridge

        self._pose_input = PoseInput()

        self._btn_move = QPushButton("MoveToPose (action)")
        self._btn_move.setObjectName("PrimaryButton")
        self._btn_move.setMinimumHeight(56)
        self._btn_move.clicked.connect(self._send_move)

        self._btn_ik = QPushButton("MoveToPoseIK (service)")
        self._btn_ik.setMinimumHeight(56)
        self._btn_ik.clicked.connect(self._send_ik)

        self._lbl_current = QLabel("Pose atual do TCP: --")
        self._lbl_current.setObjectName("MutedLabel")
        self._lbl_current.setAlignment(Qt.AlignCenter)

        buttons = QHBoxLayout()
        buttons.setSpacing(12)
        buttons.addWidget(self._btn_ik)
        buttons.addWidget(self._btn_move)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(14)
        lay.addWidget(self._pose_input)
        lay.addStretch(1)
        lay.addWidget(self._lbl_current)
        lay.addLayout(buttons)

    # ------------------------------------------------------------------
    def update_joint_state(self, _msg) -> None:
        """Placeholder for joint-state updates (TCP pose may be computed later)."""

    def _values(self):
        return self._pose_input.values()

    def _send_move(self) -> None:
        x, y, z, qx, qy, qz, qw, duration = self._values()
        pose = make_pose(x, y, z, qx, qy, qz, qw)
        self._bridge.move_to_pose(pose, duration)

    def _send_ik(self) -> None:
        x, y, z, qx, qy, qz, qw, _duration = self._values()
        pose = make_pose(x, y, z, qx, qy, qz, qw)
        self._bridge.move_to_pose_ik(pose)
