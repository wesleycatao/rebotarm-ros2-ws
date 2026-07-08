"""Joints tab: per-joint sliders and trajectory execution."""

from __future__ import annotations

from typing import Dict

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from ..widgets.joint_slider import JointSlider


class JointsTab(QWidget):
    """Tab to command arm joints via FollowJointTrajectory action."""

    def __init__(self, bridge, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._sliders: Dict[str, JointSlider] = {}

        # ---- Sliders container ----
        self._sliders_host = QWidget()
        self._sliders_lay = QVBoxLayout(self._sliders_host)
        self._sliders_lay.setContentsMargins(8, 8, 8, 8)
        self._sliders_lay.setSpacing(12)
        self._sliders_lay.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self._sliders_host)

        # ---- Duration + buttons ----
        self._duration = QDoubleSpinBox()
        self._duration.setRange(0.1, 60.0)
        self._duration.setValue(2.0)
        self._duration.setSuffix(" s")
        self._duration.setSingleStep(0.5)
        self._duration.setAlignment(Qt.AlignRight)
        self._duration.setMinimumWidth(130)

        self._btn_apply = QPushButton("Move (FollowJointTrajectory)")
        self._btn_apply.setObjectName("PrimaryButton")
        self._btn_apply.setMinimumHeight(56)
        self._btn_apply.clicked.connect(self._send)

        self._btn_reset = QPushButton("Reset targets = current")
        self._btn_reset.setMinimumHeight(56)
        self._btn_reset.clicked.connect(self._reset_to_current)

        ctrl_box = QGroupBox("Execução")
        ctrl_lay = QHBoxLayout(ctrl_box)
        ctrl_lay.setSpacing(14)
        ctrl_lay.addWidget(QLabel("Duração"))
        ctrl_lay.addWidget(self._duration)
        ctrl_lay.addWidget(self._btn_reset)
        ctrl_lay.addStretch(1)
        ctrl_lay.addWidget(self._btn_apply)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(14)
        lay.addWidget(scroll, 1)
        lay.addWidget(ctrl_box)

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

    def _build_sliders(self, arm_names: list[str], msg) -> None:
        positions = {n: p for n, p in zip(msg.name, msg.position)}
        insert_at = self._sliders_lay.count() - 1  # before the stretch
        for idx, name in enumerate(arm_names, start=1):
            slider = JointSlider(
                joint_name=name,
                index=idx,
                minimum=-3.14,
                maximum=3.14,
            )
            cur = positions.get(name)
            if cur is not None:
                slider.set_value(float(cur))
                slider.set_current(float(cur))
            self._sliders[name] = slider
            self._sliders_lay.insertWidget(insert_at, slider)
            insert_at += 1

    def _reset_to_current(self) -> None:
        node = self._bridge.node
        if node is None or node.joint_state is None:
            return
        for name, pos in zip(node.joint_state.name, node.joint_state.position):
            if name in self._sliders:
                self._sliders[name].set_value(float(pos))

    def _send(self) -> None:
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
