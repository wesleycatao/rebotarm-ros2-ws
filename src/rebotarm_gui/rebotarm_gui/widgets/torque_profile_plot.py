"""Matplotlib-based torque profile viewer for the safety tab."""

from __future__ import annotations

from typing import Any

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from rebotarm_gui.widgets.combo_box import RetractingComboBox
from PySide6.QtWidgets import QVBoxLayout, QWidget
import yaml


class TorqueProfilePlot(QWidget):
    """Display a torque profile envelope for one joint at a time."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._profile_data: dict[str, Any] | None = None
        self._current_joint: str = ""
        self._latest_torque: float | None = None
        self._latest_phase: float | None = None

        self._joint_selector = RetractingComboBox()
        self._joint_selector.setAccessibleName("Junta exibida no perfil de torque")
        self._joint_selector.currentTextChanged.connect(self._on_joint_changed)

        self._figure = Figure(figsize=(5, 3), dpi=100)
        self._canvas = FigureCanvas(self._figure)
        self._canvas.setAccessibleName("Gráfico do perfil de torque")
        self._canvas.setAccessibleDescription(
            "Envelope WARN e FAIL por fase para a junta selecionada; os valores "
            "equivalentes também estão disponíveis na tabela de verificação."
        )
        self._ax = self._figure.add_subplot(111)
        self._ax.set_xlabel("Fase")
        self._ax.set_ylabel("Torque")
        self._ax.set_title("Perfil de torque WARN/FAIL")
        self._ax.grid(True, linestyle=":", alpha=0.6)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._joint_selector)
        layout.addWidget(self._canvas, 1)

    def load_profile(self, path: str) -> None:
        """Load a torque profile YAML and refresh the joint list."""
        try:
            with open(path, "r", encoding="utf-8") as f:
                self._profile_data = yaml.safe_load(f) or {}
        except Exception:
            self._profile_data = None

        self._joint_selector.blockSignals(True)
        self._joint_selector.clear()
        joints = self._joint_names()
        if joints:
            self._joint_selector.addItems(joints)
            self._current_joint = joints[0]
        else:
            self._current_joint = ""
        self._joint_selector.blockSignals(False)
        self._refresh()

    def clear_profile(self) -> None:
        """Clear the loaded profile."""
        self._profile_data = None
        self._joint_selector.blockSignals(True)
        self._joint_selector.clear()
        self._joint_selector.blockSignals(False)
        self._current_joint = ""
        self._refresh()

    def update_current_state(self, joint_name: str, torque: float, phase: float) -> None:
        """Update the marker showing current torque at current phase.

        Pass a negative phase to hide the live marker.
        """
        self._latest_torque = float(torque)
        self._latest_phase = float(phase) if phase >= 0.0 else None
        if joint_name and self._current_joint != joint_name:
            self._current_joint = joint_name
            index = self._joint_selector.findText(joint_name)
            if index >= 0:
                self._joint_selector.setCurrentIndex(index)
        self._refresh()

    def _joint_names(self) -> list[str]:
        if not self._profile_data:
            return []
        steps = self._profile_data.get("steps", {}) or {}
        names: set[str] = set()
        for step_data in steps.values():
            joints = (step_data or {}).get("joints", {}) or {}
            names.update(joints.keys())
        return sorted(names)

    def _step_for_joint(self, joint_name: str) -> dict[str, Any] | None:
        if not self._profile_data:
            return None
        steps = self._profile_data.get("steps", {}) or {}
        for step_data in steps.values():
            joints = (step_data or {}).get("joints", {}) or {}
            if joint_name in joints:
                return joints[joint_name]
        return None

    def _on_joint_changed(self, text: str) -> None:
        self._current_joint = str(text)
        self._refresh()

    def _refresh(self) -> None:
        self._ax.clear()
        self._ax.set_xlabel("Fase")
        self._ax.set_ylabel("Torque")
        self._ax.set_title("Perfil de torque WARN/FAIL")
        self._ax.grid(True, linestyle=":", alpha=0.6)

        joint_data = self._step_for_joint(self._current_joint)
        if joint_data is None:
            self._canvas.draw()
            return

        phase = [float(v) for v in joint_data.get("phase", [])]
        if not phase:
            self._canvas.draw()
            return

        def _series(key: str) -> np.ndarray | None:
            values = joint_data.get(key)
            if not values or len(values) != len(phase):
                return None
            return np.asarray(values, dtype=np.float64)

        mean = _series("torque_mean")
        warn_min = _series("warn_min")
        warn_max = _series("warn_max")
        fail_min = _series("fail_min")
        fail_max = _series("fail_max")

        if mean is not None:
            self._ax.plot(phase, mean, label="média", color="blue", linewidth=1.5)
        if warn_min is not None and warn_max is not None:
            self._ax.fill_between(
                phase,
                warn_min,
                warn_max,
                color="yellow",
                alpha=0.3,
                label="WARN",
            )
        if fail_min is not None and fail_max is not None:
            self._ax.fill_between(
                phase,
                fail_min,
                fail_max,
                color="red",
                alpha=0.15,
                label="FAIL",
            )

        if (
            self._latest_phase is not None
            and self._latest_torque is not None
        ):
            self._ax.axvline(
                self._latest_phase, color="green", linestyle="--", alpha=0.7, label="fase atual"
            )
            self._ax.scatter(
                [self._latest_phase],
                [self._latest_torque],
                color="green",
                s=60,
                zorder=5,
                label="torque atual",
            )

        self._ax.legend(loc="best", fontsize="small")
        self._ax.set_xlim(0.0, 1.0)
        self._canvas.draw()
