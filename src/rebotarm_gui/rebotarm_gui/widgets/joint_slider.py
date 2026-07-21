"""Joint slider widget with live numeric display."""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from .layout_utils import shrink_spinbox


class JointSlider(QWidget):
    """A labelled slider (radians) with a synchronized numeric input.

    Emits :pyattr:`target_changed` when the user edits the target and
    :pyattr:`apply_clicked`. The ``−``/``+`` buttons emit
    :pyattr:`jog_pressed` / :pyattr:`jog_released` with the joint name and
    direction (-1/+1) so the parent can implement press-and-hold jogging.
    """

    target_changed = Signal(float)
    apply_clicked = Signal(float)
    jog_pressed = Signal(str, int)
    jog_released = Signal(str, int)

    def __init__(
        self,
        joint_name: str = "joint",
        index: int = 1,
        minimum: float = -3.14,
        maximum: float = 3.14,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._joint_name = joint_name
        self._index = index
        self._minimum = float(minimum)
        self._maximum = float(maximum)
        self._scale = 1000.0  # slider resolution

        title = QLabel(f"<b>{joint_name}</b>")
        title.setObjectName("JointTitle")

        self._value_label = QLabel("+0.000 rad")
        self._value_label.setObjectName("JointValue")
        self._value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._value_label.setMinimumWidth(96)

        self._current_label = QLabel("atual: --")
        self._current_label.setObjectName("JointCurrent")
        self._current_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(int(self._minimum * self._scale), int(self._maximum * self._scale))
        self._slider.setValue(0)
        self._slider.setMinimumHeight(34)
        self._slider.valueChanged.connect(self._on_slider_changed)

        self._btn_jog_minus = QPushButton("−")
        self._btn_jog_plus = QPushButton("+")
        for btn, direction in ((self._btn_jog_minus, -1), (self._btn_jog_plus, +1)):
            btn.setFixedWidth(40)
            btn.setMinimumHeight(34)
            btn.setToolTip("Manter pressionado para jog da junta")
            btn.pressed.connect(
                lambda n=self._joint_name, d=direction: self.jog_pressed.emit(n, d)
            )
            btn.released.connect(
                lambda n=self._joint_name, d=direction: self.jog_released.emit(n, d)
            )
        self._btn_jog_minus.setAccessibleName(f"{self._joint_name}: jog negativo")
        self._btn_jog_plus.setAccessibleName(f"{self._joint_name}: jog positivo")
        self._slider.setAccessibleName(f"{self._joint_name}: posição alvo")

        self._spin = QDoubleSpinBox()
        self._spin.setAccessibleName(f"{self._joint_name}: posição alvo em radianos")
        self._spin.setRange(self._minimum, self._maximum)
        self._spin.setSingleStep(0.05)
        self._spin.setDecimals(3)
        self._spin.setSuffix(" rad")
        self._spin.setAlignment(Qt.AlignRight)
        self._spin.setMinimumWidth(110)
        self._spin.setMaximumWidth(170)
        shrink_spinbox(self._spin)
        self._spin.valueChanged.connect(self._on_spin_changed)

        top = QHBoxLayout()
        top.setSpacing(16)
        top.addWidget(title)
        top.addStretch(1)
        top.addWidget(self._current_label)
        top.addWidget(self._value_label)

        mid = QHBoxLayout()
        mid.setSpacing(18)
        mid.addWidget(self._btn_jog_minus)
        mid.addWidget(self._slider, 1)
        mid.addWidget(self._btn_jog_plus)
        mid.addWidget(self._spin)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 12, 14, 12)
        lay.setSpacing(6)
        lay.addLayout(top)
        lay.addLayout(mid)
        self.setMinimumHeight(82)

    @property
    def joint_name(self) -> str:
        return self._joint_name

    @property
    def index(self) -> int:
        return self._index

    def value(self) -> float:
        return float(self._spin.value())

    def set_value(self, value: float) -> None:
        self._spin.setValue(float(value))

    def set_current(self, value: float | None) -> None:
        if value is None:
            self._current_label.setText("atual: --")
        else:
            self._current_label.setText(f"atual: {value:+.3f} rad")

    def set_command_enabled(self, enabled: bool) -> None:
        """Enable target editing and jog controls while preserving live readout."""
        for widget in (
            self._slider,
            self._spin,
            self._btn_jog_minus,
            self._btn_jog_plus,
        ):
            widget.setEnabled(bool(enabled))

    def limits(self) -> tuple[float, float]:
        """Current target range (minimum, maximum) in radians."""
        return self._minimum, self._maximum

    def set_range(self, minimum: float, maximum: float) -> None:
        """Update the slider/spin range (e.g. when the URDF limits arrive)."""
        self._minimum = float(minimum)
        self._maximum = float(maximum)
        self._slider.setRange(
            int(self._minimum * self._scale), int(self._maximum * self._scale)
        )
        self._spin.setRange(self._minimum, self._maximum)

    def _on_slider_changed(self, raw: int) -> None:
        val = raw / self._scale
        self._spin.blockSignals(True)
        self._spin.setValue(val)
        self._spin.blockSignals(False)
        self._refresh_value_label(val)
        self.target_changed.emit(val)

    def _on_spin_changed(self, val: float) -> None:
        raw = int(val * self._scale)
        self._slider.blockSignals(True)
        self._slider.setValue(raw)
        self._slider.blockSignals(False)
        self._refresh_value_label(val)
        self.target_changed.emit(val)

    def _refresh_value_label(self, val: float) -> None:
        self._value_label.setText(f"{val:+.3f} rad")
