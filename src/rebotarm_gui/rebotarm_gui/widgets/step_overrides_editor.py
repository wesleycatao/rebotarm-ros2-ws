"""Editor for per-step torque profile tolerances."""

from __future__ import annotations

from typing import Any

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QHeaderView,
    QLabel,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


class StepOverridesEditor(QWidget):
    """Allow configuring WARN/FAIL tolerances and min margin per step."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._rows: list[tuple[str, QDoubleSpinBox, QDoubleSpinBox, QDoubleSpinBox]] = []

        self._table = QTableWidget(0, 4)
        self._table.setHorizontalHeaderLabels(
            ["Step ID", "WARN %", "FAIL %", "Margem mín."]
        )
        self._table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(QLabel("Tolerâncias por etapa (vazio = usar valor global)"))
        layout.addWidget(self._table, 1)

    def set_steps(
        self,
        steps: list[str],
        defaults: dict[str, float] | None = None,
    ) -> None:
        """Populate the editor with step IDs and default values."""
        defaults = defaults or {}
        self._table.blockSignals(True)
        self._table.setRowCount(len(steps))
        self._rows = []
        for row, step_id in enumerate(sorted(steps)):
            item = QTableWidgetItem(step_id)
            item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self._table.setItem(row, 0, item)

            warn_spin = self._make_spin(defaults.get("warn_tolerance_percent", 10.0))
            fail_spin = self._make_spin(defaults.get("fail_tolerance_percent", 20.0))
            margin_spin = self._make_spin(defaults.get("min_margin", 0.25))

            self._table.setCellWidget(row, 1, warn_spin)
            self._table.setCellWidget(row, 2, fail_spin)
            self._table.setCellWidget(row, 3, margin_spin)
            self._rows.append((step_id, warn_spin, fail_spin, margin_spin))
        self._table.blockSignals(False)

    def clear(self) -> None:
        """Remove all rows."""
        self._table.setRowCount(0)
        self._rows = []

    def overrides(self) -> dict[str, dict[str, float]]:
        """Return a dict of step_id -> {warn_tolerance_percent, fail_tolerance_percent, min_margin}."""
        result: dict[str, dict[str, float]] = {}
        for step_id, warn_spin, fail_spin, margin_spin in self._rows:
            result[step_id] = {
                "warn_tolerance_percent": float(warn_spin.value()),
                "fail_tolerance_percent": float(fail_spin.value()),
                "min_margin": float(margin_spin.value()),
            }
        return result

    @staticmethod
    def _make_spin(value: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 1000.0)
        spin.setValue(float(value))
        spin.setSingleStep(1.0)
        spin.setDecimals(3)
        spin.setSpecialValueText("global")
        return spin
