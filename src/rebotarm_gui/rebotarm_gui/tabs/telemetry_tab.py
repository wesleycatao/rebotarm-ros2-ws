"""Telemetry tab: live tables for joint states and motor details."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QHeaderView,
    QLabel,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


class TelemetryTab(QWidget):
    """Read-only telemetry tables."""

    def __init__(self, bridge, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._bridge = bridge

        # ---- Joint states table ----
        self._joint_tbl = QTableWidget(0, 4)
        self._joint_tbl.setHorizontalHeaderLabels(["Junta", "Posição (rad)", "Velocidade", "Esforço"])
        self._joint_tbl.setAccessibleName("Telemetria de posição, velocidade e esforço das juntas")
        self._joint_tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._joint_tbl.verticalHeader().setVisible(False)
        self._joint_tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        self._joint_tbl.setAlternatingRowColors(True)
        self._joint_tbl.verticalHeader().setDefaultSectionSize(32)

        # ---- Motor detail table ----
        self._detail_tbl = QTableWidget(0, 6)
        self._detail_tbl.setHorizontalHeaderLabels(
            ["Junta", "Posição", "Velocidade", "Esforço", "Temp (C)", "Erro"]
        )
        self._detail_tbl.setAccessibleName("Telemetria detalhada dos motores")
        self._detail_tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._detail_tbl.verticalHeader().setVisible(False)
        self._detail_tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        self._detail_tbl.setAlternatingRowColors(True)
        self._detail_tbl.verticalHeader().setDefaultSectionSize(32)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(12)
        lay.addWidget(QLabel("<b>Joint States</b> (/rebotarm/joint_states)"))
        lay.addWidget(self._joint_tbl, 1)
        lay.addWidget(QLabel("<b>Detalhe por motor</b> (/rebotarm/joints/<j>/state)"))
        lay.addWidget(self._detail_tbl, 1)

    # ------------------------------------------------------------------
    def update_joint_state(self, msg) -> None:
        rows = len(msg.name)
        self._joint_tbl.setRowCount(rows)
        for r, (name, pos, vel, eff) in enumerate(
            zip(msg.name, msg.position, msg.velocity, msg.effort)
        ):
            self._joint_tbl.setItem(r, 0, QTableWidgetItem(str(name)))
            self._joint_tbl.setItem(r, 1, QTableWidgetItem(f"{float(pos):+.4f}"))
            self._joint_tbl.setItem(r, 2, QTableWidgetItem(f"{float(vel):+.4f}"))
            self._joint_tbl.setItem(r, 3, QTableWidgetItem(f"{float(eff):+.4f}"))

    def update_joint_detail(self, joint_name: str, msg) -> None:
        # Find or create row
        names = [self._detail_tbl.item(r, 0).text() if self._detail_tbl.item(r, 0) else "" for r in range(self._detail_tbl.rowCount())]
        if joint_name in names:
            row = names.index(joint_name)
        else:
            row = self._detail_tbl.rowCount()
            self._detail_tbl.insertRow(row)

        pos = getattr(msg, "position", 0.0)
        vel = getattr(msg, "velocity", 0.0)
        eff = getattr(msg, "torque", getattr(msg, "effort", 0.0))
        temp = getattr(msg, "temperature", getattr(msg, "temp", None))
        err = getattr(msg, "error", getattr(msg, "fault", None))

        cells = [
            joint_name,
            f"{float(pos):+.4f}",
            f"{float(vel):+.4f}",
            f"{float(eff):+.4f}",
            f"{float(temp):.1f}" if temp is not None else "--",
            str(err) if err is not None else "--",
        ]
        for c, text in enumerate(cells):
            item = QTableWidgetItem(text)
            if c == 0:
                item.setForeground(Qt.GlobalColor.cyan)
            self._detail_tbl.setItem(row, c, item)
