"""I/O tab: Raspberry Pi digital inputs (opto-isolated) and outputs (relay).

Displays every configured GPIO point with its ID, user TAG, enable/disable
toggle, activation counter and live status. Relay outputs can be driven
on/off/toggled directly from here; tags and the enabled flag are pushed to the
DioBridge node (running on the Pi) via the ``set_config`` service and persisted
there.

The same I/O points are referenced by the motion planner (``WAIT_FOR_SIGNAL``
for inputs, ``OUTPUT_ON / OFF / TOGGLE`` for outputs).
"""

from __future__ import annotations

from typing import Dict

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..widgets.layout_utils import fit_button_text
from ..widgets.status_led import StatusLed

INPUT_IDS = [f"IN{i}" for i in range(1, 6)]
OUTPUT_IDS = [f"OUT{i}" for i in range(1, 6)]


class _IORow:
    """Holds the live widgets for a single I/O point."""

    __slots__ = ("io_id", "is_output", "tag", "enabled", "gpio", "count",
                 "led", "status", "btn_on", "btn_off", "btn_toggle")

    def __init__(self, io_id: str, is_output: bool) -> None:
        self.io_id = io_id
        self.is_output = is_output
        self.tag: QLineEdit | None = None
        self.enabled: QCheckBox | None = None
        self.gpio: QLabel | None = None
        self.count: QLabel | None = None
        self.led: StatusLed | None = None
        self.status: QLabel | None = None
        self.btn_on: QPushButton | None = None
        self.btn_off: QPushButton | None = None
        self.btn_toggle: QPushButton | None = None


class IoTab(QWidget):
    """Tab for inspecting and driving the digital I/O points."""

    def __init__(self, bridge, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._updating = False
        self._out_rows: Dict[str, _IORow] = {}
        self._in_rows: Dict[str, _IORow] = {}

        self._lbl_link = QLabel("Aguardando DioBridge (tópico /rebotarm/dio/state)...")
        self._lbl_link.setObjectName("MutedLabel")

        self._out_table = self._make_table(
            ("ID", "TAG", "Habilitado", "GPIO", "Ativações", "Status", "Ações"),
            with_actions=True,
        )
        self._in_table = self._make_table(
            ("ID", "TAG", "Habilitado", "GPIO", "Ativações", "Status"),
            with_actions=False,
        )

        for io_id in OUTPUT_IDS:
            self._add_row(self._out_table, self._out_rows, io_id, True)
        for io_id in INPUT_IDS:
            self._add_row(self._in_table, self._in_rows, io_id, False)

        out_box = QGroupBox("Saídas digitais (relé)")
        ov = QVBoxLayout(out_box)
        ov.setContentsMargins(10, 14, 10, 10)
        ov.addWidget(self._out_table)

        in_box = QGroupBox("Entradas digitais (optoacopladas)")
        iv = QVBoxLayout(in_box)
        iv.setContentsMargins(10, 14, 10, 10)
        iv.addWidget(self._in_table)

        self._btn_all_off = QPushButton("Desligar todas as saídas")
        self._btn_all_off.setObjectName("WarningButton")
        self._btn_all_off.setMinimumHeight(40)
        self._btn_all_off.clicked.connect(self._all_off)
        fit_button_text(self._btn_all_off)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(12)
        lay.addWidget(self._lbl_link)
        lay.addWidget(out_box)
        lay.addWidget(in_box)
        lay.addWidget(self._btn_all_off)
        lay.addStretch(1)

        self._bridge.dio_ios_changed.connect(self._on_dio_changed)

    # ------------------------------------------------------------------
    # table construction
    # ------------------------------------------------------------------
    @staticmethod
    def _make_table(headers: tuple, with_actions: bool) -> QTableWidget:
        table = QTableWidget(0, len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.verticalHeader().setVisible(False)
        table.setSelectionMode(QAbstractItemView.NoSelection)
        table.setFocusPolicy(Qt.NoFocus)
        table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        table.setAlternatingRowColors(True)
        table.horizontalHeader().setHighlightSections(False)
        h = table.horizontalHeader()
        h.setSectionResizeMode(QHeaderView.ResizeToContents)
        if with_actions:
            h.setSectionResizeMode(1, QHeaderView.Stretch)  # TAG stretches
        else:
            h.setSectionResizeMode(1, QHeaderView.Stretch)
        return table

    def _add_row(self, table: QTableWidget, store: dict, io_id: str, is_output: bool) -> None:
        row = _IORow(io_id, is_output)
        r = table.rowCount()
        table.insertRow(r)

        id_item = QTableWidgetItem(io_id)
        id_item.setFlags(Qt.ItemIsEnabled)
        table.setItem(r, 0, id_item)

        row.tag = QLineEdit()
        row.tag.setPlaceholderText("Sem tag")
        row.tag.setAccessibleName(f"{io_id}: TAG")
        row.tag.editingFinished.connect(lambda _=False, i=io_id: self._on_tag_edited(i))
        table.setCellWidget(r, 1, row.tag)

        row.enabled = QCheckBox()
        row.enabled.setAccessibleName(f"{io_id}: habilitado")
        row.enabled.setChecked(True)
        row.enabled.toggled.connect(lambda v, i=io_id: self._on_enabled_toggled(i, v))
        wrap = QWidget()
        QHBoxLayout(wrap).addWidget(row.enabled)
        wrap.setLayoutDirection(Qt.LeftToRight)
        table.setCellWidget(r, 2, wrap)

        row.gpio = QLabel("--")
        row.gpio.setObjectName("MutedLabel")
        row.gpio.setAlignment(Qt.AlignCenter)
        table.setCellWidget(r, 3, row.gpio)

        row.count = QLabel("0")
        row.count.setObjectName("MetricValue")
        row.count.setAlignment(Qt.AlignCenter)
        table.setCellWidget(r, 4, row.count)

        row.led = StatusLed("off", 16)
        row.led.setAccessibleName(f"{io_id}: indicador de estado")
        row.status = QLabel("BAIXO")
        row.status.setObjectName("MetricValue")
        row.status.setMinimumWidth(64)
        sw = QWidget()
        sl = QHBoxLayout(sw)
        sl.setContentsMargins(0, 0, 0, 0)
        sl.setSpacing(8)
        sl.addWidget(row.led)
        sl.addWidget(row.status)
        sl.addStretch(1)
        table.setCellWidget(r, 5, sw)

        if is_output:
            row.btn_on = QPushButton("Ligar")
            row.btn_on.setObjectName("PrimaryButton")
            row.btn_off = QPushButton("Desligar")
            row.btn_off.setObjectName("WarningButton")
            row.btn_toggle = QPushButton("Alternar")
            row.btn_on.setAccessibleName(f"{io_id}: ligar saída")
            row.btn_off.setAccessibleName(f"{io_id}: desligar saída")
            row.btn_toggle.setAccessibleName(f"{io_id}: alternar saída")
            for b in (row.btn_on, row.btn_off, row.btn_toggle):
                fit_button_text(b)
            row.btn_on.clicked.connect(lambda _=False, i=io_id: self._set_output(i, True))
            row.btn_off.clicked.connect(lambda _=False, i=io_id: self._set_output(i, False))
            row.btn_toggle.clicked.connect(lambda _=False, i=io_id: self._toggle_output(i))
            aw = QWidget()
            al = QHBoxLayout(aw)
            al.setContentsMargins(0, 0, 0, 0)
            al.setSpacing(6)
            al.addWidget(row.btn_on)
            al.addWidget(row.btn_off)
            al.addWidget(row.btn_toggle)
            table.setCellWidget(r, 6, aw)

        table.setRowHeight(r, 40)
        store[io_id] = row

    # ------------------------------------------------------------------
    # live updates
    # ------------------------------------------------------------------
    def _on_dio_changed(self, ios: dict) -> None:
        self._updating = True
        try:
            self._lbl_link.setText(
                f"DioBridge conectado — {len(ios)} pontos de I/O ativos."
            )
            for io_id, io in ios.items():
                row = self._out_rows.get(io_id) or self._in_rows.get(io_id)
                if row is None:
                    continue
                self._apply_io(row, io)
        finally:
            self._updating = False

    def _apply_io(self, row: _IORow, io) -> None:
        if row.tag is not None and not row.tag.hasFocus():
            row.tag.blockSignals(True)
            row.tag.setText(str(getattr(io, "tag", "") or ""))
            row.tag.blockSignals(False)
        if row.enabled is not None:
            row.enabled.blockSignals(True)
            row.enabled.setChecked(bool(getattr(io, "enabled", True)))
            row.enabled.blockSignals(False)
        if row.gpio is not None:
            row.gpio.setText(f"BCM {int(getattr(io, 'pin', 0))}")
        if row.count is not None:
            row.count.setText(str(int(getattr(io, "activation_count", 0))))
        value = bool(getattr(io, "value", False))
        enabled = bool(getattr(io, "enabled", True))
        if row.led is not None:
            if not enabled:
                row.led.set_color("off")
            elif value:
                row.led.set_color("green")
            else:
                row.led.set_color("off")
        if row.status is not None:
            if not enabled:
                txt = "DESAB."
            elif row.is_output:
                txt = "LIGADO" if value else "DESLIGADO"
            else:
                txt = "ALTO" if value else "BAIXO"
            row.status.setText(txt)
        active = enabled
        if row.btn_on is not None:
            row.btn_on.setEnabled(active)
            row.btn_off.setEnabled(active)
            row.btn_toggle.setEnabled(active)

    # ------------------------------------------------------------------
    # actions
    # ------------------------------------------------------------------
    def _set_output(self, io_id: str, value: bool) -> None:
        self._bridge.set_dio_output(io_id, value)

    def _toggle_output(self, io_id: str) -> None:
        ios = self._bridge.dio_ios_snapshot()
        io = ios.get(io_id)
        cur = bool(getattr(io, "value", False)) if io is not None else False
        self._bridge.set_dio_output(io_id, not cur)

    def _all_off(self) -> None:
        for io_id in OUTPUT_IDS:
            self._bridge.set_dio_output(io_id, False)

    def _on_tag_edited(self, io_id: str) -> None:
        if self._updating:
            return
        row = self._out_rows.get(io_id) or self._in_rows.get(io_id)
        tag = row.tag.text().strip() if row is not None and row.tag is not None else ""
        enabled = row.enabled.isChecked() if row is not None and row.enabled is not None else True
        self._bridge.set_dio_config(io_id, tag, enabled)

    def _on_enabled_toggled(self, io_id: str, enabled: bool) -> None:
        if self._updating:
            return
        row = self._out_rows.get(io_id) or self._in_rows.get(io_id)
        tag = row.tag.text().strip() if row is not None and row.tag is not None else ""
        self._bridge.set_dio_config(io_id, tag, enabled)
