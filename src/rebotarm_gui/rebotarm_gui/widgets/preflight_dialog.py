"""Dialog that presents the pre-flight validation report.

Used by the Motion tab both as a manual "Validar" surface and (in a lighter
form) to explain why a real execution was blocked. The dialog owns a
:class:`~rebotarm_gui.motion.preflight.ReachabilityProbe` so the IK results
populate the table incrementally without freezing the UI.
"""

from __future__ import annotations

from typing import Callable, Optional

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QProgressBar,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..motion.preflight import (
    SEV_ERROR,
    SEV_INFO,
    SEV_WARNING,
    STATUS_FAIL,
    STATUS_INFO,
    STATUS_PASS,
    STATUS_SKIP,
    PreflightReport,
    ReachabilityProbe,
    REACHABILITY_DEFAULT_TIMEOUT_MS,
)
from .layout_utils import fit_button_text


# Visual mapping for severity/status cells.
_SEV_LABEL = {SEV_ERROR: "ERRO", SEV_WARNING: "AVISO", SEV_INFO: "INFO"}
_SEV_COLOR = {
    SEV_ERROR: QColor("#ff9aa5"),
    SEV_WARNING: QColor("#ffd37a"),
    SEV_INFO: QColor("#9ff0d8"),
}
_STATUS_LABEL = {
    STATUS_PASS: "OK",
    STATUS_FAIL: "Falha",
    STATUS_SKIP: "Ignorado",
    STATUS_INFO: "Info",
}


class PreflightDialog(QDialog):
    """Modal report viewer with a re-validate action.

    The caller supplies ``get_program_cb`` returning ``(Program, for_real_run)``
    so the dialog always validates the *current* program/mode when the user
    clicks "Revalidar".
    """

    def __init__(
        self,
        bridge,
        get_program_cb: Callable[[], tuple],
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Validação pré-execução (preflight)")
        self.resize(880, 560)
        self._bridge = bridge
        self._get_program_cb = get_program_cb
        self._probe: Optional[ReachabilityProbe] = None

        self._summary = QLabel("Clique em \"Revalidar\" para executar a validação.")
        self._summary.setObjectName("PreflightSummary")
        self._summary.setWordWrap(True)
        self._summary.setTextFormat(Qt.RichText)

        self._progress = QProgressBar()
        self._progress.setRange(0, 1)
        self._progress.setValue(0)
        self._progress.setTextVisible(True)
        self._progress.setFormat("Alcance: %v/%m")
        self._progress.setVisible(False)

        self._table = QTableWidget(0, 5)
        self._table.setHorizontalHeaderLabels(
            ["Categoria", "Verificação", "Severidade", "Status", "Mensagem"]
        )
        self._table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._table.setSelectionBehavior(QTableWidget.SelectRows)
        self._table.setAlternatingRowColors(True)
        header = self._table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.Stretch)

        legend = QLabel(
            "<span style='color:#ff9aa5'>■ ERRO bloqueia execução real</span> · "
            "<span style='color:#ffd37a'>■ AVISO não bloqueia</span> · "
            "<span style='color:#9ff0d8'>■ INFO/OK</span>"
        )
        legend.setObjectName("MutedLabel")

        self._btn_revalidate = QPushButton("Revalidar")
        self._btn_revalidate.setObjectName("PrimaryButton")
        self._btn_revalidate.clicked.connect(self.revalidate)
        self._btn_close = QPushButton("Fechar")
        self._btn_close.clicked.connect(self.accept)
        fit_button_text(self._btn_revalidate)
        fit_button_text(self._btn_close)

        buttons = QHBoxLayout()
        buttons.addWidget(self._btn_revalidate)
        buttons.addStretch(1)
        buttons.addWidget(self._btn_close)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(8)
        layout.addWidget(self._summary)
        layout.addWidget(self._progress)
        layout.addWidget(self._table, 1)
        layout.addWidget(legend)
        layout.addLayout(buttons)

    # ------------------------------------------------------------------
    def revalidate(self) -> None:
        """Run sync checks, then reachability, updating the table live."""
        if self._probe is not None:
            return  # a run is already in flight
        program, for_real_run = self._get_program_cb()
        report = PreflightReport()
        # import lazily to avoid a module-load cycle in tests
        from ..motion.preflight import run_sync_checks

        report = run_sync_checks(program, self._bridge, for_real_run=for_real_run)
        self.set_report(report)
        if report.errors or not program.steps:
            self._progress.setVisible(False)
            return
        self._start_reachability(program, report)

    def _start_reachability(self, program, report: PreflightReport) -> None:
        self._progress.setVisible(True)
        self._progress.setRange(0, 1)
        self._progress.setValue(0)
        self._progress.setFormat("Alcance: iniciando…")
        self._btn_revalidate.setEnabled(False)
        self._probe = ReachabilityProbe(
            program, self._bridge, report,
            timeout_ms=REACHABILITY_DEFAULT_TIMEOUT_MS,
        )
        self._probe.progress.connect(self._on_progress)
        self._probe.finished.connect(self._on_finished)
        self._probe.start()

    def _on_progress(self, done: int, total: int) -> None:
        if total <= 0:
            return
        self._progress.setRange(0, total)
        self._progress.setValue(done)
        self._progress.setFormat("Alcance: %v/%m")

    def _on_finished(self, report: PreflightReport) -> None:
        self._probe = None
        self._progress.setVisible(False)
        self._btn_revalidate.setEnabled(True)
        self.set_report(report)

    # ------------------------------------------------------------------
    def set_report(self, report: PreflightReport) -> None:
        """Render ``report`` into the table and summary."""
        self._table.setRowCount(len(report.items))
        for row, item in enumerate(report.items):
            cells = [
                item.category,
                item.name,
                _SEV_LABEL.get(item.severity, item.severity.upper()),
                _STATUS_LABEL.get(item.status, item.status),
                item.message,
            ]
            for col, text in enumerate(cells):
                cell = QTableWidgetItem(str(text))
                cell.setToolTip(item.detail or str(text))
                if col == 2:
                    cell.setForeground(_SEV_COLOR.get(item.severity, QColor("#dce5ed")))
                self._table.setItem(row, col, cell)
        self._update_summary(report)

    def _update_summary(self, report: PreflightReport) -> None:
        n_err = len(report.errors)
        n_warn = len(report.warnings)
        reach = ""
        if report.reachability_done:
            reach = " · alcance verificado"
        elif report.reachability_skipped_reason:
            reach = f" · alcance não verificado ({report.reachability_skipped_reason})"
        if n_err:
            color = "#ff9aa5"
            head = f"{n_err} erro(s) bloqueante(s)"
        elif n_warn:
            color = "#ffd37a"
            head = "OK com avisos"
        else:
            color = "#9ff0d8"
            head = "Tudo OK"
        body = report.summary()
        self._summary.setText(
            f"<span style='color:{color}; font-weight:700'>{head}:</span> {body}{reach}"
        )
