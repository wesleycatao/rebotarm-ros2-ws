"""Centralized operational alarms for the rebotArm GUI.

Replaces the bare persistent banner with a richer, single-source diagnostic
surface. An :class:`Alarm` carries severity, origin, timestamp and a
recommended action. The :class:`AlarmBanner` (header) shows the most recent
active alarm; the :class:`AlarmCenterPanel` (dock) keeps the full history with
filtering so an operator can review the whole session in one place.

All alarm producers in the application route through one ``emit_alarm`` call
site, so the banner and the history stay consistent.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Deque, Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
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

# Severities ------------------------------------------------------------------
SEV_ERROR = "error"
SEV_WARNING = "warning"
SEV_INFO = "info"

SEVERITY_LABELS = {SEV_ERROR: "ERRO", SEV_WARNING: "AVISO", SEV_INFO: "INFO"}

# Cap for the in-memory history so the dock stays responsive over long sessions.
MAX_HISTORY = 500


def _default_action(origin: str, severity: str) -> str:
    """Sensible recommended action when the producer does not supply one."""
    origin_l = origin.lower()
    if "heartbeat" in origin_l or "arm_status" in origin_l:
        return ("Aguarde o controlador publicar arm_status; comandos de "
                "movimento permanecem bloqueados.")
    if "safety" in origin_l or "segurança" in origin_l or "torque" in origin_l:
        return "Inspecione a célula antes de prosseguir; considere resetar a falha."
    if "action" in origin_l:
        return "Verifique a célula e o controlador; consulte Eventos ROS para detalhes."
    if "service" in origin_l or "serviço" in origin_l:
        return "Confirme o estado do serviço no controlador e tente novamente."
    if severity == SEV_ERROR:
        return "Consulte Eventos ROS e corrija a condição antes de prosseguir."
    return "Avalie o aviso; a operação pode prosseguir."


@dataclass
class Alarm:
    """A single operational alarm record."""

    message: str
    severity: str = SEV_WARNING
    origin: str = "Sistema"
    recommended_action: str = ""
    timestamp: datetime = field(default_factory=datetime.now)
    code: str = ""
    acknowledged: bool = False
    count: int = 1

    def display_message(self) -> str:
        """Message with a repetition suffix when coalesced."""
        return self.message if self.count <= 1 else f"{self.message} (x{self.count})"

    def __post_init__(self) -> None:
        if not self.recommended_action:
            self.recommended_action = _default_action(self.origin, self.severity)
        self.severity = self.severity if self.severity in SEVERITY_LABELS else SEV_WARNING


class AlarmBanner(QWidget):
    """Header banner: shows the most recent active alarm with full context."""

    dismiss_requested = Signal()
    next_requested = Signal()
    show_history_requested = Signal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setObjectName("AlertBanner")
        self.setProperty("severity", SEV_ERROR)
        self.setAccessibleName("Alarme operacional ativo")

        lay = QHBoxLayout(self)
        lay.setContentsMargins(16, 8, 12, 8)
        lay.setSpacing(10)

        self.severity_label = QLabel("ERRO")
        self.severity_label.setObjectName("AlertSeverity")
        lay.addWidget(self.severity_label)

        text_col = QVBoxLayout()
        text_col.setContentsMargins(0, 0, 0, 0)
        text_col.setSpacing(2)

        self.message_label = QLabel("")
        self.message_label.setObjectName("AlertText")
        self.message_label.setWordWrap(True)
        self.message_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.message_label.setAccessibleName("Mensagem do alarme")
        text_col.addWidget(self.message_label)

        meta_row = QHBoxLayout()
        meta_row.setContentsMargins(0, 0, 0, 0)
        meta_row.setSpacing(10)
        self.origin_label = QLabel("")
        self.origin_label.setObjectName("AlertOrigin")
        self.time_label = QLabel("")
        self.time_label.setObjectName("AlertTime")
        self.action_label = QLabel("")
        self.action_label.setObjectName("AlertAction")
        self.action_label.setWordWrap(True)
        meta_row.addWidget(self.origin_label)
        meta_row.addWidget(self.time_label)
        meta_row.addWidget(self.action_label, 1)
        text_col.addLayout(meta_row)
        lay.addLayout(text_col, 1)

        self.counter_button = QPushButton("")
        self.counter_button.setObjectName("AlertCounterButton")
        self.counter_button.setToolTip(
            "Outros alarmes pendentes — clique para ver o próximo."
        )
        self.counter_button.clicked.connect(self.next_requested.emit)
        self.counter_button.hide()
        lay.addWidget(self.counter_button)

        self.btn_history = QPushButton("Histórico")
        self.btn_history.setObjectName("StatusBarButton")
        self.btn_history.setToolTip("Abrir o centro de alarmes (histórico)")
        self.btn_history.clicked.connect(self.show_history_requested.emit)
        lay.addWidget(self.btn_history)

        self.btn_dismiss = QPushButton("Dispensar")
        self.btn_dismiss.setObjectName("StatusBarButton")
        self.btn_dismiss.clicked.connect(self.dismiss_requested.emit)
        lay.addWidget(self.btn_dismiss)
        self.hide()

    def render_alarm(self, alarm: Optional[Alarm], pending: int = 0) -> None:
        """Update the banner for ``alarm`` (None hides it)."""
        if alarm is None:
            self.hide()
            return
        self.severity_label.setText(SEVERITY_LABELS.get(alarm.severity, "AVISO"))
        self.setProperty("severity", alarm.severity)
        # Re-evaluate the severity-driven QSS colours.
        self.style().unpolish(self)
        self.style().polish(self)
        self.message_label.setText(alarm.message)
        self.origin_label.setText(f"Origem: {alarm.origin}")
        self.time_label.setText(f"Horário: {alarm.timestamp.strftime('%H:%M:%S')}")
        self.action_label.setText(f"Ação: {alarm.recommended_action}")
        if pending > 0:
            self.counter_button.setText(f"+{pending}")
            self.counter_button.show()
        else:
            self.counter_button.hide()
        self.show()
        self.raise_()


class AlarmCenterPanel(QWidget):
    """Dockable panel listing every alarm in the session with filters."""

    clear_requested = Signal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._alarms: Deque[Alarm] = deque(maxlen=MAX_HISTORY)
        self._severity_filter = "all"
        self._search_text = ""

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(4)

        toolbar = QHBoxLayout()
        toolbar.setSpacing(6)
        toolbar.addWidget(QLabel("Filtro"))
        self.combo = QComboBox()
        self.combo.addItem("Todos os níveis", "all")
        self.combo.addItem("Erros", SEV_ERROR)
        self.combo.addItem("Avisos", SEV_WARNING)
        self.combo.addItem("Informações", SEV_INFO)
        self.combo.currentIndexChanged.connect(self._on_combo_changed)
        toolbar.addWidget(self.combo)

        self.search = QLineEdit()
        self.search.setPlaceholderText("Buscar nos alarmes…")
        self.search.setClearButtonEnabled(True)
        self.search.textChanged.connect(self._on_search_changed)
        toolbar.addWidget(self.search, 1)

        self.ack_active = QCheckBox("Ocultar ativos")
        self.ack_active.setToolTip("Quando marcado, oculta os alarmes ainda não dispensados do banner.")
        self.ack_active.toggled.connect(self._render)
        toolbar.addWidget(self.ack_active)

        self.btn_clear = QPushButton("Limpar")
        self.btn_clear.clicked.connect(self.clear)
        toolbar.addWidget(self.btn_clear)
        root.addLayout(toolbar)

        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(
            ["Horário", "Severidade", "Origem", "Mensagem", "Ação recomendada"]
        )
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.Stretch)
        header.setSectionResizeMode(4, QHeaderView.Stretch)
        root.addWidget(self.table, 1)

        self.count_label = QLabel("0 alarme(s)")
        self.count_label.setObjectName("MutedLabel")
        root.addWidget(self.count_label)

    # ------------------------------------------------------------------
    def add_alarm(self, alarm: Alarm) -> None:
        self._alarms.append(alarm)
        self._render()

    def refresh(self) -> None:
        """Re-render without changing history (used after in-place coalescing)."""
        self._render()

    def clear(self) -> None:
        self._alarms.clear()
        self.clear_requested.emit()
        self._render()

    def alarm_count(self) -> int:
        return len(self._alarms)

    # ------------------------------------------------------------------
    def _on_combo_changed(self, _index: int) -> None:
        self._severity_filter = self.combo.currentData() or "all"
        self._render()

    def _on_search_changed(self, text: str) -> None:
        self._search_text = text.strip().lower()
        self._render()

    def _matches(self, alarm: Alarm) -> bool:
        if self._severity_filter != "all" and alarm.severity != self._severity_filter:
            return False
        if self.ack_active.isChecked() and not alarm.acknowledged:
            return False
        if self._search_text:
            haystack = " ".join(
                [alarm.message, alarm.origin, alarm.recommended_action]
            ).lower()
            if self._search_text not in haystack:
                return False
        return True

    def _render(self) -> None:
        visible = [a for a in self._alarms if self._matches(a)]
        # Newest first.
        visible.reverse()
        self.table.setRowCount(len(visible))
        for row, alarm in enumerate(visible):
            cells = [
                alarm.timestamp.strftime("%H:%M:%S"),
                SEVERITY_LABELS.get(alarm.severity, alarm.severity.upper()),
                alarm.origin,
                alarm.display_message(),
                alarm.recommended_action,
            ]
            for col, text in enumerate(cells):
                item = QTableWidgetItem(str(text))
                item.setToolTip(str(text))
                self.table.setItem(row, col, item)
        self.count_label.setText(
            f"{len(visible)} de {len(self._alarms)} alarme(s)"
        )
