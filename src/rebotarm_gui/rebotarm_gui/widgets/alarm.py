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
import json
import os
from pathlib import Path
from typing import Deque, Optional
import uuid

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
    QSizePolicy,
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
STATE_ACTIVE = "active"
STATE_ACKNOWLEDGED = "acknowledged"
STATE_CLEARED = "cleared"
ALARM_STATES = (STATE_ACTIVE, STATE_ACKNOWLEDGED, STATE_CLEARED)


def default_alarm_path() -> Path:
    base = os.environ.get("XDG_STATE_HOME")
    root = Path(base).expanduser() if base else Path.home() / ".local" / "state"
    return root / "rebotarm_gui" / "alarms" / "alarms.jsonl"


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
    alarm_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    state: str = STATE_ACTIVE
    updated_at: datetime = field(default_factory=datetime.now)

    def display_message(self) -> str:
        """Message with a repetition suffix when coalesced."""
        return self.message if self.count <= 1 else f"{self.message} (x{self.count})"

    def __post_init__(self) -> None:
        if not self.recommended_action:
            self.recommended_action = _default_action(self.origin, self.severity)
        self.severity = self.severity if self.severity in SEVERITY_LABELS else SEV_WARNING
        if self.state not in ALARM_STATES:
            self.state = STATE_ACTIVE
        self.acknowledged = self.state in (STATE_ACKNOWLEDGED, STATE_CLEARED)

    def to_record(self, event: str) -> dict:
        return {
            "schema_version": 1,
            "event": str(event),
            "alarm_id": self.alarm_id,
            "timestamp": self.timestamp.isoformat(timespec="milliseconds"),
            "updated_at": self.updated_at.isoformat(timespec="milliseconds"),
            "message": self.message,
            "severity": self.severity,
            "origin": self.origin,
            "recommended_action": self.recommended_action,
            "code": self.code,
            "state": self.state,
            "count": int(self.count),
        }

    @classmethod
    def from_record(cls, record: dict) -> "Alarm":
        def parse(value: object) -> datetime:
            try:
                return datetime.fromisoformat(str(value))
            except (TypeError, ValueError):
                return datetime.now()

        return cls(
            message=str(record.get("message", "")),
            severity=str(record.get("severity", SEV_WARNING)),
            origin=str(record.get("origin", "Sistema")),
            recommended_action=str(record.get("recommended_action", "")),
            timestamp=parse(record.get("timestamp")),
            code=str(record.get("code", "")),
            count=max(1, int(record.get("count", 1))),
            alarm_id=str(record.get("alarm_id") or uuid.uuid4()),
            state=str(record.get("state", STATE_ACTIVE)),
            updated_at=parse(record.get("updated_at")),
        )


class AlarmStore:
    """Durable append-only alarm lifecycle journal."""

    def __init__(self, path: str | Path | None = None) -> None:
        self.path = Path(path).expanduser() if path else default_alarm_path()
        self.last_error = ""

    def append(self, alarm: Alarm, event: str) -> bool:
        try:
            self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            line = json.dumps(
                alarm.to_record(event),
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
            with self.path.open("a", encoding="utf-8") as stream:
                stream.write(line + "\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(self.path, 0o600)
            return True
        except (OSError, TypeError, ValueError) as exc:
            self.last_error = str(exc)
            return False

    def load(self, limit: int = MAX_HISTORY) -> list[Alarm]:
        latest: dict[str, Alarm] = {}
        order: list[str] = []
        if not self.path.is_file():
            return []
        try:
            with self.path.open("r", encoding="utf-8") as stream:
                for line in stream:
                    try:
                        record = json.loads(line)
                        alarm = Alarm.from_record(record)
                    except (json.JSONDecodeError, TypeError, ValueError):
                        continue
                    if alarm.alarm_id not in latest:
                        order.append(alarm.alarm_id)
                    latest[alarm.alarm_id] = alarm
        except OSError as exc:
            self.last_error = str(exc)
            return []
        return [latest[key] for key in order[-max(1, int(limit)):]]

    def active(self) -> list[Alarm]:
        return [alarm for alarm in self.load() if alarm.state == STATE_ACTIVE]


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

        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(
            ["Horário", "Severidade", "Estado", "Origem", "Mensagem", "Ação recomendada"]
        )
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.Stretch)
        header.setSectionResizeMode(5, QHeaderView.Stretch)
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
        self.clear_requested.emit()
        self._alarms.clear()
        self._render()

    def alarm_count(self) -> int:
        return len(self._alarms)

    def alarms(self) -> list[Alarm]:
        """Return the lifecycle objects currently represented in the center."""
        return list(self._alarms)

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
                {
                    STATE_ACTIVE: "ATIVO",
                    STATE_ACKNOWLEDGED: "RECONHECIDO",
                    STATE_CLEARED: "NORMALIZADO",
                }.get(alarm.state, alarm.state.upper()),
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


class HeaderAlarmIndicator(QWidget):
    """Compact, persistent header indicator for active alarms.

    Shows the count of active (not cleared) alarms, the highest active
    severity, a short summary of the most critical alarm and its recommended
    action.  It does not rely on colour alone: severity is rendered as text,
    an icon shape and an accessible name.  Clicking opens the alarm centre.
    """

    clicked = Signal()

    # severity -> (label text, icon character, accessible name suffix, tooltip colour)
    _SEVERITY_META = {
        SEV_ERROR: ("ERRO", "⚠", "erro", "#ff6b6b"),
        SEV_WARNING: ("AVISO", "▲", "aviso", "#f0a500"),
        SEV_INFO: ("INFO", "ℹ", "informação", "#7aa2ff"),
    }

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setObjectName("HeaderAlarmIndicator")
        self.setAccessibleName("Indicador de alarmes ativos")
        self.setCursor(Qt.PointingHandCursor)
        self.setToolTip("Nenhum alarme ativo")

        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.setSpacing(6)

        self._icon = QLabel("●")
        self._icon.setObjectName("HeaderAlarmIndicatorIcon")
        self._icon.setAlignment(Qt.AlignCenter)
        lay.addWidget(self._icon)

        self._count = QLabel("0")
        self._count.setObjectName("HeaderAlarmIndicatorCount")
        lay.addWidget(self._count)

        self._severity = QLabel("")
        self._severity.setObjectName("HeaderAlarmIndicatorSeverity")
        lay.addWidget(self._severity)

        self._summary = QLabel("—")
        self._summary.setObjectName("HeaderAlarmIndicatorSummary")
        self._summary.setWordWrap(False)
        self._summary.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self._summary.setMinimumWidth(40)
        self._summary.setMaximumWidth(240)
        lay.addWidget(self._summary, 1)

        self._compact = False
        self._latest_alarm: Optional[Alarm] = None
        self._active_count = 0
        self._highest_severity = SEV_INFO

    def mousePressEvent(self, event) -> None:  # noqa: N802
        self.clicked.emit()
        super().mousePressEvent(event)

    def set_compact(self, compact: bool) -> None:
        compact = bool(compact)
        if compact == self._compact:
            return
        self._compact = compact
        self._summary.setVisible(not compact)
        self._severity.setVisible(not compact)
        self._refresh()

    def update_alarms(self, alarms: list[Alarm]) -> None:
        """Refresh from a snapshot of alarm lifecycle objects.

        Active means any state other than cleared, so acknowledged alarms
        that are still live continue to be counted.
        """
        active = [a for a in alarms if a.state != STATE_CLEARED]
        self._active_count = len(active)
        severity_order = (SEV_ERROR, SEV_WARNING, SEV_INFO)
        self._highest_severity = SEV_INFO
        for candidate in severity_order:
            if any(a.severity == candidate for a in active):
                self._highest_severity = candidate
                break
        # Most critical and most recent first.
        sorted_active = sorted(
            active,
            key=lambda a: (
                severity_order.index(a.severity) if a.severity in severity_order else 99,
                -a.timestamp.timestamp(),
            ),
        )
        self._latest_alarm = sorted_active[0] if sorted_active else None
        self._refresh()

    def _refresh(self) -> None:
        meta = self._SEVERITY_META.get(self._highest_severity, self._SEVERITY_META[SEV_INFO])
        label, icon_char, suffix, _colour = meta
        self._icon.setText(icon_char if not self._compact else "●")
        # Icon colour is resolved by the theme-aware #HeaderAlarmIndicatorIcon
        # [severity="..."] rule (generated from theme_tokens), so it follows
        # the active theme instead of a dark-only inline literal.
        self._icon.setProperty("severity", self._highest_severity)
        self._icon.style().unpolish(self._icon)
        self._icon.style().polish(self._icon)
        self._count.setText(str(self._active_count))
        self._severity.setText(label)
        self.setProperty("severity", self._highest_severity)
        self.style().unpolish(self)
        self.style().polish(self)

        if self._latest_alarm is not None:
            summary = self._latest_alarm.message
            if self._latest_alarm.count > 1:
                summary += f" (x{self._latest_alarm.count})"
            self._summary.setText(summary)
            self.setAccessibleName(
                f"Indicador de alarmes: {self._active_count} ativo(s), "
                f"maior severidade {suffix}, {summary}. Ação recomendada: "
                f"{self._latest_alarm.recommended_action}. Clique para abrir o centro de alarmes."
            )
            self.setToolTip(
                f"<b>{label}</b> — {self._active_count} alarme(s) ativo(s)<br>"
                f"Origem: {self._latest_alarm.origin}<br>"
                f"Horário: {self._latest_alarm.timestamp.strftime('%H:%M:%S')}<br>"
                f"Mensagem: {summary}<br>"
                f"Ação recomendada: {self._latest_alarm.recommended_action}"
            )
        else:
            self._summary.setText("—")
            self.setAccessibleName("Indicador de alarmes: nenhum alarme ativo")
            self.setToolTip("Nenhum alarme ativo")

        self.setVisible(self._active_count > 0)


# ------------------------------------------------------------------------------
# Alarm sorting helper used by the alarm centre and the header indicator.
# ------------------------------------------------------------------------------
def sort_alarms(alarms: list[Alarm]) -> list[Alarm]:
    """Sort by: active first, then severity, then most recent."""
    severity_order = {SEV_ERROR: 0, SEV_WARNING: 1, SEV_INFO: 2}
    state_order = {STATE_ACTIVE: 0, STATE_ACKNOWLEDGED: 1, STATE_CLEARED: 2}
    return sorted(
        alarms,
        key=lambda a: (
            state_order.get(a.state, 99),
            severity_order.get(a.severity, 99),
            -a.timestamp.timestamp(),
        ),
    )
