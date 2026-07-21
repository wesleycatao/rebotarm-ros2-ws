"""Read-only ROS 2 health dashboard for operators and maintainers."""

from __future__ import annotations

from datetime import datetime

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


_OK = QColor("#79e6ce")
_WARN = QColor("#ffd37a")
_ERROR = QColor("#ff9aa7")
_MUTED = QColor("#9aa8b5")


class DiagnosticsTab(QWidget):
    """Present heartbeat, topic metrics and ROS endpoint discovery."""

    def __init__(self, bridge, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._last_snapshot: dict[str, object] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 16, 18, 18)
        root.setSpacing(12)

        toolbar = QHBoxLayout()
        intro = QLabel(
            "Diagnóstico local da conexão usada pela GUI. A coleta é somente leitura e não "
            "substitui a supervisão do controlador."
        )
        intro.setObjectName("MutedLabel")
        intro.setWordWrap(True)
        toolbar.addWidget(intro, 1)
        self._btn_refresh = QPushButton("Atualizar agora")
        self._btn_refresh.setToolTip("Atualiza descoberta e métricas sem bloquear a interface")
        self._btn_refresh.clicked.connect(self.refresh_now)
        toolbar.addWidget(self._btn_refresh)
        root.addLayout(toolbar)

        summary = QGridLayout()
        summary.setHorizontalSpacing(10)
        self._heartbeat_value, self._heartbeat_hint, self._heartbeat_card = self._summary_card(
            "HEARTBEAT", "SEM SINAL", "Aguardando /arm_status"
        )
        self._latency_value, self._latency_hint, self._latency_card = self._summary_card(
            "LATÊNCIA", "N/D", "Timestamp de /arm_status"
        )
        self._topics_value, self._topics_hint, self._topics_card = self._summary_card(
            "TÓPICOS", "0 / 0", "Com atualização recente"
        )
        self._endpoints_value, self._endpoints_hint, self._endpoints_card = self._summary_card(
            "ENDPOINTS", "0 / 0", "Serviços e actions disponíveis"
        )
        for column, card in enumerate(
            (
                self._heartbeat_card,
                self._latency_card,
                self._topics_card,
                self._endpoints_card,
            )
        ):
            summary.addWidget(card, 0, column)
            summary.setColumnStretch(column, 1)
        root.addLayout(summary)

        details = QTabWidget()
        details.setObjectName("DiagnosticsDetails")
        self._topic_table = self._make_table(
            ["Estado", "Tópico", "Frequência", "Idade", "Latência"]
        )
        self._topic_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self._topic_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        for column in (2, 3, 4):
            self._topic_table.horizontalHeader().setSectionResizeMode(
                column, QHeaderView.ResizeToContents
            )
        self._topic_table.setToolTip(
            "Frequência em janela móvel de 5 s. Tópicos orientados a eventos podem ficar "
            "sem atualização sem indicar falha."
        )

        self._endpoint_table = self._make_table(["Tipo", "Estado", "Endpoint"])
        self._endpoint_table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeToContents
        )
        self._endpoint_table.horizontalHeader().setSectionResizeMode(
            1, QHeaderView.ResizeToContents
        )
        self._endpoint_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        details.addTab(self._topic_table, "Tópicos")
        details.addTab(self._endpoint_table, "Serviços e actions")
        root.addWidget(details, 1)

        error_box = QGroupBox("Último erro desta sessão")
        error_lay = QHBoxLayout(error_box)
        error_text = QVBoxLayout()
        self._error_meta = QLabel("Nenhum erro registrado")
        self._error_meta.setObjectName("MutedLabel")
        self._error_message = QLabel("—")
        self._error_message.setWordWrap(True)
        self._error_message.setTextInteractionFlags(Qt.TextSelectableByMouse)
        error_text.addWidget(self._error_meta)
        error_text.addWidget(self._error_message)
        error_lay.addLayout(error_text, 1)
        self._btn_clear_error = QPushButton("Limpar")
        self._btn_clear_error.clicked.connect(self.clear_error)
        self._btn_clear_error.setEnabled(False)
        error_lay.addWidget(self._btn_clear_error)
        root.addWidget(error_box)

        self.update_snapshot(self._bridge.diagnostic_snapshot())

    @staticmethod
    def _summary_card(
        title: str, value: str, hint: str
    ) -> tuple[QLabel, QLabel, QFrame]:
        card = QFrame()
        card.setObjectName("DiagnosticsCard")
        card.setProperty("state", "neutral")
        lay = QVBoxLayout(card)
        lay.setContentsMargins(12, 9, 12, 9)
        lay.setSpacing(2)
        title_label = QLabel(title)
        title_label.setObjectName("DashboardKey")
        value_label = QLabel(value)
        value_label.setObjectName("DashboardValue")
        hint_label = QLabel(hint)
        hint_label.setObjectName("DashboardHint")
        lay.addWidget(title_label)
        lay.addWidget(value_label)
        lay.addWidget(hint_label)
        return value_label, hint_label, card

    @staticmethod
    def _make_table(headers: list[str]) -> QTableWidget:
        table = QTableWidget(0, len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.verticalHeader().setVisible(False)
        table.verticalHeader().setDefaultSectionSize(30)
        table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        table.setSelectionBehavior(QAbstractItemView.SelectRows)
        table.setAlternatingRowColors(True)
        table.setSortingEnabled(False)
        return table

    def refresh_now(self) -> None:
        self.update_snapshot(self._bridge.diagnostic_snapshot())

    def update_snapshot(self, snapshot: dict[str, object]) -> None:
        """Refresh every diagnostic surface from one immutable-style snapshot."""
        self._last_snapshot = dict(snapshot or {})
        connected = bool(self._last_snapshot.get("connected", False))
        heartbeat = dict(self._last_snapshot.get("heartbeat", {}) or {})
        topics = list(self._last_snapshot.get("topics", []) or [])
        services = list(self._last_snapshot.get("services", []) or [])
        actions = list(self._last_snapshot.get("actions", []) or [])

        alive = bool(heartbeat.get("alive", False))
        age_ms = heartbeat.get("age_ms")
        hz = heartbeat.get("hz")
        if alive:
            self._heartbeat_value.setText(
                f"OK · {self._format_hz(hz)}" if hz is not None else "OK"
            )
            self._heartbeat_hint.setText(f"Última mensagem há {self._format_ms(age_ms)}")
            self._set_card_state(self._heartbeat_card, "ok")
        elif connected and heartbeat.get("received"):
            self._heartbeat_value.setText(f"EXPIRADO · {self._format_ms(age_ms)}")
            timeout = self._format_ms(heartbeat.get("timeout_ms"))
            self._heartbeat_hint.setText(f"Idade acima do limite de {timeout}")
            self._set_card_state(self._heartbeat_card, "error")
        elif connected:
            self._heartbeat_value.setText("AGUARDANDO")
            self._heartbeat_hint.setText("Nenhuma mensagem /arm_status recebida")
            self._set_card_state(self._heartbeat_card, "warn")
        else:
            self._heartbeat_value.setText("DESCONECTADO")
            self._heartbeat_hint.setText("Bridge ROS não iniciada")
            self._set_card_state(self._heartbeat_card, "error")

        latency = heartbeat.get("latency_ms")
        self._latency_value.setText(self._format_ms(latency))
        self._latency_hint.setText(
            "Calculada por header.stamp" if latency is not None else "Timestamp não disponível"
        )
        self._set_card_state(
            self._latency_card,
            "ok" if latency is not None and abs(float(latency)) <= 250.0 else "neutral",
        )

        active_topics = sum(1 for row in topics if self._topic_state(row)[0] == "ATIVO")
        self._topics_value.setText(f"{active_topics} / {len(topics)}")
        self._set_card_state(
            self._topics_card,
            "ok" if active_topics else ("warn" if connected else "error"),
        )

        endpoints = services + actions
        ready_endpoints = sum(1 for row in endpoints if bool(row.get("ready", False)))
        self._endpoints_value.setText(f"{ready_endpoints} / {len(endpoints)}")
        endpoint_state = "neutral"
        if endpoints:
            endpoint_state = "ok" if ready_endpoints == len(endpoints) else "warn"
        elif not connected:
            endpoint_state = "error"
        self._set_card_state(self._endpoints_card, endpoint_state)

        self._fill_topics(topics)
        self._fill_endpoints(services, actions)
        captured = self._last_snapshot.get("captured_at")
        if captured is not None:
            stamp = datetime.fromtimestamp(float(captured)).strftime("%H:%M:%S")
            self._btn_refresh.setToolTip(f"Última coleta: {stamp}. Clique para atualizar agora.")

    def _fill_topics(self, topics: list[dict[str, object]]) -> None:
        self._topic_table.setRowCount(len(topics))
        for row_index, row in enumerate(topics):
            state, color = self._topic_state(row)
            values = (
                state,
                str(row.get("name", "")),
                self._format_hz(row.get("hz")),
                self._format_ms(row.get("age_ms")),
                self._format_ms(row.get("latency_ms")),
            )
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                if column == 0:
                    item.setForeground(color)
                if column >= 2:
                    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self._topic_table.setItem(row_index, column, item)

    def _fill_endpoints(
        self,
        services: list[dict[str, object]],
        actions: list[dict[str, object]],
    ) -> None:
        rows = [("Serviço", item) for item in services] + [
            ("Action", item) for item in actions
        ]
        self._endpoint_table.setRowCount(len(rows))
        for row_index, (kind, row) in enumerate(rows):
            ready = bool(row.get("ready", False))
            values = (kind, "DISPONÍVEL" if ready else "INDISPONÍVEL", str(row.get("name", "")))
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                if column == 1:
                    item.setForeground(_OK if ready else _ERROR)
                self._endpoint_table.setItem(row_index, column, item)

    @staticmethod
    def _topic_state(row: dict[str, object]) -> tuple[str, QColor]:
        if not bool(row.get("received", False)):
            return "SEM DADOS", _MUTED
        age = row.get("age_ms")
        if age is not None and float(age) > 3000.0:
            return "SEM ATUALIZAÇÃO", _WARN
        return "ATIVO", _OK

    @staticmethod
    def _format_hz(value: object) -> str:
        return "N/D" if value is None else f"{float(value):.1f} Hz"

    @staticmethod
    def _format_ms(value: object) -> str:
        if value is None:
            return "N/D"
        number = float(value)
        if abs(number) >= 1000.0:
            return f"{number / 1000.0:.1f} s"
        return f"{number:.1f} ms"

    @staticmethod
    def _set_card_state(card: QFrame, state: str) -> None:
        card.setProperty("state", state)
        card.style().unpolish(card)
        card.style().polish(card)

    def record_error(self, source: str, message: str) -> None:
        """Keep the latest service, action or ROS bridge error visible."""
        now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
        self._error_meta.setText(f"{now} · {source}")
        self._error_message.setText(str(message) or "Erro sem mensagem")
        self._error_message.setStyleSheet("color: #ff9aa7;")
        self._btn_clear_error.setEnabled(True)

    def clear_error(self) -> None:
        self._error_meta.setText("Nenhum erro registrado")
        self._error_message.setText("—")
        self._error_message.setStyleSheet("")
        self._btn_clear_error.setEnabled(False)
