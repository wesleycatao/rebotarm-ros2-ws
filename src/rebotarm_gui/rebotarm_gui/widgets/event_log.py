"""Filtered event log widget used by the ROS event dock.

Replaces the bare ``QTextEdit`` that grew unbounded. The widget keeps an
in-memory ring buffer (capped), supports level filtering and free-text
search, exposes a clear button and an auto-scroll toggle.

The underlying :class:`QTextEdit` is exposed as :attr:`text_edit` so existing
call sites that append to a plain text edit (e.g. the Vision tab receiving
``ros_log=``) keep working unchanged.
"""

from __future__ import annotations

from collections import deque
from typing import Iterable

from PySide6.QtGui import QColor, QTextCharFormat, QTextCursor, QTextOption
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

LEVEL_INFO = "INFO"
LEVEL_WARNING = "WARN"
LEVEL_ERROR = "ERROR"

# Order matters: the combo index maps to these keys plus "ALL".
_LEVEL_FILTERS = [
    ("all", "Todos os níveis"),
    (LEVEL_INFO, "Info"),
    (LEVEL_WARNING, "Avisos"),
    (LEVEL_ERROR, "Erros"),
]

# Render hints per level: (foreground hex, optional tag prefix).
_LEVEL_STYLE: dict[str, tuple[str, str]] = {
    LEVEL_INFO: ("#9ff0d8", "INFO"),
    LEVEL_WARNING: ("#ffd37a", "WARN"),
    LEVEL_ERROR: ("#ff9aa5", "ERRO"),
}

MAX_ENTRIES = 5000


def _infer_level(text: str) -> str:
    """Best-effort level inference for legacy callers that pass raw strings."""
    lowered = text.lower()
    if " erro" in lowered or "[erro" in lowered or "falha" in lowered or "fail" in lowered:
        return LEVEL_ERROR
    if "warn" in lowered or "aviso" in lowered or "alarme" in lowered:
        return LEVEL_WARNING
    return LEVEL_INFO


class EventLogPanel(QWidget):
    """Toolbar + read-only text surface with filtering and a capped buffer."""

    def __init__(self, text_edit: QTextEdit | None = None, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._entries: deque[tuple[str, str]] = deque(maxlen=MAX_ENTRIES)
        self._level_filter = "all"
        self._search_text = ""
        self._autoscroll = True

        self.text_edit = text_edit if text_edit is not None else QTextEdit()
        self.text_edit.setReadOnly(True)
        self.text_edit.setPlaceholderText("Os eventos ROS aparecerão aqui.")
        self.text_edit.setWordWrapMode(QTextOption.WrapAtWordBoundaryOrAnywhere)
        # Keep the monospace look already provided by the global QSS.

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(4)

        toolbar = QHBoxLayout()
        toolbar.setSpacing(6)
        toolbar.addWidget(QLabel("Filtro"))
        self._combo = QComboBox()
        for key, label in _LEVEL_FILTERS:
            self._combo.addItem(label, key)
        self._combo.currentIndexChanged.connect(self._on_combo_changed)
        toolbar.addWidget(self._combo)

        self._search = QLineEdit()
        self._search.setPlaceholderText("Buscar nos eventos…")
        self._search.setClearButtonEnabled(True)
        self._search.textChanged.connect(self._on_search_changed)
        toolbar.addWidget(self._search, 1)

        self._chk_autoscroll = QCheckBox("Auto-rolagem")
        self._chk_autoscroll.setChecked(True)
        self._chk_autoscroll.toggled.connect(self._on_autoscroll_toggled)
        toolbar.addWidget(self._chk_autoscroll)

        self._btn_clear = QPushButton("Limpar")
        self._btn_clear.clicked.connect(self.clear_log)
        toolbar.addWidget(self._btn_clear)

        root.addLayout(toolbar)
        root.addWidget(self.text_edit, 1)

        self._render()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def append(self, text: str, level: str | None = None) -> None:
        """Append a single line, inferring the level when not provided."""
        self.append_message(level or _infer_level(text), text)

    def append_message(self, level: str, text: str) -> None:
        """Append an explicit (level, text) entry and refresh the view."""
        normalised = level.upper()
        if normalised not in _LEVEL_STYLE:
            normalised = _infer_level(text)
        self._entries.append((normalised, text.rstrip("\n")))
        if self._matches(normalised, text):
            self._render_latest()
            if self._autoscroll:
                self._scroll_to_end()
            self._update_count_hint()

    def extend(self, items: Iterable[tuple[str, str]]) -> None:
        for level, text in items:
            self.append_message(level, text)

    def clear_log(self) -> None:
        self._entries.clear()
        self._render()
        self._update_count_hint()

    def entry_count(self) -> int:
        return len(self._entries)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _on_combo_changed(self, _index: int) -> None:
        self._level_filter = self._combo.currentData() or "all"
        self._render()

    def _on_search_changed(self, text: str) -> None:
        self._search_text = text.strip().lower()
        self._render()

    def _on_autoscroll_toggled(self, checked: bool) -> None:
        self._autoscroll = bool(checked)
        if self._autoscroll:
            self._scroll_to_end()

    def _matches(self, level: str, text: str) -> bool:
        if self._level_filter != "all" and level != self._level_filter:
            return False
        if self._search_text and self._search_text not in text.lower():
            return False
        return True

    def _format_for(self, level: str, text: str) -> str:
        prefix = _LEVEL_STYLE[level][1]
        return f"[{prefix}] {text}" if not text.startswith(f"[{prefix}]") else text

    def _render_latest(self) -> None:
        if not self._entries:
            return
        level, text = self._entries[-1]
        if not self._matches(level, text):
            return
        color = _LEVEL_STYLE[level][0]
        cursor = self.text_edit.textCursor()
        cursor.movePosition(QTextCursor.End)
        fmt = QTextCharFormat()
        fmt.setForeground(QColor(color))
        cursor.setCharFormat(fmt)
        line = self._format_for(level, text)
        if self.text_edit.toPlainText():
            cursor.insertText("\n" + line)
        else:
            cursor.insertText(line)

    def _render(self) -> None:
        self.text_edit.clear()
        cursor = self.text_edit.textCursor()
        first = True
        for level, text in self._entries:
            if not self._matches(level, text):
                continue
            fmt = QTextCharFormat()
            fmt.setForeground(QColor(_LEVEL_STYLE[level][0]))
            cursor.setCharFormat(fmt)
            line = self._format_for(level, text)
            cursor.insertText(("\n" if not first else "") + line)
            first = False
        if self._autoscroll:
            self._scroll_to_end()
        self._update_count_hint()

    def _scroll_to_end(self) -> None:
        sb = self.text_edit.verticalScrollBar()
        if sb is not None:
            sb.setValue(sb.maximum())
        cursor = self.text_edit.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.text_edit.setTextCursor(cursor)

    def _update_count_hint(self) -> None:
        total = len(self._entries)
        self._combo.setToolTip(f"{total} evento(s) armazenado(s) (limite {MAX_ENTRIES}).")
