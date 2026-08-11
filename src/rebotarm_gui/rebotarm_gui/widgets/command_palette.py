"""Command palette (Ctrl+K).

A quick-jump dialog modelled on VS Code's palette: type to fuzzy-filter a flat
list of every navigable page and a handful of global actions, then Enter to
run the selected one. Complements the Ctrl+1..9 page shortcuts, which only
cover the primary nav.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import (
    QDialog,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QVBoxLayout,
    QWidget,
)


@dataclass(frozen=True)
class Command:
    id: str
    title: str
    group: str
    hint: str = ""
    callback: Callable[[], None] | None = None


class CommandPalette(QDialog):
    """Modal-free filterable launcher over a list of :class:`Command`."""

    command_activated = Signal(str)

    def __init__(self, commands: list[Command], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Paleta de comandos")
        self.setObjectName("CommandPalette")
        self.setWindowFlags(
            Qt.Dialog | Qt.FramelessWindowHint | Qt.WindowCloseButtonHint
        )
        self.setMinimumWidth(480)
        self._commands = list(commands)

        self._search = QLineEdit(self)
        self._search.setObjectName("PaletteSearch")
        self._search.setPlaceholderText(
            "Digite para buscar página ou ação (Esc para fechar)…"
        )
        self._search.setClearButtonEnabled(True)
        self._search.textChanged.connect(self._on_search_changed)

        self._hint = QLabel(
            "↑↓ navegar · Enter executar · Esc fechar", self
        )
        self._hint.setObjectName("MutedLabel")

        self._list = QListWidget(self)
        self._list.setObjectName("PaletteList")
        self._list.itemActivated.connect(self._on_item_activated)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 12, 12, 10)
        lay.setSpacing(6)
        lay.addWidget(self._search)
        lay.addWidget(self._list, 1)
        lay.addWidget(self._hint)

        self._populate("")

    # ------------------------------------------------------------------
    def rebuild(self, commands: list[Command]) -> None:
        self._commands = list(commands)
        self._populate(self._search.text())

    def _populate(self, query: str) -> None:
        self._list.clear()
        scored = self._rank(query)
        for cmd, _score in scored:
            item = QListWidgetItem(self._format_label(cmd))
            item.setData(Qt.UserRole, cmd.id)
            item.setToolTip(cmd.hint or cmd.title)
            self._list.addItem(item)
        if self._list.count():
            self._list.setCurrentRow(0)

    def _format_label(self, cmd: Command) -> str:
        if cmd.group:
            return f"{cmd.group}  ›  {cmd.title}"
        return cmd.title

    def _rank(self, query: str) -> list[tuple[Command, int]]:
        q = query.strip().lower()
        if not q:
            return [(cmd, 0) for cmd in self._commands]
        results: list[tuple[Command, int]] = []
        for cmd in self._commands:
            haystack = f"{cmd.title} {cmd.group} {cmd.hint}".lower()
            score = _fuzzy_score(q, haystack)
            if score is not None:
                results.append((cmd, score))
        results.sort(key=lambda pair: (-pair[1], pair[0].title.lower()))
        return results

    # ------------------------------------------------------------------
    def _on_search_changed(self, text: str) -> None:
        self._populate(text)

    def _on_item_activated(self, item: QListWidgetItem) -> None:
        cmd_id = str(item.data(Qt.UserRole))
        self.command_activated.emit(cmd_id)
        for cmd in self._commands:
            if cmd.id == cmd_id and cmd.callback is not None:
                cmd.callback()
                break
        self.accept()

    def keyPressEvent(self, event: QKeyEvent) -> None:  # noqa: N802 (Qt naming)
        key = event.key()
        if key in (Qt.Key_Return, Qt.Key_Enter):
            item = self._list.currentItem()
            if item is not None:
                self._on_item_activated(item)
            return
        if key == Qt.Key_Escape:
            self.reject()
            return
        if key in (Qt.Key_Down, Qt.Key_Up) and not self._list.count():
            return
        super().keyPressEvent(event)

    def showEvent(self, event) -> None:  # noqa: N802 (Qt naming)
        super().showEvent(event)
        self._search.clear()
        self._search.setFocus()


def _fuzzy_score(needle: str, haystack: str) -> int | None:
    """Return a relevance score (>0) or ``None`` when not a subsequence match.

    Prefers contiguous prefixes and exact words. The exact substring gets the
    highest weight so common queries ("prog", "vis", "pose") rank first.
    """
    if needle in haystack:
        # earlier substring position => higher score
        pos = haystack.index(needle)
        score = 1000 - pos
        # bonus for word-boundary start
        if pos == 0 or haystack[pos - 1] in " ›":
            score += 200
        return score
    # subsequence fallback: every char of needle must appear in order
    it = iter(haystack)
    if all(char in it for char in needle):
        return 10
    return None
