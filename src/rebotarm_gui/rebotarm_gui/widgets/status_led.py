"""Colored status LED widget."""

from __future__ import annotations

from PySide6.QtCore import QSize
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QWidget


class StatusLed(QWidget):
    """A small circular indicator that changes color by state."""

    _COLORS = {
        "off": "#444b54",
        "green": "#3ddc84",
        "amber": "#ffb300",
        "red": "#ff5252",
        "blue": "#3f97e3",
    }

    def __init__(self, color: str = "off", size: int = 14, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._color_key = color
        self._size = size
        self.setFixedSize(QSize(self._size, self._size))

    def set_color(self, color: str) -> None:
        if color != self._color_key:
            self._color_key = color
            self.update()

    def sizeHint(self) -> QSize:  # noqa: N802 - Qt API
        return QSize(self._size, self._size)

    def paintEvent(self, _event) -> None:  # noqa: N802 - Qt API
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        base = QColor(self._COLORS.get(self._color_key, self._COLORS["off"]))
        painter.setBrush(base)
        painter.setPen(QColor(0, 0, 0, 80))
        painter.drawEllipse(1, 1, self._size - 2, self._size - 2)
        # subtle highlight
        painter.setPen(QColor(255, 255, 255, 40))
        painter.drawEllipse(3, 3, max(2, self._size - 8), max(2, self._size - 8))
        painter.end()