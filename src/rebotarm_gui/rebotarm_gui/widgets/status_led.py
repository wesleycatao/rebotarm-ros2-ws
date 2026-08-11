"""Colored status LED widget.

The indicator encodes state with BOTH color and shape so the meaning is still
legible to color-blind operators: green = filled circle, amber = triangle,
red = square, blue = diamond, off = hollow ring.
"""

from __future__ import annotations

from typing import Mapping

from PySide6.QtCore import QPointF, QSize, Qt
from PySide6.QtGui import QColor, QPainter, QPolygonF
from PySide6.QtWidgets import QWidget

_SHAPES = ("circle", "triangle", "square", "diamond", "ring")

_STATE_LABELS = {
    "off": "inativo ou sem sinal",
    "green": "normal",
    "amber": "atenção ou aguardando",
    "red": "erro",
    "blue": "informação",
}


class StatusLed(QWidget):
    """A small indicator that changes color (and shape) by state."""

    _COLORS: Mapping[str, str] = {
        "off": "#444b54",
        "green": "#3ddc84",
        "amber": "#ffb300",
        "red": "#ff5252",
        "blue": "#3f97e3",
    }
    _SHAPES: Mapping[str, str] = {
        "off": "ring",
        "green": "circle",
        "amber": "triangle",
        "red": "square",
        "blue": "diamond",
    }

    def __init__(self, color: str = "off", size: int = 14, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._color_key = color
        self._shape_key = self._SHAPES.get(color, "ring")
        self._size = size
        # Slightly larger so the triangle/square read at the same visual weight.
        self.setFixedSize(QSize(self._size, self._size))
        self.setAccessibleName("Indicador de estado")
        self._update_text_alternative()

    def set_color(self, color: str) -> None:
        shape = self._SHAPES.get(color, "ring")
        if color != self._color_key or shape != self._shape_key:
            self._color_key = color
            self._shape_key = shape
            self._update_text_alternative()
            self.update()

    def set_shape(self, shape: str) -> None:
        if shape in _SHAPES and shape != self._shape_key:
            self._shape_key = shape
            self.update()

    def _default_tooltip(self) -> str:
        return f"Estado: {_STATE_LABELS.get(self._color_key, self._color_key)}"

    def _update_text_alternative(self) -> None:
        text = self._default_tooltip()
        self.setToolTip(text)
        self.setAccessibleDescription(text)

    def sizeHint(self) -> QSize:  # noqa: N802 - Qt API
        return QSize(self._size, self._size)

    def paintEvent(self, _event) -> None:  # noqa: N802 - Qt API
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        base = QColor(self._COLORS.get(self._color_key, self._COLORS["off"]))
        painter.setBrush(base)
        painter.setPen(QColor(0, 0, 0, 90))
        m = self._size
        self._draw_shape(painter, self._shape_key, m)
        # subtle highlight
        painter.setPen(QColor(255, 255, 255, 55))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(m / 2, m / 2), max(1.0, m / 2 - 3), max(1.0, m / 2 - 3))
        painter.end()

    def _draw_shape(self, painter: QPainter, shape: str, m: int) -> None:
        pad = 2
        if shape == "circle":
            painter.drawEllipse(pad, pad, m - 2 * pad, m - 2 * pad)
            return
        if shape == "square":
            painter.drawRoundedRect(pad, pad, m - 2 * pad, m - 2 * pad, 2, 2)
            return
        if shape == "ring":
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QColor(self._COLORS.get(self._color_key, self._COLORS["off"])))
            painter.drawEllipse(pad, pad, m - 2 * pad, m - 2 * pad)
            return
        if shape == "triangle":
            poly = QPolygonF([
                QPointF(m / 2, pad + 1),
                QPointF(m - pad - 1, m - pad - 1),
                QPointF(pad + 1, m - pad - 1),
            ])
            painter.drawPolygon(poly)
            return
        if shape == "diamond":
            c = m / 2
            r = m / 2 - pad
            poly = QPolygonF([
                QPointF(c, c - r),
                QPointF(c + r, c),
                QPointF(c, c + r),
                QPointF(c - r, c),
            ])
            painter.drawPolygon(poly)
            return
