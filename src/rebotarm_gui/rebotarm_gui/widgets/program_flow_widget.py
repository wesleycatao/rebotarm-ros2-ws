"""Read-only program flowchart widget (TMflow-style visualization).

Renders a motion :class:`Program` as a vertical flow of rounded blocks
connected by arrows. Each block shows the step's ``short_description()``
and a border color that encodes the step category:

- movement (move/line/arc/pallet): ``#48d5b5``
- gripper: ``#d8a03d``
- waits / digital I/O: ``#7aa2ff``
- control flow (LOOP / IF-ELSE): ``#cf5fcf``

Compound steps (``LoopStep`` / ``IfStep``) are drawn as container blocks
with their child steps nested inside; ``IfStep`` shows ENTÃO / SENÃO
branches. The widget is pure visualization: no editing, no drag-and-drop.
During execution, :meth:`ProgramFlowWidget.set_active_root_index`
highlights the root step that is currently running.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from PySide6.QtCore import QRect, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QScrollArea, QVBoxLayout, QWidget

from ..motion.program import (
    KIND_ARC,
    KIND_GRIPPER,
    KIND_IF,
    KIND_LINE,
    KIND_LOOP,
    KIND_MOVE,
    KIND_OUTPUT,
    KIND_PALLET,
    KIND_WAIT,
    KIND_WAIT_SIG,
    KIND_WAIT_SM,
    IfStep,
    LoopStep,
    Program,
    Step,
)

# category colors (match the dark theme accent palette)
_COLOR_MOVE = QColor("#48d5b5")
_COLOR_GRIPPER = QColor("#d8a03d")
_COLOR_WAIT = QColor("#7aa2ff")
_COLOR_CONTROL = QColor("#cf5fcf")
_COLOR_UNKNOWN = QColor("#8a93a1")

_COLOR_BG = QColor("#12161c")
_COLOR_BLOCK = QColor("#1c232c")
_COLOR_CONTAINER = QColor("#171d25")
_COLOR_TEXT = QColor("#e6e9ee")
_COLOR_MUTED = QColor("#8a93a1")
_COLOR_CONNECTOR = QColor("#5a6472")
_COLOR_ACTIVE = QColor("#ffffff")

# layout metrics (px)
_MARGIN = 16
_GAP = 26              # vertical gap between sibling blocks (connector)
_LEAF_H = 40
_HEADER_H = 28         # container header height
_PAD = 10              # container inner padding
_INDENT = 16           # child indent inside containers
_BRANCH_LABEL_H = 18   # ENTÃO / SENÃO label height
_RADIUS = 8
_TEXT_PAD = 14         # horizontal text padding inside a block
_MIN_WIDTH = 340
_EMPTY_H = 140


def _category_color(kind: str) -> QColor:
    if kind in (KIND_MOVE, KIND_LINE, KIND_ARC, KIND_PALLET):
        return _COLOR_MOVE
    if kind == KIND_GRIPPER:
        return _COLOR_GRIPPER
    if kind in (KIND_WAIT, KIND_WAIT_SM, KIND_WAIT_SIG, KIND_OUTPUT):
        return _COLOR_WAIT
    if kind in (KIND_LOOP, KIND_IF):
        return _COLOR_CONTROL
    return _COLOR_UNKNOWN


@dataclass
class _Block:
    rect: QRect
    title: str
    color: QColor
    root_index: int
    container: bool = False


class _FlowCanvas(QWidget):
    """The actual painted surface; sized to the laid-out content."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._program: Optional[Program] = None
        self._blocks: List[_Block] = []
        self._labels: List[Tuple[QRect, str]] = []
        self._connectors: List[Tuple[int, int, int]] = []  # (x, y1, y2)
        self._active_root_index: Optional[int] = None
        self._edit_enabled = True
        self._relayouting = False
        self.setMinimumWidth(_MIN_WIDTH)
        self.setFixedHeight(_EMPTY_H)

    # ---- public (mirrors ProgramFlowWidget API) ----
    def set_program(self, program: Optional[Program]) -> None:
        self._program = program
        self._relayout()
        self.update()

    def set_active_root_index(self, index: Optional[int]) -> None:
        index = None if index is None else int(index)
        if index == self._active_root_index:
            return
        self._active_root_index = index
        self.update()

    def set_edit_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled == self._edit_enabled:
            return
        self._edit_enabled = enabled
        self.update()

    def active_rect(self) -> Optional[QRect]:
        """Bounding rect of the highlighted root block, if any."""
        if self._active_root_index is None:
            return None
        for b in self._blocks:
            if b.root_index == self._active_root_index:
                return b.rect
        return None

    # ---- layout ----
    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        # Block widths follow the available width, so relayout on resize.
        self._relayout()
        self.update()

    def _relayout(self) -> None:
        if self._relayouting:
            return
        self._relayouting = True
        try:
            self._blocks.clear()
            self._labels.clear()
            self._connectors.clear()
            steps = self._program.steps if self._program is not None else []
            x = _MARGIN
            w = max(self.width(), _MIN_WIDTH) - 2 * _MARGIN
            total = _EMPTY_H
            if steps:
                _first, last = self._layout_siblings(steps, -1, x, _MARGIN, w, numbered=True)
                total = last.bottom() + 1 + _MARGIN if last is not None else _EMPTY_H
            self.setFixedHeight(max(total, _EMPTY_H))
        finally:
            self._relayouting = False

    def _layout_siblings(
        self,
        steps: List[Step],
        root_index: int,
        x: int,
        y: int,
        w: int,
        numbered: bool = False,
    ) -> Tuple[Optional[QRect], Optional[QRect]]:
        """Lay out a sibling list vertically; returns (first, last) rects."""
        first: Optional[QRect] = None
        prev: Optional[QRect] = None
        for i, step in enumerate(steps):
            ri = i if root_index < 0 else root_index
            title = step.short_description()
            if numbered:
                title = f"{i + 1}. {title}"
            rect = self._layout_step(step, ri, x, y, w, title)
            if first is None:
                first = rect
            if prev is not None:
                self._connectors.append((x + w // 2, prev.bottom() + 1, rect.top()))
            prev = rect
            y = rect.bottom() + 1 + _GAP
        return first, prev

    def _layout_step(self, step: Step, root_index: int, x: int, y: int, w: int, title: str) -> QRect:
        color = _category_color(step.kind)
        if isinstance(step, (LoopStep, IfStep)):
            return self._layout_container(step, root_index, x, y, w, title, color)
        rect = QRect(x, y, w, _LEAF_H)
        self._blocks.append(_Block(rect, title, color, root_index, container=False))
        return rect

    def _layout_container(
        self, step: Step, root_index: int, x: int, y: int, w: int, title: str, color: QColor
    ) -> QRect:
        # Children are laid out first to know the height; the container block
        # is then inserted *before* them so it paints underneath.
        insert_pos = len(self._blocks)
        inner_x = x + _INDENT
        inner_w = w - 2 * _INDENT
        cy = y + _HEADER_H + _PAD
        if isinstance(step, LoopStep):
            _f, last = self._layout_siblings(step.steps, root_index, inner_x, cy, inner_w)
            if last is not None:
                cy = last.bottom() + 1
        elif isinstance(step, IfStep):
            cy = self._layout_branch("ENTÃO", step.then_steps, root_index, inner_x, cy, inner_w)
            if step.else_steps:
                cy = self._layout_branch("SENÃO", step.else_steps, root_index, inner_x, cy + 4, inner_w)
        bottom = cy + _PAD
        rect = QRect(x, y, w, bottom - y)
        self._blocks.insert(insert_pos, _Block(rect, title, color, root_index, container=True))
        return rect

    def _layout_branch(
        self, label: str, steps: List[Step], root_index: int, x: int, y: int, w: int
    ) -> int:
        self._labels.append((QRect(x, y, w, _BRANCH_LABEL_H), label))
        y += _BRANCH_LABEL_H + 4
        _f, last = self._layout_siblings(steps, root_index, x, y, w)
        return last.bottom() + 1 if last is not None else y

    # ---- painting ----
    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), _COLOR_BG)
        if not self._blocks:
            p.setPen(_COLOR_MUTED)
            p.drawText(self.rect(), Qt.AlignCenter,
                       "Programa vazio — adicione etapas na visualização em lista.")
            p.end()
            return
        for b in self._blocks:
            self._paint_block(p, b)
        self._paint_labels(p)
        self._paint_connectors(p)
        p.end()

    def _paint_block(self, p: QPainter, b: _Block) -> None:
        active = self._active_root_index is not None and b.root_index == self._active_root_index
        if active:
            # soft glow behind the active block
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(255, 255, 255, 36))
            p.drawRoundedRect(b.rect.adjusted(-4, -4, 4, 4), _RADIUS + 3, _RADIUS + 3)
        p.setBrush(_COLOR_CONTAINER if b.container else _COLOR_BLOCK)
        pen = QPen(_COLOR_ACTIVE if active else b.color)
        pen.setWidth(3 if active else 2)
        p.setPen(pen)
        p.drawRoundedRect(b.rect, _RADIUS, _RADIUS)
        # left accent bar keeps the category readable even when active
        p.setPen(Qt.NoPen)
        p.setBrush(b.color)
        p.drawRoundedRect(b.rect.left() + 5, b.rect.top() + 7, 4, b.rect.height() - 14, 2, 2)

        if b.container:
            header = QRect(b.rect.left(), b.rect.top(), b.rect.width(), _HEADER_H)
            font = p.font()
            font.setBold(True)
            p.setFont(font)
            p.setPen(b.color)
            self._draw_elided(p, header, b.title)
        else:
            p.setPen(_COLOR_TEXT if self._edit_enabled else _COLOR_MUTED)
            self._draw_elided(p, b.rect, b.title)

    def _draw_elided(self, p: QPainter, rect: QRect, text: str) -> None:
        elided = p.fontMetrics().elidedText(text, Qt.ElideRight, rect.width() - 2 * _TEXT_PAD)
        p.drawText(rect.adjusted(_TEXT_PAD, 0, -_TEXT_PAD, 0),
                   Qt.AlignVCenter | Qt.AlignLeft, elided)

    def _paint_labels(self, p: QPainter) -> None:
        p.setPen(_COLOR_MUTED)
        for rect, text in self._labels:
            p.drawText(rect, Qt.AlignVCenter | Qt.AlignLeft, text)

    def _paint_connectors(self, p: QPainter) -> None:
        pen = QPen(_COLOR_CONNECTOR)
        pen.setWidth(2)
        p.setPen(pen)
        for x, y1, y2 in self._connectors:
            p.drawLine(x, y1, x, y2 - 7)
            arrow = QPainterPath()
            arrow.moveTo(x - 5, y2 - 7)
            arrow.lineTo(x + 5, y2 - 7)
            arrow.lineTo(x, y2)
            arrow.closeSubpath()
            p.fillPath(arrow, _COLOR_CONNECTOR)


class ProgramFlowWidget(QWidget):
    """Scrollable, read-only flowchart view of a motion :class:`Program`."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._canvas = _FlowCanvas()
        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QScrollArea.NoFrame)
        self._scroll.setWidget(self._canvas)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self._scroll)

    def set_program(self, program: Optional[Program]) -> None:
        """Render ``program`` (may be empty; ``None`` clears the view)."""
        self._canvas.set_program(program)

    def set_active_root_index(self, index: Optional[int]) -> None:
        """Highlight the top-level step ``index`` currently executing."""
        self._canvas.set_active_root_index(index)
        rect = self._canvas.active_rect()
        if rect is not None:
            self._scroll.ensureVisible(0, rect.center().y(), 0, 60)

    def set_edit_enabled(self, enabled: bool) -> None:
        """Visual-only: dims step text while a program is running."""
        self._canvas.set_edit_enabled(enabled)
