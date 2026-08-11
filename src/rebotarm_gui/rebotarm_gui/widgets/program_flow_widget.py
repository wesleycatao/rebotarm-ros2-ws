"""Editable visual program flowchart widget (TMflow-style blocks).

This widget renders a motion :class:`Program` as a vertical flow of rounded
blocks connected by arrows.  Leaves and compound steps (``LoopStep`` /
``IfStep``) are represented as blocks.  The widget is now editable:

* click selects a top-level step;
* drag a block to reorder top-level steps;
* arrow keys navigate between top-level steps;
* ``+``/``-`` and Ctrl+wheel zoom;
* invalid steps are drawn with a red warning stroke.

All mutations are emitted as requests and applied by the owner
(:class:`MotionTab`) so the ``Program`` data model remains the single source of
truth and undo/redo stay centralized.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from PySide6.QtCore import QRect, Qt, Signal, QPoint
from PySide6.QtGui import QColor, QCursor, QKeyEvent, QMouseEvent, QPainter, QPainterPath, QPen, QWheelEvent
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
_COLOR_SELECTED = QColor("#f5d742")
_COLOR_INVALID = QColor("#e74c3c")

# layout metrics (px, logical coordinates before zoom)
_MARGIN = 16
_GAP = 26
_LEAF_H = 40
_HEADER_H = 28
_PAD = 10
_INDENT = 16
_BRANCH_LABEL_H = 18
_RADIUS = 8
_TEXT_PAD = 14
_MIN_WIDTH = 340
_EMPTY_H = 140

_INVALID_EMPTY_IDS = {"", "step", "etapa"}


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
    """Painted canvas with mouse/keyboard selection and drag-to-reorder."""

    step_selected = Signal(int)              # root index selected
    reorder_requested = Signal(int, int)     # (source_root_index, target_root_index)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._program: Optional[Program] = None
        self._blocks: List[_Block] = []
        self._labels: List[Tuple[QRect, str]] = []
        self._connectors: List[Tuple[int, int, int]] = []  # (x, y1, y2)
        self._active_root_index: Optional[int] = None
        self._selected_root_index: Optional[int] = None
        self._edit_enabled = True
        self._relayouting = False
        self._zoom = 1.0
        self._drag_index: Optional[int] = None
        self._drag_hover_index: Optional[int] = None
        self._drag_start_pos: Optional[QPoint] = None
        self._invalid_indices: set[int] = set()
        self._highlighted_index: Optional[int] = None
        self.setMinimumWidth(int(_MIN_WIDTH * self._zoom))
        self.setFixedHeight(int(_EMPTY_H * self._zoom))
        self.setAccessibleName("Editor visual do programa em diagrama de blocos")
        self.setToolTip("Clique para selecionar, arraste para reordenar etapas, setas para navegar, +/- para zoom")

    # ---- public ----
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

    def set_selected_index(self, index: Optional[int]) -> None:
        index = None if index is None else int(index)
        if index == self._selected_root_index:
            return
        self._selected_root_index = index
        self._scroll_to_selected()
        self.update()

    def set_edit_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled == self._edit_enabled:
            return
        self._edit_enabled = enabled
        if not enabled:
            self._drag_index = None
            self._drag_hover_index = None
            self._drag_start_pos = None
            self.unsetCursor()
        self.update()

    def set_invalid_indices(self, indices: set[int]) -> None:
        self._invalid_indices = set(indices)
        self.update()

    def highlight_step(self, index: Optional[int]) -> None:
        """Brief visual highlight of a step (e.g. preflight error)."""
        self._highlighted_index = index
        self._scroll_to_index(index)
        self.update()

    def active_rect(self) -> Optional[QRect]:
        if self._active_root_index is None:
            return None
        for b in self._blocks:
            if b.root_index == self._active_root_index:
                return b.rect
        return None

    def selected_rect(self) -> Optional[QRect]:
        if self._selected_root_index is None:
            return None
        for b in self._blocks:
            if b.root_index == self._selected_root_index:
                return b.rect
        return None

    def set_zoom(self, zoom: float) -> None:
        zoom = max(0.5, min(2.0, float(zoom)))
        if zoom == self._zoom:
            return
        self._zoom = zoom
        self._relayout()
        self.update()

    def zoom(self) -> float:
        return self._zoom

    def fit_all(self) -> None:
        """Reset zoom to 1 and scroll to the top of the program."""
        self._zoom = 1.0
        self._relayout()
        self.update()
        parent = self.parent()
        if isinstance(parent, QScrollArea):
            parent.ensureVisible(0, 0, 0, 0)

    # ---- layout ----
    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
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
            logical_width = max(int(self.width() / self._zoom), _MIN_WIDTH) - 2 * _MARGIN
            x = _MARGIN
            total = _EMPTY_H
            if steps:
                _first, last = self._layout_siblings(steps, -1, x, _MARGIN, logical_width, numbered=True)
                total = last.bottom() + 1 + _MARGIN if last is not None else _EMPTY_H
            self.setMinimumWidth(int(_MIN_WIDTH * self._zoom))
            self.setFixedHeight(int(max(total, _EMPTY_H) * self._zoom))
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

    # ---- interaction ----
    def _root_block_at(self, pos: QPoint) -> Optional[_Block]:
        # pos is in widget coordinates; map to logical coordinates
        logical = self._logical_pos(pos)
        for b in reversed(self._blocks):
            if b.rect.contains(logical):
                return b
        return None

    def _logical_pos(self, pos: QPoint) -> QPoint:
        return QPoint(int(pos.x() / self._zoom), int(pos.y() / self._zoom))

    def _block_center_widget(self, b: _Block) -> QPoint:
        return QPoint(int((b.rect.center().x()) * self._zoom), int((b.rect.center().y()) * self._zoom))

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if not self._edit_enabled:
            return
        if event.button() == Qt.LeftButton:
            b = self._root_block_at(event.pos())
            if b is not None:
                self._drag_index = b.root_index
                self._drag_start_pos = event.pos()
                self.set_selected_index(b.root_index)
                self.step_selected.emit(b.root_index)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if not self._edit_enabled or self._drag_index is None:
            return
        if self._drag_start_pos is not None:
            dist = (event.pos() - self._drag_start_pos).manhattanLength()
            if dist > 6:
                self.setCursor(QCursor(Qt.DragMoveCursor))
        b = self._root_block_at(event.pos())
        hover = b.root_index if b is not None else None
        if hover != self._drag_hover_index:
            self._drag_hover_index = hover
            self.update()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if not self._edit_enabled or self._drag_index is None:
            self._drag_index = None
            self._drag_hover_index = None
            self._drag_start_pos = None
            self.unsetCursor()
            super().mouseReleaseEvent(event)
            return
        if event.button() == Qt.LeftButton:
            b = self._root_block_at(event.pos())
            target = b.root_index if b is not None else None
            source = self._drag_index
            self._drag_index = None
            self._drag_hover_index = None
            self._drag_start_pos = None
            self.unsetCursor()
            if target is not None and target != source:
                self.reorder_requested.emit(source, target)
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        b = self._root_block_at(event.pos())
        if b is not None:
            self.set_selected_index(b.root_index)
            self.step_selected.emit(b.root_index)
        super().mouseDoubleClickEvent(event)

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if not self._program or not self._program.steps:
            super().keyPressEvent(event)
            return
        steps = self._program.steps
        current = self._selected_root_index
        if current is None:
            current = self._active_root_index
        if current is None:
            current = 0 if event.key() in (Qt.Key_Down, Qt.Key_Home, Qt.Key_PageDown) else len(steps) - 1

        if event.key() == Qt.Key_Down:
            new_index = min(len(steps) - 1, current + 1)
        elif event.key() == Qt.Key_Up:
            new_index = max(0, current - 1)
        elif event.key() == Qt.Key_Home:
            new_index = 0
        elif event.key() == Qt.Key_End:
            new_index = len(steps) - 1
        elif event.key() == Qt.Key_Plus or event.key() == Qt.Key_Equal:
            self.set_zoom(self._zoom + 0.1)
            return
        elif event.key() == Qt.Key_Minus:
            self.set_zoom(self._zoom - 0.1)
            return
        elif event.key() == Qt.Key_0:
            self.fit_all()
            return
        else:
            super().keyPressEvent(event)
            return
        self.set_selected_index(new_index)
        self.step_selected.emit(new_index)

    def wheelEvent(self, event: QWheelEvent) -> None:
        if event.modifiers() & Qt.ControlModifier:
            delta = event.angleDelta().y()
            if delta > 0:
                self.set_zoom(self._zoom + 0.1)
            elif delta < 0:
                self.set_zoom(self._zoom - 0.1)
            return
        super().wheelEvent(event)

    # ---- painting ----
    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), _COLOR_BG)
        p.scale(self._zoom, self._zoom)
        if not self._blocks:
            p.setPen(_COLOR_MUTED)
            p.drawText(QRect(0, 0, int(self.width() / self._zoom), int(self.height() / self._zoom)),
                       Qt.AlignCenter,
                       "Programa vazio — adicione etapas na visualização em lista.")
            p.end()
            return

        for b in self._blocks:
            self._paint_block(p, b)
        self._paint_labels(p)
        self._paint_connectors(p)
        self._paint_drop_indicator(p)
        self._paint_highlight(p)
        p.end()

    def _paint_block(self, p: QPainter, b: _Block) -> None:
        active = self._active_root_index is not None and b.root_index == self._active_root_index
        selected = self._selected_root_index is not None and b.root_index == self._selected_root_index
        invalid = b.root_index in self._invalid_indices

        if active:
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(255, 255, 255, 36))
            p.drawRoundedRect(b.rect.adjusted(-4, -4, 4, 4), _RADIUS + 3, _RADIUS + 3)

        p.setBrush(_COLOR_CONTAINER if b.container else _COLOR_BLOCK)
        if invalid:
            pen = QPen(_COLOR_INVALID)
            pen.setWidth(3)
        elif active or selected:
            pen = QPen(_COLOR_SELECTED if selected else _COLOR_ACTIVE)
            pen.setWidth(3 if active else 2)
        else:
            pen = QPen(b.color)
            pen.setWidth(2)
        p.setPen(pen)
        p.drawRoundedRect(b.rect, _RADIUS, _RADIUS)

        # left accent bar
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

    def _paint_drop_indicator(self, p: QPainter) -> None:
        if self._drag_hover_index is None or self._drag_index is None:
            return
        target = None
        for b in self._blocks:
            if b.root_index == self._drag_hover_index:
                target = b.rect
                break
        if target is None:
            return
        pen = QPen(_COLOR_SELECTED)
        pen.setWidth(3)
        pen.setStyle(Qt.DashLine)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(target.adjusted(-6, -6, 6, 6), _RADIUS + 4, _RADIUS + 4)

    def _paint_highlight(self, p: QPainter) -> None:
        if self._highlighted_index is None:
            return
        for b in self._blocks:
            if b.root_index == self._highlighted_index:
                p.setPen(Qt.NoPen)
                p.setBrush(QColor(231, 76, 60, 80))
                p.drawRoundedRect(b.rect.adjusted(-8, -8, 8, 8), _RADIUS + 6, _RADIUS + 6)
                return

    def _scroll_to_selected(self) -> None:
        rect = self.selected_rect()
        if rect is None:
            return
        parent = self.parent()
        if isinstance(parent, QScrollArea):
            parent.ensureVisible(0, int(rect.center().y() * self._zoom), 0, 60)

    def _scroll_to_index(self, index: Optional[int]) -> None:
        if index is None:
            return
        for b in self._blocks:
            if b.root_index == index:
                parent = self.parent()
                if isinstance(parent, QScrollArea):
                    parent.ensureVisible(0, int(b.rect.center().y() * self._zoom), 0, 60)
                return


class ProgramFlowWidget(QWidget):
    """Scrollable editable flowchart view of a motion :class:`Program`."""

    step_selected = Signal(int)        # root index selected in the diagram
    reorder_requested = Signal(int, int)  # source root index, target root index

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._canvas = _FlowCanvas()
        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QScrollArea.NoFrame)
        self._scroll.setAccessibleName("Rolagem do diagrama de blocos do programa")
        self._scroll.setWidget(self._canvas)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self._scroll)
        self.setAccessibleName("Editor visual do programa em diagrama de blocos")

        # Re-emit canvas signals so external owners connect to the public widget.
        self._canvas.step_selected.connect(self.step_selected)
        self._canvas.reorder_requested.connect(self.reorder_requested)

    def set_program(self, program: Optional[Program]) -> None:
        self._canvas.set_program(program)

    def set_active_root_index(self, index: Optional[int]) -> None:
        """Highlight the top-level step ``index`` currently executing."""
        self._canvas.set_active_root_index(index)

    def set_selected_index(self, index: Optional[int]) -> None:
        """Synchronize diagram selection with the list editor."""
        self._canvas.set_selected_index(index)

    def set_edit_enabled(self, enabled: bool) -> None:
        """Block or allow editing interactions."""
        self._canvas.set_edit_enabled(enabled)

    def set_invalid_indices(self, indices: set[int]) -> None:
        """Mark root step indices as invalid (e.g. failed validation)."""
        self._canvas.set_invalid_indices(indices)

    def highlight_step(self, index: Optional[int]) -> None:
        """Visually emphasize a step (e.g. error from preflight)."""
        self._canvas.highlight_step(index)

    def fit_all(self) -> None:
        """Reset zoom and scroll to the top of the program."""
        self._canvas.fit_all()

    def zoom(self) -> float:
        return self._canvas.zoom()

    def set_zoom(self, zoom: float) -> None:
        self._canvas.set_zoom(zoom)
