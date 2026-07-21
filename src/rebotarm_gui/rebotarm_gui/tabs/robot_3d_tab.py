"""Robot 3D tab: real-time skeleton view of the arm posture.

Renders the URDF kinematic chain (base_link -> gripper_link) with a custom
QPainter widget — no Qt3D/matplotlib involved. The camera supports orbiting
(left-drag: yaw/pitch) and zooming (mouse wheel). Redraws are throttled to
~10 Hz so a 20+ Hz joint-state stream does not flood the GUI thread.
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Sequence, Tuple

from PySide6.QtCore import QPointF, QTimer, Qt
from PySide6.QtGui import QColor, QMouseEvent, QPainter, QPaintEvent, QPen, QWheelEvent
from PySide6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from .. import kinematics
from ..widgets.layout_utils import fit_button_text

Vec3 = Tuple[float, float, float]

# Theme-aligned palette (dark.qss).
_COLOR_BG = QColor("#121417")
_COLOR_GRID = QColor("#2b3540")
_COLOR_LINK = QColor("#48d5b5")
_COLOR_JOINT = QColor("#e6edf3")
_COLOR_TCP = QColor("#f0b429")
_COLOR_TEXT = QColor("#8b98a5")

_WAITING_TEXT = "Aguardando /robot_description..."

# Redraw throttle (~10 Hz).
_REDRAW_INTERVAL_S = 0.1


class Robot3dView(QWidget):
    """Custom-painted 3D wireframe view of the arm skeleton.

    World points are rotated by the camera yaw (around Z) and pitch (around
    X), then projected with a mild perspective (scale factor inversely
    proportional to depth). Left-drag orbits the camera; the wheel zooms.
    """

    _DEFAULT_YAW = 0.6
    _DEFAULT_PITCH = 0.35
    _DEFAULT_ZOOM = 1.0
    _MIN_PITCH = -1.5
    _MAX_PITCH = 1.5
    _MIN_ZOOM = 0.3
    _MAX_ZOOM = 6.0
    _CAMERA_DISTANCE = 3.0  # m; mild perspective, arm stays well inside
    _GRID_HALF_EXTENT = 0.6  # m
    _GRID_STEP = 0.2  # m

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._yaw = self._DEFAULT_YAW
        self._pitch = self._DEFAULT_PITCH
        self._zoom = self._DEFAULT_ZOOM
        self._points: Optional[List[Vec3]] = None
        self._message: str = _WAITING_TEXT
        self._drag_last: Optional[QPointF] = None
        self.setMinimumSize(320, 240)
        self.setMouseTracking(False)

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------
    def set_skeleton(self, points: Sequence[Vec3]) -> None:
        """Set the ordered frame origins (root -> TCP) to draw."""
        self._points = [(float(p[0]), float(p[1]), float(p[2])) for p in points]
        self._message = ""
        self.update()

    def show_message(self, text: str) -> None:
        """Clear the skeleton and show a centered status message."""
        self._points = None
        self._message = text
        self.update()

    def reset_view(self) -> None:
        """Restore the default camera yaw/pitch/zoom."""
        self._yaw = self._DEFAULT_YAW
        self._pitch = self._DEFAULT_PITCH
        self._zoom = self._DEFAULT_ZOOM
        self.update()

    # ------------------------------------------------------------------
    # camera / projection
    # ------------------------------------------------------------------
    def _project(self, p: Vec3) -> Optional[QPointF]:
        """Project a world point to widget coordinates (None if degenerate)."""
        cy, sy = math.cos(self._yaw), math.sin(self._yaw)
        cp, sp = math.cos(self._pitch), math.sin(self._pitch)
        # Orbit: yaw around world Z, then pitch around the rotated X axis.
        x1 = cy * p[0] - sy * p[1]
        y1 = sy * p[0] + cy * p[1]
        y2 = cp * y1 - sp * p[2]
        z2 = sp * y1 + cp * p[2]
        depth = self._CAMERA_DISTANCE - y2
        if depth < 0.2:
            depth = 0.2
        scale = min(self.width(), self.height()) * 0.6 * self._zoom
        factor = self._CAMERA_DISTANCE / depth
        sx = self.width() * 0.5 + x1 * scale * factor
        sy_ = self.height() * 0.5 - z2 * scale * factor
        return QPointF(sx, sy_)

    # ------------------------------------------------------------------
    # Qt events
    # ------------------------------------------------------------------
    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), _COLOR_BG)

        self._draw_ground_grid(painter)

        if not self._points:
            painter.setPen(QPen(_COLOR_TEXT))
            painter.drawText(self.rect(), Qt.AlignCenter, self._message)
            painter.end()
            return

        projected = [self._project(p) for p in self._points]

        # Links: thick teal segments between consecutive frame origins.
        link_pen = QPen(_COLOR_LINK, 4, Qt.SolidLine, Qt.RoundCap)
        painter.setPen(link_pen)
        for a, b in zip(projected, projected[1:]):
            if a is not None and b is not None:
                painter.drawLine(a, b)

        # Joints: dots at every frame origin except the TCP.
        painter.setPen(QPen(_COLOR_JOINT.darker(130), 1))
        painter.setBrush(_COLOR_JOINT)
        for pt in projected[:-1]:
            if pt is not None:
                painter.drawEllipse(pt, 4.0, 4.0)

        # TCP: highlighted amber marker + label.
        tcp = projected[-1]
        if tcp is not None:
            painter.setPen(QPen(_COLOR_TCP.darker(140), 1))
            painter.setBrush(_COLOR_TCP)
            painter.drawEllipse(tcp, 6.0, 6.0)
            painter.setPen(QPen(_COLOR_TCP))
            painter.drawText(tcp + QPointF(10.0, -10.0), "TCP")

        painter.end()

    def _draw_ground_grid(self, painter: QPainter) -> None:
        painter.setPen(QPen(_COLOR_GRID, 1))
        extent = self._GRID_HALF_EXTENT
        step = self._GRID_STEP
        n = int(round(extent / step))
        for i in range(-n, n + 1):
            c = i * step
            a = self._project((c, -extent, 0.0))
            b = self._project((c, extent, 0.0))
            if a is not None and b is not None:
                painter.drawLine(a, b)
            a = self._project((-extent, c, 0.0))
            b = self._project((extent, c, 0.0))
            if a is not None and b is not None:
                painter.drawLine(a, b)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.LeftButton:
            self._drag_last = event.position()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self._drag_last is not None and event.buttons() & Qt.LeftButton:
            pos = event.position()
            dx = pos.x() - self._drag_last.x()
            dy = pos.y() - self._drag_last.y()
            self._drag_last = pos
            self._yaw += dx * 0.01
            self._pitch = max(self._MIN_PITCH, min(self._MAX_PITCH, self._pitch + dy * 0.01))
            self.update()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.LeftButton:
            self._drag_last = None
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event: QWheelEvent) -> None:
        delta = event.angleDelta().y()
        if delta:
            factor = 1.15 if delta > 0 else 1.0 / 1.15
            self._zoom = max(self._MIN_ZOOM, min(self._MAX_ZOOM, self._zoom * factor))
            self.update()
        event.accept()


class Robot3dTab(QWidget):
    """Real-time 3D visualization of the arm posture (FK over the URDF)."""

    def __init__(self, bridge, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._bridge = bridge

        self._chain: Optional[List[kinematics.ChainJoint]] = None
        self._chain_urdf: Optional[str] = None

        self._last_redraw = 0.0  # monotonic timestamp of the last FK redraw
        self._pending_msg = None  # latest joint state skipped by the throttle
        self._redraw_timer = QTimer(self)
        self._redraw_timer.setSingleShot(True)
        self._redraw_timer.timeout.connect(self._flush_pending)

        # ---- 3D viewport ----
        self._view = Robot3dView()

        # ---- Bottom bar: TCP pose + camera reset ----
        self._lbl_tcp = QLabel("TCP  x: --- m   y: --- m   z: --- m")
        self._lbl_tcp.setObjectName("MetricValue")

        self._btn_reset_view = QPushButton("Resetar vista")
        self._btn_reset_view.setMinimumHeight(40)
        self._btn_reset_view.clicked.connect(self._view.reset_view)
        fit_button_text(self._btn_reset_view)

        bar = QHBoxLayout()
        bar.setSpacing(14)
        bar.addWidget(self._lbl_tcp, 1)
        bar.addWidget(self._btn_reset_view)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(12)
        lay.addWidget(self._view, 1)
        lay.addLayout(bar)

    # ------------------------------------------------------------------
    def update_joint_state(self, msg) -> None:
        """Joint-state slot (same signature as the other tabs).

        Throttled to ~10 Hz: the first update draws immediately; bursts keep
        only the newest message, flushed by a single-shot timer.
        """
        now = time.monotonic()
        if now - self._last_redraw >= _REDRAW_INTERVAL_S:
            self._redraw(msg)
        else:
            self._pending_msg = msg
            if not self._redraw_timer.isActive():
                remaining_ms = int((_REDRAW_INTERVAL_S - (now - self._last_redraw)) * 1000)
                self._redraw_timer.start(max(remaining_ms, 1))

    # ------------------------------------------------------------------
    def _flush_pending(self) -> None:
        if self._pending_msg is not None:
            msg = self._pending_msg
            self._pending_msg = None
            self._redraw(msg)

    def _redraw(self, msg) -> None:
        self._last_redraw = time.monotonic()

        node = self._bridge.node
        urdf = node.robot_description if node is not None else None
        if not urdf:
            self._view.show_message(_WAITING_TEXT)
            self._set_tcp_label(None)
            return

        # Rebuild the kinematic chain only when the URDF changes.
        if urdf != self._chain_urdf:
            try:
                self._chain = kinematics.load_chain(urdf)
            except Exception:
                self._chain = None
            self._chain_urdf = urdf

        if not self._chain:
            self._view.show_message("Cadeia cinemática indisponível no URDF.")
            self._set_tcp_label(None)
            return

        q = {name: float(pos) for name, pos in zip(msg.name, msg.position)}
        try:
            frames = kinematics.fk(self._chain, q)
        except Exception:
            self._view.show_message("Falha ao calcular a cinemática direta.")
            self._set_tcp_label(None)
            return

        points = [tuple(map(float, t[:3, 3])) for t in frames]
        self._view.set_skeleton(points)
        self._set_tcp_label(points[-1] if points else None)

    def _set_tcp_label(self, point: Optional[Vec3]) -> None:
        if point is None:
            self._lbl_tcp.setText("TCP  x: --- m   y: --- m   z: --- m")
        else:
            self._lbl_tcp.setText(
                f"TCP  x: {point[0]:.3f} m   y: {point[1]:.3f} m   z: {point[2]:.3f} m"
            )
