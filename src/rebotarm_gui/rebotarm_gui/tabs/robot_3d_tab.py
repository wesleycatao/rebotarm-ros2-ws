"""Robot 3D tab: real-time skeleton view of the arm posture.

Renders the URDF kinematic chain (base_link -> gripper_link) with a custom
QPainter widget — no Qt3D/matplotlib involved. The camera supports orbiting
(left-drag: yaw/pitch) and zooming (mouse wheel). Redraws are throttled to
~10 Hz so a 20+ Hz joint-state stream does not flood the GUI thread.
"""

from __future__ import annotations

import math
import time
from datetime import datetime, timezone
from typing import List, Optional, Sequence, Tuple

from PySide6.QtCore import QPointF, QTimer, Qt, Signal
from PySide6.QtGui import QColor, QMouseEvent, QPainter, QPaintEvent, QPen, QWheelEvent
from PySide6.QtWidgets import (
    QAbstractItemView,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from .. import kinematics
from ..motion.cell_scene import (
    KEEP_OUT,
    OBSTACLE,
    CellObstacle,
    CellScene,
    CellSceneStore,
    MonitoredGeometry,
    monitored_world_vertices,
)
from ..widgets.layout_utils import fit_button_text

Vec3 = Tuple[float, float, float]

# Theme-aligned palette (dark.qss).
_COLOR_BG = QColor("#121417")
_COLOR_GRID = QColor("#2b3540")
_COLOR_LINK = QColor("#48d5b5")
_COLOR_JOINT = QColor("#e6edf3")
_COLOR_TCP = QColor("#f0b429")
_COLOR_TEXT = QColor("#8b98a5")
_COLOR_OBSTACLE = QColor("#f0b429")
_COLOR_KEEP_OUT = QColor("#f85149")
_COLOR_EXTERNAL = QColor("#58a6ff")
_COLOR_ATTACHED = QColor("#bc8cff")

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
        self._scene = CellScene()
        self._monitored_geometries: tuple[MonitoredGeometry, ...] = ()
        self._monitored_draw_items: list[
            tuple[
                MonitoredGeometry,
                tuple[Vec3, ...],
                tuple[tuple[int, int], ...],
            ]
        ] = []
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

    def set_scene(self, scene: CellScene) -> None:
        """Display the same cell geometry that is mirrored into MoveIt."""

        self._scene = scene.validated()
        self.update()

    def set_monitored_geometries(
        self,
        geometries: Sequence[MonitoredGeometry],
    ) -> None:
        """Display external MoveIt objects as read-only blue wireframes."""

        self._monitored_geometries = tuple(
            geometry.validated()
            for geometry in geometries
            if not geometry.gui_managed
        )
        self._monitored_draw_items = []
        for geometry in self._monitored_geometries:
            vertices = monitored_world_vertices(geometry)
            if geometry.shape == "mesh" and geometry.triangles:
                edges = tuple(
                    sorted(
                        {
                            tuple(sorted((first, second)))
                            for triangle in geometry.triangles
                            for first, second in (
                                (triangle[0], triangle[1]),
                                (triangle[1], triangle[2]),
                                (triangle[2], triangle[0]),
                            )
                        }
                    )[:3000]
                )
            elif len(vertices) == 8:
                edges = (
                    (0, 1), (1, 3), (3, 2), (2, 0),
                    (4, 5), (5, 7), (7, 6), (6, 4),
                    (0, 4), (1, 5), (2, 6), (3, 7),
                )
            elif geometry.shape == "plane" and len(vertices) == 4:
                edges = ((0, 1), (1, 2), (2, 3), (3, 0), (0, 2))
            else:
                edges = ()
            self._monitored_draw_items.append((geometry, vertices, edges))
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
        self._draw_obstacles(painter)
        self._draw_monitored_geometries(painter)

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

    def _draw_obstacles(self, painter: QPainter) -> None:
        for obstacle in self._scene.enabled_obstacles:
            lower, upper = obstacle.bounds()
            x0, y0, z0 = lower
            x1, y1, z1 = upper
            corners = [
                (x0, y0, z0),
                (x1, y0, z0),
                (x1, y1, z0),
                (x0, y1, z0),
                (x0, y0, z1),
                (x1, y0, z1),
                (x1, y1, z1),
                (x0, y1, z1),
            ]
            projected = [self._project(corner) for corner in corners]
            color = _COLOR_KEEP_OUT if obstacle.kind == KEEP_OUT else _COLOR_OBSTACLE
            painter.setPen(QPen(color, 2, Qt.DashLine if obstacle.kind == KEEP_OUT else Qt.SolidLine))
            for first, second in (
                (0, 1), (1, 2), (2, 3), (3, 0),
                (4, 5), (5, 6), (6, 7), (7, 4),
                (0, 4), (1, 5), (2, 6), (3, 7),
            ):
                a, b = projected[first], projected[second]
                if a is not None and b is not None:
                    painter.drawLine(a, b)
            label_point = self._project((obstacle.center[0], obstacle.center[1], z1))
            if label_point is not None:
                painter.setPen(QPen(color))
                painter.drawText(label_point + QPointF(5.0, -5.0), obstacle.name)

    def _draw_monitored_geometries(self, painter: QPainter) -> None:
        for geometry, vertices, edges in self._monitored_draw_items:
            # Without TF in the GUI, drawing coordinates from a different
            # frame would be dangerously misleading.
            if geometry.frame_id != self._scene.frame_id:
                continue
            if not vertices:
                continue
            color = _COLOR_ATTACHED if geometry.attached_to else _COLOR_EXTERNAL
            painter.setPen(QPen(color, 1.5, Qt.DotLine))
            used_indexes = {
                index for edge in edges for index in edge
                if 0 <= index < len(vertices)
            }
            projected = {
                index: self._project(vertices[index])
                for index in used_indexes
            }
            for first, second in edges:
                a, b = projected.get(first), projected.get(second)
                if a is not None and b is not None:
                    painter.drawLine(a, b)
            highest = max(vertices, key=lambda point: point[2])
            label_point = self._project(highest)
            if label_point is not None:
                suffix = f" [{geometry.attached_to}]" if geometry.attached_to else ""
                if geometry.shape == "plane":
                    suffix += " [plano ∞; vista 2 m]"
                if (
                    geometry.transform_status == "resolved"
                    and geometry.source_frame_id != geometry.frame_id
                ):
                    suffix += f" ← {geometry.source_frame_id}"
                painter.setPen(QPen(color))
                painter.drawText(
                    label_point + QPointF(5.0, -5.0),
                    f"{geometry.object_id}{suffix}",
                )

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
    """Real-time arm posture plus an engineering editor for cell volumes."""

    async_operation_started = Signal(str, str)
    async_operation_finished = Signal(str, bool, str)

    def __init__(
        self,
        bridge,
        parent: QWidget | None = None,
        *,
        cell_scene_store: CellSceneStore | None = None,
    ) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._scene_store = cell_scene_store or CellSceneStore(parent=self)
        self._pending_scene_signature = ""

        self._chain: Optional[List[kinematics.ChainJoint]] = None
        self._chain_urdf: Optional[str] = None

        self._last_redraw = 0.0  # monotonic timestamp of the last FK redraw
        self._pending_msg = None  # latest joint state skipped by the throttle
        self._redraw_timer = QTimer(self)
        self._redraw_timer.setSingleShot(True)
        self._redraw_timer.timeout.connect(self._flush_pending)

        # ---- 3D viewport ----
        self._view = Robot3dView()
        self._view.set_scene(self._scene_store.scene)
        self._view.set_monitored_geometries(
            self._scene_store.external_geometries
        )

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

        viewport = QWidget()
        viewport_layout = QVBoxLayout(viewport)
        viewport_layout.setContentsMargins(0, 0, 0, 0)
        viewport_layout.addWidget(self._view, 1)
        viewport_layout.addLayout(bar)

        self._scene_editor = self._build_scene_editor()
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(viewport)
        splitter.addWidget(self._scene_editor)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([760, 500])

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(12)
        lay.addWidget(splitter, 1)

        self._scene_store.changed.connect(self._on_scene_changed)
        self._scene_store.applied_changed.connect(
            lambda _ok: self._refresh_scene_status()
        )
        self._scene_store.monitored_changed.connect(
            self._on_monitored_scene_changed
        )
        self._scene_store.monitored_status_changed.connect(
            lambda _ok: self._refresh_scene_status()
        )
        self._bridge.service_result.connect(self._on_service_result)
        self._load_scene_table(self._scene_store.scene)
        self.set_interface_mode("supervisor")

    def _build_scene_editor(self) -> QGroupBox:
        group = QGroupBox("Cena da célula")
        layout = QVBoxLayout(group)

        form = QFormLayout()
        self._scene_frame = QLineEdit()
        self._scene_frame.setAccessibleName("Frame de referência da cena")
        self._minimum_clearance = QDoubleSpinBox()
        self._minimum_clearance.setRange(0.0, 1.0)
        self._minimum_clearance.setDecimals(3)
        self._minimum_clearance.setSingleStep(0.005)
        self._minimum_clearance.setSuffix(" m")
        self._tcp_radius = QDoubleSpinBox()
        self._tcp_radius.setRange(0.0, 0.5)
        self._tcp_radius.setDecimals(3)
        self._tcp_radius.setSingleStep(0.005)
        self._tcp_radius.setSuffix(" m")
        form.addRow("Frame:", self._scene_frame)
        form.addRow("Folga mínima:", self._minimum_clearance)
        form.addRow("Raio do envelope TCP:", self._tcp_radius)
        layout.addLayout(form)

        self._scene_table = QTableWidget(0, 11)
        self._scene_table.setAccessibleName("Volumes de colisão da célula")
        self._scene_table.setAccessibleDescription(
            "Tabela editável de obstáculos e zonas proibidas no frame da célula"
        )
        self._scene_table.setHorizontalHeaderLabels(
            [
                "ID",
                "Nome",
                "Tipo",
                "X",
                "Y",
                "Z",
                "Dim. X",
                "Dim. Y",
                "Dim. Z",
                "Margem",
                "Ativo",
            ]
        )
        self._scene_table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows
        )
        self._scene_table.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection
        )
        self._scene_table.verticalHeader().setVisible(False)
        header = self._scene_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        header.setStretchLastSection(True)
        layout.addWidget(self._scene_table, 1)

        actions = QHBoxLayout()
        self._btn_add_obstacle = QPushButton("Adicionar")
        self._btn_remove_obstacle = QPushButton("Remover")
        self._btn_save_scene = QPushButton("Salvar cena")
        self._btn_apply_scene = QPushButton("Aplicar no MoveIt")
        self._btn_refresh_scene = QPushButton("Atualizar do MoveIt")
        self._btn_add_obstacle.clicked.connect(self._add_obstacle_row)
        self._btn_remove_obstacle.clicked.connect(self._remove_obstacle_row)
        self._btn_save_scene.clicked.connect(self._save_scene)
        self._btn_apply_scene.clicked.connect(self._apply_scene)
        self._btn_refresh_scene.clicked.connect(self._refresh_monitored_scene)
        for button in (
            self._btn_add_obstacle,
            self._btn_remove_obstacle,
            self._btn_save_scene,
            self._btn_apply_scene,
            self._btn_refresh_scene,
        ):
            fit_button_text(button)
            actions.addWidget(button)
        layout.addLayout(actions)

        self._scene_status = QLabel()
        self._scene_status.setWordWrap(True)
        layout.addWidget(self._scene_status)
        return group

    def set_interface_mode(self, profile: str, production: bool = False) -> None:
        """Keep scene changes exclusive to Engineering and outside production."""

        editable = str(profile) == "engenharia" and not bool(production)
        self._scene_frame.setEnabled(editable)
        self._minimum_clearance.setEnabled(editable)
        self._tcp_radius.setEnabled(editable)
        self._scene_table.setEditTriggers(
            QAbstractItemView.EditTrigger.DoubleClicked
            | QAbstractItemView.EditTrigger.EditKeyPressed
            if editable
            else QAbstractItemView.EditTrigger.NoEditTriggers
        )
        for button in (
            self._btn_add_obstacle,
            self._btn_remove_obstacle,
            self._btn_save_scene,
            self._btn_apply_scene,
        ):
            button.setEnabled(editable)
        self._btn_refresh_scene.setEnabled(True)
        self._refresh_scene_status()

    def _load_scene_table(self, scene: CellScene) -> None:
        self._scene_frame.setText(scene.frame_id)
        self._minimum_clearance.setValue(scene.minimum_clearance_m)
        self._tcp_radius.setValue(scene.tcp_radius_m)
        self._scene_table.setRowCount(0)
        for obstacle in scene.obstacles:
            self._append_obstacle(obstacle)
        self._refresh_scene_status()

    def _append_obstacle(self, obstacle: CellObstacle) -> None:
        row = self._scene_table.rowCount()
        self._scene_table.insertRow(row)
        values = (
            obstacle.object_id,
            obstacle.name,
            obstacle.kind,
            *(f"{value:.3f}" for value in obstacle.center),
            *(f"{value:.3f}" for value in obstacle.size),
            f"{obstacle.margin_m:.3f}",
        )
        for column, value in enumerate(values):
            self._scene_table.setItem(row, column, QTableWidgetItem(str(value)))
        active = QTableWidgetItem("sim" if obstacle.enabled else "não")
        active.setFlags(active.flags() | Qt.ItemIsUserCheckable)
        active.setCheckState(Qt.Checked if obstacle.enabled else Qt.Unchecked)
        self._scene_table.setItem(row, 10, active)

    def _add_obstacle_row(self) -> None:
        existing = {
            self._scene_table.item(row, 0).text().strip()
            for row in range(self._scene_table.rowCount())
            if self._scene_table.item(row, 0) is not None
        }
        index = 1
        while f"volume_{index}" in existing:
            index += 1
        self._append_obstacle(
            CellObstacle(
                object_id=f"volume_{index}",
                name=f"Volume {index}",
                kind=OBSTACLE,
            )
        )
        self._scene_table.selectRow(self._scene_table.rowCount() - 1)

    def _remove_obstacle_row(self) -> None:
        row = self._scene_table.currentRow()
        if row >= 0:
            self._scene_table.removeRow(row)

    def _scene_from_table(self) -> CellScene:
        obstacles: list[CellObstacle] = []
        for row in range(self._scene_table.rowCount()):
            def text(column: int) -> str:
                item = self._scene_table.item(row, column)
                return item.text().strip() if item is not None else ""

            obstacles.append(
                CellObstacle(
                    object_id=text(0),
                    name=text(1),
                    kind=text(2),
                    center=tuple(float(text(column)) for column in range(3, 6)),
                    size=tuple(float(text(column)) for column in range(6, 9)),
                    margin_m=float(text(9)),
                    enabled=(
                        self._scene_table.item(row, 10) is not None
                        and self._scene_table.item(row, 10).checkState() == Qt.Checked
                    ),
                )
            )
        return CellScene(
            frame_id=self._scene_frame.text(),
            minimum_clearance_m=self._minimum_clearance.value(),
            tcp_radius_m=self._tcp_radius.value(),
            obstacles=tuple(obstacles),
            revision=self._scene_store.scene.revision + 1,
            updated_at=datetime.now(timezone.utc).isoformat(),
        ).validated()

    def _save_scene(self) -> bool:
        try:
            scene = self._scene_from_table()
            self._scene_store.replace(scene)
        except (OSError, TypeError, ValueError) as exc:
            QMessageBox.critical(self, "Cena inválida", str(exc))
            return False
        self._scene_status.setText(
            f"Cena salva · revisão {scene.revision} · "
            f"assinatura {scene.signature()[:12]}…"
        )
        return True

    def _apply_scene(self) -> None:
        if not self._save_scene():
            return
        scene = self._scene_store.scene
        self._pending_scene_signature = scene.signature()
        self.async_operation_started.emit(
            "apply_cell_scene",
            "Aplicação da cena da célula no MoveIt iniciada.",
        )
        if not self._bridge.apply_cell_scene(scene):
            self.async_operation_finished.emit(
                "apply_cell_scene",
                False,
                "Serviço /apply_planning_scene indisponível.",
            )
            self._scene_store.mark_applied(
                self._pending_scene_signature,
                False,
                "serviço /apply_planning_scene indisponível",
            )
        else:
            self._scene_status.setText("Aplicando cena no MoveIt…")

    def _on_scene_changed(self, scene: CellScene) -> None:
        self._view.set_scene(scene)
        self._load_scene_table(scene)

    def _on_monitored_scene_changed(
        self,
        geometries: Sequence[MonitoredGeometry],
    ) -> None:
        del geometries
        self._view.set_monitored_geometries(
            self._scene_store.external_geometries
        )
        self._refresh_scene_status()

    def _refresh_monitored_scene(self) -> None:
        self._scene_status.setText("Lendo cena monitorada do MoveIt…")
        if not self._bridge.refresh_planning_scene(
            self._scene_store.scene.frame_id
        ):
            self._refresh_scene_status()

    def _on_service_result(self, label: str, success: bool, message: str) -> None:
        if label != "apply_cell_scene":
            return
        signature = self._pending_scene_signature
        self._pending_scene_signature = ""
        self._scene_store.mark_applied(signature, success, message)
        if success:
            self._bridge.refresh_planning_scene(
                self._scene_store.scene.frame_id
            )
        self._refresh_scene_status()

    def _refresh_scene_status(self) -> None:
        scene = self._scene_store.scene
        if self._scene_store.load_error:
            text = (
                "ARQUIVO DE CENA INVÁLIDO: "
                f"{self._scene_store.load_error}. Salve uma configuração válida."
            )
        elif not scene.configured:
            text = (
                "Cena local sem volumes. O MoveIt ainda pode conter objetos "
                "publicados por outros nós."
            )
        elif self._scene_store.is_applied:
            text = (
                f"{len(scene.enabled_obstacles)} volume(s) · aplicado no MoveIt · "
                f"assinatura {scene.signature()[:12]}…"
            )
            if self._scene_store.last_apply_message:
                text += f"\n{self._scene_store.last_apply_message}"
        else:
            text = (
                f"{len(scene.enabled_obstacles)} volume(s) · NÃO aplicado nesta sessão. "
                "Aplique no MoveIt antes de liberar ou executar."
            )
        external = self._scene_store.external_geometries
        if self._scene_store.monitored_error:
            text += (
                "\nCena monitorada: indisponível — "
                f"{self._scene_store.monitored_error}."
            )
        elif self._scene_store.monitored_ready:
            resolved_tf = sum(
                geometry.transform_status == "resolved"
                for geometry in external
            )
            unresolved_tf = sum(
                geometry.transform_status == "unresolved"
                or geometry.frame_id != scene.frame_id
                for geometry in external
            )
            text += (
                f"\nMoveIt monitorado: {len(external)} geometria(s) externa(s) "
                f"somente leitura · hash "
                f"{self._scene_store.monitored_signature[:12]}…"
            )
            if resolved_tf:
                text += f" · {resolved_tf} resolvida(s) por TF"
            if unresolved_tf:
                text += (
                    f" · {unresolved_tf} sem TF para {scene.frame_id} "
                    "não desenhada(s)"
                )
            if self._scene_store.monitored_tf_dynamic_enabled:
                text += (
                    " · TF dinâmico "
                    f"{self._scene_store.monitored_tf_refresh_rate_hz:g} Hz"
                )
            plane_count = sum(
                geometry.shape == "plane" for geometry in external
            )
            if plane_count:
                text += (
                    f"\n{plane_count} plano(s) infinito(s) do MoveIt · "
                    "recorte visual de 2 m"
                )
            octomap = self._scene_store.monitored_octomap
            if bool(octomap.get("present", False)):
                size_kib = int(octomap.get("data_bytes", 0)) / 1024.0
                text += (
                    "\nOctoMap ativo no MoveIt · "
                    f"frame {octomap['frame_id']} · "
                    f"resolução {float(octomap['resolution_m']) * 1000.0:g} mm"
                    f" · {size_kib:.1f} KiB · voxels não desenhados"
                )
        else:
            text += "\nCena monitorada: ainda não atualizada nesta sessão."
        self._scene_status.setText(text)

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
