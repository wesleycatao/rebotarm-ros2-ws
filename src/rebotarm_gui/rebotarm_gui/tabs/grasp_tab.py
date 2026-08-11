"""Grasp planning tab based on detected vision objects."""

from __future__ import annotations

import json
from pathlib import Path

from PySide6.QtCore import QPointF, Qt, Signal
from PySide6.QtGui import QColor, QBrush, QImage, QPainter, QPen, QPixmap, QPolygonF
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSplitter,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from ..motion import GripperStep, MoveStep, Program, WaitStep, pose
from ..grasp_planner import (
    GraspCandidate,
    GraspPlannerConfig,
    best_candidates,
    plan_grasps,
)
from ..widgets.layout_utils import fit_button_text


class GraspTab(QWidget):
    """Visual grasp strategy planner for the current vision frame."""

    motion_program_generated = Signal(object)

    def __init__(self, bridge, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._last_objects: list[object] = []
        self._candidates: list[GraspCandidate] = []
        self._last_raw_pixmap: QPixmap | None = None
        self._last_overlay_pixmap: QPixmap | None = None
        self._selected_candidate_index: int | None = None
        self._workspace_limit: dict[str, object] = {"enabled": False}

        self._open_width = self._spin(0, 2000, 140, " px")
        self._closed_width = self._spin(0, 2000, 12, " px")
        self._body_width = self._spin(0, 2000, 95, " px")
        self._body_length = self._spin(0, 2000, 150, " px")
        self._finger_thickness = self._spin(0, 500, 12, " px")
        self._side_clearance = self._spin(0, 1000, 4, " px")
        self._front_clearance = self._spin(0, 1000, 20, " px")
        self._approach_distance = self._spin(0, 2000, 55, " px")
        self._vertical_margin = self._spin(0, 500, 8, " px")
        self._lateral_angle_step = self._spin(1, 90, 10, " °")
        self._lateral_candidate_count = self._spin(1, 12, 3, "")
        self._mm_per_px = self._dspin(0.001, 100.0, 1.0, " mm/px", step=0.1)
        self._safe_z = self._dspin(-1.0, 2.0, 0.200, " m", step=0.005)
        self._pick_z = self._dspin(-1.0, 2.0, 0.050, " m", step=0.005)
        self._move_duration = self._dspin(0.1, 60.0, 2.0, " s", step=0.1)
        self._gripper_effort = self._dspin(0.0, 100.0, 0.0, "", step=0.5)
        self._gripper_wait = self._dspin(0.0, 10.0, 0.4, " s", step=0.1)
        self._vision_uncertainty_xy = self._dspin(
            0.0, 0.100, 0.005, " m", step=0.001
        )
        self._vision_uncertainty_z = self._dspin(
            0.0, 0.100, 0.003, " m", step=0.001
        )
        self._lying_aspect = self._spin(10, 100, 16, " /10")
        self._standing_aspect = self._spin(10, 100, 14, " /10")
        self._top_enabled = QCheckBox("Pega superior vertical")
        self._lateral_enabled = QCheckBox("Pega lateral horizontal")
        self._end_enabled = QCheckBox("Pega pela ponta (experimental)")
        self._prefer_top = QCheckBox("Preferir superior para objeto deitado")
        self._show_only_best = QCheckBox("Desenhar só melhor pega válida por objeto")
        for checkbox in (
            self._top_enabled,
            self._lateral_enabled,
            self._end_enabled,
            self._prefer_top,
            self._show_only_best,
        ):
            checkbox.setChecked(True)
        self._end_enabled.setChecked(False)
        for checkbox in (
            self._top_enabled,
            self._lateral_enabled,
            self._end_enabled,
            self._prefer_top,
        ):
            checkbox.stateChanged.connect(self._mark_strategy_stale)
        self._show_only_best.stateChanged.connect(lambda _state: self._render_overlay())
        for spin in (
            self._open_width,
            self._closed_width,
            self._body_width,
            self._body_length,
            self._finger_thickness,
            self._side_clearance,
            self._front_clearance,
            self._approach_distance,
            self._vertical_margin,
            self._lateral_angle_step,
            self._lateral_candidate_count,
            self._lying_aspect,
            self._standing_aspect,
            self._mm_per_px,
        ):
            spin.valueChanged.connect(self._mark_strategy_stale)
        for spin in (
            self._safe_z,
            self._pick_z,
            self._move_duration,
            self._gripper_effort,
            self._gripper_wait,
            self._vision_uncertainty_xy,
            self._vision_uncertainty_z,
        ):
            spin.valueChanged.connect(self._update_instructions)

        dimensions = QGroupBox("Dimensões do gripper na imagem")
        dim_form = QFormLayout(dimensions)
        dim_form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        dim_form.addRow("Abertura máx.", self._open_width)
        dim_form.addRow("Abertura mín.", self._closed_width)
        dim_form.addRow("Largura deitado", self._body_width)
        dim_form.addRow("Comprimento deitado", self._body_length)
        dim_form.addRow("Espessura dedos", self._finger_thickness)
        dim_form.addRow("Margem pega topo", self._vertical_margin)
        dim_form.addRow("Escala câmera", self._mm_per_px)

        clearance = QGroupBox("Regras de clearance")
        clearance_form = QFormLayout(clearance)
        clearance_form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        clearance_form.addRow("Lateral mín.", self._side_clearance)
        clearance_form.addRow("Frontal mín.", self._front_clearance)
        clearance_form.addRow("Dist. aproximação", self._approach_distance)
        clearance_form.addRow("Deitado se razão >=", self._lying_aspect)
        clearance_form.addRow("Em pé se razão <=", self._standing_aspect)
        clearance_form.addRow("Passo busca lateral", self._lateral_angle_step)
        clearance_form.addRow("Alternativas laterais", self._lateral_candidate_count)

        modes = QGroupBox("Estratégias habilitadas")
        modes_lay = QVBoxLayout(modes)
        modes_lay.addWidget(self._top_enabled)
        modes_lay.addWidget(self._lateral_enabled)
        modes_lay.addWidget(self._end_enabled)
        modes_lay.addWidget(self._prefer_top)
        modes_lay.addWidget(self._show_only_best)

        motion = QGroupBox("Geração de movimento")
        motion_form = QFormLayout(motion)
        motion_form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        motion_form.addRow("Z seguro", self._safe_z)
        motion_form.addRow("Z coleta", self._pick_z)
        motion_form.addRow("Duração mov.", self._move_duration)
        motion_form.addRow("Esforço grip.", self._gripper_effort)
        motion_form.addRow("Espera grip.", self._gripper_wait)
        motion_form.addRow("Incerteza visão XY", self._vision_uncertainty_xy)
        motion_form.addRow("Incerteza visão Z", self._vision_uncertainty_z)

        self._btn_replan = QPushButton("Recalcular estratégias")
        self._btn_send_program = QPushButton("Enviar ao Programa")
        self._btn_save_program = QPushButton("Salvar programa")
        self._btn_save = QPushButton("Salvar config.")
        self._btn_load = QPushButton("Carregar config.")
        self._btn_replan.setObjectName("PrimaryButton")
        self._btn_send_program.setObjectName("PrimaryButton")
        for b in (
            self._btn_replan,
            self._btn_send_program,
            self._btn_save_program,
            self._btn_save,
            self._btn_load,
        ):
            fit_button_text(b)
        self._btn_replan.clicked.connect(self._replan)
        self._btn_send_program.clicked.connect(self._send_motion_program)
        self._btn_save_program.clicked.connect(self._save_motion_program)
        self._btn_save.clicked.connect(self._save_config)
        self._btn_load.clicked.connect(self._load_config)
        button_grid = QGridLayout()
        button_grid.addWidget(self._btn_replan, 0, 0, 1, 2)
        button_grid.addWidget(self._btn_send_program, 1, 0)
        button_grid.addWidget(self._btn_save_program, 1, 1)
        config_button_row = QHBoxLayout()
        config_button_row.addWidget(self._btn_save)
        config_button_row.addWidget(self._btn_load)

        help_text = QLabel(
            "A aba usa os objetos detectados na Visão. Clearances laterais vêm dos BBoxes "
            "e são usados para validar se o gripper cabe na aproximação escolhida. O limite "
            "operacional desenhado na Visão também limita o retângulo do gripper e o corredor "
            "de aproximação. Garrafas deitadas usam pega superior vertical; pega lateral fica "
            "reservada para objetos classificados como em pé e busca o maior corredor livre ao redor do objeto. "
            "Novos frames deixam as estratégias pendentes; clique em Recalcular estratégias para popular a tabela."
        )
        help_text.setObjectName("MutedLabel")
        help_text.setWordWrap(True)

        side = QWidget()
        side_lay = QVBoxLayout(side)
        side_lay.setContentsMargins(0, 0, 0, 0)
        side_lay.setSpacing(12)
        side_lay.addWidget(dimensions)
        side_lay.addWidget(clearance)
        side_lay.addWidget(modes)
        side_lay.addWidget(motion)
        side_lay.addLayout(button_grid)
        side_lay.addLayout(config_button_row)
        side_lay.addWidget(help_text)
        side_lay.addStretch(1)

        side_scroll = QScrollArea()
        side_scroll.setWidgetResizable(True)
        side_scroll.setWidget(side)
        side_scroll.setMinimumWidth(360)
        side_scroll.setMaximumWidth(500)
        side_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self._preview = QLabel("Sem imagem da visão")
        self._preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._preview.setMinimumSize(360, 240)
        self._preview.setObjectName("VisionPreview")
        self._preview.setAccessibleName("Pré-visualização das estratégias de pega")

        self._status = QLabel("Aguardando objetos detectados.")
        self._status.setObjectName("MutedLabel")

        self._table = QTableWidget(0, 14)
        self._table.setHorizontalHeaderLabels(
            [
                "N",
                "Tipo",
                "Estratégia",
                "Abordagem",
                "Válida",
                "Score",
                "Yaw",
                "Abrir",
                "Clear. disp.",
                "Clear. req.",
                "Centro gripper",
                "Motivo",
                "Classe",
                "Rank",
            ]
        )
        self._table.setObjectName("VisionObjectsTable")
        self._table.setAccessibleName("Estratégias de pega calculadas")
        self._table.horizontalHeader().setStretchLastSection(True)
        self._table.verticalHeader().setVisible(False)
        self._table.itemSelectionChanged.connect(self._on_selection_changed)
        self._chk_technical_columns = QCheckBox("Detalhes técnicos")
        self._chk_technical_columns.setToolTip(
            "Mostra yaw, clearances, centro do gripper, classe e ranking."
        )
        self._chk_technical_columns.toggled.connect(self._set_technical_columns_visible)
        self._set_technical_columns_visible(False)

        self._instructions = QTextEdit()
        self._instructions.setReadOnly(True)
        self._instructions.setAccessibleName("Instruções da estratégia de pega")
        self._instructions.setMaximumHeight(130)
        self._instructions.setPlaceholderText("Selecione uma estratégia válida para ver as instruções.")

        right_panel = QWidget()
        right = QVBoxLayout(right_panel)
        right.setContentsMargins(0, 0, 0, 0)
        right.addWidget(self._preview, 1)
        status_row = QHBoxLayout()
        status_row.addWidget(self._status, 1)
        status_row.addWidget(self._chk_technical_columns)
        right.addLayout(status_row)
        right.addWidget(self._table, 0)
        right.addWidget(self._instructions, 0)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(side_scroll)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([430, 760])
        splitter.setChildrenCollapsible(False)

        main = QVBoxLayout(self)
        main.setContentsMargins(12, 12, 12, 12)
        main.addWidget(splitter, 1)

    def _set_technical_columns_visible(self, visible: bool) -> None:
        for column in (6, 7, 8, 9, 10, 12, 13):
            self._table.setColumnHidden(column, not visible)

    def update_objects(self, msg) -> None:
        self._last_objects = list(getattr(msg, "objects", []))
        self._mark_strategy_stale()

    def update_debug_image(self, msg) -> None:
        if self._last_raw_pixmap is not None:
            return
        self.update_raw_image(msg)

    def update_raw_image(self, msg) -> None:
        image = self._image_msg_to_qimage(msg)
        if image is None:
            return
        self._last_raw_pixmap = QPixmap.fromImage(image)
        self._render_overlay()

    def update_workspace_limit(self, limit: object) -> None:
        self._workspace_limit = limit if isinstance(limit, dict) else {"enabled": False}
        self._mark_strategy_stale()

    def apply_external_config(self, data: object) -> None:
        """Apply the grasp-related portion of the unified vision JSON."""
        if not isinstance(data, dict):
            return
        grasp = data.get("grasp", {})
        object_config = data.get("object", {})
        if not isinstance(grasp, dict):
            grasp = {}
        if not isinstance(object_config, dict):
            object_config = {}
        general = float(grasp.get("clearance_px", 0.0) or 0.0)
        horizontal = float(grasp.get("min_clearance_horizontal_px", self._side_clearance.value()))
        vertical = float(grasp.get("min_clearance_vertical_px", self._front_clearance.value()))
        self._side_clearance.setValue(int(round(max(general, horizontal))))
        self._front_clearance.setValue(int(round(max(general, vertical))))
        orientation = str(object_config.get("orientation", "any")).lower()
        self._top_enabled.setChecked(orientation in {"any", "horizontal"})
        self._lateral_enabled.setChecked(orientation in {"any", "vertical"})
        self._mark_strategy_stale()

    def _replan(self) -> None:
        self._candidates = plan_grasps(self._last_objects, self._config())
        self._fill_table()
        self._render_overlay()

    def _mark_strategy_stale(self, *_args) -> None:
        self._selected_candidate_index = None
        self._candidates = []
        self._table.setRowCount(0)
        if self._last_objects:
            self._status.setText(
                f"{len(self._last_objects)} objeto(s) recebido(s). Clique em Recalcular estratégias."
            )
        else:
            self._status.setText("Aguardando objetos detectados.")
        self._update_instructions()
        self._render_overlay()

    def _fill_table(self) -> None:
        self._table.setUpdatesEnabled(False)
        try:
            self._table.setRowCount(len(self._candidates))
            ranks: dict[int, int] = {}
            for row, cand in enumerate(self._candidates):
                rank = ranks.get(cand.object_number, 0) + 1
                ranks[cand.object_number] = rank
                values = [
                    str(cand.object_number),
                    cand.object_type,
                    cand.strategy,
                    cand.approach,
                    "sim" if cand.valid else "não",
                    f"{cand.score:.2f}",
                    f"{cand.yaw:.2f}",
                    f"{cand.gripper_opening_mm:.1f} mm",
                    f"{cand.available_clearance_px:.0f}",
                    f"{cand.required_clearance_px:.0f}",
                    f"{cand.center[0]:.0f}, {cand.center[1]:.0f}",
                    cand.reason,
                    cand.object_label,
                    str(rank),
                ]
                for col, value in enumerate(values):
                    item = QTableWidgetItem(value)
                    if col == 4:
                        item.setForeground(Qt.GlobalColor.green if cand.valid else Qt.GlobalColor.red)
                    self._table.setItem(row, col, item)
        finally:
            self._table.setUpdatesEnabled(True)
        valid_count = sum(1 for cand in self._candidates if cand.valid)
        self._status.setText(
            f"{len(self._last_objects)} objeto(s), {len(self._candidates)} estratégia(s), "
            f"{valid_count} válida(s)."
        )
        self._update_instructions()

    def _render_overlay(self) -> None:
        base = self._last_raw_pixmap
        if base is None or base.isNull():
            self._preview.setText("Sem imagem da visão")
            return
        pixmap = QPixmap(base)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        self._draw_workspace_limit(painter)
        self._draw_objects(painter)
        self._draw_candidates(painter)
        painter.end()
        self._last_overlay_pixmap = pixmap
        self._refresh_preview()

    def _draw_objects(self, painter: QPainter) -> None:
        pen = QPen(QColor(0, 180, 255))
        pen.setWidth(2)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        for index, obj in enumerate(self._last_objects, start=1):
            points = list(getattr(obj, "bounding_box_pixels", []))
            if len(points) >= 4:
                for point_index in range(4):
                    a = points[point_index]
                    b = points[(point_index + 1) % 4]
                    painter.drawLine(int(round(a.x)), int(round(a.y)), int(round(b.x)), int(round(b.y)))
                painter.drawText(int(round(points[0].x)) + 4, int(round(points[0].y)) - 4, str(index))
            center = getattr(obj, "center_pixel", None)
            if center is not None:
                x = int(round(center.x))
                y = int(round(center.y))
                painter.drawLine(x - 5, y, x + 5, y)
                painter.drawLine(x, y - 5, x, y + 5)

    def _draw_candidates(self, painter: QPainter) -> None:
        if self._selected_candidate_index is not None and 0 <= self._selected_candidate_index < len(self._candidates):
            candidates = [self._candidates[self._selected_candidate_index]]
        elif self._show_only_best.isChecked():
            candidates = list(best_candidates(self._candidates).values())
        else:
            candidates = [cand for cand in self._candidates if cand.valid]
        for cand in candidates:
            self._draw_candidate(painter, cand, selected=len(candidates) == 1)

    def _draw_candidate(self, painter: QPainter, cand: GraspCandidate, selected: bool = False) -> None:
        color = QColor(80, 255, 130) if cand.valid else QColor(255, 110, 80)
        if selected:
            color = QColor(255, 220, 70)
        if cand.approach_rect is not None:
            fill = QColor(color)
            fill.setAlpha(42)
            painter.setBrush(QBrush(fill))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawPolygon(self._polygon(cand.approach_rect))
        pen = QPen(color)
        pen.setWidth(4 if selected else 2)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(pen)
        self._draw_polygon_outline(painter, cand.gripper_rect)
        cx, cy = cand.center
        painter.drawText(int(round(cx)) + 6, int(round(cy)) - 6, f"{cand.object_number} {cand.approach}")

    def _draw_workspace_limit(self, painter: QPainter) -> None:
        if not bool(self._workspace_limit.get("enabled", False)):
            return
        shape = str(self._workspace_limit.get("shape", "")).lower()
        margin = float(self._workspace_limit.get("margin_px", 0.0) or 0.0)
        pen = QPen(QColor(255, 120, 80))
        pen.setWidth(3)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        if shape == "circle":
            circle = self._workspace_limit.get("circle", [])
            if not isinstance(circle, (list, tuple)) or len(circle) < 3:
                return
            cx, cy, radius = (float(circle[0]), float(circle[1]), float(circle[2]))
            if radius <= 0.0:
                return
            painter.drawEllipse(
                int(round(cx - radius)),
                int(round(cy - radius)),
                int(round(radius * 2.0)),
                int(round(radius * 2.0)),
            )
            if margin > 0.0 and radius > margin:
                margin_pen = QPen(QColor(120, 255, 120))
                margin_pen.setWidth(2)
                margin_pen.setStyle(Qt.PenStyle.DashLine)
                painter.setPen(margin_pen)
                inner = radius - margin
                painter.drawEllipse(
                    int(round(cx - inner)),
                    int(round(cy - inner)),
                    int(round(inner * 2.0)),
                    int(round(inner * 2.0)),
                )
            return
        if shape != "polygon":
            return
        points = self._workspace_polygon()
        if len(points) < 2:
            return
        self._draw_polygon_outline(painter, tuple(points))
        if margin > 0.0 and len(points) >= 3:
            inset = self._inset_polygon_for_display(points, margin)
            if len(inset) >= 3:
                margin_pen = QPen(QColor(120, 255, 120))
                margin_pen.setWidth(2)
                margin_pen.setStyle(Qt.PenStyle.DashLine)
                painter.setPen(margin_pen)
                self._draw_polygon_outline(painter, tuple(inset))

    @staticmethod
    def _draw_polygon_outline(painter: QPainter, points: tuple[tuple[float, float], ...]) -> None:
        for index in range(len(points)):
            x0, y0 = points[index]
            x1, y1 = points[(index + 1) % len(points)]
            painter.drawLine(int(round(x0)), int(round(y0)), int(round(x1)), int(round(y1)))

    @staticmethod
    def _polygon(points: tuple[tuple[float, float], ...]) -> QPolygonF:
        polygon = QPolygonF()
        for x, y in points:
            polygon.append(QPointF(float(x), float(y)))
        return polygon

    def _on_selection_changed(self) -> None:
        rows = sorted({index.row() for index in self._table.selectedIndexes()})
        self._selected_candidate_index = rows[0] if rows else None
        self._update_instructions()
        self._render_overlay()

    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._refresh_preview()

    def _refresh_preview(self) -> None:
        if self._last_overlay_pixmap is None or self._last_overlay_pixmap.isNull():
            return
        target = self._preview.size()
        if target.width() < 2 or target.height() < 2:
            target = self._preview.minimumSize()
        self._preview.setPixmap(
            self._last_overlay_pixmap.scaled(
                target,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.FastTransformation,
            )
        )

    def _config(self) -> GraspPlannerConfig:
        return GraspPlannerConfig(
            gripper_open_width_px=float(self._open_width.value()),
            gripper_closed_width_px=float(self._closed_width.value()),
            gripper_body_width_px=float(self._body_width.value()),
            gripper_body_length_px=float(self._body_length.value()),
            finger_thickness_px=float(self._finger_thickness.value()),
            min_side_clearance_px=float(self._side_clearance.value()),
            min_front_clearance_px=float(self._front_clearance.value()),
            approach_distance_px=float(self._approach_distance.value()),
            vertical_pick_margin_px=float(self._vertical_margin.value()),
            mm_per_px=float(self._mm_per_px.value()),
            lying_aspect_min=float(self._lying_aspect.value()) / 10.0,
            standing_aspect_max=float(self._standing_aspect.value()) / 10.0,
            lateral_search_step_deg=float(self._lateral_angle_step.value()),
            lateral_candidate_count=int(self._lateral_candidate_count.value()),
            enable_top_pick=bool(self._top_enabled.isChecked()),
            enable_lateral_pick=bool(self._lateral_enabled.isChecked()),
            enable_end_pick=bool(self._end_enabled.isChecked()),
            prefer_vertical_for_lying=bool(self._prefer_top.isChecked()),
            workspace_shape=self._workspace_shape(),
            workspace_polygon=tuple(self._workspace_polygon()),
            workspace_circle=self._workspace_circle(),
            workspace_margin_px=float(self._workspace_limit.get("margin_px", 0.0) or 0.0),
        )

    def _workspace_shape(self) -> str:
        if not bool(self._workspace_limit.get("enabled", False)):
            return "none"
        return str(self._workspace_limit.get("shape", "none")).lower()

    def _workspace_polygon(self) -> list[tuple[float, float]]:
        points = self._workspace_limit.get("points", [])
        if not isinstance(points, (list, tuple)):
            return []
        parsed: list[tuple[float, float]] = []
        for point in points:
            if isinstance(point, (list, tuple)) and len(point) >= 2:
                parsed.append((float(point[0]), float(point[1])))
        return parsed

    def _workspace_circle(self) -> tuple[float, float, float] | None:
        circle = self._workspace_limit.get("circle", [])
        if not isinstance(circle, (list, tuple)) or len(circle) < 3:
            return None
        return (float(circle[0]), float(circle[1]), float(circle[2]))

    @staticmethod
    def _inset_polygon_for_display(points: list[tuple[float, float]], margin: float) -> list[tuple[float, float]]:
        center_x = sum(point[0] for point in points) / len(points)
        center_y = sum(point[1] for point in points) / len(points)
        inset: list[tuple[float, float]] = []
        for x, y in points:
            dx = center_x - x
            dy = center_y - y
            length = max((dx * dx + dy * dy) ** 0.5, 1e-6)
            inset.append((x + dx * margin / length, y + dy * margin / length))
        return inset

    def _save_config(self) -> None:
        path, _selected = QFileDialog.getSaveFileName(
            self,
            "Salvar configuração de pega",
            str(self._default_config_path()),
            "JSON (*.json)",
        )
        if not path:
            return
        data = {
            "gripper_open_width_px": self._open_width.value(),
            "gripper_closed_width_px": self._closed_width.value(),
            "gripper_body_width_px": self._body_width.value(),
            "gripper_body_length_px": self._body_length.value(),
            "finger_thickness_px": self._finger_thickness.value(),
            "min_side_clearance_px": self._side_clearance.value(),
            "min_front_clearance_px": self._front_clearance.value(),
            "approach_distance_px": self._approach_distance.value(),
            "vertical_pick_margin_px": self._vertical_margin.value(),
            "mm_per_px": self._mm_per_px.value(),
            "safe_z_m": self._safe_z.value(),
            "pick_z_m": self._pick_z.value(),
            "move_duration_s": self._move_duration.value(),
            "gripper_effort": self._gripper_effort.value(),
            "gripper_wait_s": self._gripper_wait.value(),
            "vision_uncertainty_xy_m": self._vision_uncertainty_xy.value(),
            "vision_uncertainty_z_m": self._vision_uncertainty_z.value(),
            "lying_aspect_x10": self._lying_aspect.value(),
            "standing_aspect_x10": self._standing_aspect.value(),
            "lateral_search_step_deg": self._lateral_angle_step.value(),
            "lateral_candidate_count": self._lateral_candidate_count.value(),
            "enable_top_pick": self._top_enabled.isChecked(),
            "enable_lateral_pick": self._lateral_enabled.isChecked(),
            "enable_end_pick": self._end_enabled.isChecked(),
            "prefer_vertical_for_lying": self._prefer_top.isChecked(),
        }
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
        self._status.setText(f"Configuração salva em {target}")

    def _load_config(self) -> None:
        path, _selected = QFileDialog.getOpenFileName(
            self,
            "Carregar configuração de pega",
            str(self._default_config_path()),
            "JSON (*.json)",
        )
        if not path:
            return
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        self._open_width.setValue(int(data.get("gripper_open_width_px", self._open_width.value())))
        self._closed_width.setValue(int(data.get("gripper_closed_width_px", self._closed_width.value())))
        self._body_width.setValue(int(data.get("gripper_body_width_px", self._body_width.value())))
        self._body_length.setValue(int(data.get("gripper_body_length_px", self._body_length.value())))
        self._finger_thickness.setValue(int(data.get("finger_thickness_px", self._finger_thickness.value())))
        self._side_clearance.setValue(int(data.get("min_side_clearance_px", self._side_clearance.value())))
        self._front_clearance.setValue(int(data.get("min_front_clearance_px", self._front_clearance.value())))
        self._approach_distance.setValue(int(data.get("approach_distance_px", self._approach_distance.value())))
        self._vertical_margin.setValue(int(data.get("vertical_pick_margin_px", self._vertical_margin.value())))
        self._mm_per_px.setValue(float(data.get("mm_per_px", self._mm_per_px.value())))
        self._safe_z.setValue(float(data.get("safe_z_m", self._safe_z.value())))
        self._pick_z.setValue(float(data.get("pick_z_m", self._pick_z.value())))
        self._move_duration.setValue(float(data.get("move_duration_s", self._move_duration.value())))
        self._gripper_effort.setValue(float(data.get("gripper_effort", self._gripper_effort.value())))
        self._gripper_wait.setValue(float(data.get("gripper_wait_s", self._gripper_wait.value())))
        self._vision_uncertainty_xy.setValue(
            float(
                data.get(
                    "vision_uncertainty_xy_m",
                    self._vision_uncertainty_xy.value(),
                )
            )
        )
        self._vision_uncertainty_z.setValue(
            float(
                data.get(
                    "vision_uncertainty_z_m",
                    self._vision_uncertainty_z.value(),
                )
            )
        )
        self._lying_aspect.setValue(int(data.get("lying_aspect_x10", self._lying_aspect.value())))
        self._standing_aspect.setValue(int(data.get("standing_aspect_x10", self._standing_aspect.value())))
        self._lateral_angle_step.setValue(int(data.get("lateral_search_step_deg", self._lateral_angle_step.value())))
        self._lateral_candidate_count.setValue(int(data.get("lateral_candidate_count", self._lateral_candidate_count.value())))
        self._top_enabled.setChecked(bool(data.get("enable_top_pick", self._top_enabled.isChecked())))
        self._lateral_enabled.setChecked(bool(data.get("enable_lateral_pick", self._lateral_enabled.isChecked())))
        self._end_enabled.setChecked(bool(data.get("enable_end_pick", self._end_enabled.isChecked())))
        self._prefer_top.setChecked(bool(data.get("prefer_vertical_for_lying", self._prefer_top.isChecked())))
        self._mark_strategy_stale()

    def _selected_candidate(self) -> GraspCandidate | None:
        if self._selected_candidate_index is not None and 0 <= self._selected_candidate_index < len(self._candidates):
            return self._candidates[self._selected_candidate_index]
        for cand in self._candidates:
            if cand.valid:
                return cand
        return None

    def _update_instructions(self) -> None:
        cand = self._selected_candidate()
        if cand is None:
            self._instructions.setPlainText("Nenhuma estratégia válida disponível.")
            return
        obj = self._object_for_candidate(cand)
        pose_msg = getattr(getattr(obj, "pose", None), "position", None) if obj is not None else None
        xyz = "XYZ indisponível" if pose_msg is None else f"X={pose_msg.x:+.3f} Y={pose_msg.y:+.3f}"
        text = "\n".join(
            [
                f"Objeto {cand.object_number}: {cand.strategy} / {cand.approach} / score {cand.score:.2f}",
                f"Abrir gripper para {cand.gripper_opening_mm:.1f} mm antes da aproximação.",
                f"{xyz}; yaw gripper={cand.yaw:+.2f} rad; Z seguro={self._safe_z.value():+.3f} m; Z coleta={self._pick_z.value():+.3f} m.",
                "Sequência: abrir -> aproximar em Z seguro -> descer/coletar -> fechar -> aguardar -> retrair.",
                self._vision_envelope_guidance(),
            ]
        )
        self._instructions.setPlainText(text)

    def _vision_envelope_guidance(self) -> str:
        base_shape = str(self._workspace_limit.get("base_shape", "")).lower()
        calibration_hash = str(
            self._workspace_limit.get("calibration_integrity_hash", "")
        ).strip()
        uncertainty_xy = self._vision_uncertainty_xy.value() * 1000.0
        uncertainty_z = self._vision_uncertainty_z.value() * 1000.0
        if base_shape in {"circle", "polygon"} and calibration_hash:
            return (
                "Envelope de Produção pronto: área operacional calibrada; "
                f"incerteza adicional ±{uncertainty_xy:.1f} mm XY / "
                f"±{uncertainty_z:.1f} mm Z."
            )
        return (
            "Produção será bloqueada: desenhe/aplique uma área operacional "
            "com a calibração ativa na página Visão."
        )

    def _send_motion_program(self) -> None:
        program = self._motion_program_from_selection()
        if program is None:
            return
        self.motion_program_generated.emit(program)
        self._status.setText("Programa de pega enviado para a página Programa.")

    def _save_motion_program(self) -> None:
        program = self._motion_program_from_selection()
        if program is None:
            return
        path, _selected = QFileDialog.getSaveFileName(
            self,
            "Salvar programa de movimento da pega",
            str(Path.home() / ".config" / "rebotarm_gui" / "grasp_motion_program.json"),
            "Programas JSON (*.json)",
        )
        if not path:
            return
        target = Path(path).with_suffix(".json")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(program.to_json(), encoding="utf-8")
        self._status.setText(f"Programa de movimento salvo em {target}")

    def _motion_program_from_selection(self) -> Program | None:
        cand = self._selected_candidate()
        if cand is None or not cand.valid:
            QMessageBox.information(self, "Sem pega válida", "Selecione uma estratégia válida.")
            return None
        obj = self._object_for_candidate(cand)
        position = getattr(getattr(obj, "pose", None), "position", None) if obj is not None else None
        center = getattr(obj, "center_pixel", None) if obj is not None else None
        if position is None or center is None:
            QMessageBox.information(
                self,
                "XYZ indisponível",
                "O objeto ainda não possui pose XYZ. Aplique a calibração pixel -> XY antes de gerar movimento.",
            )
            return None
        dx_m = (cand.center[0] - float(center.x)) * float(self._mm_per_px.value()) / 1000.0
        dy_m = (cand.center[1] - float(center.y)) * float(self._mm_per_px.value()) / 1000.0
        target_x = float(position.x) + dx_m
        target_y = float(position.y) + dy_m
        safe_z = float(self._safe_z.value())
        pick_z = float(self._pick_z.value())
        duration = float(self._move_duration.value())
        close_mm = float(self._closed_width.value()) * float(self._mm_per_px.value())
        effort = float(self._gripper_effort.value())
        wait_s = float(self._gripper_wait.value())

        steps = [
            GripperStep(
                label=f"abrir gripper obj {cand.object_number}",
                position_mm=float(cand.gripper_opening_mm),
                max_effort=effort,
                timeout=3.0,
            ),
            MoveStep(
                label=f"aproximar obj {cand.object_number}",
                pose=pose(target_x, target_y, safe_z, 0.0, 0.0, float(cand.yaw)),
                duration=duration,
            ),
            MoveStep(
                label=f"coletar obj {cand.object_number}",
                pose=pose(target_x, target_y, pick_z, 0.0, 0.0, float(cand.yaw)),
                duration=duration,
            ),
            GripperStep(
                label=f"fechar gripper obj {cand.object_number}",
                position_mm=close_mm,
                max_effort=effort,
                timeout=3.0,
            ),
            WaitStep(label="aguardar fechamento", time=wait_s),
            MoveStep(
                label=f"retrair obj {cand.object_number}",
                pose=pose(target_x, target_y, safe_z, 0.0, 0.0, float(cand.yaw)),
                duration=duration,
            ),
        ]
        affected_labels = [
            f"aproximar obj {cand.object_number}",
            f"coletar obj {cand.object_number}",
            f"retrair obj {cand.object_number}",
        ]
        envelope: dict[str, object] = {
            "version": 1,
            "anchor_m": [target_x, target_y, pick_z],
            "affected_step_labels": affected_labels,
            "z_bounds_m": [pick_z, pick_z],
            "uncertainty_m": [
                float(self._vision_uncertainty_xy.value()),
                float(self._vision_uncertainty_xy.value()),
                float(self._vision_uncertainty_z.value()),
            ],
            "calibration_error_m": 0.0,
            "source": str(getattr(obj, "object_id", "")),
        }
        base_shape = str(self._workspace_limit.get("base_shape", "")).lower()
        calibration_hash = str(
            self._workspace_limit.get("calibration_integrity_hash", "")
        ).strip()
        if base_shape == "circle":
            circle = self._workspace_limit.get("base_circle", ())
            if (
                calibration_hash
                and isinstance(circle, (list, tuple))
                and len(circle) >= 3
            ):
                envelope.update(
                    shape="circle",
                    circle_m=[float(value) for value in circle[:3]],
                    calibration_integrity_hash=calibration_hash,
                    production_ready=True,
                )
        elif base_shape == "polygon":
            polygon = self._workspace_limit.get("base_polygon", ())
            if (
                calibration_hash
                and isinstance(polygon, (list, tuple))
                and len(polygon) >= 3
            ):
                envelope.update(
                    shape="polygon",
                    polygon_m=[
                        [float(point[0]), float(point[1])]
                        for point in polygon
                        if isinstance(point, (list, tuple)) and len(point) >= 2
                    ],
                    calibration_integrity_hash=calibration_hash,
                    production_ready=True,
                )
        if "shape" not in envelope:
            # Commissioning remains possible, but production release fails
            # closed until a calibrated operational area is configured.
            envelope.update(
                shape="point",
                point_m=[target_x, target_y],
                production_ready=False,
            )
        return Program(
            name=f"pega_objeto_{cand.object_number}",
            steps=steps,
            parameters={
                "_vision_guided": True,
                "_vision_envelope": envelope,
            },
        )

    def _object_for_candidate(self, cand: GraspCandidate) -> object | None:
        index = cand.object_number - 1
        if 0 <= index < len(self._last_objects):
            return self._last_objects[index]
        return None

    @staticmethod
    def _image_msg_to_qimage(msg) -> QImage | None:
        width = int(getattr(msg, "width", 0))
        height = int(getattr(msg, "height", 0))
        step = int(getattr(msg, "step", 0))
        data = bytes(getattr(msg, "data", b""))
        encoding = str(getattr(msg, "encoding", ""))
        if width <= 0 or height <= 0 or not data:
            return None
        if encoding == "bgr8":
            row_bytes = width * 3
            if step < row_bytes or len(data) < step * height:
                return None
            rgb = bytearray(row_bytes * height)
            for row in range(height):
                src_start = row * step
                dst_start = row * row_bytes
                row_data = data[src_start : src_start + row_bytes]
                rgb[dst_start : dst_start + row_bytes] = row_data
                for offset in range(dst_start, dst_start + row_bytes, 3):
                    rgb[offset], rgb[offset + 2] = rgb[offset + 2], rgb[offset]
            return QImage(bytes(rgb), width, height, row_bytes, QImage.Format.Format_RGB888).copy()
        if encoding == "rgb8":
            return QImage(data, width, height, step, QImage.Format.Format_RGB888).copy()
        if encoding in ("mono8", "8UC1"):
            return QImage(data, width, height, step, QImage.Format.Format_Grayscale8).copy()
        return None

    @staticmethod
    def _spin(minimum: int, maximum: int, value: int, suffix: str = "") -> QSpinBox:
        spin = QSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSuffix(suffix)
        spin.lineEdit().setAlignment(Qt.AlignmentFlag.AlignRight)
        spin.setMinimumWidth(96)
        spin.setMaximumWidth(132)
        return spin

    @staticmethod
    def _dspin(
        minimum: float,
        maximum: float,
        value: float,
        suffix: str = "",
        step: float = 0.1,
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(3)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setSuffix(suffix)
        spin.lineEdit().setAlignment(Qt.AlignmentFlag.AlignRight)
        spin.setMinimumWidth(112)
        spin.setMaximumWidth(154)
        return spin

    @staticmethod
    def _label(text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("FieldLabel")
        return label

    def _add_field(self, grid: QGridLayout, row: int, col: int, label: str, widget: QWidget) -> None:
        grid.addWidget(self._label(label), row, col)
        grid.addWidget(widget, row, col + 1)

    @staticmethod
    def _default_config_path() -> Path:
        return Path.home() / ".config" / "rebotarm_gui" / "grasp_config.json"
