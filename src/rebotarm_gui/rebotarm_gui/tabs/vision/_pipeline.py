# ruff: noqa: F401,F403,F405
"""VisionTab pipeline mixin — extracted from vision_tab.py."""

from __future__ import annotations

from ._common import *  # noqa: F403


class VisionTabPipelineMixin:
    """Pipeline concern methods for VisionTab."""

    def _apply(self) -> None:
        active_filters = self._active_filter_keys()
        orientation_profiles = self._orientation_profiles_for_apply()
        backend = self._detector_backend
        if backend == "yolo" and not self._yolo_model_path:
            backend = "opencv"
        backend_name = "YOLO" if backend == "yolo" else "OpenCV"
        detail = Path(self._yolo_model_path).name if backend == "yolo" else self._selected_detection_mode()
        self._active_backend_label.setText(f"Detector ativo: {backend_name} · {detail}")
        target_classes = [
            value.strip()
            for value in self._target_classes.text().split(",")
            if value.strip()
        ]
        params = {
            "backend": backend,
            "model_path": self._yolo_model_path,
            "confidence_threshold": float(self._yolo_confidence_percent.value()) / 100.0,
            "image_size": int(self._yolo_image_size.value()),
            "target_classes": target_classes,
            "auxiliary_attribute": self._auxiliary_attribute.text().strip(),
            "use_orientation_profiles": bool(orientation_profiles),
            "vertical_profile_json": json.dumps(orientation_profiles.get("vertical", {}), ensure_ascii=False),
            "horizontal_profile_json": json.dumps(orientation_profiles.get("horizontal", {}), ensure_ascii=False),
            "opencv_mode": self._selected_detection_mode(),
            "process_mode": self._process_mode.currentText(),
            "max_processing_fps": float(self._max_processing_fps.value()),
            "center_method": self._center.currentText(),
            "direction_method": self._direction.currentText(),
            "direction_sense_method": self._sense.currentText(),
            "draw_clearance": bool(self._draw_clearance.isChecked()),
            "shape_filter": self._shape_filter.currentText() if "shape_filter" in active_filters else "any",
            "min_area_px": float(self._min_area.value()) if "min_area" in active_filters else 0.0,
            "max_area_px": float(self._max_area.value()) if "max_area" in active_filters else 0.0,
            "min_bbox_long_side_px": 0.0,
            "min_bbox_short_side_px": 0.0,
            "max_bbox_long_side_px": 0.0,
            "max_bbox_short_side_px": 0.0,
            "morph_kernel": int(self._morph.value()),
            "morph_open_iterations": self._negative_part(self._morph_operation.value()),
            "morph_close_iterations": self._positive_part(self._morph_operation.value()),
            "roi_margin_px": int(self._roi_margin.value()),
            "brightness_threshold": int(self._brightness.value()),
            "brightness_invert": bool(self._brightness_invert.isChecked()),
            "blur_kernel": int(self._blur_kernel.value()),
            "canny_low": int(self._canny_low.value()),
            "canny_high": int(self._canny_high.value()),
            "edge_aperture_size": int(self._edge_aperture.currentText()),
            "edge_dilate_iterations": self._positive_part(self._edge_operation.value()),
            "edge_erode_iterations": self._negative_part(self._edge_operation.value()),
            "hsv_lower": [self._h_min.value(), self._s_min.value(), self._v_min.value()],
            "hsv_upper": [self._h_max.value(), self._s_max.value(), self._v_max.value()],
            "shape_epsilon_ratio": float(self._shape_eps.value()),
            "circle_min_circularity": float(self._circle_circ.value()),
            "square_aspect_tolerance": float(self._square_tol.value()),
            "lying_aspect_ratio_threshold": float(self._lying_ratio.value()),
            "min_solidity": float(self._min_solidity.value()) if "min_solidity" in active_filters else 0.0,
            "min_extent": float(self._min_extent.value()) if "min_extent" in active_filters else 0.0,
            "max_extent": float(self._max_extent.value()) if "max_extent" in active_filters else 0.0,
            "min_aspect_ratio": float(self._min_aspect.value()) if "min_aspect" in active_filters else 0.0,
            "max_aspect_ratio": float(self._max_aspect.value()) if "max_aspect" in active_filters else 0.0,
        }
        params.update(self._workspace_limit_params())
        self._bridge.vision_set_parameters(params)
        allowed_orientation = self._object_orientation.currentText()
        allowed_orientations = (
            ["horizontal", "vertical", "ambiguous"]
            if allowed_orientation == "any"
            else [allowed_orientation]
        )
        scheduler_params = {
            "min_confidence": float(self._yolo_confidence_percent.value()) / 100.0,
            "allowed_orientations": allowed_orientations,
            "min_clearance_px": float(self._grasp_clearance.value()),
            "min_clearance_horizontal_px": float(self._min_clearance_horizontal.value()),
            "min_clearance_vertical_px": float(self._min_clearance_vertical.value()),
            "auxiliary_attribute": self._auxiliary_attribute.text().strip(),
            "strategy": self._pick_route.currentText(),
            "max_objects": int(self._pick_max_objects.value()),
            "stable_frames": int(self._pick_stable_frames.value()),
            "route_origin_xy": [
                float(self._route_origin_x.value()),
                float(self._route_origin_y.value()),
            ],
        }
        setter = getattr(self._bridge, "pick_scheduler_set_parameters", None)
        if callable(setter):
            setter(scheduler_params)
        self.planning_config_changed.emit(self._current_task_config().to_dict())


    def _orientation_profiles_for_apply(self) -> dict[str, dict[str, object]]:
        profiles = {
            name: dict(profile)
            for name, profile in self._orientation_profiles.items()
            if name in {"vertical", "horizontal"} and isinstance(profile, dict) and profile
        }
        if not profiles:
            return {}
        workspace = self._workspace_limit_params()
        for profile in profiles.values():
            profile.update(workspace)
        return profiles


    def _process_frame(self) -> None:
        if self._ai_autotune_busy:
            self._set_ai_autotune_status(
                "Auto-calibração IA: aguarde a iteração atual terminar antes de processar outra imagem.",
                busy=True,
            )
            return
        self._clear_detection_view("Processando 1 frame...")
        self._apply()
        self.workspace_limit_changed.emit(self._current_workspace_limit())
        processor = getattr(self._bridge, "vision_process_frame", None)
        if callable(processor):
            QTimer.singleShot(300, processor)


    def _draw_detection_overlay(self) -> None:
        self._draw_clearance.setChecked(True)
        self._limit_mode.setCurrentText("limite desenhado")
        self._limit_margin_applied = self._limit_mask_enabled()
        self._profile_status.setText(
            "Desenho ativado: BBoxes, clearances, sentido dos objetos e área de trabalho serão mostrados no próximo frame."
        )
        self._process_frame()


    def _apply_external_profiles(self) -> None:
        loaded = sorted(self._orientation_profiles)
        if not {"horizontal", "vertical"}.issubset(set(loaded)):
            missing = []
            if "horizontal" not in self._orientation_profiles:
                missing.append("deitado/horizontal")
            if "vertical" not in self._orientation_profiles:
                missing.append("em pé/vertical")
            self._profile_status.setText(f"Perfil incompleto: carregue também {', '.join(missing)}.")
            QMessageBox.warning(
                self,
                "Perfis externos de visão",
                f"Carregue os dois perfis antes de aplicar: {', '.join(missing)}.",
            )
            return
        self._apply()
        self._apply_limit()
        self._profile_status.setText("Perfis externos aplicados ao pipeline de visão.")


    def _apply_task_configuration(self) -> None:
        self._apply()
        self._apply_limit()
        self._profile_status.setText("Modelo, área de trabalho e planejamento de pega aplicados.")


    def _on_preview_clicked(self, u: float, v: float) -> None:
        if self._is_drawn_limit_mode() and (
            self._btn_draw_limit_mode.isChecked() or self._click_target.currentText() == "limite"
        ):
            self._add_limit_point(u, v)
            return
        index = int(self._calibration_index.value()) - 1
        if not (0 <= index < len(self._pixel_spins)):
            return
        u_spin, v_spin = self._pixel_spins[index]
        u_spin.setValue(int(round(u)))
        v_spin.setValue(int(round(v)))
        self._calibration_enabled[index].setChecked(True)


    def _set_limit_drawing_mode(self, enabled: bool) -> None:
        if enabled:
            self._limit_mode.setCurrentText("limite desenhado")
            self._click_target.setCurrentText("limite")
            self._preview_status.setText("Clique na imagem para desenhar o limite operacional")
        elif self._click_target.currentText() == "limite":
            self._click_target.setCurrentText("calibracao")


    def _on_limit_mode_changed(self, _mode: str) -> None:
        drawing = self._is_drawn_limit_mode()
        self._limit_draw_controls.setVisible(drawing)
        self._btn_draw_limit_mode.setEnabled(drawing)
        if not drawing and self._btn_draw_limit_mode.isChecked():
            self._btn_draw_limit_mode.setChecked(False)
        if not drawing:
            self._limit_margin_applied = False
        self._render_local_overlay()


    def _on_limit_shape_changed(self, _shape: str) -> None:
        self._limit_points = []
        self._limit_margin_applied = False
        self._refresh_limit_summary()
        self._render_local_overlay()


    def _add_limit_point(self, u: float, v: float) -> None:
        self._limit_mode.setCurrentText("limite desenhado")
        self._limit_margin_applied = False
        if self._limit_shape.currentText() == "circle":
            if not self._limit_points:
                self._limit_points = [(float(u), float(v))]
                self._limit_circle = [float(u), float(v), 0.0]
            else:
                cx, cy = self._limit_points[0]
                radius = math.hypot(float(u) - cx, float(v) - cy)
                self._limit_circle = [cx, cy, radius]
                self._limit_points = []
        else:
            self._limit_points.append((float(u), float(v)))
        self._refresh_limit_summary()
        self._schedule_overlay_render()


    def _clear_limit(self) -> None:
        self._limit_points = []
        self._limit_circle = [320.0, 240.0, 180.0]
        self._limit_margin_applied = False
        self._refresh_limit_summary()
        self._apply_limit()
        self._render_local_overlay()


    def _apply_limit(self) -> None:
        use_operation_limits = self._is_operation_limit_mode()
        self._bridge.vision_set_parameters(self._workspace_limit_params())
        pose_params = self._operation_limit_pose_params(use_operation_limits)
        if pose_params:
            self._bridge.vision_pose_set_parameters(pose_params)
        self._limit_margin_applied = self._limit_mask_enabled()
        if self._btn_draw_limit_mode.isChecked():
            self._btn_draw_limit_mode.setChecked(False)
        self.workspace_limit_changed.emit(self._current_workspace_limit())
        self._render_local_overlay()


    def _workspace_limit_params(self) -> dict[str, object]:
        points: list[float] = []
        for u, v in self._limit_points:
            points.extend([float(u), float(v)])
        use_workspace_mask = self._limit_mask_enabled()
        return {
            "use_workspace_mask": use_workspace_mask,
            "draw_workspace_limit": use_workspace_mask,
            "workspace_limit_shape": self._limit_shape.currentText(),
            "workspace_limit_pixels": points,
            "workspace_circle_pixel": [float(value) for value in self._limit_circle],
            "workspace_mask_margin_px": int(self._limit_margin_px.value()),
        }


    def _limit_mask_enabled(self) -> bool:
        if not self._is_drawn_limit_mode():
            return False
        shape = self._limit_shape.currentText()
        if shape == "circle":
            return float(self._limit_circle[2]) > 0.0
        return len(self._limit_points) >= 3


    def _current_workspace_limit(self) -> dict[str, object]:
        enabled = self._is_drawn_limit_mode() or self._is_operation_limit_mode()
        shape = self._limit_shape.currentText()
        base_geometry: dict[str, object] = {}
        if enabled:
            base_params = self._operation_limit_pose_params(True)
            if bool(base_params.get("use_operation_limits", False)):
                base_shape = str(
                    base_params.get("operation_limit_shape", "")
                ).lower()
                base_geometry["base_shape"] = base_shape
                if (
                    self._calibration_profile.status == STATUS_APPROVED
                    and self._calibration_profile.integrity_hash
                ):
                    base_geometry["calibration_integrity_hash"] = (
                        self._calibration_profile.integrity_hash
                    )
                if base_shape == "circle":
                    base_geometry["base_circle"] = list(
                        base_params.get("operation_limit_circle", [])
                    )
                elif base_shape == "polygon":
                    flat = list(
                        base_params.get("operation_limit_xy_vertices", [])
                    )
                    base_geometry["base_polygon"] = [
                        [float(flat[index]), float(flat[index + 1])]
                        for index in range(0, len(flat) - 1, 2)
                    ]
        if shape == "circle":
            circle_valid = float(self._limit_circle[2]) > 0.0
            return {
                "enabled": bool(enabled and circle_valid),
                "mode": self._limit_mode.currentText(),
                "shape": "circle",
                "margin_px": int(self._limit_margin_px.value()),
                "points": [],
                "circle": [float(value) for value in self._limit_circle],
                **base_geometry,
            }
        return {
            "enabled": bool(enabled and len(self._limit_points) >= 3),
            "mode": self._limit_mode.currentText(),
            "shape": "polygon",
            "margin_px": int(self._limit_margin_px.value()),
            "points": [(float(u), float(v)) for u, v in self._limit_points],
            "circle": [float(value) for value in self._limit_circle],
            **base_geometry,
        }


    def _refresh_limit_summary(self) -> None:
        if self._limit_shape.currentText() == "circle":
            cx, cy, radius = self._limit_circle
            self._limit_summary.setText(f"círculo: centro=({cx:.0f},{cy:.0f}) raio={radius:.0f}px")
            return
        if not self._limit_points:
            self._limit_summary.setText("Sem limite desenhado")
            return
        self._limit_summary.setText(f"polígono com {len(self._limit_points)} pontos")


    def _operation_limit_pose_params(self, enabled: bool | None = None) -> dict[str, object]:
        if enabled is None:
            enabled = self._is_operation_limit_mode()
        if not enabled:
            return {"use_operation_limits": False}
        transform = self._calibration_transform()
        if transform is None:
            return {"use_operation_limits": False}
        shape = self._limit_shape.currentText()
        if shape == "circle":
            cx, cy, radius_px = self._limit_circle
            center_xy = self._apply_transform(transform, cx, cy)
            edge_xy = self._apply_transform(transform, cx + radius_px, cy)
            if center_xy is None or edge_xy is None:
                return {"use_operation_limits": False}
            radius_m = math.hypot(edge_xy[0] - center_xy[0], edge_xy[1] - center_xy[1])
            return {
                "use_operation_limits": True,
                "operation_limit_shape": "circle",
                "operation_limit_circle": [center_xy[0], center_xy[1], radius_m],
                "operation_limit_margin_m": 0.0,
            }
        if len(self._limit_points) < 3:
            return {"use_operation_limits": False}
        vertices: list[float] = []
        for u, v in self._limit_points:
            xy = self._apply_transform(transform, u, v)
            if xy is None:
                return {"use_operation_limits": False}
            vertices.extend([xy[0], xy[1]])
        return {
            "use_operation_limits": True,
            "operation_limit_shape": "polygon",
            "operation_limit_xy_vertices": vertices,
            "operation_limit_margin_m": 0.0,
        }


    def _is_drawn_limit_mode(self) -> bool:
        return self._limit_mode.currentText() == "limite desenhado"


    def _is_operation_limit_mode(self) -> bool:
        return self._limit_mode.currentText() == "limites operacionais"


    def _render_local_overlay(self) -> None:
        self._overlay_refresh_pending = False
        base = self._last_raw_pixmap if self._last_raw_pixmap is not None else self._last_pixmap
        if base is None or base.isNull():
            return
        pixmap = QPixmap(base)
        painter = QPainter(pixmap)
        pen = QPen(QColor(0, 180, 255))
        pen.setWidth(max(2, int(round(max(pixmap.width(), pixmap.height()) / 700.0))))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        text_pen = QPen(QColor(255, 255, 255))
        center_pen = QPen(QColor(255, 80, 80))
        center_pen.setWidth(pen.width() + 1)
        direction_pen = QPen(QColor(255, 190, 50))
        direction_pen.setWidth(pen.width() + 1)
        clearance_pen = QPen(QColor(80, 255, 180))
        clearance_pen.setWidth(max(1, pen.width() - 1))
        limit_pen = QPen(QColor(255, 120, 80))
        limit_pen.setWidth(pen.width() + 1)

        self._draw_limit_overlay(painter, limit_pen)
        for row, obj in enumerate(self._last_objects):
            points = list(getattr(obj, "bounding_box_pixels", []))
            if len(points) >= 4:
                painter.setPen(pen)
                for index in range(4):
                    start = points[index]
                    end = points[(index + 1) % 4]
                    painter.drawLine(int(round(start.x)), int(round(start.y)), int(round(end.x)), int(round(end.y)))
                label_x = int(round(points[0].x))
                label_y = int(round(points[0].y))
            else:
                center = getattr(obj, "center_pixel", None)
                if center is None:
                    continue
                label_x = int(round(center.x))
                label_y = int(round(center.y))
            center = getattr(obj, "center_pixel", None)
            if center is not None:
                painter.setPen(center_pen)
                cx = int(round(center.x))
                cy = int(round(center.y))
                painter.drawLine(cx - 6, cy, cx + 6, cy)
                painter.drawLine(cx, cy - 6, cx, cy + 6)
                if bool(self._draw_clearance.isChecked()) and len(points) >= 4:
                    painter.setPen(clearance_pen)
                    self._draw_clearance_overlay(painter, obj, cx, cy, pixmap.width(), pixmap.height())
            painter.setPen(direction_pen)
            self._draw_direction_overlay(painter, obj)
            painter.setPen(text_pen)
            painter.drawText(label_x + 4, label_y - 4, str(row + 1))
        painter.end()
        self._last_pixmap = pixmap
        if self._last_objects:
            self._preview_status.setText(f"Preview: overlay local com {len(self._last_objects)} objeto(s)")
        else:
            self._preview_status.setText("Preview: limite operacional desenhado")
        self._refresh_preview()


    def _schedule_overlay_render(self, delay_ms: int = 20) -> None:
        if self._overlay_refresh_pending:
            return
        self._overlay_refresh_pending = True
        QTimer.singleShot(delay_ms, self._render_local_overlay)


    def _draw_limit_overlay(self, painter: QPainter, pen: QPen) -> None:
        if not self._is_drawn_limit_mode():
            return
        painter.setPen(pen)
        if self._limit_shape.currentText() == "circle":
            if self._limit_points:
                cx, cy = self._limit_points[0]
                painter.drawEllipse(int(round(cx - 5)), int(round(cy - 5)), 10, 10)
                painter.drawLine(int(round(cx - 10)), int(round(cy)), int(round(cx + 10)), int(round(cy)))
                painter.drawLine(int(round(cx)), int(round(cy - 10)), int(round(cx)), int(round(cy + 10)))
                return
            cx, cy, radius = self._limit_circle
            if radius <= 0.0:
                return
            painter.drawEllipse(
                int(round(cx - radius)),
                int(round(cy - radius)),
                int(round(radius * 2.0)),
                int(round(radius * 2.0)),
            )
            self._draw_circle_margin_overlay(painter, cx, cy, radius, pen.width())
            return
        if not self._limit_points:
            return
        for x, y in self._limit_points:
            painter.drawEllipse(int(round(x - 3)), int(round(y - 3)), 6, 6)
        if len(self._limit_points) < 2:
            return
        for index in range(len(self._limit_points) - 1):
            x0, y0 = self._limit_points[index]
            x1, y1 = self._limit_points[index + 1]
            painter.drawLine(int(round(x0)), int(round(y0)), int(round(x1)), int(round(y1)))
        if len(self._limit_points) >= 3:
            x0, y0 = self._limit_points[-1]
            x1, y1 = self._limit_points[0]
            painter.drawLine(int(round(x0)), int(round(y0)), int(round(x1)), int(round(y1)))
            self._draw_polygon_margin_overlay(painter, pen.width())


    def _draw_circle_margin_overlay(self, painter: QPainter, cx: float, cy: float, radius: float, width: int) -> None:
        if not self._limit_margin_applied:
            return
        margin = int(self._limit_margin_px.value())
        inner_radius = radius - float(margin)
        if margin <= 0 or inner_radius <= 1.0:
            return
        margin_pen = QPen(QColor(120, 255, 120))
        margin_pen.setWidth(max(1, width))
        margin_pen.setStyle(Qt.PenStyle.DashLine)
        painter.setPen(margin_pen)
        painter.drawEllipse(
            int(round(cx - inner_radius)),
            int(round(cy - inner_radius)),
            int(round(inner_radius * 2.0)),
            int(round(inner_radius * 2.0)),
        )


    def _draw_polygon_margin_overlay(self, painter: QPainter, width: int) -> None:
        if not self._limit_margin_applied:
            return
        margin = float(self._limit_margin_px.value())
        if margin <= 0.0 or len(self._limit_points) < 3:
            return
        center_x = sum(point[0] for point in self._limit_points) / len(self._limit_points)
        center_y = sum(point[1] for point in self._limit_points) / len(self._limit_points)
        inset_points: list[tuple[int, int]] = []
        for x, y in self._limit_points:
            dx = center_x - x
            dy = center_y - y
            length = max(math.hypot(dx, dy), 1e-6)
            inset_points.append((int(round(x + dx * margin / length)), int(round(y + dy * margin / length))))
        margin_pen = QPen(QColor(120, 255, 120))
        margin_pen.setWidth(max(1, width))
        margin_pen.setStyle(Qt.PenStyle.DashLine)
        painter.setPen(margin_pen)
        for index in range(len(inset_points)):
            x0, y0 = inset_points[index]
            x1, y1 = inset_points[(index + 1) % len(inset_points)]
            painter.drawLine(x0, y0, x1, y1)


    def _draw_direction_overlay(self, painter: QPainter, obj) -> None:
        start = getattr(obj, "direction_start_pixel", None)
        end = getattr(obj, "direction_end_pixel", None)
        if start is None or end is None:
            return
        x0 = float(start.x)
        y0 = float(start.y)
        x1 = float(end.x)
        y1 = float(end.y)
        painter.drawLine(int(round(x0)), int(round(y0)), int(round(x1)), int(round(y1)))
        angle = math.atan2(y1 - y0, x1 - x0)
        head = 14.0
        for delta in (2.55, -2.55):
            hx = x1 + math.cos(angle + delta) * head
            hy = y1 + math.sin(angle + delta) * head
            painter.drawLine(int(round(x1)), int(round(y1)), int(round(hx)), int(round(hy)))


    @staticmethod
    def _draw_clearance_overlay(
        painter: QPainter,
        obj,
        _cx: int,
        _cy: int,
        _image_width: int,
        _image_height: int,
    ) -> None:
        oriented = VisionTab._oriented_bbox_from_object(obj)
        if oriented is None:
            return
        center_x, center_y, long_axis, short_axis, half_long, half_short = oriented
        max_draw = 80.0
        spans = [
            (short_axis, -1.0, half_short, min(float(getattr(obj, "clearance_left_px", 0.0)), max_draw)),
            (short_axis, 1.0, half_short, min(float(getattr(obj, "clearance_right_px", 0.0)), max_draw)),
            (long_axis, -1.0, half_long, min(float(getattr(obj, "clearance_up_px", 0.0)), max_draw)),
            (long_axis, 1.0, half_long, min(float(getattr(obj, "clearance_down_px", 0.0)), max_draw)),
        ]
        for axis, sign, half_size, clearance in spans:
            start_x = center_x + axis[0] * sign * half_size
            start_y = center_y + axis[1] * sign * half_size
            end_x = start_x + axis[0] * sign * clearance
            end_y = start_y + axis[1] * sign * clearance
            painter.drawLine(
                int(round(start_x)),
                int(round(start_y)),
                int(round(end_x)),
                int(round(end_y)),
            )


    @staticmethod
    def _oriented_bbox_from_object(
        obj,
    ) -> tuple[float, float, tuple[float, float], tuple[float, float], float, float] | None:
        points = list(getattr(obj, "bounding_box_pixels", []))
        if len(points) < 4:
            return None
        xy = [(float(point.x), float(point.y)) for point in points[:4]]
        center_x = sum(point[0] for point in xy) / 4.0
        center_y = sum(point[1] for point in xy) / 4.0
        edges: list[tuple[float, float, float]] = []
        for index in range(4):
            x0, y0 = xy[index]
            x1, y1 = xy[(index + 1) % 4]
            dx = x1 - x0
            dy = y1 - y0
            edges.append((math.hypot(dx, dy), dx, dy))
        long_index, long_edge = max(enumerate(edges), key=lambda item: item[1][0])
        short_edge = edges[(long_index + 1) % 4]
        long_length = max(long_edge[0], 1e-6)
        short_length = max(short_edge[0], 1e-6)
        long_axis = (long_edge[1] / long_length, long_edge[2] / long_length)
        short_axis = (short_edge[1] / short_length, short_edge[2] / short_length)
        return center_x, center_y, long_axis, short_axis, long_length * 0.5, short_length * 0.5


    def update_objects(self, msg) -> None:
        objects = list(getattr(msg, "objects", []))
        self._last_objects = objects
        self._btn_continue_grasp.setEnabled(bool(objects))
        self._objects.setRowCount(len(objects))
        for row, obj in enumerate(objects):
            object_number = row + 1
            pose = getattr(obj, "pose", None)
            position = getattr(pose, "position", None)
            center = getattr(obj, "center_pixel", None)
            xyz = "--"
            if position is not None:
                xyz = f"{position.x:+.3f}, {position.y:+.3f}, {position.z:+.3f}"
            center_text = "--"
            if center is not None:
                center_text = f"{center.x:.0f}, {center.y:.0f}"
            bbox = " ".join(
                f"({point.x:.0f},{point.y:.0f})"
                for point in list(getattr(obj, "bounding_box_pixels", []))
            ) or "--"
            bbox_short, bbox_long, bbox_ratio = self._bbox_side_metrics(obj)
            dimensions = getattr(obj, "dimensions", None)
            dim_text = "--"
            if dimensions is not None:
                dim_text = f"{dimensions.x:.3f}, {dimensions.y:.3f}, {dimensions.z:.3f}"
            values = [
                str(object_number),
                str(getattr(obj, "class_name", "")),
                f"{float(getattr(obj, 'score', 0.0)):.2f}",
                center_text,
                xyz,
                f"{float(getattr(obj, 'yaw', 0.0)):.2f}",
                f"{float(getattr(obj, 'axis_yaw', 0.0)):.2f}",
                str(getattr(obj, "direction_sense", "")),
                f"{float(getattr(obj, 'gripper_yaw', 0.0)):.2f}",
                f"{float(getattr(obj, 'area_px', 0.0)):.0f}",
                bbox,
                self._format_optional_float(bbox_short, 1),
                self._format_optional_float(bbox_long, 1),
                self._format_optional_float(bbox_ratio, 2),
                dim_text,
                f"{float(getattr(obj, 'clearance_left_px', 0.0)):.0f}",
                f"{float(getattr(obj, 'clearance_right_px', 0.0)):.0f}",
                f"{float(getattr(obj, 'clearance_up_px', 0.0)):.0f}",
                f"{float(getattr(obj, 'clearance_down_px', 0.0)):.0f}",
                "sim" if bool(getattr(obj, "reachable", False)) else "não",
                "" if bool(getattr(obj, "reachable", False)) else str(getattr(obj, "reject_reason", "")),
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                if col == 1 and "lying" in value:
                    item.setForeground(Qt.GlobalColor.cyan)
                self._objects.setItem(row, col, item)
        self._render_local_overlay()


    def update_debug_image(self, msg) -> None:
        image = self._image_msg_to_qimage(msg)
        if image is None:
            encoding = str(getattr(msg, "encoding", ""))
            width = int(getattr(msg, "width", 0))
            height = int(getattr(msg, "height", 0))
            self._preview_status.setText(f"Preview: imagem invalida ({encoding}, {width}x{height})")
            return
        self._set_preview_image(image, f"Preview: debug {image.width()}x{image.height()} recebido")
        self._schedule_overlay_render(delay_ms=35)


    def update_raw_image(self, msg) -> None:
        image = self._image_msg_to_qimage(msg)
        if image is None:
            return
        header = getattr(msg, "header", None)
        previous_signature = (
            self._last_camera_frame,
            self._runtime_image_width,
            self._runtime_image_height,
        )
        self._last_camera_frame = str(getattr(header, "frame_id", "")).strip()
        self._runtime_image_width = int(image.width())
        self._runtime_image_height = int(image.height())
        runtime_signature = (
            self._last_camera_frame,
            self._runtime_image_width,
            self._runtime_image_height,
        )
        if runtime_signature != previous_signature:
            valid, reason, _reference = self.production_calibration_status()
            self.calibration_status_changed.emit(valid, reason)
        self._last_raw_pixmap = QPixmap.fromImage(image)
        self._last_image_width = int(image.width())
        self._last_image_height = int(image.height())
        self._preview.set_source_size(image.width(), image.height())
        if self._last_pixmap is None:
            self._last_pixmap = self._last_raw_pixmap
            self._preview_status.setText(f"Preview: frame cru {image.width()}x{image.height()}")
            self._refresh_preview()


    def _set_preview_image(self, image: QImage, status: str, raw: bool = False) -> None:
        pixmap = QPixmap.fromImage(image)
        if pixmap.isNull():
            self._preview_status.setText(f"Preview: pixmap vazio ({image.width()}x{image.height()})")
            return
        self._last_pixmap = pixmap
        if raw:
            self._last_raw_pixmap = pixmap
        self._last_image_width = int(image.width())
        self._last_image_height = int(image.height())
        self._preview.set_source_size(image.width(), image.height())
        self._preview_status.setText(status)
        self._refresh_preview()


    def _refresh_preview(self) -> None:
        if self._last_pixmap is None:
            return
        target_size = self._preview.size()
        if target_size.width() < 2 or target_size.height() < 2:
            target_size = self._preview.minimumSize()
        scaled = self._last_pixmap.scaled(
            target_size,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.FastTransformation,
        )
        self._preview.setPixmap(scaled)


    def _current_image_width(self) -> int:
        if self._last_image_width > 0:
            return self._last_image_width
        if self._test_image is not None and not self._test_image.isNull():
            return int(self._test_image.width())
        return 0


    def _current_image_height(self) -> int:
        if self._last_image_height > 0:
            return self._last_image_height
        if self._test_image is not None and not self._test_image.isNull():
            return int(self._test_image.height())
        return 0


    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._refresh_preview()


    @staticmethod
    def _default_task_config_path() -> Path:
        return Path.home() / ".config" / "rebotarm_gui" / "vision_pick_config.json"


