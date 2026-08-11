# ruff: noqa: F401,F403,F405
"""VisionTab calibration mixin — extracted from vision_tab.py."""

from __future__ import annotations

from ._common import *  # noqa: F403


class VisionTabCalibrationMixin:
    """Calibration concern methods for VisionTab."""

    def _apply_workspace_from_profile(self, profile: dict[str, object]) -> None:
        if "use_workspace_mask" in profile:
            self._limit_mode.setCurrentText("limite desenhado" if bool(profile.get("use_workspace_mask")) else "desativado")
        if "workspace_limit_shape" in profile:
            self._limit_shape.setCurrentText(str(profile.get("workspace_limit_shape")))
        self._set_spin_value(self._limit_margin_px, profile.get("workspace_mask_margin_px"))
        points = profile.get("workspace_limit_pixels")
        if isinstance(points, list) and len(points) >= 6:
            parsed_points: list[tuple[float, float]] = []
            for index in range(0, len(points) - 1, 2):
                try:
                    parsed_points.append((float(points[index]), float(points[index + 1])))
                except Exception:
                    continue
            if len(parsed_points) >= 3:
                self._limit_points = parsed_points
        circle = profile.get("workspace_circle_pixel")
        if isinstance(circle, list) and len(circle) >= 3:
            self._limit_circle = [float(circle[0]), float(circle[1]), float(circle[2])]
        self._limit_margin_applied = self._limit_mask_enabled()
        self._refresh_limit_summary()
        self._render_local_overlay()


    def _apply_workspace(self) -> None:
        enabled = bool(self._use_calibration.isChecked())
        if enabled and len(self._calibration_profile.homography) != 9:
            if not self._validate_cell_calibration(show_dialog=False):
                QMessageBox.warning(
                    self,
                    "Calibração da célula",
                    "A calibração não foi aplicada porque o ajuste é inválido.",
                )
                return
        profile = self._calibration_profile
        if enabled and profile.status == STATUS_APPROVED:
            active = self._calibration_store.active()
            already_active = bool(
                active is not None
                and active.profile_id == profile.profile_id
                and active.revision == profile.revision
                and active.integrity_hash == profile.integrity_hash
            )
            if not already_active:
                allowed, identity_or_reason = (
                    self._calibration_approval_identity()
                )
                if not allowed:
                    QMessageBox.warning(
                        self, "Ativação bloqueada", identity_or_reason
                    )
                    return
                try:
                    self._calibration_store.save(profile, activate=True)
                except (OSError, TypeError, ValueError) as exc:
                    QMessageBox.critical(
                        self,
                        "Ativação bloqueada",
                        f"Não foi possível ativar o perfil:\n{exc}",
                    )
                    return
        params = {
            # The fitted 3x3 matrix uses every accepted sample.  Keep the
            # legacy workspace fit disabled because that node path truncates
            # calibration pairs to four points.
            "use_workspace_homography": False,
            "pixel_to_base_homography": (
                list(profile.homography) if enabled else [0.0] * 9
            ),
            "calibration_pixel_points": [],
            "calibration_xy_points": [],
            "image_width": float(profile.image_width or self._current_image_width()),
            "image_height": float(
                profile.image_height or self._current_image_height()
            ),
            "target_frame": profile.target_frame or "base_link",
            "table_z": float(profile.table_z_m),
        }
        self._bridge.vision_pose_set_parameters(params)
        state = (
            f"Perfil {profile.name} r{profile.revision} aplicado ao projetor."
            if enabled
            else "Projeção calibrada desativada."
        )
        self._calibration_status.setText(state)
        self._preview_status.setText(state)


    def _points_from_calibration_form(self) -> list[CalibrationPoint]:
        points: list[CalibrationPoint] = []
        for enabled, pixel_pair, xy_pair in zip(
            self._calibration_enabled,
            self._pixel_spins,
            self._xy_spins,
        ):
            u_spin, v_spin = pixel_pair
            x_spin, y_spin = xy_pair
            points.append(
                CalibrationPoint(
                    u=float(u_spin.value()),
                    v=float(v_spin.value()),
                    x=float(x_spin.value()),
                    y=float(y_spin.value()),
                    enabled=bool(enabled.isChecked()),
                )
            )
        return points


    def _profile_from_calibration_form(self) -> CalibrationProfile:
        previous = self._calibration_profile
        revision = max(1, int(previous.revision))
        if previous.status == STATUS_APPROVED:
            revision += 1
        width = self._current_image_width() or previous.image_width
        height = self._current_image_height() or previous.image_height
        camera_frame = self._calibration_camera_frame.text().strip()
        if self._last_camera_frame and camera_frame in {"", "camera_link"}:
            camera_frame = self._last_camera_frame
            self._calibration_camera_frame.setText(camera_frame)
        return CalibrationProfile(
            name=self._calibration_name.text().strip() or "calibração da célula",
            profile_id=previous.profile_id,
            revision=revision,
            camera_frame=camera_frame or "camera_link",
            target_frame=self._calibration_target_frame.text().strip()
            or "base_link",
            image_width=int(width),
            image_height=int(height),
            table_z_m=float(self._calibration_table_z.value()),
            tcp_frame=self._calibration_tcp_frame.text().strip()
            or "gripper_tcp",
            tool_id=self._calibration_tool_id.text().strip() or "gripper",
            payload_kg=float(self._calibration_payload.value()),
            validity_days=int(self._calibration_validity_days.value()),
            points=self._points_from_calibration_form(),
            created_at_utc=previous.created_at_utc,
        )


    def _populate_calibration_form(self, profile: CalibrationProfile) -> None:
        self._calibration_profile = profile
        self._calibration_name.setText(profile.name)
        self._calibration_camera_frame.setText(profile.camera_frame)
        self._calibration_target_frame.setText(profile.target_frame)
        self._calibration_tcp_frame.setText(profile.tcp_frame)
        self._calibration_tool_id.setText(profile.tool_id)
        self._calibration_table_z.setValue(profile.table_z_m)
        self._calibration_payload.setValue(profile.payload_kg)
        self._calibration_validity_days.setValue(profile.validity_days)
        for index, (enabled, pixel_pair, xy_pair, residual_label) in enumerate(
            zip(
                self._calibration_enabled,
                self._pixel_spins,
                self._xy_spins,
                self._calibration_residual_labels,
            )
        ):
            point = profile.points[index] if index < len(profile.points) else None
            if point is not None:
                enabled.setChecked(point.enabled)
                pixel_pair[0].setValue(int(round(point.u)))
                pixel_pair[1].setValue(int(round(point.v)))
                xy_pair[0].setValue(point.x)
                xy_pair[1].setValue(point.y)
            residual_label.setText("--")
        self._render_calibration_metrics()
        self._use_calibration.setChecked(
            profile.status == STATUS_APPROVED and len(profile.homography) == 9
        )
        self._update_calibration_editability()


    def _update_calibration_editability(self) -> None:
        privileged = self._interface_mode in {"supervisor", "engenharia"}
        editable = privileged and self._calibration_profile.status != STATUS_APPROVED
        form_controls = [
            self._calibration_name,
            self._calibration_camera_frame,
            self._calibration_target_frame,
            self._calibration_tcp_frame,
            self._calibration_tool_id,
            self._calibration_table_z,
            self._calibration_payload,
            self._calibration_validity_days,
            self._calibration_index,
            self._click_target,
            *self._calibration_enabled,
            *(spin for pair in self._pixel_spins for spin in pair),
            *(spin for pair in self._xy_spins for spin in pair),
        ]
        for control in form_controls:
            control.setEnabled(editable)
        for control in (
            self._btn_capture_pose,
            self._btn_validate_calibration,
            self._btn_save_calibration,
            self._btn_approve_cell_calibration,
        ):
            control.setEnabled(editable)
        self._btn_new_calibration_revision.setEnabled(privileged)
        self._btn_load_cell_calibration.setEnabled(privileged)
        self._btn_apply_table.setEnabled(privileged)


    def _render_calibration_metrics(self) -> None:
        profile = self._calibration_profile
        metrics = profile.metrics
        residual_index = 0
        for enabled, label in zip(
            self._calibration_enabled, self._calibration_residual_labels
        ):
            if not enabled.isChecked():
                label.setText("--")
                continue
            if residual_index < len(metrics.residuals_m):
                residual = metrics.residuals_m[residual_index] * 1000.0
                inlier = (
                    metrics.inliers[residual_index]
                    if residual_index < len(metrics.inliers)
                    else True
                )
                label.setText(f"{residual:.2f} mm" if inlier else f"{residual:.2f} mm ✕")
            else:
                label.setText("--")
            residual_index += 1
        state = {
            "approved": "APROVADO",
            "draft": "RASCUNHO",
            "obsolete": "OBSOLETO",
        }.get(profile.status, profile.status.upper())
        text = (
            f"{state} · {profile.name} r{profile.revision} · "
            f"{profile.metrics.summary()}"
        )
        if profile.status == STATUS_APPROVED:
            text += (
                f" · por {profile.approved_by} · válido até "
                f"{profile.expires_at_utc or '--'}"
            )
        else:
            ready, reason = profile.approval_readiness()
            text += " · pronto para aprovação" if ready else f" · {reason}"
        self._calibration_status.setText(text)
        self._active_calibration_label.setText(
            (
                f"Calibração: r{profile.revision} aprovada"
                if profile.status == STATUS_APPROVED
                else f"Calibração: r{profile.revision} rascunho"
            )
        )


    def _validate_cell_calibration(self, *, show_dialog: bool = True) -> bool:
        profile = self._profile_from_calibration_form()
        try:
            profile.fit()
        except (ValueError, ArithmeticError) as exc:
            self._calibration_profile = profile
            self._render_calibration_metrics()
            message = f"Falha no ajuste: {exc}"
            self._calibration_status.setText(message)
            self.calibration_status_changed.emit(False, message)
            if show_dialog:
                QMessageBox.warning(self, "Validar calibração", message)
            return False
        self._calibration_profile = profile
        self._render_calibration_metrics()
        ready, reason = profile.approval_readiness()
        message = profile.metrics.summary()
        if not ready:
            message = f"{message}. Aprovação bloqueada: {reason}."
        self.calibration_status_changed.emit(False, message)
        return True


    def _save_cell_calibration(self) -> None:
        if len(self._calibration_profile.homography) != 9:
            if not self._validate_cell_calibration(show_dialog=True):
                return
        try:
            path = self._calibration_store.save(self._calibration_profile)
        except (OSError, TypeError, ValueError) as exc:
            QMessageBox.critical(
                self, "Salvar calibração", f"Falha ao salvar o perfil:\n{exc}"
            )
            return
        self._calibration_status.setText(
            f"Rascunho r{self._calibration_profile.revision} salvo em {path}."
        )


    def _calibration_approval_identity(self) -> tuple[bool, str]:
        if self._interface_mode not in {"supervisor", "engenharia"}:
            return False, "a aprovação exige o perfil Supervisor ou Engenharia"
        if self._authorization is None:
            return True, self._settings.operator_id.strip() or "supervisor-comissionamento"
        allowed, reason = self._authorization.require(CAP_CALIBRATION_APPROVE)
        if not allowed:
            return False, reason
        identity = self._authorization.identity
        if not identity.authenticated:
            return False, "usuário aprovador não autenticado"
        return True, identity.user


    def _approve_cell_calibration(self) -> None:
        if len(self._calibration_profile.homography) != 9:
            if not self._validate_cell_calibration(show_dialog=True):
                return
        allowed, identity_or_reason = self._calibration_approval_identity()
        if not allowed:
            QMessageBox.warning(
                self, "Aprovação bloqueada", identity_or_reason
            )
            return
        reply = QMessageBox.question(
            self,
            "Aprovar calibração da célula",
            f"Aprovar e ativar {self._calibration_profile.name} "
            f"r{self._calibration_profile.revision}?\n\n"
            f"{self._calibration_profile.metrics.summary()}\n"
            f"Limites: RMS ≤ {DEFAULT_MAX_RMS_ERROR_M * 1000:.1f} mm; "
            f"máximo ≤ {DEFAULT_MAX_ERROR_M * 1000:.1f} mm.\n\n"
            "A receita liberada ficará vinculada ao hash deste perfil.",
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if reply != QMessageBox.Yes:
            return
        try:
            self._calibration_profile.approve(identity_or_reason)
            self._calibration_store.save(
                self._calibration_profile, activate=True
            )
        except (OSError, TypeError, ValueError) as exc:
            QMessageBox.critical(
                self, "Aprovação bloqueada", f"Perfil não aprovado:\n{exc}"
            )
            return
        self._use_calibration.setChecked(True)
        self._render_calibration_metrics()
        self._update_calibration_editability()
        self._apply_workspace()
        message = (
            f"Calibração {self._calibration_profile.name} "
            f"r{self._calibration_profile.revision} aprovada e ativa."
        )
        self.calibration_status_changed.emit(True, message)


    def _new_cell_calibration_revision(self) -> None:
        profile = self._calibration_profile
        if profile.status == STATUS_APPROVED:
            profile.start_new_revision()
        else:
            profile = CalibrationProfile(
                name=self._calibration_name.text().strip()
                or "calibração da célula",
                camera_frame=self._calibration_camera_frame.text().strip()
                or "camera_link",
                target_frame=self._calibration_target_frame.text().strip()
                or "base_link",
                tcp_frame=self._calibration_tcp_frame.text().strip()
                or "gripper_tcp",
                tool_id=self._calibration_tool_id.text().strip() or "gripper",
            )
        self._populate_calibration_form(profile)
        self._use_calibration.setChecked(False)
        self.calibration_status_changed.emit(False, "Nova revisão em edição.")


    def _load_cell_calibration_dialog(self) -> None:
        path, _selected = QFileDialog.getOpenFileName(
            self,
            "Carregar perfil de calibração",
            str(self._calibration_store.directory),
            "Calibração JSON (*.json);;Todos os arquivos (*)",
        )
        if not path:
            return
        try:
            profile = self._calibration_store.load_path(path)
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            QMessageBox.warning(
                self, "Carregar calibração", f"Perfil inválido:\n{exc}"
            )
            return
        self._populate_calibration_form(profile)
        if profile.status == STATUS_APPROVED:
            valid, reason = profile.verify_integrity()
            if not valid:
                self._use_calibration.setChecked(False)
                self.calibration_status_changed.emit(False, reason)
                QMessageBox.warning(
                    self, "Integridade da calibração", reason
                )


    def _load_active_cell_calibration(self) -> None:
        profile = self._calibration_store.active()
        if profile is None:
            reason = self._calibration_store.last_error or "calibração não configurada"
            self._active_calibration_label.setText("Calibração: não aprovada")
            self._calibration_status.setText(
                f"Produção bloqueada: {reason}. Crie e aprove um perfil."
            )
            self.calibration_status_changed.emit(False, reason)
            return
        self._populate_calibration_form(profile)
        valid, reason = self.production_calibration_status()
        self.calibration_status_changed.emit(valid, reason)
        if valid:
            self._apply_workspace()


    def production_calibration_status(
        self,
    ) -> tuple[bool, str, dict[str, object]]:
        valid, reason, profile = self._calibration_store.production_status(
            camera_frame=self._last_camera_frame,
            image_width=self._runtime_image_width,
            image_height=self._runtime_image_height,
            target_frame="base_link",
            tcp_frame="gripper_tcp",
            tool_id="gripper",
        )
        if profile is None:
            self._active_calibration_label.setText("Calibração: inválida")
            return False, reason, {}
        if valid:
            self._active_calibration_label.setText(
                f"Calibração: r{profile.revision} aprovada"
            )
            reason = (
                f"{profile.name} r{profile.revision} · "
                f"{profile.metrics.summary()}"
            )
        else:
            self._active_calibration_label.setText("Calibração: incompatível")
        return valid, reason, profile.production_reference()


    def _capture_arm_pose(self) -> None:
        pose = None
        getter = getattr(self._bridge, "current_tcp_pose", None)
        if callable(getter):
            pose = getter()
        if pose is None:
            pose = getattr(self._bridge, "last_reached_pose", None)
        position = getattr(pose, "position", None)
        if position is None:
            return
        index = int(self._calibration_index.value()) - 1
        if not (0 <= index < len(self._xy_spins)):
            return
        x_spin, y_spin = self._xy_spins[index]
        x_spin.setValue(float(position.x))
        y_spin.setValue(float(position.y))
        self._calibration_table_z.setValue(float(position.z))
        self._calibration_enabled[index].setChecked(True)
        if index + 1 < len(self._xy_spins):
            self._calibration_index.setValue(index + 2)
        self._calibration_status.setText(
            f"P{index + 1}: TCP capturado em "
            f"({position.x:+.4f}, {position.y:+.4f}, {position.z:+.4f}) m."
        )


    def _calibration_transform(self):
        try:
            import numpy as np
        except Exception:
            return None
        if len(self._calibration_profile.homography) != 9:
            if not self._validate_cell_calibration(show_dialog=False):
                return None
        try:
            return np.asarray(
                self._calibration_profile.homography, dtype=float
            ).reshape((3, 3))
        except (TypeError, ValueError):
            return None


    @staticmethod
    def _apply_transform(transform, u: float, v: float) -> tuple[float, float] | None:
        denom = float(transform[2, 0]) * float(u) + float(transform[2, 1]) * float(v) + float(transform[2, 2])
        if abs(denom) < 1e-9:
            return None
        x = (float(transform[0, 0]) * float(u) + float(transform[0, 1]) * float(v) + float(transform[0, 2])) / denom
        y = (float(transform[1, 0]) * float(u) + float(transform[1, 1]) * float(v) + float(transform[1, 2])) / denom
        return x, y


    def _load_horizontal_profile_dialog(self) -> None:
        self._load_orientation_profile_dialog("horizontal")


    def _load_vertical_profile_dialog(self) -> None:
        self._load_orientation_profile_dialog("vertical")


    def _load_orientation_profile_dialog(self, orientation: str) -> None:
        title = "Carregar perfil deitado/horizontal" if orientation == "horizontal" else "Carregar perfil em pé/vertical"
        path, _selected_filter = QFileDialog.getOpenFileName(
            self,
            title,
            str(self._default_external_profile_dir()),
            "JSON (*.json);;Todos os arquivos (*)",
        )
        if not path:
            return
        try:
            self._load_orientation_profile(path, orientation)
        except Exception as exc:
            self._profile_status.setText(f"Falha ao carregar perfil: {exc}")
            QMessageBox.warning(self, "Perfis externos de visão", f"Falha ao carregar perfil:\n{exc}")
            return
        self._apply()


    def _load_orientation_profile(self, path: str, orientation: str) -> None:
        if orientation not in {"horizontal", "vertical"}:
            raise ValueError(f"orientação inválida: {orientation}")
        config_path = Path(path).expanduser()
        with config_path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
        if not isinstance(data, dict):
            raise ValueError("o JSON deve conter um objeto")
        profile = self._extract_orientation_profile(data, orientation)
        self._orientation_profiles[orientation] = profile
        self._profile_files[orientation] = str(config_path)
        self._apply_workspace_from_profile(profile)
        label = "Deitado" if orientation == "horizontal" else "Em pé"
        if orientation == "horizontal":
            self._horizontal_profile_path.setText(f"{label}: {config_path}")
        else:
            self._vertical_profile_path.setText(f"{label}: {config_path}")
        self._profile_status.setText(f"Perfil {label.lower()} carregado. Clique em “Aplicar perfis” para usar os dois JSONs.")


    def _extract_orientation_profile(self, data: dict[str, object], orientation: str) -> dict[str, object]:
        candidate: object = None
        profiles = data.get("profiles")
        if isinstance(profiles, dict):
            candidate = profiles.get(orientation)
        if not isinstance(candidate, dict):
            orientation_profiles = data.get("orientation_profiles")
            if isinstance(orientation_profiles, dict):
                candidate = orientation_profiles.get(orientation)
        if not isinstance(candidate, dict) and self._looks_like_profile(data):
            candidate = data
        if not isinstance(candidate, dict):
            raise ValueError(
                f"não encontrei um perfil {orientation!r}. Use um JSON completo com profiles.{orientation} "
                "ou um JSON contendo diretamente os parâmetros do perfil."
            )

        profile = dict(candidate)
        workspace = data.get("workspace_limit")
        if isinstance(workspace, dict):
            self._merge_workspace_limit_into_profile(profile, workspace)

        try:
            from rebotarm_vision.vision_profile import CalibratedProfile

            if "morph_operation" not in profile:
                profile["morph_operation"] = self._signed_operation_value(
                    None,
                    profile.get("morph_open_iterations"),
                    profile.get("morph_close_iterations"),
                )
            if "edge_operation" not in profile:
                profile["edge_operation"] = self._signed_operation_value(
                    None,
                    profile.get("edge_erode_iterations"),
                    profile.get("edge_dilate_iterations"),
                )
            return CalibratedProfile.from_dict(profile).to_dict()
        except Exception:
            return profile


    @staticmethod
    def _looks_like_profile(data: dict[str, object]) -> bool:
        profile_keys = {
            "opencv_mode",
            "active_filters",
            "center_method",
            "direction_method",
            "direction_sense_method",
            "min_area_px",
            "max_area_px",
            "brightness_threshold",
            "workspace_limit_pixels",
        }
        return any(key in data for key in profile_keys)


    @staticmethod
    def _merge_workspace_limit_into_profile(profile: dict[str, object], workspace: dict[str, object]) -> None:
        shape = workspace.get("shape")
        if shape is not None:
            profile["workspace_limit_shape"] = str(shape)
        margin = workspace.get("margin_px")
        if margin is not None:
            profile["workspace_mask_margin_px"] = margin
        points = workspace.get("points")
        if isinstance(points, list):
            flat: list[float] = []
            for point in points:
                if isinstance(point, (list, tuple)) and len(point) >= 2:
                    flat.extend([float(point[0]), float(point[1])])
            if flat:
                profile["workspace_limit_pixels"] = flat
        circle = workspace.get("circle")
        if isinstance(circle, list) and len(circle) >= 3:
            profile["workspace_circle_pixel"] = [float(circle[0]), float(circle[1]), float(circle[2])]
        enabled = bool(workspace.get("enabled", False) or workspace.get("draw", False))
        if enabled:
            profile["use_workspace_mask"] = True
            profile["draw_workspace_limit"] = True


    @staticmethod
    def _default_external_profile_dir() -> Path:
        return Path.home()



    def _on_service_result(self, label: str, success: bool, message: str) -> None:
        if label != "vision_calibrate":
            return
        if not success:
            self._warn_ai_autotune(f"Auto-calibração IA falhou: {message}")
            return
        self._set_ai_autotune_status("Auto-calibração IA: concluída.", busy=False)


