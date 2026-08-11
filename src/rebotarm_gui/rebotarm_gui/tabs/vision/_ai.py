# ruff: noqa: F401,F403,F405
"""VisionTab ai mixin — extracted from vision_tab.py."""

from __future__ import annotations

from ._common import *  # noqa: F403


class VisionTabAiMixin:
    """Ai concern methods for VisionTab."""

    def _set_ai_autotune_status(self, message: str, *, busy: bool = False) -> None:
        self._ai_autotune_busy = bool(busy)
        self._autotune_status.setText(message)
        self._preview_status.setText(message)
        self._btn_autotune_ai.setEnabled(not busy)
        self._btn_autotune_ai.setText("IA processando..." if busy else "Auto-calibrar com IA")
        self._btn_load_image.setEnabled(not busy)
        self._btn_publish_image.setEnabled(not busy)
        self._btn_apply.setEnabled(not busy)
        self._btn_process.setEnabled(not busy)
        self._btn_bottles.setEnabled(not busy)
        self._btn_color.setEnabled(not busy)
        if busy:
            self._btn_approve_calibration.setEnabled(False)
            self._btn_retry_calibration.setEnabled(False)
            self._btn_complement_calibration.setEnabled(False)
        fit_button_text(self._btn_autotune_ai)
        self._audit_ai_autotune_event(message)
        if self._ros_log is not None:
            level = "info" if busy or not message.lower().startswith("auto-tune ia falhou") else "erro"
            self._ros_log.append(f"[{level}] {message}")


    def _warn_ai_autotune(self, message: str) -> None:
        self._set_ai_autotune_status(message, busy=False)
        QMessageBox.warning(self, "Auto-calibração por IA", message)


    def _audit_ai_autotune_event(self, message: str) -> None:
        try:
            log_dir = Path.home() / ".ros" / "log"
            log_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().isoformat(timespec="seconds")
            line = f"{timestamp} {message}\n"
            with (log_dir / "rebotarm_gui_ai_autotune.log").open("a", encoding="utf-8") as handle:
                handle.write(line)
        except Exception:
            pass


    def _start_ai_autotune(self) -> None:
        self._set_ai_autotune_status("Auto-calibração IA: preparando imagem e prompt...", busy=False)
        image = self._image_for_autotune()
        if image is None or image.isNull():
            self._warn_ai_autotune(
                "Auto-calibração IA: carregue uma imagem de teste ou publique/capture um frame antes de ajustar."
            )
            return
        ros_image = self._qimage_to_ros_image(image)
        if ros_image is None:
            self._warn_ai_autotune("Auto-calibração IA: falha ao converter imagem para ROS.")
            return

        prompt = self._autotune_prompt.toPlainText().strip()
        self._set_ai_autotune_status(
            f"Auto-calibração IA: enviando imagem ({ros_image.width}x{ros_image.height}) para o serviço de calibração...",
            busy=True,
        )
        self._last_calibration_image = image
        calibrate = getattr(self._bridge, "vision_calibrate", None)
        if not callable(calibrate):
            self._warn_ai_autotune("Auto-calibração IA: bridge não possui vision_calibrate.")
            return
        self.async_operation_started.emit(
            "vision_ai_calibration",
            "Calibração de visão por IA iniciada.",
        )
        # Credenciais e provedor são responsabilidade do nó de calibração; a
        # GUI envia apenas os dados necessários ao comando.
        ok = calibrate(ros_image, prompt)
        if not ok:
            self.async_operation_finished.emit(
                "vision_ai_calibration",
                False,
                "Serviço de calibração indisponível.",
            )
            self._warn_ai_autotune("Auto-calibração IA: serviço de calibração indisponível.")


    def _on_ai_calibrate_progress(self, text: str) -> None:
        self._set_ai_autotune_status(f"Auto-calibração IA: {text}", busy=True)


    def _on_ai_calibrate_result(self, profile_json: str, report: str) -> None:
        try:
            from rebotarm_vision.vision_profile import VisionProfile

            profile = VisionProfile.from_json(profile_json)
        except Exception as exc:
            self._warn_ai_autotune(f"Auto-calibração IA: resposta inválida: {exc}")
            return

        self._orientation_profiles = {
            name: profile.profiles[name].to_dict()
            for name in ("vertical", "horizontal")
            if name in profile.profiles
        }
        self._set_ai_autotune_status(
            f"Auto-calibração IA: perfil recebido ({profile.status.reason})", busy=False
        )
        self._preview_status.setText("Auto-calibração IA: processando frame com perfil calibrado...")
        if self._last_calibration_image is not None and not self._last_calibration_image.isNull():
            self._publish_test_image(self._last_calibration_image)
        processor = getattr(self._bridge, "vision_process_frame", None)
        if callable(processor):
            QTimer.singleShot(250, processor)

        if report:
            if self._ros_log is not None:
                self._ros_log.append(f"[info] Auto-calibração IA — relatório:\n{report}")
        self._btn_approve_calibration.setEnabled(profile.status.converged)
        self._btn_retry_calibration.setEnabled(True)
        self._btn_complement_calibration.setEnabled(True)


    def _approve_calibration(self) -> None:
        self._set_ai_autotune_status("Calibração aprovada pelo operador.", busy=False)
        self._audit_ai_autotune_event("Calibração aprovada pelo operador.")
        self._btn_approve_calibration.setEnabled(False)


    def _complement_calibration_text(self) -> None:
        self._autotune_prompt.setFocus()
        self._set_ai_autotune_status(
            "Edite o prompt do cenário e pressione Auto-calibrar com IA para repetir.", busy=False
        )


    @staticmethod
    def _qimage_to_ros_image(
        image: QImage, *, msg: SensorImage | None = None
    ) -> SensorImage | None:
        """Convert a QImage into a sensor_msgs/Image (rgb8 or bgr8).

        When *image* is already in a 3-channel RGB/BGR format the expensive
        ``convertToFormat`` deep copy is skipped and the native pixel buffer is
        published directly.  Pass a reusable *msg* to avoid reallocating the
        message object (and its internal ``data`` array) on every frame.
        """
        fmt = image.format()
        if fmt == QImage.Format.Format_BGR888:
            rgb = image
            encoding = "bgr8"
        elif fmt == QImage.Format.Format_RGB888:
            rgb = image
            encoding = "rgb8"
        else:
            rgb = image.convertToFormat(QImage.Format.Format_RGB888)
            encoding = "rgb8"
        if rgb.isNull():
            return None

        out = msg if msg is not None else SensorImage()
        out.height = int(rgb.height())
        out.width = int(rgb.width())
        out.encoding = encoding
        out.is_bigendian = False
        out.step = int(rgb.bytesPerLine())

        size = rgb.sizeInBytes()
        ptr = rgb.constBits()
        if ptr is None:
            return None
        try:
            ptr.setsize(size)
        except (AttributeError, TypeError):
            pass
        out.data = bytes(ptr)
        return out


    def _image_for_autotune(self) -> QImage | None:
        if self._test_image is not None and not self._test_image.isNull():
            return self._test_image.convertToFormat(QImage.Format.Format_RGB888)
        if self._last_raw_pixmap is not None and not self._last_raw_pixmap.isNull():
            return self._last_raw_pixmap.toImage().convertToFormat(QImage.Format.Format_RGB888)
        if self._last_pixmap is not None and not self._last_pixmap.isNull():
            return self._last_pixmap.toImage().convertToFormat(QImage.Format.Format_RGB888)
        return None


    def _load_yolo_model_dialog(self) -> None:
        path, _selected = QFileDialog.getOpenFileName(
            self,
            "Carregar modelo YOLO pré-calibrado",
            str(self._default_external_profile_dir()),
            "Modelos YOLO (*.pt *.onnx *.engine *.tflite *.torchscript *.xml);;Todos os arquivos (*)",
        )
        if path:
            self._load_yolo_model(path)
            self._apply()


    def _load_yolo_model(self, path: str) -> None:
        model_path = Path(path).expanduser()
        if not model_path.exists():
            raise FileNotFoundError(f"modelo YOLO não encontrado: {model_path}")
        self._yolo_model_path = str(model_path.resolve())
        self._detector_backend = "yolo"
        self._yolo_model_label.setText(f"YOLO: {self._yolo_model_path}")
        self._preview_status.setText("Modelo selecionado; aguardando confirmação do nó de visão.")


    def _load_task_config_dialog(self) -> None:
        path, _selected = QFileDialog.getOpenFileName(
            self,
            "Carregar configuração completa de visão e pega",
            str(self._default_external_profile_dir()),
            "JSON (*.json);;Todos os arquivos (*)",
        )
        if not path:
            return
        try:
            self._load_task_config(path)
            self._apply_task_configuration()
        except Exception as exc:
            QMessageBox.warning(self, "Configuração de visão", f"Falha ao carregar JSON:\n{exc}")


    def _save_task_config_dialog(self) -> None:
        path, _selected = QFileDialog.getSaveFileName(
            self,
            "Salvar configuração completa de visão e pega",
            str(self._default_task_config_path()),
            "JSON (*.json);;Todos os arquivos (*)",
        )
        if not path:
            return
        target = self._current_task_config().save(path)
        self._preview_status.setText(f"Configuração completa salva: {target}")


    def _load_task_config(self, path: str) -> None:
        source = Path(path).expanduser().resolve()
        config = VisionTaskConfig.load(source)
        model_path = Path(config.detector.model_path).expanduser() if config.detector.model_path else None
        if model_path is not None and not model_path.is_absolute():
            model_path = source.parent / model_path
        self._yolo_model_path = str(model_path.resolve()) if model_path is not None else ""
        self._detector_backend = config.detector.backend
        self._yolo_model_label.setText(
            f"YOLO: {self._yolo_model_path}" if self._yolo_model_path else "Nenhum modelo YOLO carregado"
        )
        self._yolo_confidence_percent.setValue(config.detector.confidence_percent)
        self._yolo_image_size.setValue(config.detector.image_size)
        self._target_classes.setText(", ".join(config.detector.target_classes))
        self._object_orientation.setCurrentText(config.object.orientation)
        self._auxiliary_attribute.setText(config.object.auxiliary_attribute)
        self._grasp_clearance.setValue(int(round(config.grasp.clearance_px)))
        self._min_clearance_horizontal.setValue(int(round(config.grasp.min_clearance_horizontal_px)))
        self._min_clearance_vertical.setValue(int(round(config.grasp.min_clearance_vertical_px)))
        self._pick_route.setCurrentText(config.pick_route.strategy)
        self._pick_max_objects.setValue(config.pick_route.max_objects)
        self._pick_stable_frames.setValue(config.pick_route.stable_frames)
        self._route_origin_x.setValue(config.pick_route.origin_xy_m[0])
        self._route_origin_y.setValue(config.pick_route.origin_xy_m[1])

        workspace = config.workspace
        self._limit_mode.setCurrentText("limite desenhado" if workspace.enabled else "desativado")
        self._limit_shape.setCurrentText(workspace.shape)
        self._limit_points = [(float(x), float(y)) for x, y in workspace.vertices]
        self._limit_circle = [float(value) for value in workspace.circle]
        self._limit_margin_px.setValue(int(round(workspace.clearance_px)))
        self._orientation_profiles = {
            name: dict(profile) for name, profile in config.orientation_profiles.items()
        }
        self._refresh_limit_summary()
        self._on_limit_mode_changed(self._limit_mode.currentText())
        self._profile_status.setText(f"Configuração completa carregada: {source}")


    def _current_task_config(self) -> VisionTaskConfig:
        workspace = self._current_workspace_limit()
        return VisionTaskConfig.from_dict(
            {
                "schema_version": 3,
                "detector": {
                    "backend": self._detector_backend,
                    "model_path": self._yolo_model_path,
                    "confidence_percent": self._yolo_confidence_percent.value(),
                    "image_size": self._yolo_image_size.value(),
                    "target_classes": [
                        value.strip()
                        for value in self._target_classes.text().split(",")
                        if value.strip()
                    ],
                },
                "object": {
                    "orientation": self._object_orientation.currentText(),
                    "auxiliary_attribute": self._auxiliary_attribute.text().strip(),
                },
                "workspace": {
                    "enabled": workspace.get("enabled", False),
                    "shape": workspace.get("shape", "polygon"),
                    "vertices": workspace.get("points", []),
                    "circle": workspace.get("circle", []),
                    "clearance_px": workspace.get("margin_px", 0),
                },
                "grasp": {
                    "clearance_px": self._grasp_clearance.value(),
                    "min_clearance_horizontal_px": self._min_clearance_horizontal.value(),
                    "min_clearance_vertical_px": self._min_clearance_vertical.value(),
                },
                "pick_route": {
                    "strategy": self._pick_route.currentText(),
                    "max_objects": self._pick_max_objects.value(),
                    "stable_frames": self._pick_stable_frames.value(),
                    "origin_xy_m": [
                        self._route_origin_x.value(),
                        self._route_origin_y.value(),
                    ],
                },
                "orientation_profiles": self._orientation_profiles,
            }
        )
