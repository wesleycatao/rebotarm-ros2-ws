# ruff: noqa: F401,F403,F405
"""VisionTab classic mixin — extracted from vision_tab.py."""

from __future__ import annotations

from ._common import *  # noqa: F403


class VisionTabClassicMixin:
    """Classic concern methods for VisionTab."""

    def _preset_bottles(self) -> None:
        self._mode.setCurrentText("bright_or_edge")
        self._process_mode.setCurrentText("manual")
        self._max_processing_fps.setValue(1.0)
        self._center.setCurrentText("moments")
        self._direction.setCurrentText("pca")
        self._sense.setCurrentText("bright_end")
        self._shape_filter.setCurrentText("any")
        self._brightness.setValue(105)
        self._brightness_invert.setChecked(False)
        self._blur_kernel.setValue(5)
        self._canny_low.setValue(45)
        self._canny_high.setValue(130)
        self._edge_aperture.setCurrentText("3")
        self._morph.setValue(7)
        self._morph_operation.setValue(1)
        self._edge_operation.setValue(1)
        self._min_area.setValue(500)
        self._max_area.setValue(70000)
        self._lying_ratio.setValue(1.8)
        self._min_solidity.setValue(0.0)
        self._min_extent.setValue(0.0)
        self._max_extent.setValue(1.0)
        self._min_aspect.setValue(1.2)
        self._max_aspect.setValue(6.5)
        self._set_active_filters(["detect_bright", "detect_edge", "min_area", "max_area", "min_aspect", "max_aspect"])
        self._apply()


    def _preset_color(self) -> None:
        self._mode.setCurrentText("hsv")
        self._shape_filter.setCurrentText("any")
        self._s_min.setValue(45)
        self._v_min.setValue(45)
        self._set_active_filters(["detect_hsv", "min_area", "max_area"])
        self._apply()


    @staticmethod
    def _detection_keys_from_mode(mode: str) -> list[str]:
        tokens = set(str(mode).split("_or_"))
        keys: list[str] = []
        if "hsv" in tokens:
            keys.append("detect_hsv")
        if "bright" in tokens:
            keys.append("detect_bright")
        if "edge" in tokens:
            keys.append("detect_edge")
        return keys


    def _selected_detection_mode(self) -> str:
        selected: list[str] = []
        active_filters = self._active_filter_keys()
        if "detect_hsv" in active_filters:
            selected.append("hsv")
        if "detect_bright" in active_filters:
            selected.append("bright")
        if "detect_edge" in active_filters:
            selected.append("edge")
        if not selected:
            return "bright_or_edge"
        if len(selected) == 1:
            return selected[0]
        return "_or_".join(selected)


    def _add_filter_limit(self) -> None:
        label = self._filter_selector.currentText()
        key = self._filter_key_for_label(label)
        if not key or key in self._active_filter_keys():
            return
        item = QListWidgetItem(label)
        item.setData(Qt.ItemDataRole.UserRole, key)
        self._filter_list.addItem(item)


    def _remove_filter_limit(self) -> None:
        for item in self._filter_list.selectedItems():
            row = self._filter_list.row(item)
            self._filter_list.takeItem(row)


    def _active_filter_keys(self) -> set[str]:
        keys: set[str] = set()
        for row in range(self._filter_list.count()):
            item = self._filter_list.item(row)
            key = item.data(Qt.ItemDataRole.UserRole)
            if key:
                keys.add(str(key))
        return keys


    def _set_active_filters(self, keys: list[str]) -> None:
        self._filter_list.clear()
        seen: set[str] = set()
        for key in keys:
            if key in seen:
                continue
            label = self._filter_options.get(key)
            if not label:
                continue
            seen.add(key)
            item = QListWidgetItem(label)
            item.setData(Qt.ItemDataRole.UserRole, key)
            self._filter_list.addItem(item)


    def _filter_key_for_label(self, label: str) -> str | None:
        for key, option_label in self._filter_options.items():
            if option_label == label:
                return key
        return None


    def _clear_detection_view(self, status: str = "Preview aguardando imagem") -> None:
        self._last_objects = []
        self._last_pixmap = None
        self._preview.clear()
        if self._last_raw_pixmap is not None and not self._last_raw_pixmap.isNull():
            self._last_pixmap = self._last_raw_pixmap
            status = f"{status} usando frame cru {self._last_image_width}x{self._last_image_height}"
            self._refresh_preview()
        else:
            self._preview.setText("Sem imagem processada")
        self._preview_status.setText(status)
        self._objects.setRowCount(0)
        self._btn_continue_grasp.setEnabled(False)


    def _load_test_image(self) -> None:
        if self._ai_autotune_busy:
            self._set_ai_autotune_status(
                "Auto-calibração IA: aguarde a iteração atual terminar antes de trocar a imagem.",
                busy=True,
            )
            return
        path, _selected_filter = QFileDialog.getOpenFileName(
            self,
            "Escolher imagem de teste",
            "",
            "Imagens (*.png *.jpg *.jpeg *.bmp);;Todos os arquivos (*)",
        )
        if not path:
            return
        image = QImage(path)
        if image.isNull():
            self._test_path.setText("Falha ao carregar imagem")
            return
        self._test_image = image
        self._test_path.setText(path)
        self._set_preview_image(image, f"Preview: imagem carregada {image.width()}x{image.height()}", raw=True)


    def _load_default_test_image(self) -> None:
        path = self._default_test_image_path()
        if path is None:
            self._test_path.setText("Nenhuma imagem carregada")
            return
        image = QImage(str(path))
        if image.isNull():
            self._test_path.setText("Falha ao carregar imagem padrão")
            return
        self._test_image = image
        self._test_path.setText(f"Padrão: {path}")
        self._set_preview_image(
            image,
            f"Preview: imagem padrão {image.width()}x{image.height()}",
            raw=True,
        )


    def _publish_test_image(self, image: QImage | None = None) -> None:
        if self._ai_autotune_busy and image is None:
            self._set_ai_autotune_status(
                "Auto-calibração IA: aguarde a iteração atual terminar antes de publicar outra imagem.",
                busy=True,
            )
            return
        source_image = image if image is not None else self._test_image
        if source_image is None:
            return
        self._apply()
        if source_image.format() == QImage.Format.Format_RGB888:
            rgb = source_image
        else:
            rgb = source_image.convertToFormat(QImage.Format.Format_RGB888)
        ptr = rgb.constBits()
        if ptr is None:
            return
        try:
            ptr.setsize(rgb.sizeInBytes())
            data = bytes(ptr)
        except (AttributeError, TypeError):
            bits = rgb.bits()
            if bits is None:
                return
            data = bytes(bits)
        self._bridge.publish_vision_test_image(
            rgb.width(),
            rgb.height(),
            data,
            rgb.bytesPerLine(),
        )
        processor = getattr(self._bridge, "vision_process_frame", None)
        if callable(processor):
            QTimer.singleShot(250, processor)


    @staticmethod
    def _set_spin_value(spin, value) -> None:
        if value is None:
            return
        try:
            if isinstance(spin, QSpinBox):
                spin.setValue(int(float(value)))
            elif isinstance(spin, QDoubleSpinBox):
                spin.setValue(float(value))
            elif hasattr(spin, "setValue"):
                spin.setValue(value)
            elif isinstance(spin, QComboBox):
                spin.setCurrentText(str(value))
            else:
                spin.setValue(float(value))
        except Exception:
            return


    @staticmethod
    def _positive_part(value: int | float) -> int:
        return max(0, int(round(float(value))))


    @staticmethod
    def _negative_part(value: int | float) -> int:
        return max(0, -int(round(float(value))))


    @staticmethod
    def _signed_operation_value(value, negative_source, positive_source) -> int | None:
        if value is not None:
            try:
                return int(round(float(value)))
            except Exception:
                return None
        try:
            positive = int(round(float(positive_source)))
        except Exception:
            positive = 0
        try:
            negative = int(round(float(negative_source)))
        except Exception:
            negative = 0
        if positive > 0:
            return positive
        if negative > 0:
            return -negative
        return 0


    @staticmethod
    def _default_test_image_path() -> Path | None:
        current = Path(__file__).resolve()
        candidates = [
            Path.cwd() / "src" / "rebotarm_vision" / "test_images" / DEFAULT_TEST_IMAGE,
            Path.cwd() / "install" / "rebotarm_vision" / "share" / "rebotarm_vision" / "test_images" / DEFAULT_TEST_IMAGE,
            current.parents[3] / "rebotarm_vision" / "test_images" / DEFAULT_TEST_IMAGE,
            Path.cwd() / "src" / "rebotarm_vision" / "test_images" / FALLBACK_TEST_IMAGE,
            Path.cwd() / "install" / "rebotarm_vision" / "share" / "rebotarm_vision" / "test_images" / FALLBACK_TEST_IMAGE,
            current.parents[3] / "rebotarm_vision" / "test_images" / FALLBACK_TEST_IMAGE,
        ]
        for path in candidates:
            if path.is_file():
                return path
        return None


    @staticmethod
    def _combo(items: list[str]) -> RetractingComboBox:
        combo = RetractingComboBox()
        combo.addItems(items)
        combo.setMinimumContentsLength(12)
        combo.setMinimumWidth(128)
        combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        combo.view().pressed.connect(lambda _index, widget=combo: QTimer.singleShot(0, widget.hidePopup))
        combo.view().clicked.connect(lambda _index, widget=combo: QTimer.singleShot(0, widget.hidePopup))
        return combo


    @staticmethod
    def _int_combo(minimum: int, maximum: int, value: int) -> IntComboBox:
        combo = IntComboBox(minimum, maximum, value)
        combo.setMinimumContentsLength(max(len(str(minimum)), len(str(maximum)), len(str(value))))
        combo.setMinimumWidth(58)
        combo.setMaximumWidth(74)
        combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        combo.view().pressed.connect(lambda _index, widget=combo: QTimer.singleShot(0, widget.hidePopup))
        combo.view().clicked.connect(lambda _index, widget=combo: QTimer.singleShot(0, widget.hidePopup))
        return combo


    @staticmethod
    def _spin(minimum: int, maximum: int, value: int, suffix: str = "") -> QSpinBox:
        spin = QSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSuffix(suffix)
        spin.lineEdit().setAlignment(Qt.AlignmentFlag.AlignRight)
        widest = max(len(f"{minimum}{suffix}"), len(f"{maximum}{suffix}"), len(f"{value}{suffix}"))
        width = max(92, min(180, 44 + widest * 9))
        spin.setMinimumWidth(width)
        spin.setMaximumWidth(width + 28)
        return spin


    @staticmethod
    def _double_spin(
        minimum: float,
        maximum: float,
        value: float,
        step: float = 0.1,
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(3)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.lineEdit().setAlignment(Qt.AlignmentFlag.AlignRight)
        widest = max(len(f"{minimum:.3f}"), len(f"{maximum:.3f}"), len(f"{value:.3f}"))
        width = max(82, min(132, 34 + widest * 8))
        spin.setMinimumWidth(width)
        spin.setMaximumWidth(width + 16)
        return spin


    @staticmethod
    def _label(text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("FieldLabel")
        return label

    @staticmethod
    def _bbox_side_metrics(obj) -> tuple[float | None, float | None, float | None]:
        points = list(getattr(obj, "bounding_box_pixels", []))
        if len(points) < 4:
            return None, None, None
        lengths: list[float] = []
        for index in range(4):
            start = points[index]
            end = points[(index + 1) % 4]
            lengths.append(math.hypot(float(end.x) - float(start.x), float(end.y) - float(start.y)))
        positive = [value for value in lengths if value > 1e-6]
        if not positive:
            return None, None, None
        short_side = min(positive)
        long_side = max(positive)
        return short_side, long_side, long_side / short_side if short_side > 1e-6 else None


    @staticmethod
    def _format_optional_float(value: float | None, decimals: int) -> str:
        if value is None:
            return "--"
        return f"{value:.{decimals}f}"


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


