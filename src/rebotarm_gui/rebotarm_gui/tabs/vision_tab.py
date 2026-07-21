"""Computer-vision configuration tab."""

from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHeaderView,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from sensor_msgs.msg import Image as SensorImage
from rebotarm_vision.task_config import VisionTaskConfig

from ..settings import AppSettings
from ..widgets.combo_box import RetractingComboBox
from ..widgets.factory import make_button
from ..widgets.layout_utils import fit_button_text


VISION_SIDE_PANEL_WIDTH = 600
DEFAULT_TEST_IMAGE = "default_vision_test.png"
FALLBACK_TEST_IMAGE = "garrafas.png"
DEFAULT_AI_SCENARIO_PROMPT = (
    "25 garrafas PET transparentes e vazias. "
    "Detectar todas as unidades individuais na imagem. "
    "A zona de pega segura fica no centro do corpo de cada garrafa."
)


class VisionPreviewLabel(QLabel):
    image_clicked = Signal(float, float)

    def __init__(self, text: str = "", parent: QWidget | None = None) -> None:
        super().__init__(text, parent)
        self._source_width = 0
        self._source_height = 0
        self._overlay_text = ""
        self.setAccessibleName("Pré-visualização da câmera e área de trabalho")
        self.setAccessibleDescription(
            "Imagem usada para revisar detecções e, quando habilitado, marcar limites operacionais."
        )

    def set_source_size(self, width: int, height: int) -> None:
        self._source_width = int(width)
        self._source_height = int(height)

    def set_overlay_text(self, text: str) -> None:
        self._overlay_text = str(text)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        super().paintEvent(event)
        pixmap = self.pixmap()
        if pixmap is None or not self._overlay_text:
            return
        shown_w = float(pixmap.width())
        shown_h = float(pixmap.height())
        offset_x = max(0.0, (float(self.width()) - shown_w) * 0.5)
        offset_y = max(0.0, (float(self.height()) - shown_h) * 0.5)
        margin = 10
        pad_x = 8
        pad_y = 5
        max_width = max(40, int(shown_w) - margin * 2)
        painter = QPainter(self)
        metrics = painter.fontMetrics()
        text = metrics.elidedText(self._overlay_text, Qt.TextElideMode.ElideRight, max_width - pad_x * 2)
        text_width = metrics.horizontalAdvance(text)
        box_width = min(max_width, text_width + pad_x * 2)
        box_height = metrics.height() + pad_y * 2
        x = int(round(offset_x)) + margin
        y = int(round(offset_y + shown_h)) - margin - box_height
        painter.fillRect(x, y, box_width, box_height, QColor(0, 0, 0, 220))
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(x + pad_x, y + pad_y + metrics.ascent(), text)
        painter.end()

    def mousePressEvent(self, event) -> None:  # noqa: N802
        pixmap = self.pixmap()
        if pixmap is None or self._source_width <= 0 or self._source_height <= 0:
            super().mousePressEvent(event)
            return
        pos = event.position() if hasattr(event, "position") else event.pos()
        px = float(pos.x())
        py = float(pos.y())
        shown_w = float(pixmap.width())
        shown_h = float(pixmap.height())
        offset_x = max(0.0, (float(self.width()) - shown_w) * 0.5)
        offset_y = max(0.0, (float(self.height()) - shown_h) * 0.5)
        if px < offset_x or py < offset_y or px > offset_x + shown_w or py > offset_y + shown_h:
            return
        u = (px - offset_x) * float(self._source_width) / max(shown_w, 1.0)
        v = (py - offset_y) * float(self._source_height) / max(shown_h, 1.0)
        self.image_clicked.emit(u, v)


class PreviewStatusLabel(QLabel):
    def __init__(self, preview: VisionPreviewLabel, text: str = "") -> None:
        super().__init__(text)
        self._preview = preview
        self._preview.set_overlay_text(text)

    def setText(self, text: str) -> None:  # noqa: N802
        super().setText(text)
        self._preview.set_overlay_text(text)


class IntComboBox(RetractingComboBox):
    def __init__(self, minimum: int, maximum: int, value: int) -> None:
        super().__init__()
        self.addItems([str(number) for number in range(minimum, maximum + 1)])
        self.setCurrentText(str(value))

    def value(self) -> int:
        try:
            return int(self.currentText())
        except ValueError:
            return 0

    def setValue(self, value: int | float) -> None:  # noqa: N802
        self.setCurrentText(str(int(round(float(value)))))


class VisionTab(QWidget):
    """Controls for non-neural image processing and object suggestions."""

    workspace_limit_changed = Signal(object)
    planning_config_changed = Signal(object)
    continue_to_grasp = Signal()

    def __init__(
        self,
        bridge,
        ros_log: QTextEdit | None = None,
        settings: AppSettings | None = None,
        show_ros_log: bool = True,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._ros_log = ros_log
        self._show_ros_log = bool(show_ros_log)
        self._settings = settings or AppSettings()
        self._last_pixmap: QPixmap | None = None
        self._last_raw_pixmap: QPixmap | None = None
        self._last_image_width = 0
        self._last_image_height = 0
        self._last_objects: list[object] = []
        self._test_image: QImage | None = None
        self._last_calibration_image: QImage | None = None
        self._ai_autotune_busy = False
        self._overlay_refresh_pending = False
        self._orientation_profiles: dict[str, dict[str, object]] = {}
        self._profile_files: dict[str, str] = {}
        self._yolo_model_path = ""
        self._detector_backend = "yolo"

        self._mode = self._combo(
            ["bright_or_edge", "edge", "bright", "hsv", "hsv_or_edge", "hsv_or_bright"]
        )
        self._center = self._combo(["moments", "min_area_rect", "bounding_box", "contour_mean"])
        self._direction = self._combo(["min_area_rect", "pca", "none"])
        self._sense = self._combo(
            ["none", "bright_end", "dark_end", "positive_x", "negative_x", "positive_y", "negative_y"]
        )
        self._process_mode = self._combo(["manual", "continuous"])
        self._max_processing_fps = self._double_spin(0.0, 30.0, 1.0, step=0.5)
        self._draw_clearance = QCheckBox("Desenhar clearances")
        self._draw_clearance.setChecked(True)
        self._shape_filter = self._combo(
            ["any", "circle", "square", "rectangle", "triangle", "object"]
        )

        self._min_area = self._spin(0, 2073600, 1500, suffix=" px²")
        self._max_area = self._spin(0, 2073600, 0, suffix=" px²")
        self._min_aspect = self._double_spin(0.0, 20.0, 0.0, step=0.1)
        self._max_aspect = self._double_spin(0.0, 20.0, 0.0, step=0.1)
        self._morph = self._spin(1, 31, 7)
        self._morph_operation = self._int_combo(-8, 8, 1)
        self._roi_margin = self._spin(0, 200, 12, suffix=" px")
        self._brightness = self._spin(0, 255, 105)
        self._brightness_invert = QCheckBox("Inverter brilho")
        self._blur_kernel = self._spin(1, 31, 5)
        self._canny_low = self._spin(0, 255, 45)
        self._canny_high = self._spin(0, 255, 130)
        self._edge_aperture = self._combo(["3", "5", "7"])
        self._edge_aperture.setMinimumWidth(56)
        self._edge_aperture.setMaximumWidth(64)
        self._edge_operation = self._int_combo(-8, 8, 1)
        self._min_area.setToolTip("Área mínima do contorno em pixels quadrados (px²).")
        self._max_area.setToolTip("Área máxima do contorno em pixels quadrados (px²). Zero desativa o limite absoluto.")
        self._min_aspect.setToolTip("Razão mínima lado longo/lado curto do bbox rotacionado. Zero desativa.")
        self._max_aspect.setToolTip("Razão máxima lado longo/lado curto do bbox rotacionado. Zero desativa.")
        self._brightness.setToolTip("Limiar de brilho no canal cinza, de 0 a 255.")
        self._canny_low.setToolTip("Limiar inferior do detector de bordas Canny, de 0 a 255.")
        self._canny_high.setToolTip("Limiar superior do detector de bordas Canny, de 0 a 255.")
        self._edge_aperture.setToolTip("Tamanho da janela do operador Sobel usado pelo Canny, em pixels.")
        self._morph_operation.setToolTip("Operação morfológica: valores negativos abrem, positivos fecham, zero desativa.")
        self._edge_operation.setToolTip("Ajuste da borda: valores negativos erodem, positivos dilatam, zero desativa.")

        self._filter_options = {
            "detect_bright": "Detecção: brilho",
            "detect_edge": "Detecção: bordas",
            "detect_hsv": "Detecção: cor HSV",
            "detect_rgb": "Detecção: cor RGB",
            "min_area": "Área mínima",
            "max_area": "Área máxima",
            "min_aspect": "Razão BBox mínima",
            "max_aspect": "Razão BBox máxima",
            "shape_filter": "Forma geométrica",
            "min_solidity": "Solidez mínima",
            "min_extent": "Extent mínimo",
            "max_extent": "Extent máximo",
        }
        self._filter_selector = self._combo(list(self._filter_options.values()))
        self._filter_list = QListWidget()
        self._filter_list.setMaximumHeight(96)
        self._btn_add_filter = QPushButton("Adicionar limitador")
        self._btn_remove_filter = QPushButton("Remover selecionado")
        self._btn_add_filter.clicked.connect(self._add_filter_limit)
        self._btn_remove_filter.clicked.connect(self._remove_filter_limit)

        self._h_min = self._spin(0, 179, 0)
        self._s_min = self._spin(0, 255, 40)
        self._v_min = self._spin(0, 255, 40)
        self._h_max = self._spin(0, 179, 179)
        self._s_max = self._spin(0, 255, 255)
        self._v_max = self._spin(0, 255, 255)

        self._shape_eps = self._double_spin(0.005, 0.20, 0.035, step=0.005)
        self._circle_circ = self._double_spin(0.10, 1.0, 0.72, step=0.01)
        self._square_tol = self._double_spin(0.01, 1.0, 0.22, step=0.01)
        self._lying_ratio = self._double_spin(1.0, 10.0, 1.8, step=0.1)
        self._min_solidity = self._double_spin(0.0, 1.0, 0.0, step=0.02)
        self._min_extent = self._double_spin(0.0, 1.0, 0.0, step=0.02)
        self._max_extent = self._double_spin(0.0, 1.0, 1.0, step=0.02)

        self._btn_apply = make_button("Aplicar visão")
        self._btn_process = make_button("Processar 1 frame", primary=True)
        self._btn_bottles = make_button("Preset garrafas")
        self._btn_color = make_button("Preset cor")
        self._btn_apply.clicked.connect(self._apply)
        self._btn_process.clicked.connect(self._process_frame)
        self._btn_bottles.clicked.connect(self._preset_bottles)
        self._btn_color.clicked.connect(self._preset_color)

        self._autotune_prompt = QTextEdit()
        self._autotune_prompt.setPlainText(DEFAULT_AI_SCENARIO_PROMPT)
        self._autotune_prompt.setPlaceholderText(
            "Informe a quantidade esperada, o tipo de objeto e, se necessário, a zona de pega permitida."
        )
        self._autotune_prompt.setMinimumHeight(120)
        self._autotune_prompt.setMaximumHeight(180)
        self._btn_autotune_ai = QPushButton("Auto-calibrar com IA")
        self._btn_autotune_ai.setObjectName("PrimaryButton")
        fit_button_text(self._btn_autotune_ai)
        self._btn_autotune_ai.clicked.connect(self._start_ai_autotune)
        self._btn_approve_calibration = QPushButton("Aprovar calibração")
        self._btn_retry_calibration = QPushButton("Repetir")
        self._btn_complement_calibration = QPushButton("Complementar texto")
        for button in (
            self._btn_approve_calibration,
            self._btn_retry_calibration,
            self._btn_complement_calibration,
        ):
            fit_button_text(button)
            button.setEnabled(False)
        self._btn_approve_calibration.clicked.connect(self._approve_calibration)
        self._btn_retry_calibration.clicked.connect(self._start_ai_autotune)
        self._btn_complement_calibration.clicked.connect(self._complement_calibration_text)

        # Status da auto-calibração IA: widget interno (o grupo visual não é
        # exposto no fluxo novo), atualizado por _set_ai_autotune_status.
        self._autotune_status = QLabel("Pronto para auto-calibrar com IA.")
        self._autotune_status.setObjectName("MutedLabel")
        self._autotune_status.setWordWrap(True)

        self._test_path = QLabel("Nenhuma imagem carregada")
        self._test_path.setObjectName("MutedLabel")
        self._btn_load_image = QPushButton("Carregar imagem")
        self._btn_publish_image = QPushButton("Publicar teste")
        fit_button_text(self._btn_load_image)
        fit_button_text(self._btn_publish_image)
        self._btn_publish_image.setToolTip(
            "Publica a imagem carregada em /camera/image_raw para o pipeline de visão processar como se fosse a webcam."
        )
        self._btn_load_image.clicked.connect(self._load_test_image)
        self._btn_publish_image.clicked.connect(self._publish_test_image)

        test_box = QGroupBox("Teste com imagem")
        test_grid = QGridLayout(test_box)
        test_grid.addWidget(self._label("Arquivo"), 0, 0)
        test_grid.addWidget(self._test_path, 0, 1, 1, 3)
        test_grid.addWidget(self._btn_load_image, 1, 0, 1, 2)
        test_grid.addWidget(self._btn_publish_image, 1, 2, 1, 2)

        self._use_calibration = QCheckBox("Usar calibração por pontos marcados")
        self._use_calibration.setChecked(False)
        self._click_target = self._combo(["calibracao", "limite"])
        self._calibration_index = self._int_combo(1, 4, 1)
        self._calibration_enabled: list[QCheckBox] = []
        self._pixel_spins: list[tuple[QSpinBox, QSpinBox]] = []
        self._xy_spins: list[tuple[QDoubleSpinBox, QDoubleSpinBox]] = []
        defaults_px = [(120.0, 120.0), (520.0, 120.0), (520.0, 360.0), (120.0, 360.0)]
        defaults_xy = [(0.42, 0.18), (0.42, -0.18), (0.20, -0.18), (0.20, 0.18)]

        table_box = QGroupBox("Calibração pixel -> XY por pontos do braço")
        table_grid = QGridLayout(table_box)
        table_grid.addWidget(self._use_calibration, 0, 0, 1, 6)
        table_grid.addWidget(self._label("Clique"), 1, 0)
        table_grid.addWidget(self._click_target, 1, 1)
        table_grid.addWidget(self._label("Ponto ativo"), 1, 2)
        table_grid.addWidget(self._calibration_index, 1, 3)
        headers = ["Usar", "Ponto", "u px", "v px", "X m", "Y m"]
        for col, title in enumerate(headers):
            table_grid.addWidget(self._label(title), 2, col)
        for row, name in enumerate(["P1", "P2", "P3", "P4"], start=3):
            px_u, px_v = defaults_px[row - 3]
            xy_x, xy_y = defaults_xy[row - 3]
            enabled = QCheckBox()
            enabled.setChecked(row <= 6)
            u_spin = self._spin(0, 10000, int(round(px_u)), suffix=" px")
            v_spin = self._spin(0, 10000, int(round(px_v)), suffix=" px")
            x_spin = self._double_spin(-2.0, 2.0, xy_x, step=0.01)
            y_spin = self._double_spin(-2.0, 2.0, xy_y, step=0.01)
            self._calibration_enabled.append(enabled)
            self._pixel_spins.append((u_spin, v_spin))
            self._xy_spins.append((x_spin, y_spin))
            table_grid.addWidget(enabled, row, 0)
            table_grid.addWidget(QLabel(name), row, 1)
            table_grid.addWidget(u_spin, row, 2)
            table_grid.addWidget(v_spin, row, 3)
            table_grid.addWidget(x_spin, row, 4)
            table_grid.addWidget(y_spin, row, 5)

        self._btn_capture_pose = QPushButton("Capturar XY do braço")
        self._btn_apply_table = QPushButton("Aplicar calibração")
        fit_button_text(self._btn_capture_pose)
        fit_button_text(self._btn_apply_table)
        self._btn_capture_pose.clicked.connect(self._capture_arm_pose)
        self._btn_apply_table.clicked.connect(self._apply_workspace)
        table_grid.addWidget(self._btn_capture_pose, 7, 0, 1, 3)
        table_grid.addWidget(self._btn_apply_table, 7, 3, 1, 3)

        self._limit_mode = self._combo(["desativado", "limite desenhado", "limites operacionais"])
        self._limit_mode.setAccessibleName("Modo dos limites de operação")
        self._limit_shape = self._combo(["polygon", "circle"])
        self._limit_margin_px = self._spin(0, 500, 12, suffix=" px")
        self._limit_margin_px.setToolTip(
            "Margem de segurança em pixels: encolhe a área válida do limite desenhado."
        )
        self._limit_points: list[tuple[float, float]] = []
        self._limit_circle = [320.0, 240.0, 180.0]
        self._limit_margin_applied = False
        self._limit_summary = QLabel("Sem limite desenhado")
        self._limit_summary.setObjectName("MutedLabel")
        self._limit_help = QLabel("Polígono: clique os vértices na imagem. Círculo: clique o centro e depois o raio.")
        self._limit_help.setObjectName("MutedLabel")
        self._limit_help.setWordWrap(True)
        self._btn_draw_limit_mode = QPushButton("Desenhar na imagem")
        self._btn_draw_limit_mode.setCheckable(True)
        self._btn_draw_limit_mode.setObjectName("DrawModeButton")
        self._btn_clear_limit = QPushButton("Limpar limite")
        self._btn_apply_limit = QPushButton("Aplicar limite")
        for b in (self._btn_draw_limit_mode, self._btn_clear_limit, self._btn_apply_limit):
            fit_button_text(b)
        self._btn_draw_limit_mode.toggled.connect(self._set_limit_drawing_mode)
        self._btn_clear_limit.clicked.connect(self._clear_limit)
        self._btn_apply_limit.clicked.connect(self._apply_limit)
        self._limit_shape.currentTextChanged.connect(self._on_limit_shape_changed)
        self._limit_mode.currentTextChanged.connect(self._on_limit_mode_changed)

        limit_box = QGroupBox("Limites de operação")
        limit_grid = QGridLayout(limit_box)
        limit_grid.addWidget(self._label("Modo"), 0, 0)
        limit_grid.addWidget(self._limit_mode, 0, 1, 1, 3)

        self._limit_draw_controls = QWidget()
        draw_grid = QGridLayout(self._limit_draw_controls)
        draw_grid.setContentsMargins(0, 0, 0, 0)
        draw_grid.addWidget(self._label("Forma"), 0, 0)
        draw_grid.addWidget(self._limit_shape, 0, 1)
        draw_grid.addWidget(self._label("Margem px"), 0, 2)
        draw_grid.addWidget(self._limit_margin_px, 0, 3)
        draw_grid.addWidget(self._label("Desenho"), 1, 0)
        draw_grid.addWidget(self._limit_summary, 1, 1, 1, 3)
        draw_grid.addWidget(self._limit_help, 2, 0, 1, 4)
        draw_grid.addWidget(self._btn_draw_limit_mode, 3, 0, 1, 2)
        draw_grid.addWidget(self._btn_clear_limit, 3, 2)
        draw_grid.addWidget(self._btn_apply_limit, 3, 3)
        limit_grid.addWidget(self._limit_draw_controls, 1, 0, 1, 4)

        self._btn_load_horizontal_profile = QPushButton("Carregar perfil deitado")
        self._btn_load_vertical_profile = QPushButton("Carregar perfil em pé")
        self._btn_draw_detection_overlay = QPushButton("Desenhar BBoxes/clearances")
        self._btn_apply_profiles = QPushButton("Aplicar perfis")
        self._btn_apply_profiles.setObjectName("PrimaryButton")
        self._horizontal_profile_path = QLabel("Deitado: nenhum arquivo carregado")
        self._vertical_profile_path = QLabel("Em pé: nenhum arquivo carregado")
        self._profile_status = QLabel(
            "Carregue dois JSONs externos: um para objetos deitados/horizontais e outro para objetos em pé/verticais."
        )
        for label in (
            self._horizontal_profile_path,
            self._vertical_profile_path,
            self._profile_status,
        ):
            label.setObjectName("MutedLabel")
            label.setWordWrap(True)
        for button in (
            self._btn_load_horizontal_profile,
            self._btn_load_vertical_profile,
            self._btn_draw_detection_overlay,
            self._btn_apply_profiles,
        ):
            fit_button_text(button)
        self._btn_load_horizontal_profile.clicked.connect(self._load_horizontal_profile_dialog)
        self._btn_load_vertical_profile.clicked.connect(self._load_vertical_profile_dialog)
        self._btn_draw_detection_overlay.clicked.connect(self._draw_detection_overlay)
        self._btn_apply_profiles.clicked.connect(self._apply_external_profiles)

        profile_box = QGroupBox("Perfis externos de visão")
        profile_grid = QGridLayout(profile_box)
        profile_grid.addWidget(self._profile_status, 0, 0, 1, 4)
        profile_grid.addWidget(self._horizontal_profile_path, 1, 0, 1, 4)
        profile_grid.addWidget(self._btn_load_horizontal_profile, 2, 0, 1, 2)
        profile_grid.addWidget(self._vertical_profile_path, 3, 0, 1, 4)
        profile_grid.addWidget(self._btn_load_vertical_profile, 4, 0, 1, 2)
        profile_grid.addWidget(self._btn_apply_profiles, 5, 0, 1, 2)
        profile_grid.addWidget(self._btn_draw_detection_overlay, 5, 2, 1, 2)

        self._yolo_model_label = QLabel("Nenhum modelo YOLO carregado")
        self._yolo_model_label.setObjectName("MutedLabel")
        self._yolo_model_label.setWordWrap(True)
        self._btn_load_yolo_model = QPushButton("Carregar modelo YOLO")
        self._btn_load_task_config = QPushButton("Carregar JSON")
        self._btn_save_task_config = QPushButton("Salvar JSON")
        self._btn_apply_task_config = QPushButton("Aplicar")
        self._btn_apply_task_config.setObjectName("PrimaryButton")
        self._yolo_confidence_percent = self._double_spin(0.0, 100.0, 35.0, step=1.0)
        self._yolo_confidence_percent.setSuffix(" %")
        self._yolo_image_size = self._spin(64, 4096, 416, suffix=" px")
        self._target_classes = QLineEdit()
        self._target_classes.setPlaceholderText("ex.: bottle, cap")
        self._object_orientation = self._combo(["any", "horizontal", "vertical"])
        self._auxiliary_attribute = QLineEdit()
        self._auxiliary_attribute.setPlaceholderText("ex.: PET transparente")
        self._grasp_clearance = self._spin(0, 5000, 0, suffix=" px")
        self._min_clearance_horizontal = self._spin(0, 5000, 4, suffix=" px")
        self._min_clearance_vertical = self._spin(0, 5000, 20, suffix=" px")
        self._pick_route = self._combo(
            [
                "score",
                "nearest_neighbor",
                "left_to_right",
                "right_to_left",
                "front_to_back",
                "back_to_front",
            ]
        )
        self._pick_max_objects = self._spin(0, 1000, 8)
        self._pick_stable_frames = self._spin(1, 100, 1)
        self._route_origin_x = self._double_spin(-10.0, 10.0, 0.0, step=0.01)
        self._route_origin_x.setSuffix(" m")
        self._route_origin_y = self._double_spin(-10.0, 10.0, 0.0, step=0.01)
        self._route_origin_y.setSuffix(" m")
        for button in (
            self._btn_load_yolo_model,
            self._btn_load_task_config,
            self._btn_save_task_config,
            self._btn_apply_task_config,
        ):
            fit_button_text(button)
        self._btn_load_yolo_model.clicked.connect(self._load_yolo_model_dialog)
        self._btn_load_task_config.clicked.connect(self._load_task_config_dialog)
        self._btn_save_task_config.clicked.connect(self._save_task_config_dialog)
        self._btn_apply_task_config.clicked.connect(self._apply_task_configuration)

        task_box = QGroupBox("YOLO e planejamento de pega")
        task_grid = QGridLayout(task_box)
        task_grid.addWidget(self._label("Modelo"), 0, 0)
        task_grid.addWidget(self._yolo_model_label, 0, 1, 1, 3)
        task_grid.addWidget(self._btn_load_yolo_model, 1, 0, 1, 2)
        task_grid.addWidget(self._label("Assertividade YOLO"), 2, 0)
        task_grid.addWidget(self._yolo_confidence_percent, 2, 1)
        task_grid.addWidget(self._label("Entrada YOLO"), 2, 2)
        task_grid.addWidget(self._yolo_image_size, 2, 3)
        task_grid.addWidget(self._label("Classes de interesse"), 3, 0)
        task_grid.addWidget(self._target_classes, 3, 1, 1, 3)
        task_grid.addWidget(self._label("Disposição"), 4, 0)
        task_grid.addWidget(self._object_orientation, 4, 1)
        task_grid.addWidget(self._label("Atributo auxiliar"), 4, 2)
        task_grid.addWidget(self._auxiliary_attribute, 4, 3)
        task_grid.addWidget(self._label("Clearance geral"), 5, 0)
        task_grid.addWidget(self._grasp_clearance, 5, 1)
        task_grid.addWidget(self._label("Mín. horizontal"), 5, 2)
        task_grid.addWidget(self._min_clearance_horizontal, 5, 3)
        task_grid.addWidget(self._label("Mín. vertical"), 6, 0)
        task_grid.addWidget(self._min_clearance_vertical, 6, 1)
        task_grid.addWidget(self._label("Rota de pega"), 6, 2)
        task_grid.addWidget(self._pick_route, 6, 3)
        task_grid.addWidget(self._label("Máx. objetos"), 7, 0)
        task_grid.addWidget(self._pick_max_objects, 7, 1)
        task_grid.addWidget(self._label("Frames estáveis"), 7, 2)
        task_grid.addWidget(self._pick_stable_frames, 7, 3)
        task_grid.addWidget(self._label("Origem rota X"), 8, 0)
        task_grid.addWidget(self._route_origin_x, 8, 1)
        task_grid.addWidget(self._label("Origem rota Y"), 8, 2)
        task_grid.addWidget(self._route_origin_y, 8, 3)
        task_grid.addWidget(self._btn_load_task_config, 9, 0, 1, 2)
        task_grid.addWidget(self._btn_save_task_config, 9, 2, 1, 2)
        task_grid.addWidget(self._btn_apply_task_config, 10, 0, 1, 4)
        task_help = QLabel(
            "O JSON reúne modelo, área de trabalho, disposição, clearances e ordenação da rota. "
            "A confiança é informada em percentual e convertida para o limiar 0–1 do YOLO."
        )
        task_help.setObjectName("MutedLabel")
        task_help.setWordWrap(True)
        task_grid.addWidget(task_help, 11, 0, 1, 4)

        self._task_box = task_box
        self._profile_box = profile_box
        self._test_box = test_box
        self._calibration_box = table_box
        self._limit_box = limit_box

        # The advanced configuration tools (YOLO task, external profiles, test
        # image and pixel→XY calibration) are grouped into sub-sections so the
        # operator is not overwhelmed by a single long form. The workspace
        # limit panel stays visible as part of the daily setup; the sub-tab
        # group is revealed by the "Configuração avançada" toggle.
        self._config_tabs = QTabWidget()
        self._config_tabs.setObjectName("VisionConfigTabs")
        self._config_tabs.addTab(task_box, "Tarefa / YOLO")
        self._config_tabs.addTab(profile_box, "Perfis externos")
        self._config_tabs.addTab(test_box, "Imagem de teste")
        self._config_tabs.addTab(table_box, "Calibração px→XY")

        controls = QWidget()
        controls_lay = QVBoxLayout(controls)
        controls_lay.setContentsMargins(0, 0, 0, 0)
        controls_lay.setSpacing(12)
        controls_lay.addWidget(limit_box)
        controls_lay.addWidget(self._config_tabs, 1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(controls)
        scroll.setMinimumWidth(VISION_SIDE_PANEL_WIDTH)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        operation_bar = QFrame()
        operation_bar.setObjectName("OperationBar")
        operation_lay = QHBoxLayout(operation_bar)
        operation_lay.setContentsMargins(12, 8, 12, 8)
        operation_lay.setSpacing(8)
        self._active_backend_label = QLabel("Detector ativo: aguardando aplicação")
        self._active_backend_label.setObjectName("StatusChip")
        self._active_backend_label.setAccessibleName("Detector de visão ativo")
        operation_lay.addWidget(self._active_backend_label)
        operation_lay.addStretch(1)
        operation_lay.addWidget(self._btn_apply)
        operation_lay.addWidget(self._btn_process)

        self._btn_continue_grasp = QPushButton("Continuar para Pegas")
        self._btn_continue_grasp.setEnabled(False)
        self._btn_continue_grasp.setToolTip(
            "Abre Pegas quando o frame atual contém objetos detectados."
        )
        self._btn_continue_grasp.clicked.connect(self.continue_to_grasp.emit)
        operation_lay.addWidget(self._btn_continue_grasp)

        self._chk_technical_columns = QCheckBox("Detalhes técnicos")
        self._chk_technical_columns.setToolTip(
            "Mostra geometria, bounding boxes e clearances na tabela."
        )
        self._chk_technical_columns.toggled.connect(self._set_technical_columns_visible)
        operation_lay.addWidget(self._chk_technical_columns)

        self._btn_advanced = QPushButton("Configuração avançada")
        self._btn_advanced.setCheckable(True)
        self._btn_advanced.toggled.connect(self._set_advanced_visible)
        operation_lay.addWidget(self._btn_advanced)

        self._preview = VisionPreviewLabel("Sem /rebotarm/vision/debug_image")
        self._preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._preview.setMinimumSize(440, 280)
        self._preview.setObjectName("VisionPreview")
        self._preview.image_clicked.connect(self._on_preview_clicked)

        self._objects = QTableWidget(0, 21)
        self._objects.setHorizontalHeaderLabels(
            [
                "N",
                "Classe",
                "Score",
                "Centro px",
                "XYZ",
                "Yaw",
                "Eixo",
                "Sentido",
                "Grip yaw",
                "Área",
                "BBox px",
                "BBox curto",
                "BBox longo",
                "Razão BBox",
                "Dim XYZ",
                "Clear curto-",
                "Clear curto+",
                "Clear longo-",
                "Clear longo+",
                "Reach",
                "Motivo",
            ]
        )
        self._objects.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self._objects.horizontalHeader().setStretchLastSection(True)
        self._objects.verticalHeader().setVisible(False)
        self._objects.verticalHeader().setDefaultSectionSize(22)
        self._objects.setEditTriggers(QTableWidget.NoEditTriggers)
        self._objects.setAlternatingRowColors(True)
        self._objects.setObjectName("VisionObjectsTable")
        self._objects.setAccessibleName("Objetos detectados no frame atual")
        table_font = self._objects.font()
        table_font.setPointSize(max(7, table_font.pointSize() - 2))
        self._objects.setFont(table_font)
        header_font = self._objects.horizontalHeader().font()
        header_font.setPointSize(max(7, header_font.pointSize() - 2))
        self._objects.horizontalHeader().setFont(header_font)
        self._objects.setStyleSheet(
            "QTableWidget#VisionObjectsTable { font-size: 9pt; }"
            "QTableWidget#VisionObjectsTable QHeaderView::section {"
            " font-size: 8pt; min-height: 24px; padding: 3px;"
            "}"
        )

        clearance_help = QLabel(
            "Clearances: distância livre em px nos eixos curto/longo do BBox rotacionado, medida perpendicular às faces do BBox. "
            "Objetos filtrados ou não detectados não entram no cálculo."
        )
        clearance_help.setObjectName("MutedLabel")
        clearance_help.setWordWrap(True)

        preview_area = QVBoxLayout()
        preview_area.addWidget(self._preview, 1)
        self._preview_status = PreviewStatusLabel(self._preview, "Preview aguardando imagem")

        log_box = None
        if self._show_ros_log:
            if self._ros_log is None:
                self._ros_log = QTextEdit()
                self._ros_log.setReadOnly(True)
            self._ros_log.setMinimumHeight(120)
            self._ros_log.setMaximumHeight(180)
            log_box = QGroupBox("Eventos ROS")
            log_lay = QVBoxLayout(log_box)
            log_lay.setContentsMargins(8, 8, 8, 8)
            log_lay.addWidget(self._ros_log)

        objects_box = QGroupBox("Objetos detectados")
        objects_lay = QVBoxLayout(objects_box)
        objects_lay.setContentsMargins(8, 8, 8, 8)
        objects_lay.addWidget(self._objects)
        objects_lay.addWidget(clearance_help)
        objects_box.setMinimumHeight(170)
        objects_box.setMaximumHeight(230)
        if log_box is not None:
            log_box.setMinimumHeight(170)
            log_box.setMaximumHeight(230)

        lay = QGridLayout(self)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(12)
        lay.addWidget(operation_bar, 0, 0, 1, 2)
        lay.addWidget(scroll, 1, 0)
        lay.addLayout(preview_area, 1, 1)
        if log_box is not None:
            lay.addWidget(log_box, 2, 0)
            lay.addWidget(objects_box, 2, 1)
        else:
            lay.addWidget(objects_box, 2, 0, 1, 2)
        lay.setColumnMinimumWidth(0, VISION_SIDE_PANEL_WIDTH)
        lay.setColumnStretch(0, 1)
        lay.setColumnStretch(1, 3)
        lay.setRowStretch(0, 0)
        lay.setRowStretch(1, 1)
        lay.setRowStretch(2, 0)
        self._set_advanced_visible(False)
        self._set_technical_columns_visible(False)
        self._set_active_filters(["detect_bright", "detect_edge", "min_area", "max_area", "min_aspect", "max_aspect"])
        self._on_limit_mode_changed(self._limit_mode.currentText())
        self._connect_bridge_signals()
        self._load_default_test_image()

    def set_interface_mode(self, mode: str) -> None:
        """Keep the operator surface concise while preserving expert tools."""
        advanced_allowed = mode in {"supervisor", "engenharia"}
        self._btn_advanced.setVisible(advanced_allowed)
        if not advanced_allowed:
            self._btn_advanced.setChecked(False)
            self._set_advanced_visible(False)

    def _set_advanced_visible(self, visible: bool) -> None:
        # The whole sub-tab group is shown/hidden at once; individual boxes
        # (task/profile/test/calibration) are the tab pages and stay addressable.
        self._config_tabs.setVisible(bool(visible))
        self._btn_advanced.setText(
            "Ocultar configuração" if visible else "Configuração avançada"
        )

    def _set_technical_columns_visible(self, visible: bool) -> None:
        # The operator summary keeps identity, confidence, pose and validity.
        technical_columns = range(6, 19)
        for column in technical_columns:
            self._objects.setColumnHidden(column, not visible)

    def _connect_bridge_signals(self) -> None:
        if self._bridge is None:
            return
        try:
            self._bridge.vision_calibration_progress.connect(self._on_ai_calibrate_progress)
            self._bridge.vision_calibration_result.connect(self._on_ai_calibrate_result)
            self._bridge.service_result.connect(self._on_service_result)
        except Exception:
            pass

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

    @staticmethod
    def _default_external_profile_dir() -> Path:
        return Path.home()


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
        api_key = self._settings.openai.effective_api_key()
        if not api_key:
            self._warn_ai_autotune(
                "Auto-calibração IA: API key não encontrada. Em Ferramentas e ajustes > Configurações, informe a API key e clique "
                "em “Salvar configurações gerais” — ou defina OPENAI_API_KEY no ambiente."
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
        ok = calibrate(ros_image, prompt, api_key)
        if not ok:
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

    def _on_service_result(self, label: str, success: bool, message: str) -> None:
        if label != "vision_calibrate":
            return
        if not success:
            self._warn_ai_autotune(f"Auto-calibração IA falhou: {message}")
            return
        self._set_ai_autotune_status("Auto-calibração IA: concluída.", busy=False)

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
    def _qimage_to_ros_image(image: QImage) -> SensorImage | None:
        """Convert a QImage into a sensor_msgs/Image (rgb8)."""
        rgb = image.convertToFormat(QImage.Format.Format_RGB888)
        if rgb.isNull():
            return None
        msg = SensorImage()
        msg.height = int(rgb.height())
        msg.width = int(rgb.width())
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = int(rgb.bytesPerLine())
        try:
            msg.data = bytes(rgb.constBits())
        except Exception:
            # PySide6 may expose constBits as a memoryview-like object that needs copy.
            bits = rgb.bits()
            if bits is None:
                return None
            msg.data = bytes(bits)
        return msg

    def _image_for_autotune(self) -> QImage | None:
        if self._test_image is not None and not self._test_image.isNull():
            return self._test_image.convertToFormat(QImage.Format.Format_RGB888)
        if self._last_raw_pixmap is not None and not self._last_raw_pixmap.isNull():
            return self._last_raw_pixmap.toImage().convertToFormat(QImage.Format.Format_RGB888)
        if self._last_pixmap is not None and not self._last_pixmap.isNull():
            return self._last_pixmap.toImage().convertToFormat(QImage.Format.Format_RGB888)
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
        image = source_image.convertToFormat(QImage.Format.Format_RGB888)
        bits = image.bits()
        try:
            data = bits.tobytes()
        except AttributeError:
            data = bytes(bits)
        self._bridge.publish_vision_test_image(
            image.width(),
            image.height(),
            data,
            image.bytesPerLine(),
        )
        processor = getattr(self._bridge, "vision_process_frame", None)
        if callable(processor):
            QTimer.singleShot(250, processor)

    def _apply_workspace(self) -> None:
        pixel_points: list[float] = []
        xy_points: list[float] = []
        for enabled, pixel_pair, xy_pair in zip(
            self._calibration_enabled,
            self._pixel_spins,
            self._xy_spins,
        ):
            if not enabled.isChecked():
                continue
            u_spin, v_spin = pixel_pair
            x_spin, y_spin = xy_pair
            pixel_points.extend([float(u_spin.value()), float(v_spin.value())])
            xy_points.extend([float(x_spin.value()), float(y_spin.value())])
        self._bridge.vision_pose_set_parameters(
            {
                "use_workspace_homography": bool(self._use_calibration.isChecked()),
                "calibration_pixel_points": pixel_points,
                "calibration_xy_points": xy_points,
                "image_width": float(self._current_image_width()),
                "image_height": float(self._current_image_height()),
            }
        )

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
        self._calibration_enabled[index].setChecked(True)

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
        if shape == "circle":
            circle_valid = float(self._limit_circle[2]) > 0.0
            return {
                "enabled": bool(enabled and circle_valid),
                "mode": self._limit_mode.currentText(),
                "shape": "circle",
                "margin_px": int(self._limit_margin_px.value()),
                "points": [],
                "circle": [float(value) for value in self._limit_circle],
            }
        return {
            "enabled": bool(enabled and len(self._limit_points) >= 3),
            "mode": self._limit_mode.currentText(),
            "shape": "polygon",
            "margin_px": int(self._limit_margin_px.value()),
            "points": [(float(u), float(v)) for u, v in self._limit_points],
            "circle": [float(value) for value in self._limit_circle],
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

    def _calibration_transform(self):
        pixel_points: list[float] = []
        xy_points: list[float] = []
        for enabled, pixel_pair, xy_pair in zip(
            self._calibration_enabled,
            self._pixel_spins,
            self._xy_spins,
        ):
            if not enabled.isChecked():
                continue
            u_spin, v_spin = pixel_pair
            x_spin, y_spin = xy_pair
            pixel_points.extend([float(u_spin.value()), float(v_spin.value())])
            xy_points.extend([float(x_spin.value()), float(y_spin.value())])
        count = min(len(pixel_points), len(xy_points)) // 2
        if count < 3:
            return None
        try:
            import cv2
            import numpy as np
        except Exception:
            return None
        src = np.array(pixel_points[: count * 2], dtype=np.float32).reshape((count, 2))
        dst = np.array(xy_points[: count * 2], dtype=np.float32).reshape((count, 2))
        try:
            if count == 3:
                affine = cv2.getAffineTransform(src, dst)
                return np.vstack([affine, np.array([0.0, 0.0, 1.0], dtype=np.float32)])
            return cv2.getPerspectiveTransform(src[:4], dst[:4])
        except Exception:
            return None

    @staticmethod
    def _apply_transform(transform, u: float, v: float) -> tuple[float, float] | None:
        denom = float(transform[2, 0]) * float(u) + float(transform[2, 1]) * float(v) + float(transform[2, 2])
        if abs(denom) < 1e-9:
            return None
        x = (float(transform[0, 0]) * float(u) + float(transform[0, 1]) * float(v) + float(transform[0, 2])) / denom
        y = (float(transform[1, 0]) * float(u) + float(transform[1, 1]) * float(v) + float(transform[1, 2])) / denom
        return x, y

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

    def _is_drawn_limit_mode(self) -> bool:
        return self._limit_mode.currentText() == "limite desenhado"

    def _is_operation_limit_mode(self) -> bool:
        return self._limit_mode.currentText() == "limites operacionais"

    def _on_limit_shape_changed(self, _shape: str) -> None:
        self._limit_points = []
        self._limit_margin_applied = False
        self._refresh_limit_summary()
        self._render_local_overlay()

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

    def _apply_task_configuration(self) -> None:
        self._apply()
        self._apply_limit()
        self._profile_status.setText("Modelo, área de trabalho e planejamento de pega aplicados.")

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
    def _default_task_config_path() -> Path:
        return Path.home() / ".config" / "rebotarm_gui" / "vision_pick_config.json"

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
        self._last_raw_pixmap = QPixmap.fromImage(image)
        self._last_image_width = int(image.width())
        self._last_image_height = int(image.height())
        self._preview.set_source_size(image.width(), image.height())
        if self._last_pixmap is None:
            self._last_pixmap = self._last_raw_pixmap
            self._preview_status.setText(f"Preview: frame cru {image.width()}x{image.height()}")
            self._refresh_preview()

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
