# ruff: noqa: F401,F403,F405
"""Composed VisionTab — pipeline, classic CV, calibration and AI mixins."""

from __future__ import annotations

from ._common import *  # noqa: F403
from ._pipeline import VisionTabPipelineMixin
from ._classic import VisionTabClassicMixin
from ._calibration import VisionTabCalibrationMixin
from ._ai import VisionTabAiMixin


class VisionTab(
    QWidget,
    VisionTabPipelineMixin,
    VisionTabClassicMixin,
    VisionTabCalibrationMixin,
    VisionTabAiMixin,
):
    """Controls for non-neural image processing and object suggestions."""

    workspace_limit_changed = Signal(object)
    planning_config_changed = Signal(object)
    continue_to_grasp = Signal()
    calibration_status_changed = Signal(bool, str)
    async_operation_started = Signal(str, str)
    async_operation_finished = Signal(str, bool, str)

    def __init__(
        self,
        bridge,
        ros_log: QTextEdit | None = None,
        settings: AppSettings | None = None,
        show_ros_log: bool = True,
        calibration_store: CalibrationStore | None = None,
        authorization: AuthorizationContext | None = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._ros_log = ros_log
        self._show_ros_log = bool(show_ros_log)
        self._settings = settings or AppSettings()
        self._calibration_store = calibration_store or CalibrationStore()
        self._authorization = authorization
        self._interface_mode = "supervisor"
        self._calibration_profile = CalibrationProfile()
        self._last_camera_frame = ""
        self._last_pixmap: QPixmap | None = None
        self._last_raw_pixmap: QPixmap | None = None
        self._last_image_width = 0
        self._last_image_height = 0
        self._runtime_image_width = 0
        self._runtime_image_height = 0
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

        self._use_calibration = QCheckBox("Usar perfil de calibração validado")
        self._use_calibration.setChecked(False)
        self._click_target = self._combo(["calibracao", "limite"])
        self._calibration_index = self._int_combo(1, 8, 1)
        self._calibration_enabled: list[QCheckBox] = []
        self._pixel_spins: list[tuple[QSpinBox, QSpinBox]] = []
        self._xy_spins: list[tuple[QDoubleSpinBox, QDoubleSpinBox]] = []
        self._calibration_residual_labels: list[QLabel] = []
        defaults_px = [
            (120.0, 120.0),
            (520.0, 120.0),
            (520.0, 360.0),
            (120.0, 360.0),
            (320.0, 240.0),
            (320.0, 120.0),
            (520.0, 240.0),
            (120.0, 240.0),
        ]
        defaults_xy = [
            (0.42, 0.18),
            (0.42, -0.18),
            (0.20, -0.18),
            (0.20, 0.18),
            (0.31, 0.00),
            (0.42, 0.00),
            (0.31, -0.18),
            (0.31, 0.18),
        ]

        self._calibration_name = QLineEdit("calibração da célula")
        self._calibration_camera_frame = QLineEdit("camera_link")
        self._calibration_target_frame = QLineEdit("base_link")
        self._calibration_tcp_frame = QLineEdit("gripper_tcp")
        self._calibration_tool_id = QLineEdit("gripper")
        self._calibration_table_z = self._double_spin(
            -2.0, 2.0, 0.0, step=0.001
        )
        self._calibration_table_z.setSuffix(" m")
        self._calibration_payload = self._double_spin(
            0.0, 100.0, 0.0, step=0.05
        )
        self._calibration_payload.setSuffix(" kg")
        self._calibration_validity_days = self._spin(1, 3650, 180, suffix=" dias")
        self._calibration_status = QLabel(
            f"Rascunho — colete ao menos {MIN_APPROVAL_POINTS} pontos distribuídos."
        )
        self._calibration_status.setObjectName("MutedLabel")
        self._calibration_status.setWordWrap(True)

        table_box = QGroupBox("Gerenciador de calibração da célula")
        table_grid = QGridLayout(table_box)
        table_grid.addWidget(self._label("Perfil"), 0, 0)
        table_grid.addWidget(self._calibration_name, 0, 1, 1, 3)
        table_grid.addWidget(self._label("Validade"), 0, 4)
        table_grid.addWidget(self._calibration_validity_days, 0, 5, 1, 2)
        table_grid.addWidget(self._label("Câmera"), 1, 0)
        table_grid.addWidget(self._calibration_camera_frame, 1, 1)
        table_grid.addWidget(self._label("Base"), 1, 2)
        table_grid.addWidget(self._calibration_target_frame, 1, 3)
        table_grid.addWidget(self._label("TCP"), 1, 4)
        table_grid.addWidget(self._calibration_tcp_frame, 1, 5, 1, 2)
        table_grid.addWidget(self._label("Ferramenta"), 2, 0)
        table_grid.addWidget(self._calibration_tool_id, 2, 1)
        table_grid.addWidget(self._label("Payload"), 2, 2)
        table_grid.addWidget(self._calibration_payload, 2, 3)
        table_grid.addWidget(self._label("Plano Z"), 2, 4)
        table_grid.addWidget(self._calibration_table_z, 2, 5, 1, 2)
        table_grid.addWidget(self._use_calibration, 3, 0, 1, 4)
        table_grid.addWidget(self._label("Clique"), 4, 0)
        table_grid.addWidget(self._click_target, 4, 1)
        table_grid.addWidget(self._label("Ponto ativo"), 4, 2)
        table_grid.addWidget(self._calibration_index, 4, 3)
        headers = ["Usar", "Ponto", "u px", "v px", "X m", "Y m", "Resíduo"]
        for col, title in enumerate(headers):
            table_grid.addWidget(self._label(title), 5, col)
        for row, name in enumerate(
            ["P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8"], start=6
        ):
            px_u, px_v = defaults_px[row - 6]
            xy_x, xy_y = defaults_xy[row - 6]
            enabled = QCheckBox()
            enabled.setChecked(row <= 9)
            u_spin = self._spin(0, 10000, int(round(px_u)), suffix=" px")
            v_spin = self._spin(0, 10000, int(round(px_v)), suffix=" px")
            x_spin = self._double_spin(-2.0, 2.0, xy_x, step=0.001)
            y_spin = self._double_spin(-2.0, 2.0, xy_y, step=0.001)
            residual = QLabel("--")
            residual.setObjectName("MutedLabel")
            self._calibration_enabled.append(enabled)
            self._pixel_spins.append((u_spin, v_spin))
            self._xy_spins.append((x_spin, y_spin))
            self._calibration_residual_labels.append(residual)
            table_grid.addWidget(enabled, row, 0)
            table_grid.addWidget(QLabel(name), row, 1)
            table_grid.addWidget(u_spin, row, 2)
            table_grid.addWidget(v_spin, row, 3)
            table_grid.addWidget(x_spin, row, 4)
            table_grid.addWidget(y_spin, row, 5)
            table_grid.addWidget(residual, row, 6)

        self._btn_capture_pose = QPushButton("Capturar XYZ do TCP")
        self._btn_apply_table = QPushButton("Aplicar calibração")
        self._btn_validate_calibration = QPushButton("Validar ajuste")
        self._btn_save_calibration = QPushButton("Salvar rascunho")
        self._btn_approve_cell_calibration = QPushButton("Aprovar e ativar")
        self._btn_new_calibration_revision = QPushButton("Nova revisão")
        self._btn_load_cell_calibration = QPushButton("Carregar perfil")
        for button in (
            self._btn_capture_pose,
            self._btn_apply_table,
            self._btn_validate_calibration,
            self._btn_save_calibration,
            self._btn_approve_cell_calibration,
            self._btn_new_calibration_revision,
            self._btn_load_cell_calibration,
        ):
            fit_button_text(button)
        self._btn_capture_pose.clicked.connect(self._capture_arm_pose)
        self._btn_apply_table.clicked.connect(self._apply_workspace)
        self._btn_validate_calibration.clicked.connect(
            self._validate_cell_calibration
        )
        self._btn_save_calibration.clicked.connect(self._save_cell_calibration)
        self._btn_approve_cell_calibration.clicked.connect(
            self._approve_cell_calibration
        )
        self._btn_new_calibration_revision.clicked.connect(
            self._new_cell_calibration_revision
        )
        self._btn_load_cell_calibration.clicked.connect(
            self._load_cell_calibration_dialog
        )
        table_grid.addWidget(self._calibration_status, 14, 0, 1, 7)
        table_grid.addWidget(self._btn_capture_pose, 15, 0, 1, 2)
        table_grid.addWidget(self._btn_validate_calibration, 15, 2)
        table_grid.addWidget(self._btn_apply_table, 15, 3)
        table_grid.addWidget(self._btn_save_calibration, 15, 4)
        table_grid.addWidget(self._btn_load_cell_calibration, 15, 5)
        table_grid.addWidget(self._btn_new_calibration_revision, 16, 0, 1, 3)
        table_grid.addWidget(self._btn_approve_cell_calibration, 16, 3, 1, 4)

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
        self._active_calibration_label = QLabel("Calibração: não aprovada")
        self._active_calibration_label.setObjectName("StatusChip")
        self._active_calibration_label.setAccessibleName(
            "Estado da calibração da célula"
        )
        operation_lay.addWidget(self._active_calibration_label)
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
        self._load_active_cell_calibration()


    def set_interface_mode(self, mode: str) -> None:
        """Keep the operator surface concise while preserving expert tools."""
        self._interface_mode = str(mode)
        advanced_allowed = mode in {"supervisor", "engenharia"}
        self._btn_advanced.setVisible(advanced_allowed)
        self._update_calibration_editability()
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
