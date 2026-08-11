"""Main application window for the reBotArm GUI."""

from __future__ import annotations

from collections import deque
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import QByteArray, QSize, QTimer, Qt
from PySide6.QtGui import QAction, QFont, QFontDatabase, QIcon, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QComboBox,
    QCheckBox,
    QDockWidget,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QInputDialog,
    QMainWindow,
    QMenu,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSlider,
    QSizePolicy,
    QStackedWidget,
    QStatusBar,
    QTextEdit,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from .accessibility import announce, configure_accessibility, first_focusable, focus_first
from .audit import ExecutionAuditLog, new_run_id
from .calibration import CalibrationStore
from .i18n import tr
from .icons import load_icon
from .motion.cell_scene import CellSceneStore
from .ros_bridge import RosBridge
from .settings import (
    COMMISSIONING_CHECKLIST_KEYS,
    VALID_PROFILES,
    VALID_THEMES,
    clamp_font_scale,
    load_app_settings,
    save_app_settings,
)
from .security import (
    AuthorizationContext,
    CredentialStore,
    Identity,
    ROLE_OPERATOR,
)
from .tabs import (
    DiagnosticsTab,
    GripperTab,
    GraspTab,
    IoTab,
    JointsTab,
    MitTab,
    MotionTab,
    PoseTab,
    Robot3dTab,
    SafetyTab,
    SettingsTab,
    SystemTab,
    TelemetryTab,
    VisionTab,
)
from .widgets import Command, CommandPalette, EventLogPanel, OperatorTestPanel, make_button
from .widgets.alarm import (
    Alarm,
    AlarmBanner,
    AlarmCenterPanel,
    AlarmStore,
    HeaderAlarmIndicator,
    STATE_ACKNOWLEDGED,
    STATE_ACTIVE,
    STATE_CLEARED,
)
from .widgets.status_led import StatusLed
from .widgets.toast import ToastManager


PAGE_META = {
    "home": ("Início", "Acesso rápido ao fluxo diário e ao controle manual."),
    "vision": ("Visão", "Detecte objetos e valide a área de trabalho."),
    "grasp": ("Pegas", "Revise estratégias e gere o programa de coleta."),
    "motion": ("Programa", "Edite, simule e execute sequências de movimento."),
    "robot3d": (
        "Robô 3D",
        "Visualize o braço e configure volumes de colisão da célula.",
    ),
    "pose": ("Pose cartesiana", "Movimente o TCP por posição e orientação."),
    "joints": ("Juntas", "Ajuste e envie alvos individuais das juntas."),
    "gripper": ("Gripper", "Abra, feche e ajuste o efetuador."),
    "safety": ("Segurança", "Monitore torque, falhas e perfis supervisionados."),
    "system": ("Sistema", "Comandos e diagnóstico detalhado do controlador."),
    "io": ("I/O digital", "Monitore entradas e comande saídas digitais."),
    "telemetry": ("Telemetria", "Consulte estados detalhados das juntas e motores."),
    "diagnostics": (
        "Diagnóstico ROS",
        "Verifique heartbeat, métricas dos tópicos e disponibilidade de endpoints.",
    ),
    "mit": ("MIT / Raw", "Comandos de baixo nível para engenharia e diagnóstico."),
    "settings": ("Configurações", "Preferências persistentes e integrações da GUI."),
}

ASYNC_OPERATION_LABELS = {
    "vision_ai_calibration": "Calibração de visão por IA",
    "apply_cell_scene": "Aplicação da cena no MoveIt",
}

ASYNC_SERVICE_OPERATIONS = {
    "vision_calibrate": "vision_ai_calibration",
    "apply_cell_scene": "apply_cell_scene",
}

# Pages hidden from the daily navigation for each user profile. The profile
# gates (not just hides) engineering pages: direct navigation is rejected.
PROFILE_HIDDEN_PAGES = {
    "operador": {"system", "io", "telemetry", "diagnostics", "mit", "settings"},
    "supervisor": {"mit"},
    "engenharia": set(),
}
PROFILE_LABELS = {
    "operador": "Operador",
    "supervisor": "Supervisor",
    "engenharia": "Engenharia",
}
PRODUCTION_HIDDEN_PAGES = {
    "grasp",
    "pose",
    "joints",
    "gripper",
    "system",
    "io",
    "telemetry",
    "diagnostics",
    "mit",
    "settings",
}

# Icon asset name for each page. Missing/empty icons degrade gracefully to a
# text-only control, so this mapping never breaks layout or tests.
PAGE_ICONS = {
    "home": "home",
    "vision": "vision",
    "grasp": "grasp",
    "motion": "motion",
    "robot3d": "robot3d",
    "pose": "pose",
    "joints": "joints",
    "gripper": "gripper",
    "safety": "safety",
    "system": "system",
    "io": "io",
    "telemetry": "telemetry",
    "diagnostics": "diagnostics",
    "mit": "terminal",
    "settings": "settings",
}

# Daily-flow breadcrumb path for each page. The recommended cell workflow is
# Visão -> Pegas -> Programa; supporting pages map to their nearest flow stage.
# Pages not part of the main flow (home is the entry; settings/engineering are
# off-flow) show no breadcrumb so the indicator stays meaningful.
PAGE_FLOW = {
    "vision": ("Fluxo", ["Visão", "Pegas", "Programa"], "Visão"),
    "grasp": ("Fluxo", ["Visão", "Pegas", "Programa"], "Pegas"),
    "motion": ("Fluxo", ["Visão", "Pegas", "Programa"], "Programa"),
}

# Sidebar widths for the two modes. Collapsed keeps icons + tooltips only so
# the fixed 214 px rail frees area for the vision/3D preview at 1360 px min.
SIDEBAR_WIDTH_EXPANDED = 214
SIDEBAR_WIDTH_COLLAPSED = 60

# Bound the active-alarm queue and coalesce identical alarms within this window
# so a repeating failure (e.g. an arm fault rejecting each dispatched move)
# cannot flood the banner or rebuild the history table unbounded.
ALERT_QUEUE_MAX = 50
ALERT_DEDUP_WINDOW_S = 5.0

COMMISSIONING_STEPS = (
    (
        "calibration_approved",
        "Calibração da visão aprovada",
        "Perfil px→XY válido para a câmera e ferramenta ativas.",
        "vision",
    ),
    (
        "cell_scene_applied",
        "Cena da célula aplicada",
        "Volumes e zonas de exclusão enviados ao MoveIt.",
        "robot3d",
    ),
    (
        "vision_validated",
        "Detecção validada",
        "Objetos revisados e encaminhados de Visão para Pegas.",
        "vision",
    ),
    (
        "program_generated",
        "Programa preparado",
        "Estratégia de pega convertida em programa de movimento.",
        "grasp",
    ),
)


class MainWindow(QMainWindow):
    """Top-level window with task navigation and persistent robot status."""

    def __init__(
        self,
        namespace: str = "rebotarm",
        *,
        start_bridge: bool = True,
        usability_test: bool = False,
        restore_session: bool = False,
        alarm_store: AlarmStore | None = None,
    ) -> None:
        super().__init__()
        # When False (the default, used by tests) the window starts on Início
        # at the default geometry. The real entry point passes True so the
        # operator resumes exactly where the previous session left off.
        self._restore_session_enabled = bool(restore_session)
        self.setWindowTitle("reBotArm Controller")
        self.setMinimumSize(1360, 720)
        self.resize(1440, 860)

        self._bridge = RosBridge(namespace=namespace)
        self._settings = load_app_settings()
        self._credentials = CredentialStore()
        self._authorization = AuthorizationContext(
            enforce_rbac=self._credentials.has_users()
        )
        if self._authorization.enforce_rbac:
            # Persisted privileged UI profiles never bypass a fresh login.
            self._settings.profile = ROLE_OPERATOR
        # A production lease is deliberately session-scoped and requires a
        # fresh privileged login after every application restart.
        self._settings.production_mode = False
        self._bridge.set_authorization_context(self._authorization)
        self._authorization_timer = QTimer(self)
        self._authorization_timer.setInterval(30_000)
        self._authorization_timer.timeout.connect(
            self._bridge.publish_authorization_lease
        )
        self._bridge.set_speed_override(self._settings.speed_override)
        self._cell_scene_store = CellSceneStore(parent=self)
        self._calibration_store = CalibrationStore()
        self._audit_log = ExecutionAuditLog()
        self._bridge.planning_scene_result.connect(
            self._cell_scene_store.accept_monitored_result
        )
        self._bridge.connected.connect(
            lambda online: self._bridge.refresh_planning_scene(
                self._cell_scene_store.scene.frame_id
            )
            if online
            else None
        )
        self._speed_save_timer = QTimer(self)
        self._speed_save_timer.setSingleShot(True)
        self._speed_save_timer.setInterval(350)
        self._speed_save_timer.timeout.connect(self._persist_speed_override)
        self._shortcuts: list[QShortcut] = []
        self._nav_buttons: dict[str, QPushButton] = {}
        self._nav_labels: dict[str, str] = {}
        self._sidebar_section_labels: list[QLabel] = []
        self._sidebar_footer_widgets: list[QWidget] = []
        self._page_indexes: dict[str, int] = {}
        self._advanced_actions: dict[str, QAction] = {}
        self._bridge_started = False
        self._async_operations: dict[str, str] = {}
        self._arm_alive = False
        self._joint_limits_applied = False
        self._arm_last_seen: str = "--"

        # The event stream is useful for diagnosis, but stays out of the daily
        # workflow until explicitly opened from the status bar or Ctrl+L.
        # ``_log`` remains the underlying text surface so existing call sites
        # (e.g. Vision receiving ``ros_log=``) keep working; the EventLogPanel
        # wraps it with filtering, search and a capped buffer.
        self._log = QTextEdit()
        self._alert_queue: deque[Alarm] = deque(maxlen=ALERT_QUEUE_MAX)
        self._alarm_store = alarm_store or AlarmStore()

        self._build_pages()
        self._build_shell(namespace)
        self._build_event_dock()
        self._build_alarm_center_dock()
        if self._restore_session_enabled:
            self._restore_persistent_alarms()
        self._operator_test_dock: QDockWidget | None = None
        self._operator_test_panel: OperatorTestPanel | None = None
        if usability_test:
            self._build_operator_test_dock()
        # The toast manager floats toasts over the whole window, so it must be
        # created once the shell exists.
        self._toast = ToastManager(self)
        self._command_palette: CommandPalette | None = None
        self._wire_signals()
        self._sync_commissioning_prerequisites()
        self._install_shortcuts()
        self._configure_accessibility()
        self._apply_profile(self._settings.profile, persist=False)
        self._tab_motion.set_operating_mode(
            production=False,
            collision_planning_required=self._settings.collision_planning_required,
        )
        # The actual stylesheet is applied once by the entry point (run()), or
        # on an explicit theme change. Here we only keep the settings-page
        # selector in sync with the persisted theme so construction stays cheap
        # (applying the full QSS on every window makes a long session slower).
        self._sync_theme_selector()
        self._restore_sidebar_state()
        self._refresh_command_availability()
        self.navigate("home")

        if start_bridge:
            self._bridge.start()
            self._bridge_started = True
            if self._authorization.enforce_rbac:
                self._bridge.publish_authorization_lease()
                self._authorization_timer.start()
            self._append_log(f"GUI iniciada — namespace /{namespace}")

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def _build_pages(self) -> None:
        self._tab_settings = SettingsTab(self._settings)
        self._tab_system = SystemTab(self._bridge)
        self._tab_joints = JointsTab(self._bridge)
        self._tab_pose = PoseTab(self._bridge)
        self._tab_motion = MotionTab(
            self._bridge,
            operator_provider=lambda: self._settings.operator_id,
            cell_scene_store=self._cell_scene_store,
            production_prerequisite_provider=lambda: (
                self._tab_vision.production_calibration_status()
            ),
        )
        self._tab_robot3d = Robot3dTab(
            self._bridge,
            cell_scene_store=self._cell_scene_store,
        )
        self._tab_gripper = GripperTab(self._bridge)
        self._tab_grasp = GraspTab(self._bridge)
        self._tab_safety = SafetyTab(
            self._bridge,
            authenticate_callback=self._safety_authenticate_callback,
            audit_callback=self._safety_audit_callback,
        )
        self._tab_telemetry = TelemetryTab(self._bridge)
        self._tab_diagnostics = DiagnosticsTab(self._bridge)
        self._tab_vision = VisionTab(
            self._bridge,
            ros_log=self._log,
            settings=self._settings,
            show_ros_log=False,
            calibration_store=self._calibration_store,
            authorization=self._authorization,
        )
        self._tab_mit = MitTab(self._bridge)
        self._tab_io = IoTab(self._bridge)

        self._stack = QStackedWidget()
        pages = {
            "home": self._build_home_page(),
            "vision": self._tab_vision,
            "grasp": self._tab_grasp,
            "motion": self._tab_motion,
            "robot3d": self._tab_robot3d,
            "pose": self._tab_pose,
            "joints": self._tab_joints,
            "gripper": self._tab_gripper,
            "safety": self._tab_safety,
            "system": self._tab_system,
            "io": self._tab_io,
            "telemetry": self._tab_telemetry,
            "diagnostics": self._tab_diagnostics,
            "mit": self._tab_mit,
            "settings": self._tab_settings,
        }
        self._pages = pages
        for key, page in pages.items():
            self._page_indexes[key] = self._stack.addWidget(page)

    def _build_shell(self, namespace: str) -> None:
        shell = QWidget()
        shell_lay = QHBoxLayout(shell)
        shell_lay.setContentsMargins(0, 0, 0, 0)
        shell_lay.setSpacing(0)

        sidebar = QFrame()
        sidebar.setObjectName("SideBar")
        sidebar.setAccessibleName("Navegação principal")
        sidebar.setFixedWidth(self._sidebar_width(collapsed=False))
        self._sidebar = sidebar
        self._sidebar_collapsed = False
        side_lay = QVBoxLayout(sidebar)
        side_lay.setContentsMargins(12, 16, 12, 14)
        side_lay.setSpacing(4)

        # Brand header is a row: the wordmark on the left, the collapse toggle
        # on the right. Collapsing frees the fixed rail down to icons + tooltips.
        brand_row = QHBoxLayout()
        brand_row.setContentsMargins(0, 0, 0, 0)
        brand_row.setSpacing(6)
        brand = QLabel("reBotArm")
        brand.setObjectName("BrandLabel")
        self._sidebar_brand = brand
        product = QLabel("CONTROLE ROBÓTICO")
        product.setObjectName("SidebarCaption")
        self._sidebar_product = product
        self._btn_collapse = QToolButton()
        self._btn_collapse.setObjectName("SidebarCollapseButton")
        self._btn_collapse.setText("«")
        self._btn_collapse.setToolTip("Recolher a barra lateral (mais área para o preview)")
        self._btn_collapse.setAccessibleName("Recolher barra lateral")
        collapse_icon = load_icon("collapse")
        if not collapse_icon.isNull():
            self._btn_collapse.setIcon(collapse_icon)
            self._btn_collapse.setIconSize(QSize(16, 16))
            self._btn_collapse.setText("")
        self._btn_collapse.setCursor(Qt.PointingHandCursor)
        self._btn_collapse.clicked.connect(self._toggle_sidebar)
        brand_row.addWidget(brand, 1)
        brand_row.addWidget(self._btn_collapse, 0)
        side_lay.addLayout(brand_row)
        side_lay.addWidget(product)
        side_lay.addSpacing(16)

        self._nav_group = QButtonGroup(self)
        self._nav_group.setExclusive(True)
        self._add_section_label(side_lay, "OPERAÇÃO")
        self._add_nav_button(side_lay, "home", "Início", "Ctrl+1")
        self._add_nav_button(side_lay, "vision", "Visão", "Ctrl+2")
        self._add_nav_button(side_lay, "grasp", "Pegas", "Ctrl+3")
        self._add_nav_button(side_lay, "motion", "Programa", "Ctrl+4")

        side_lay.addSpacing(10)
        self._add_section_label(side_lay, "MOVIMENTO MANUAL")
        self._add_nav_button(side_lay, "pose", "Pose cartesiana", "Ctrl+5")
        self._add_nav_button(side_lay, "joints", "Juntas", "Ctrl+6")
        self._add_nav_button(side_lay, "gripper", "Gripper", "Ctrl+7")

        side_lay.addSpacing(10)
        self._add_section_label(side_lay, "MONITORAMENTO")
        self._add_nav_button(side_lay, "robot3d", "Robô 3D", "Ctrl+8")
        self._add_nav_button(side_lay, "safety", "Segurança", "Ctrl+9")
        side_lay.addStretch(1)

        self._add_section_label(side_lay, "OPERADOR DA AUDITORIA")
        self._operator_edit = QLineEdit(self._settings.operator_id)
        self._operator_edit.setPlaceholderText("Identifique o operador")
        self._operator_edit.setAccessibleName("Operador da auditoria")
        self._operator_edit.setToolTip(
            "Identificação gravada com execuções, etapas, cancelamentos e falhas."
        )
        self._operator_edit.editingFinished.connect(self._on_operator_changed)
        side_lay.addWidget(self._operator_edit)
        self._sidebar_footer_widgets.append(self._operator_edit)
        side_lay.addSpacing(8)

        self._add_section_label(side_lay, "MODO DA INTERFACE")
        self._profile_combo = QComboBox()
        for profile in VALID_PROFILES:
            self._profile_combo.addItem(PROFILE_LABELS[profile], profile)
        current = self._settings.profile if self._settings.profile in VALID_PROFILES else "supervisor"
        self._profile_combo.setCurrentIndex(VALID_PROFILES.index(current))
        self._profile_combo.setToolTip(
            "Fora da produção adapta os controles exibidos. No Modo Produção, "
            "elevação para Supervisor/Engenharia exige autenticação."
        )
        self._profile_combo.setAccessibleName("Modo da interface")
        self._profile_combo.currentIndexChanged.connect(self._on_profile_changed)
        side_lay.addWidget(self._profile_combo)
        self._sidebar_footer_widgets.append(self._profile_combo)
        self._production_check = QCheckBox("Modo Produção")
        self._production_check.setToolTip(
            "Somente receitas liberadas; edição e movimento manual bloqueados; "
            "planejamento de colisão obrigatório."
        )
        self._production_check.toggled.connect(self._on_production_toggled)
        side_lay.addWidget(self._production_check)
        self._sidebar_footer_widgets.append(self._production_check)
        side_lay.addSpacing(8)

        self._advanced_button = QToolButton()
        self._advanced_button.setObjectName("AdvancedButton")
        self._advanced_button.setText("Ferramentas e ajustes")
        self._advanced_button.setToolButtonStyle(Qt.ToolButtonTextOnly)
        self._advanced_button.setPopupMode(QToolButton.InstantPopup)
        self._advanced_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        menu = QMenu(self._advanced_button)
        for key in ("system", "io", "telemetry", "diagnostics", "mit"):
            title = PAGE_META[key][0]
            action = menu.addAction(title)
            action.triggered.connect(lambda _checked=False, page=key: self.navigate(page))
            self._advanced_actions[key] = action
        menu.addSeparator()
        settings_action = menu.addAction("Configurações")
        settings_action.triggered.connect(
            lambda _checked=False: self.navigate("settings")
        )
        self._advanced_actions["settings"] = settings_action
        self._advanced_button.setMenu(menu)
        side_lay.addWidget(self._advanced_button)
        self._sidebar_footer_widgets.append(self._advanced_button)

        # Settings is also reachable directly from the sidebar footer so it is
        # not buried inside the overflow menu. It mirrors the visibility rules
        # of the equivalent entry in the advanced menu (hidden for Operador).
        self._btn_sidebar_settings = QPushButton("Configurações")
        self._btn_sidebar_settings.setObjectName("SidebarFooterButton")
        self._btn_sidebar_settings.setCursor(Qt.PointingHandCursor)
        self._btn_sidebar_settings.setToolTip("Preferências e integrações (Ctrl+,)")
        self._btn_sidebar_settings.clicked.connect(
            lambda _checked=False: self.navigate("settings")
        )
        side_lay.addWidget(self._btn_sidebar_settings)
        self._sidebar_footer_widgets.append(self._btn_sidebar_settings)

        right = QWidget()
        right_lay = QVBoxLayout(right)
        right_lay.setContentsMargins(0, 0, 0, 0)
        right_lay.setSpacing(0)
        right_lay.addWidget(self._build_top_bar(namespace))
        right_lay.addWidget(self._build_alert_banner())
        right_lay.addWidget(self._stack, 1)

        shell_lay.addWidget(sidebar)
        shell_lay.addWidget(right, 1)
        self.setCentralWidget(shell)

        self._led = StatusLed("amber", 14)
        self._lbl_ros_status = QLabel("Conectando")
        self._lbl_ros_status.setObjectName("MutedLabel")
        self._lbl_ros_status.setAccessibleName("Estado da conexão")
        self._lbl_ns = QLabel(f"/{namespace}")
        self._lbl_ns.setObjectName("MutedLabel")
        self._lbl_footer_arm = QLabel("Braço: aguardando estado")
        self._lbl_footer_arm.setObjectName("MutedLabel")
        self._lbl_footer_arm.setAccessibleName("Resumo do braço")
        self._btn_events = QPushButton("Eventos ROS")
        self._btn_events.setObjectName("StatusBarButton")
        self._btn_events.setToolTip("Mostrar ou ocultar eventos ROS (Ctrl+L)")
        self._btn_alarms = QPushButton("Alarmes")
        self._btn_alarms.setObjectName("StatusBarButton")
        self._btn_alarms.setToolTip("Abrir o centro de alarmes (histórico)")
        self._btn_keyboard_help = QPushButton("Ajuda (F1)")
        self._btn_keyboard_help.setObjectName("StatusBarButton")
        self._btn_keyboard_help.setToolTip(
            "Abrir ajuda de navegação integral por teclado"
        )

        bar = QStatusBar()
        bar.addWidget(self._led)
        bar.addWidget(self._lbl_ros_status)
        bar.addWidget(self._lbl_ns)
        bar.addWidget(self._lbl_footer_arm)
        bar.addPermanentWidget(self._btn_keyboard_help)
        bar.addPermanentWidget(self._btn_alarms)
        bar.addPermanentWidget(self._btn_events)
        self.setStatusBar(bar)

        self._btn_alarms.clicked.connect(self._show_alarm_center)
        self._btn_keyboard_help.clicked.connect(self._show_keyboard_help)

    def _build_top_bar(self, namespace: str) -> QWidget:
        top = QFrame()
        top.setObjectName("TopBar")
        lay = QGridLayout(top)
        lay.setContentsMargins(20, 10, 16, 10)
        lay.setSpacing(8)
        self._top_layout = lay
        self._top_compact: bool | None = None

        title_host = QWidget()
        title_col = QVBoxLayout()
        title_col.setContentsMargins(0, 0, 0, 0)
        title_col.setSpacing(1)
        self._page_title = QLabel("Início")
        self._page_title.setObjectName("PageTitle")
        self._page_subtitle = QLabel("")
        self._page_subtitle.setObjectName("MutedLabel")
        title_col.addWidget(self._page_title)
        title_col.addWidget(self._page_subtitle)
        # Breadcrumb shows where the current page sits in the cell workflow
        # (Visão › Pegas › Programa). Off-flow pages hide it. It is built
        # lazily and refreshed by ``navigate`` so the steps always match.
        self._breadcrumb = QWidget()
        self._breadcrumb.setObjectName("Breadcrumb")
        self._breadcrumb_lay = QHBoxLayout(self._breadcrumb)
        self._breadcrumb_lay.setContentsMargins(0, 0, 0, 0)
        self._breadcrumb_lay.setSpacing(5)
        self._breadcrumb.hide()
        title_col.addWidget(self._breadcrumb)
        title_host.setLayout(title_col)
        title_host.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self._top_title_host = title_host

        self._header_ros = QLabel(f"ROS /{namespace}")
        self._header_ros.setObjectName("StatusChip")
        self._header_arm = QLabel("BRAÇO --")
        self._header_arm.setObjectName("StatusChipWarning")
        self._header_mode = QLabel("MODO --")
        self._header_mode.setObjectName("StatusChip")
        chips = QWidget()
        chips_lay = QHBoxLayout(chips)
        chips_lay.setContentsMargins(0, 0, 0, 0)
        chips_lay.setSpacing(8)
        for chip in (self._header_ros, self._header_arm, self._header_mode):
            chips_lay.addWidget(chip)
        self._header_alarm = HeaderAlarmIndicator()
        self._header_alarm.setVisible(False)
        self._header_alarm.clicked.connect(self._show_alarm_center)
        chips_lay.addWidget(self._header_alarm)
        self._top_chips = chips

        # Routine arm commands grouped together, visually distinct from the
        # safety stop below. Enable/Disable form an enable pair, Home is a
        # warning-level move.
        ops = QFrame()
        ops.setObjectName("OpsBar")
        ops_lay = QHBoxLayout(ops)
        ops_lay.setContentsMargins(8, 6, 8, 6)
        ops_lay.setSpacing(6)
        self._btn_enable = make_button(
            "Ativar", primary=True, tip="Habilita o controlador; não inicia um programa."
        )
        self._btn_home = make_button(
            "Home seguro",
            warning=True,
            tip="Solicita movimento para a posição segura após confirmação.",
        )
        self._btn_disable = make_button(
            "Desabilitar",
            danger=True,
            tip="Desabilita o braço após confirmação.",
        )
        self._stamp_icon(self._btn_enable, "power")
        self._stamp_icon(self._btn_home, "safe-home")
        self._stamp_icon(self._btn_disable, "logout")
        self._speed_caption = QLabel("VEL")
        self._speed_caption.setObjectName("MutedLabel")
        self._speed_slider = QSlider(Qt.Horizontal)
        self._speed_slider.setRange(5, 100)
        self._speed_slider.setSingleStep(5)
        self._speed_slider.setPageStep(10)
        self._speed_slider.setValue(round(self._settings.speed_override * 100))
        self._speed_slider.setFixedWidth(112)
        self._speed_slider.setAccessibleName("Controle global de velocidade")
        self._speed_slider.setAccessibleDescription(
            "Percentual de 5 a 100 aplicado pelo controlador ao relógio comum "
            "das trajetórias. Mudanças durante movimento usam rampa segura."
        )
        self._speed_slider.setToolTip(
            "Controle global de velocidade (5–100%).\n"
            "No controlador compatível, altera também o movimento em execução por "
            "uma rampa comum a todas as juntas.\n"
            "Não deforma a trajetória e não acelera além do teto certificado. "
            "Não altera Home seguro, gripper ou MIT/Raw."
        )
        self._speed_value = QLabel(f"{self._speed_slider.value()}%")
        self._speed_value.setObjectName("SpeedOverrideValue")
        self._speed_value.setMinimumWidth(96)
        self._speed_value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._speed_value.setAccessibleName("Velocidade global efetiva")
        self._speed_slider.valueChanged.connect(self._on_speed_override_changed)
        ops_lay.addWidget(self._speed_caption)
        ops_lay.addWidget(self._speed_slider)
        ops_lay.addWidget(self._speed_value)
        ops_lay.addSpacing(6)
        ops_lay.addWidget(self._btn_enable)
        ops_lay.addWidget(self._btn_home)
        ops_lay.addWidget(self._btn_disable)
        self._top_ops = ops

        # The software stop is intentionally isolated in its own panel: it is
        # safety-relevant and must not be confused with routine enable/disable.
        safety = QFrame()
        safety.setObjectName("SafetyBar")
        safety_lay = QHBoxLayout(safety)
        safety_lay.setContentsMargins(10, 6, 10, 6)
        safety_lay.setSpacing(8)
        self._btn_estop = make_button("PARADA SOFTWARE", estop=True, fit=False)
        self._btn_estop.setToolTip(
            "Cancela o movimento e solicita a desabilitação do braço.\n"
            "Comando de software: não substitui o E-stop físico da célula."
        )
        self._btn_estop.setAccessibleName("Parada de software do braço")
        self._btn_estop.setAccessibleDescription(
            "Cancela movimentos e desabilita o braço; não é uma parada de emergência física."
        )
        safety_lay.addWidget(self._btn_estop)
        self._top_safety = safety

        self._btn_enable.setEnabled(False)
        self._btn_home.setEnabled(False)
        self._btn_disable.setEnabled(False)
        self._btn_enable.clicked.connect(self._bridge.enable)
        self._btn_home.clicked.connect(self._confirm_safe_home)
        self._btn_disable.clicked.connect(self._confirm_disable)
        self._btn_estop.clicked.connect(self._on_estop)
        self._reflow_top_bar(force=True)
        return top

    def _reflow_top_bar(self, *, force: bool = False) -> None:
        """Use two rows on compact windows so safety controls never truncate."""
        layout = getattr(self, "_top_layout", None)
        if layout is None:
            return
        compact = self.width() < 1700
        if not force and compact == self._top_compact:
            return
        self._top_compact = compact
        self._header_alarm.set_compact(compact)
        widgets = (
            self._top_title_host,
            self._top_chips,
            self._top_ops,
            self._top_safety,
        )
        for widget in widgets:
            layout.removeWidget(widget)
        if compact:
            layout.addWidget(self._top_title_host, 0, 0)
            layout.addWidget(self._top_chips, 0, 1, alignment=Qt.AlignRight)
            layout.addWidget(self._top_ops, 1, 0, alignment=Qt.AlignRight)
            layout.addWidget(self._top_safety, 1, 1, alignment=Qt.AlignRight)
            layout.setColumnStretch(0, 1)
            layout.setColumnStretch(1, 0)
        else:
            layout.addWidget(self._top_title_host, 0, 0)
            layout.addWidget(self._top_chips, 0, 1)
            layout.addWidget(self._top_ops, 0, 2)
            layout.addWidget(self._top_safety, 0, 3)
            layout.setColumnStretch(0, 1)
            for column in (1, 2, 3):
                layout.setColumnStretch(column, 0)

    def _build_alert_banner(self) -> QWidget:
        """Create the centralized alarm banner (severity, origin, time, action)."""
        banner = AlarmBanner()
        # Backwards-compatible aliases used by tests and legacy call sites.
        self._alert_label = banner.message_label
        self._alert_severity_label = banner.severity_label
        self._alert_counter_button = banner.counter_button
        self._btn_alert_events = banner.btn_history
        self._btn_alert_dismiss = banner.btn_dismiss
        banner.dismiss_requested.connect(self._dismiss_alert)
        banner.next_requested.connect(self._show_next_alert)
        banner.show_history_requested.connect(self._show_alarm_center)
        self._alert_banner = banner
        return banner

    def _build_home_page(self) -> QWidget:
        content = QWidget()
        lay = QVBoxLayout(content)
        lay.setContentsMargins(22, 18, 22, 22)
        lay.setSpacing(14)

        intro = QLabel(
            "Fluxo recomendado: valide a visão, escolha a pega e revise o programa antes de executar."
        )
        intro.setObjectName("HomeIntro")
        intro.setWordWrap(True)
        lay.addWidget(intro)

        workflow = QGroupBox("Fluxo principal")
        workflow_grid = QGridLayout(workflow)
        workflow_grid.setHorizontalSpacing(12)
        workflow_grid.setVerticalSpacing(10)
        workflow_grid.addWidget(
            self._home_action("1", "Visão e detecção", "Processar e validar objetos", "vision"),
            0,
            0,
        )
        workflow_grid.addWidget(
            self._home_action("2", "Estratégia de pega", "Revisar clearances e candidatos", "grasp"),
            0,
            1,
        )
        workflow_grid.addWidget(
            self._home_action("3", "Programa e execução", "Simular, revisar e executar", "motion"),
            0,
            2,
        )
        for column in range(3):
            workflow_grid.setColumnStretch(column, 1)
        lay.addWidget(workflow)

        commissioning = QGroupBox("Checklist de comissionamento")
        commissioning_lay = QVBoxLayout(commissioning)
        commissioning_lay.setSpacing(8)
        self._commissioning_leds: dict[str, StatusLed] = {}
        self._commissioning_states: dict[str, QLabel] = {}
        for key, title, detail, page in COMMISSIONING_STEPS:
            commissioning_lay.addWidget(
                self._build_commissioning_row(key, title, detail, page)
            )
        commissioning_footer = QHBoxLayout()
        self._commissioning_summary = QLabel()
        self._commissioning_summary.setObjectName("CommissioningSummary")
        self._btn_reset_commissioning = QPushButton("Reiniciar checklist")
        self._btn_reset_commissioning.setToolTip(
            "Mantém as configurações e inicia uma nova verificação de comissionamento."
        )
        self._btn_reset_commissioning.clicked.connect(
            self._reset_commissioning_checklist
        )
        commissioning_footer.addWidget(self._commissioning_summary, 1)
        commissioning_footer.addWidget(self._btn_reset_commissioning)
        commissioning_lay.addLayout(commissioning_footer)
        lay.addWidget(commissioning)
        self._render_commissioning_checklist()

        manual = QGroupBox("Controle manual")
        manual_grid = QGridLayout(manual)
        manual_grid.setHorizontalSpacing(12)
        manual_grid.addWidget(
            self._home_action("XYZ", "Pose cartesiana", "Mover o TCP por coordenadas", "pose"),
            0,
            0,
        )
        manual_grid.addWidget(
            self._home_action("J", "Juntas", "Ajustar alvos por junta", "joints"),
            0,
            1,
        )
        manual_grid.addWidget(
            self._home_action("G", "Gripper", "Abrir, fechar e comandar", "gripper"),
            0,
            2,
        )
        for column in range(3):
            manual_grid.setColumnStretch(column, 1)
        lay.addWidget(manual)

        dashboard = QGroupBox("Estado da célula")
        dash_grid = QGridLayout(dashboard)
        dash_grid.setHorizontalSpacing(12)
        dash_grid.setVerticalSpacing(12)
        self._dash_arm = QLabel("--")
        self._dash_mode = QLabel("--")
        self._dash_program = QLabel("--")
        self._dash_vision = QLabel("--")
        self._dash_last_error = QLabel("sem erros recentes")
        self._dash_last_seen = QLabel("--")
        for label in (
            self._dash_arm,
            self._dash_mode,
            self._dash_program,
            self._dash_vision,
            self._dash_last_error,
            self._dash_last_seen,
        ):
            label.setObjectName("DashboardValue")
        self._dash_last_error.setObjectName("DashboardHint")
        self._dash_last_seen.setObjectName("DashboardHint")
        self._card_arm = self._build_dashboard_card(
            "BRAÇO", self._dash_arm, "Toque para detalhes do controlador", "system"
        )
        self._card_mode = self._build_dashboard_card(
            "MODO", self._dash_mode, "Modo de operação atual", "system"
        )
        self._card_program = self._build_dashboard_card(
            "PROGRAMA", self._dash_program, "Toque para abrir o editor", "motion"
        )
        self._card_vision = self._build_dashboard_card(
            "OBJETOS DETECTADOS", self._dash_vision, "Toque para revisar a visão", "vision"
        )
        self._card_error = self._build_dashboard_card(
            "ÚLTIMO ERRO", self._dash_last_error, "Falha mais recente do sistema", None
        )
        self._card_seen = self._build_dashboard_card(
            "SINAL DO BRAÇO", self._dash_last_seen, "Última atualização de arm_status", None
        )
        cards = [
            self._card_arm,
            self._card_mode,
            self._card_program,
            self._card_vision,
            self._card_error,
            self._card_seen,
        ]
        for index, card in enumerate(cards):
            dash_grid.addWidget(card, index // 3, index % 3)
        for column in range(3):
            dash_grid.setColumnStretch(column, 1)
        lay.addWidget(dashboard)

        notice = QLabel(
            "Segurança: esta GUI não substitui parada de emergência, dispositivo de habilitação "
            "ou a avaliação de risco da célula. Movimentos devem permanecer sob supervisão."
        )
        notice.setObjectName("SafetyNotice")
        notice.setWordWrap(True)
        lay.addWidget(notice)
        lay.addStretch(1)

        scroll = QScrollArea()
        scroll.setObjectName("HomeScroll")
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(content)
        return scroll

    def _build_commissioning_row(
        self,
        key: str,
        title: str,
        detail: str,
        page: str,
    ) -> QWidget:
        row = QFrame()
        row.setObjectName("CommissioningRow")
        row_lay = QHBoxLayout(row)
        row_lay.setContentsMargins(10, 7, 10, 7)
        row_lay.setSpacing(10)
        led = StatusLed("amber", 14)
        led.setAccessibleName(f"Estado: {title}")
        copy = QWidget()
        copy.setObjectName("CommissioningCopy")
        copy_lay = QVBoxLayout(copy)
        copy_lay.setContentsMargins(0, 0, 0, 0)
        copy_lay.setSpacing(1)
        title_label = QLabel(title)
        title_label.setObjectName("CommissioningTitle")
        detail_label = QLabel(detail)
        detail_label.setObjectName("CommissioningDetail")
        detail_label.setWordWrap(True)
        copy_lay.addWidget(title_label)
        copy_lay.addWidget(detail_label)
        state = QLabel("Pendente")
        state.setObjectName("CommissioningState")
        action = QPushButton("Abrir")
        action.setAccessibleName(f"Abrir etapa: {title}")
        action.setToolTip(f"Abrir {PAGE_META[page][0]} para concluir esta etapa")
        action.clicked.connect(
            lambda _checked=False, destination=page: self.navigate(destination)
        )
        self._commissioning_leds[key] = led
        self._commissioning_states[key] = state
        row_lay.addWidget(led)
        row_lay.addWidget(copy, 1)
        row_lay.addWidget(state)
        row_lay.addWidget(action)
        return row

    def _home_action(self, badge: str, title: str, detail: str, page: str) -> QPushButton:
        button = QPushButton(f"{badge}   {title}\n      {detail}")
        button.setObjectName("HomeActionButton")
        button.setCursor(Qt.PointingHandCursor)
        button.clicked.connect(lambda _checked=False, key=page: self.navigate(key))
        return button

    def _build_dashboard_card(
        self, title: str, value_label: QLabel, hint: str, page: str | None
    ) -> QWidget:
        """Clickable dashboard tile. ``page`` None makes it informational only."""
        card = QPushButton() if page else QFrame()
        card.setObjectName("DashboardCard")
        card.setProperty("status", "neutral")
        card.setAccessibleName(title)
        card.setAccessibleDescription(hint)
        inner = QVBoxLayout(card)
        inner.setContentsMargins(12, 10, 12, 10)
        inner.setSpacing(2)
        key = QLabel(title)
        key.setObjectName("DashboardKey")
        value_label.setObjectName("DashboardValue")
        hint_label = QLabel(hint)
        hint_label.setObjectName("DashboardHint")
        inner.addWidget(key)
        inner.addWidget(value_label)
        inner.addWidget(hint_label)
        if page:
            card.setCursor(Qt.PointingHandCursor)
            card.clicked.connect(lambda _checked=False, key=page: self.navigate(key))
        return card

    @staticmethod
    def _set_card_status(card: QWidget, status: str) -> None:
        card.setProperty("status", status)
        card.style().unpolish(card)
        card.style().polish(card)

    def _add_section_label(self, layout: QVBoxLayout, text: str) -> None:
        label = QLabel(text)
        label.setObjectName("SidebarCaption")
        layout.addWidget(label)
        self._sidebar_section_labels.append(label)

    def _add_nav_button(
        self, layout: QVBoxLayout, key: str, text: str, shortcut: str
    ) -> None:
        button = QPushButton(text)
        button.setObjectName("NavButton")
        button.setCheckable(True)
        button.setCursor(Qt.PointingHandCursor)
        button.setToolTip(f"{text} ({shortcut})")
        button.clicked.connect(lambda _checked=False, page=key: self.navigate(page))
        icon = load_icon(PAGE_ICONS.get(key, ""))
        if not icon.isNull():
            button.setIcon(icon)
            button.setIconSize(QSize(18, 18))
        self._nav_group.addButton(button)
        self._nav_buttons[key] = button
        self._nav_labels[key] = text
        layout.addWidget(button)

    def _build_event_dock(self) -> None:
        self._event_log_panel = EventLogPanel(self._log)
        self._event_dock = QDockWidget("Eventos ROS", self)
        self._event_dock.setObjectName("EventDock")
        self._event_dock.setAllowedAreas(Qt.BottomDockWidgetArea)
        self._event_dock.setFeatures(
            QDockWidget.DockWidgetClosable | QDockWidget.DockWidgetMovable
        )
        self._event_dock.setWidget(self._event_log_panel)
        self.addDockWidget(Qt.BottomDockWidgetArea, self._event_dock)
        self._event_dock.hide()
        self._btn_events.clicked.connect(self._toggle_event_dock)

    def _build_alarm_center_dock(self) -> None:
        """Dockable alarm history with filtering and search."""
        self._alarm_center = AlarmCenterPanel()
        self._alarm_dock = QDockWidget("Alarmes", self)
        self._alarm_dock.setObjectName("AlarmDock")
        self._alarm_dock.setAllowedAreas(Qt.BottomDockWidgetArea | Qt.RightDockWidgetArea)
        self._alarm_dock.setFeatures(
            QDockWidget.DockWidgetClosable | QDockWidget.DockWidgetMovable
        )
        self._alarm_dock.setWidget(self._alarm_center)
        self.addDockWidget(Qt.BottomDockWidgetArea, self._alarm_dock)
        self._alarm_dock.hide()
        # Keep the active banner queue in sync with the history: clearing the
        # dock also dismisses every pending alarm so the two views never diverge.
        self._alarm_center.clear_requested.connect(self._on_alarm_history_cleared)

    def _on_alarm_history_cleared(self) -> None:
        now = datetime.now()
        for alarm in self._alarm_center.alarms():
            if alarm.state != STATE_CLEARED:
                alarm.state = STATE_CLEARED
                alarm.acknowledged = True
                alarm.updated_at = now
                self._alarm_store.append(alarm, "cleared")
        self._alert_queue.clear()
        self._render_alert_head()

    def _restore_persistent_alarms(self) -> None:
        """Restore durable alarm history and re-open unresolved alarms."""
        for alarm in self._alarm_store.load():
            self._alarm_center.add_alarm(alarm)
            if alarm.state == STATE_ACTIVE:
                self._alert_queue.append(alarm)
        self._render_alert_head()

    def _build_operator_test_dock(self) -> None:
        """Show the moderated timer only when explicitly requested at launch."""
        self._operator_test_panel = OperatorTestPanel()
        self._operator_test_dock = QDockWidget("Avaliação com operador", self)
        self._operator_test_dock.setObjectName("OperatorTestDock")
        self._operator_test_dock.setAllowedAreas(
            Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea
        )
        self._operator_test_dock.setFeatures(QDockWidget.DockWidgetMovable)
        self._operator_test_dock.setWidget(self._operator_test_panel)
        self.addDockWidget(Qt.RightDockWidgetArea, self._operator_test_dock)
        # Keep the evaluated interface at its normal geometry: docking the
        # timer would shrink the content and bias task times at 1360 x 720.
        self._operator_test_dock.setFloating(True)
        self._operator_test_dock.resize(500, 760)

    def _configure_accessibility(self) -> None:
        """Apply names, form-label buddies and page landmarks after construction."""
        self._stack.setAccessibleName("Conteúdo da página atual")
        for key, page in self._pages.items():
            title, subtitle = PAGE_META[key]
            page.setAccessibleName(title)
            page.setAccessibleDescription(subtitle)
            configure_accessibility(page)
        configure_accessibility(self._event_log_panel)
        configure_accessibility(self._alarm_center)
        if self._operator_test_panel is not None:
            configure_accessibility(self._operator_test_panel)
        configure_accessibility(self)

    def _wire_signals(self) -> None:
        b = self._bridge
        b.connected.connect(self._on_connected)
        b.arm_alive.connect(self._on_arm_alive)
        b.joint_state_changed.connect(self._on_joint_state)
        b.arm_status_changed.connect(self._on_arm_status)
        b.timing_status_changed.connect(self._tab_system.update_timing_status)
        b.speed_override_status_changed.connect(
            self._on_speed_override_status
        )
        b.gripper_state_changed.connect(self._on_gripper_state)
        b.joint_detail_changed.connect(self._on_joint_detail)
        b.diagnostics_changed.connect(self._tab_diagnostics.update_snapshot)
        b.service_result.connect(self._on_service_result)
        b.action_result.connect(self._on_action_result)
        b.safety_check_result.connect(self._tab_safety.update_check_result)
        self._tab_safety.alarm_changed.connect(self._on_safety_alarm)
        b.vision_objects_changed.connect(self._tab_vision.update_objects)
        b.vision_objects_changed.connect(self._tab_grasp.update_objects)
        b.vision_objects_changed.connect(self._on_vision_objects)
        b.vision_debug_image_changed.connect(self._tab_vision.update_debug_image)
        b.vision_debug_image_changed.connect(self._tab_grasp.update_debug_image)
        b.vision_raw_image_changed.connect(self._tab_vision.update_raw_image)
        b.vision_raw_image_changed.connect(self._tab_grasp.update_raw_image)
        self._tab_vision.workspace_limit_changed.connect(
            self._tab_grasp.update_workspace_limit
        )
        self._tab_vision.planning_config_changed.connect(
            self._tab_grasp.apply_external_config
        )
        self._tab_vision.continue_to_grasp.connect(
            self._on_continue_to_grasp
        )
        self._tab_vision.calibration_status_changed.connect(
            self._on_calibration_status_changed
        )
        self._tab_vision.async_operation_started.connect(
            self._begin_async_operation
        )
        self._tab_vision.async_operation_finished.connect(
            self._finish_async_operation
        )
        self._tab_robot3d.async_operation_started.connect(
            self._begin_async_operation
        )
        self._tab_robot3d.async_operation_finished.connect(
            self._finish_async_operation
        )
        self._tab_diagnostics.snapshot_exported.connect(
            self._on_diagnostic_snapshot_exported
        )
        self._cell_scene_store.applied_changed.connect(
            self._on_cell_scene_applied
        )
        self._tab_grasp.motion_program_generated.connect(
            lambda program: self._load_generated_program(program)
        )
        self._tab_motion.program_loaded.connect(
            lambda: self._set_commissioning_step("program_generated", True)
        )
        self._tab_motion.toast_requested.connect(
            lambda message, kind: self.show_toast(message, kind=kind)
        )
        self._tab_settings._btn_save.clicked.connect(
            self._sync_operator_from_settings
        )
        self._tab_settings.credential_saved.connect(
            self._on_credentials_configured
        )
        b.log_message.connect(self._on_log_message)
        self._tab_system.request_safe_home.connect(self._confirm_safe_home)
        self._tab_system.request_disable.connect(self._confirm_disable)

    def _install_shortcuts(self) -> None:
        bindings = {
            "Ctrl+1": "home",
            "Ctrl+2": "vision",
            "Ctrl+3": "grasp",
            "Ctrl+4": "motion",
            "Ctrl+5": "pose",
            "Ctrl+6": "joints",
            "Ctrl+7": "gripper",
            "Ctrl+8": "robot3d",
            "Ctrl+9": "safety",
            "Ctrl+,": "settings",
        }
        for sequence, page in bindings.items():
            shortcut = QShortcut(QKeySequence(sequence), self)
            shortcut.activated.connect(
                lambda key=page: self.navigate(key, focus_content=True)
            )
            self._shortcuts.append(shortcut)
        log_shortcut = QShortcut(QKeySequence("Ctrl+L"), self)
        log_shortcut.activated.connect(self._toggle_event_dock)
        self._shortcuts.append(log_shortcut)
        alarm_shortcut = QShortcut(QKeySequence("Ctrl+Shift+A"), self)
        alarm_shortcut.activated.connect(self._show_alarm_center)
        self._shortcuts.append(alarm_shortcut)
        help_shortcut = QShortcut(QKeySequence("F1"), self)
        help_shortcut.activated.connect(self._show_keyboard_help)
        self._shortcuts.append(help_shortcut)
        palette_shortcut = QShortcut(QKeySequence("Ctrl+K"), self)
        palette_shortcut.activated.connect(self._open_command_palette)
        self._shortcuts.append(palette_shortcut)
        next_region = QShortcut(QKeySequence("F6"), self)
        next_region.activated.connect(lambda: self._cycle_focus_region(1))
        self._shortcuts.append(next_region)
        previous_region = QShortcut(QKeySequence("Shift+F6"), self)
        previous_region.activated.connect(lambda: self._cycle_focus_region(-1))
        self._shortcuts.append(previous_region)
        # A software stop intentionally has no global keyboard shortcut.  Esc
        # must remain available to cancel dialogs and editing operations.

    # ------------------------------------------------------------------
    # Navigation and global commands
    # ------------------------------------------------------------------
    def navigate(self, key: str, *, focus_content: bool = False) -> None:
        """Show a page by stable key and update the surrounding context."""
        if key not in self._page_indexes:
            raise KeyError(f"Página desconhecida: {key}")
        profile = self._settings.profile
        if key in PROFILE_HIDDEN_PAGES.get(profile, set()):
            self.statusBar().showMessage(
                f"Página bloqueada no perfil {PROFILE_LABELS.get(profile, profile)}.", 4000
            )
            return
        self._stack.setCurrentIndex(self._page_indexes[key])
        title, subtitle = PAGE_META[key]
        self._page_title.setText(tr(f"page.{key}.title", title))
        self._page_subtitle.setText(tr(f"page.{key}.subtitle", subtitle))
        self._refresh_breadcrumb(key)

        if key in self._nav_buttons:
            self._nav_buttons[key].setChecked(True)
            self._advanced_button.setText("Ferramentas e ajustes")
            self._advanced_button.setToolTip("Abrir ferramentas e ajustes")
        else:
            self._nav_group.setExclusive(False)
            for button in self._nav_buttons.values():
                button.setChecked(False)
            self._nav_group.setExclusive(True)
            # The selected page is already named in the header. Keeping the
            # menu label compact avoids truncation for titles such as
            # "Diagnóstico ROS" in the fixed-width sidebar.
            self._advanced_button.setText("Ferramentas")
            self._advanced_button.setToolTip(
                f"Página atual: {title}. Clique para abrir outras ferramentas e ajustes."
            )
        announce(self._page_title, f"Página {title}. {subtitle}")
        if focus_content:
            QTimer.singleShot(0, lambda page=self._pages[key]: focus_first(page))

    def _focus_region_roots(self) -> list[QWidget]:
        roots = [self._sidebar, self._pages[
            next(
                (k for k, idx in self._page_indexes.items()
                 if idx == self._stack.currentIndex()),
                "home",
            )
        ], self._top_ops, self.statusBar()]
        if self._alarm_dock.isVisible():
            roots.append(self._alarm_center)
        if self._event_dock.isVisible():
            roots.append(self._event_log_panel)
        if (
            self._operator_test_dock is not None
            and self._operator_test_dock.isVisible()
            and self._operator_test_panel is not None
        ):
            roots.append(self._operator_test_panel)
        return [root for root in roots if first_focusable(root) is not None]

    def _cycle_focus_region(self, direction: int) -> None:
        """Implement the desktop F6/Shift+F6 convention between UI regions."""
        roots = self._focus_region_roots()
        if not roots:
            return
        current = QApplication.focusWidget()
        current_index = -1
        for index, root in enumerate(roots):
            if current is root or (current is not None and root.isAncestorOf(current)):
                current_index = index
                break
        target_root = roots[(current_index + direction) % len(roots)]
        target = focus_first(target_root)
        if target is not None:
            announce(target, f"Foco em {target_root.accessibleName() or 'região da interface'}")

    # ------------------------------------------------------------------
    # Breadcrumb flow indicator
    # ------------------------------------------------------------------
    def _refresh_breadcrumb(self, key: str) -> None:
        """Rebuild the Visão › Pegas › Programa indicator for ``key``."""
        lay = self._breadcrumb_lay
        while lay.count():
            item = lay.takeAt(0)
            child = item.widget()
            if child is not None:
                child.deleteLater()
        flow = PAGE_FLOW.get(key)
        if flow is None:
            self._breadcrumb.hide()
            return
        _group, steps, current = flow
        for index, step in enumerate(steps):
            if index > 0:
                sep = QLabel("›")
                sep.setObjectName("BreadcrumbSeparator")
                lay.addWidget(sep)
            label = QLabel(step)
            label.setObjectName("BreadcrumbStep")
            label.setProperty("current", "true" if step == current else "false")
            lay.addWidget(label)
        lay.addStretch(1)
        self._breadcrumb.show()

    # ------------------------------------------------------------------
    # Collapsible sidebar
    # ------------------------------------------------------------------
    def _toggle_sidebar(self) -> None:
        self._set_sidebar_collapsed(not self._sidebar_collapsed)

    def _sidebar_width(self, *, collapsed: bool) -> int:
        base = (
            SIDEBAR_WIDTH_COLLAPSED
            if collapsed
            else SIDEBAR_WIDTH_EXPANDED
        )
        scale = clamp_font_scale(self._settings.font_scale_percent) / 100.0
        return int(round(base * scale))

    def _update_sidebar_width_for_scale(self) -> None:
        sidebar = getattr(self, "_sidebar", None)
        if sidebar is not None:
            sidebar.setFixedWidth(
                self._sidebar_width(collapsed=self._sidebar_collapsed)
            )

    def _set_sidebar_collapsed(self, collapsed: bool) -> None:
        """Toggle the rail between full labels and icon-only mode."""
        collapsed = bool(collapsed)
        if collapsed == self._sidebar_collapsed:
            return
        self._sidebar_collapsed = collapsed
        mode = "collapsed" if collapsed else "expanded"
        self._sidebar.setFixedWidth(
            self._sidebar_width(collapsed=collapsed)
        )
        for key, button in self._nav_buttons.items():
            button.setProperty("sidebar", mode)
            button.setText("" if collapsed else self._nav_labels.get(key, ""))
            button.style().unpolish(button)
            button.style().polish(button)
        for label in self._sidebar_section_labels:
            label.setVisible(not collapsed)
        for widget in self._sidebar_footer_widgets:
            widget.setVisible(not collapsed)
        # Brand wordmark and caption take space the icons need when collapsed.
        self._sidebar_brand.setVisible(not collapsed)
        self._sidebar_product.setVisible(not collapsed)
        # Swap the chevron direction so the toggle reads as an affordance.
        expand_icon = load_icon("expand")
        collapse_icon = load_icon("collapse")
        new_icon = expand_icon if collapsed else collapse_icon
        if not new_icon.isNull():
            self._btn_collapse.setIcon(new_icon)
            self._btn_collapse.setText("")
        else:
            self._btn_collapse.setIcon(QIcon())
            self._btn_collapse.setText("»" if collapsed else "«")
        self._btn_collapse.setToolTip(
            "Expandir a barra lateral"
            if collapsed
            else "Recolher a barra lateral (mais área para o preview)"
        )
        self._btn_collapse.setAccessibleName(
            "Expandir barra lateral" if collapsed else "Recolher barra lateral"
        )
        self._persist_sidebar_state()
        # The stacked content reflows; give it a tick to relayout.
        QTimer.singleShot(0, self._reflow_top_bar)

    def _restore_sidebar_state(self) -> None:
        if not self._restore_session_enabled:
            return
        collapsed = bool(self._session_settings().value("session/sidebarCollapsed", False))
        if collapsed:
            self._set_sidebar_collapsed(True)

    def _persist_sidebar_state(self) -> None:
        s = self._session_settings()
        try:
            s.setValue("session/sidebarCollapsed", self._sidebar_collapsed)
            s.sync()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Transient confirmation toast
    # ------------------------------------------------------------------
    def show_toast(self, text: str, *, kind: str = "info") -> None:
        """Show a short-lived, non-blocking confirmation in the corner."""
        if getattr(self, "_toast", None) is not None:
            self._toast.show(text, kind=kind)

    def _begin_async_operation(self, operation_id: str, message: str) -> None:
        """Publish one consistent visible and durable start notification."""
        operation = str(operation_id).strip()
        if not operation or operation in self._async_operations:
            return
        label = ASYNC_OPERATION_LABELS.get(operation, operation)
        self._async_operations[operation] = label
        text = str(message).strip() or f"{label} iniciada."
        self._append_log(f"[op:{operation}] INÍCIO: {text}")
        self.statusBar().showMessage(text, 5000)
        self.show_toast(text, kind="info")

    def _finish_async_operation(
        self,
        operation_id: str,
        ok: bool,
        message: str,
    ) -> bool:
        """Publish one terminal notification and suppress duplicate callbacks."""
        operation = str(operation_id).strip()
        label = self._async_operations.pop(operation, None)
        if label is None:
            return False
        detail = str(message).strip()
        if ok:
            text = f"{label} concluída"
            if detail:
                text += f": {detail}"
            kind = "success"
            tag = "OK"
        else:
            text = f"{label} falhou"
            if detail:
                text += f": {detail}"
            kind = "error"
            tag = "ERRO"
        self._append_log(f"[op:{operation}] {tag}: {text}")
        self.statusBar().showMessage(text, 7000)
        self.show_toast(text, kind=kind)
        return True

    def _on_diagnostic_snapshot_exported(self, ok: bool, message: str) -> None:
        tag = "OK" if ok else "ERRO"
        text = str(message)
        self._append_log(f"[diagnóstico:exportar] {tag}: {text}")
        self.statusBar().showMessage(text, 7000)
        self.show_toast(text, kind="success" if ok else "error")

    # ------------------------------------------------------------------
    # Command palette (Ctrl+K)
    # ------------------------------------------------------------------
    def _build_commands(self) -> list[Command]:
        commands: list[Command] = []
        profile = self._settings.profile
        hidden = PROFILE_HIDDEN_PAGES.get(profile, set())
        for key in self._page_indexes:
            if key in hidden:
                continue
            title, _subtitle = PAGE_META[key]
            commands.append(
                Command(
                    id=f"nav:{key}",
                    title=title,
                    group="Ir para",
                    hint=f"Abrir a página {title}",
                    callback=lambda _k=key: self.navigate(_k, focus_content=True),
                )
            )
        commands.append(
            Command(
                id="act:events",
                title="Eventos ROS",
                group="Ação",
                hint="Mostrar ou ocultar o painel de eventos (Ctrl+L)",
                callback=self._toggle_event_dock,
            )
        )
        commands.append(
            Command(
                id="act:alarms",
                title="Centro de alarmes",
                group="Ação",
                hint="Abrir o histórico de alarmes (Ctrl+Shift+A)",
                callback=self._show_alarm_center,
            )
        )
        commands.append(
            Command(
                id="act:sidebar",
                title="Recolher/expandir barra lateral",
                group="Ação",
                hint="Alterna o modo só-ícones da navegação",
                callback=self._toggle_sidebar,
            )
        )
        commands.append(
            Command(
                id="act:help",
                title="Ajuda de navegação",
                group="Ação",
                hint="Atalhos e navegação por teclado (F1)",
                callback=self._show_keyboard_help,
            )
        )
        commands.append(
            Command(
                id="act:theme",
                title="Alternar tema claro/escuro",
                group="Ação",
                hint="Muda o tema da interface para alta luz ambiente",
                callback=self._toggle_theme,
            )
        )
        return commands

    def _open_command_palette(self) -> None:
        if self._command_palette is None:
            self._command_palette = CommandPalette([], self)
            self._command_palette.command_activated.connect(self._on_palette_command)
        self._command_palette.rebuild(self._build_commands())
        self._command_palette.show()
        self._command_palette.raise_()
        self._command_palette.activateWindow()

    def _on_palette_command(self, cmd_id: str) -> None:
        # Callbacks are already bound to each command; this hook is reserved
        # for future telemetry/logging of which command was launched.
        _ = cmd_id

    # ------------------------------------------------------------------
    # Theme selection
    # ------------------------------------------------------------------
    def apply_theme(self, theme: str) -> None:
        """Apply ``theme`` (dark/light) immediately and persist the choice.

        The stylesheet is only re-applied when the theme actually changes:
        ``QApplication.setStyleSheet`` re-polishes every live widget, so calling
        it on every window construction makes a long session progressively
        slower. The active theme is memoised on the application object.
        """
        if theme not in VALID_THEMES:
            theme = "dark"
        self._settings.theme = theme
        app = QApplication.instance()
        scale = clamp_font_scale(self._settings.font_scale_percent)
        if app is not None and (
            app.property("rebotarm_theme") != theme
            or app.property("rebotarm_font_scale") != scale
        ):
            style = load_stylesheet(theme, scale)
            if style:
                app.setStyleSheet(style)
            app.setProperty("rebotarm_theme", theme)
            app.setProperty("rebotarm_font_scale", scale)
        self._update_sidebar_width_for_scale()
        self._sync_theme_selector()

    def apply_font_scale(self, percent: int) -> None:
        """Apply the persisted 90–130% typography scale immediately."""
        scale = clamp_font_scale(percent)
        self._settings.font_scale_percent = scale
        app = QApplication.instance()
        if app is not None and (
            app.property("rebotarm_theme") != self._settings.theme
            or app.property("rebotarm_font_scale") != scale
        ):
            style = load_stylesheet(self._settings.theme, scale)
            if style:
                app.setStyleSheet(style)
            app.setProperty("rebotarm_theme", self._settings.theme)
            app.setProperty("rebotarm_font_scale", scale)
        self._update_sidebar_width_for_scale()
        self.statusBar().showMessage(
            f"Escala das fontes aplicada: {scale}%", 4000
        )

    def _toggle_theme(self) -> None:
        new_theme = "light" if self._settings.theme == "dark" else "dark"
        self.apply_theme(new_theme)
        try:
            save_app_settings(self._settings)
        except OSError:
            pass
        self.show_toast(f"Tema: {new_theme.capitalize()}", kind="info")

    def _sync_theme_selector(self) -> None:
        """Keep the settings-page combo in sync with the active theme."""
        combo = getattr(self._tab_settings, "_theme_combo", None)
        if combo is None:
            return
        index = combo.findData(self._settings.theme)
        if index >= 0 and index != combo.currentIndex():
            combo.blockSignals(True)
            combo.setCurrentIndex(index)
            combo.blockSignals(False)

    def _show_keyboard_help(self) -> None:
        QMessageBox.information(
            self,
            "Navegação por teclado",
            "Tab / Shift+Tab: próximo ou controle anterior\n"
            "F6 / Shift+F6: próxima ou região anterior\n"
            "Setas: navegar em listas, tabelas, menus, abas e sliders\n"
            "Espaço: marcar opções ou acionar o controle focado\n"
            "Enter: acionar o botão padrão\n"
            "Esc: fechar diálogo ou cancelar edição (não para o robô)\n\n"
            "Ctrl+1…Ctrl+9: páginas principais\n"
            "Ctrl+K: paleta de comandos (busca páginas e ações)\n"
            "Ctrl+L: Eventos ROS\n"
            "Ctrl+Shift+A: Alarmes\n"
            "Ctrl+,: Configurações\n"
            "F1: esta ajuda\n\n"
            "Jog por teclado (opt-in nas abas Juntas/Pose):\n"
            "  Juntas: ↑/↓ move a junta ativa · ←/→ troca de junta\n"
            "  Pose: ←/→ eixo X · ↑/↓ eixo Y · Page Up/Down eixo Z\n\n"
            "Movimentos e a parada de software não possuem atalho global por segurança.",
        )

    def _toggle_event_dock(self) -> None:
        self._event_dock.setVisible(not self._event_dock.isVisible())
        if self._event_dock.isVisible():
            self._event_dock.raise_()
            QTimer.singleShot(0, lambda: focus_first(self._event_log_panel))

    def _show_event_dock(self) -> None:
        self._event_dock.show()
        self._event_dock.raise_()
        QTimer.singleShot(0, lambda: focus_first(self._event_log_panel))

    def _show_alarm_center(self) -> None:
        """Open the dockable alarm history panel."""
        self._alarm_dock.show()
        self._alarm_dock.raise_()
        QTimer.singleShot(0, lambda: focus_first(self._alarm_center))

    def emit_alarm(
        self,
        message: str,
        *,
        severity: str = "warning",
        origin: str = "Sistema",
        recommended_action: str = "",
        code: str = "",
    ) -> Alarm:
        """Central alarm producer: updates banner queue and history dock.

        Every operational alarm in the GUI routes through here so the banner
        (active alarm) and the dock (full session history) never diverge.
        Repeated identical alarms (same origin + severity + message) within a
        short window are coalesced into a single entry to avoid alarm storms.
        """
        now = datetime.now()
        if self._alert_queue:
            last = self._alert_queue[-1]
            if (
                last.origin == origin
                and last.severity == severity
                and last.message == message
                and (now - last.timestamp).total_seconds() < ALERT_DEDUP_WINDOW_S
            ):
                # Same alarm repeating: refresh timestamp and bump the count
                # instead of flooding the banner and the history table.
                last.timestamp = now
                last.updated_at = now
                last.count += 1
                last.state = STATE_ACTIVE
                last.acknowledged = False
                self._alarm_store.append(last, "updated")
                self._render_alert_head()
                self._alarm_center.refresh()
                return last
        alarm = Alarm(
            message=message,
            severity=severity,
            origin=origin,
            recommended_action=recommended_action,
            code=code,
        )
        self._alert_queue.append(alarm)
        self._alarm_center.add_alarm(alarm)
        self._alarm_store.append(alarm, "raised")
        self._render_alert_head()
        announce(
            self._alert_banner,
            f"{alarm.severity}: {alarm.message}. Ação recomendada: "
            f"{alarm.recommended_action}",
            assertive=alarm.severity in {"error", "warning"},
        )
        return alarm

    def _show_persistent_alert(self, text: str, severity: str = "error") -> None:
        """Backwards-compatible shim that routes into the central alarm path."""
        self.emit_alarm(text, severity=severity, origin="Sistema")

    def _render_alert_head(self) -> None:
        if not self._alert_queue:
            self._alert_banner.render_alarm(None)
            self._header_alarm.update_alarms(self._alarm_center.alarms())
            return
        head = self._alert_queue[0]
        pending = len(self._alert_queue) - 1
        self._alert_banner.render_alarm(head, pending)
        self._header_alarm.update_alarms(self._alarm_center.alarms())

    def _dismiss_alert(self) -> None:
        if self._alert_queue:
            head = self._alert_queue.popleft()
            head.acknowledged = True
            head.state = STATE_ACKNOWLEDGED
            head.updated_at = datetime.now()
            self._alarm_store.append(head, "acknowledged")
            self._alarm_center.refresh()
        self._render_alert_head()

    def clear_alarm(self, *, code: str = "", origin: str = "") -> int:
        """Normalize matching active or acknowledged alarms durably."""
        cleared = 0
        cleared_ids: set[str] = set()
        for alarm in self._alarm_center.alarms():
            matches = (not code or alarm.code == code) and (
                not origin or alarm.origin == origin
            )
            if matches and alarm.state != STATE_CLEARED:
                alarm.state = STATE_CLEARED
                alarm.acknowledged = True
                alarm.updated_at = datetime.now()
                self._alarm_store.append(alarm, "cleared")
                cleared_ids.add(alarm.alarm_id)
                cleared += 1
        self._alert_queue = deque(
            (
                alarm
                for alarm in self._alert_queue
                if alarm.alarm_id not in cleared_ids
            ),
            maxlen=ALERT_QUEUE_MAX,
        )
        self._alarm_center.refresh()
        self._render_alert_head()
        return cleared

    def _show_next_alert(self) -> None:
        """Cycle through queued alarms so the operator can review all of them."""
        if len(self._alert_queue) > 1:
            head = self._alert_queue.popleft()
            self._alert_queue.append(head)
        self._render_alert_head()

    def _confirm_safe_home(self) -> None:
        answer = QMessageBox.question(
            self,
            "Mover para Home seguro",
            "Confirma o movimento para Home seguro?\n\n"
            "Verifique se a área de trabalho está livre e mantenha supervisão do braço.",
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer == QMessageBox.Yes:
            self._bridge.safe_home()

    def _confirm_disable(self) -> None:
        answer = QMessageBox.question(
            self,
            "Desabilitar o braço",
            "Confirma desabilitar o braço?\n\n"
            "O controlador interrompe o movimento e corta a habilitação dos motores.",
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer == QMessageBox.Yes:
            self._bridge.disable()

    def _on_estop(self) -> None:
        """Software stop button: cancel motion and disable the arm."""
        self._append_log("Parada de software acionada pelo operador.")
        self.statusBar().showMessage(
            "PARADA SOFTWARE: cancelamento e desabilitação solicitados.", 5000
        )
        self._tab_motion.stop_for_software_stop()
        self._bridge.emergency_stop()

    def _on_speed_override_changed(self, percent: int) -> None:
        """Apply the global override immediately and debounce persistence."""
        fraction = self._bridge.set_speed_override(float(percent) / 100.0)
        normalized_percent = int(round(fraction * 100.0))
        self._settings.speed_override = fraction
        self._speed_value.setText(f"{normalized_percent}%")
        self._speed_value.setAccessibleDescription(
            f"Velocidade global em {normalized_percent} por cento."
        )
        scope = (
            "rampa enviada ao controlador"
            if self._bridge.dynamic_speed_override_supported
            else "aplicada aos próximos movimentos (controlador legado)"
        )
        self.statusBar().showMessage(
            f"Velocidade global: {normalized_percent}% — {scope}.",
            3000,
        )
        self._speed_save_timer.start()

    def _on_speed_override_status(self, status: object) -> None:
        if not isinstance(status, dict):
            return
        try:
            requested = max(0.0, min(1.0, float(status["requested"])))
            applied = max(0.0, min(1.0, float(status["applied"])))
        except (KeyError, TypeError, ValueError):
            return
        requested_percent = int(round(requested * 100.0))
        applied_percent = int(round(applied * 100.0))
        ramping = bool(status.get("ramping", False))
        if ramping or requested_percent != applied_percent:
            self._speed_value.setText(
                f"{requested_percent}% → {applied_percent}%"
            )
        else:
            self._speed_value.setText(f"{applied_percent}%")
        detail = str(status.get("message", "")).strip()
        self._speed_value.setToolTip(
            "Override solicitado: "
            f"{requested_percent}%\nOverride aplicado: {applied_percent}%"
            + (f"\n{detail}" if detail else "")
        )
        self._speed_value.setAccessibleDescription(
            f"Velocidade solicitada em {requested_percent} por cento; "
            f"velocidade aplicada em {applied_percent} por cento."
        )

    def _persist_speed_override(self) -> None:
        try:
            save_app_settings(self._settings)
        except OSError as exc:
            self.statusBar().showMessage(
                f"Falha ao salvar velocidade global: {exc}", 5000
            )

    # ------------------------------------------------------------------
    # user profiles
    # ------------------------------------------------------------------
    def _on_profile_changed(self, index: int) -> None:
        profile = self._profile_combo.itemData(index)
        if (
            profile != ROLE_OPERATOR
            and (
                self._authorization.production_mode
                or self._credentials.has_users()
            )
        ):
            identity = self._authenticate_privileged()
            if identity is None or (
                identity.role != profile and identity.role != "engenharia"
            ):
                self._apply_profile(ROLE_OPERATOR, persist=False)
                return
            self._authorization.set_identity(identity)
        elif profile == ROLE_OPERATOR and self._authorization.enforce_rbac:
            self._authorization.set_identity(
                Identity(
                    self._settings.operator_id.strip() or "operador",
                    ROLE_OPERATOR,
                    True,
                )
            )
        if self._authorization.enforce_rbac:
            self._bridge.publish_authorization_lease()
            self._authorization_timer.start()
        self._apply_profile(profile, persist=True)

    def _on_credentials_configured(self) -> None:
        """Close commissioning bootstrap after the first account is created."""
        self._authorization.set_enforcement(True)
        self._authorization.set_identity(
            Identity(
                self._settings.operator_id.strip() or "operador",
                ROLE_OPERATOR,
                True,
            )
        )
        self._apply_profile(ROLE_OPERATOR, persist=True)
        self._bridge.publish_authorization_lease()
        self._authorization_timer.start()
        self.statusBar().showMessage(
            "Permissões ativadas; autentique-se para Supervisor/Engenharia.",
            6000,
        )

    def _authenticate_privileged(self) -> Identity | None:
        if not self._credentials.has_users():
            QMessageBox.warning(
                self,
                "Autenticação não configurada",
                "Cadastre ao menos um Supervisor/Engenharia em Configurações → "
                "Segurança e permissões.",
            )
            return None
        user, ok = QInputDialog.getText(
            self,
            "Autenticação",
            "Usuário:",
            QLineEdit.EchoMode.Normal,
            self._operator_edit.text().strip(),
        )
        if not ok:
            return None
        pin, ok = QInputDialog.getText(
            self,
            "Autenticação",
            "PIN:",
            QLineEdit.EchoMode.Password,
        )
        if not ok:
            return None
        identity = self._credentials.authenticate(user, pin)
        if not identity.authenticated or identity.role == ROLE_OPERATOR:
            QMessageBox.critical(self, "Acesso negado", "Credencial privilegiada inválida.")
            return None
        self._settings.operator_id = identity.user
        self._operator_edit.setText(identity.user)
        return identity

    def _safety_authenticate_callback(self) -> bool:
        """Boolean gate used by SafetyTab for privileged mutations."""
        return self._authenticate_privileged() is not None

    def _safety_audit_callback(self, record: dict) -> None:
        """Forward safety-tab audit records to the persistent execution audit log."""
        try:
            event = record.pop("event", "safety_profile_change")
            self._audit_log.append(
                event,
                run_id=new_run_id(),
                **record,
            )
        except Exception:
            # Audit failures must not block safety actions; they are surfaced
            # through the persistent log's last_error field elsewhere.
            pass

    def _on_production_toggled(self, enabled: bool) -> None:
        if enabled:
            calibration_ok, calibration_reason, _reference = (
                self._tab_vision.production_calibration_status()
            )
            if not calibration_ok:
                self._production_check.blockSignals(True)
                self._production_check.setChecked(False)
                self._production_check.blockSignals(False)
                QMessageBox.critical(
                    self,
                    "Modo Produção bloqueado",
                    "A calibração da célula não está pronta:\n\n"
                    f"{calibration_reason}\n\n"
                    "Abra Visão → Configuração avançada → Calibração px→XY.",
                )
                self.emit_alarm(
                    f"Modo Produção bloqueado: {calibration_reason}",
                    severity="warning",
                    origin="Calibração",
                    code="CALIBRATION_INVALID",
                    recommended_action=(
                        "Valide e aprove um perfil compatível com a câmera, "
                        "TCP e ferramenta ativos."
                    ),
                )
                return
            identity = self._authenticate_privileged()
            if identity is None:
                self._production_check.blockSignals(True)
                self._production_check.setChecked(False)
                self._production_check.blockSignals(False)
                return
            self._authorization.set_identity(identity)
            self._authorization.set_production_mode(True)
            if not self._bridge.publish_authorization_lease():
                self._authorization.set_production_mode(False)
                self._production_check.blockSignals(True)
                self._production_check.setChecked(False)
                self._production_check.blockSignals(False)
                QMessageBox.critical(
                    self,
                    "Modo Produção bloqueado",
                    "Não foi possível publicar a autorização assinada para o controlador.",
                )
                return
            self._authorization_timer.start()
            self._settings.production_mode = True
            self._settings.collision_planning_required = True
            self._apply_profile(ROLE_OPERATOR, persist=False)
            self.emit_alarm(
                "Modo Produção ativado.",
                severity="info",
                origin="Autorização",
                code="PRODUCTION_MODE",
                recommended_action="Execute apenas a receita liberada exibida na página Programa.",
            )
        else:
            self._authorization.set_production_mode(False)
            # The controller remains fail-closed after its first valid lease.
            # Keep renewing the authenticated supervisor/engineering session
            # for commissioning commands after leaving Production mode.
            self._bridge.publish_authorization_lease()
            self._settings.production_mode = False
            self.clear_alarm(code="PRODUCTION_MODE")
        self._tab_motion.set_operating_mode(
            production=enabled,
            collision_planning_required=(
                True if enabled else self._settings.collision_planning_required
            ),
        )
        self._apply_profile(self._settings.profile, persist=False)
        try:
            save_app_settings(self._settings)
        except OSError as exc:
            self.statusBar().showMessage(f"Falha ao salvar modo operacional: {exc}", 5000)

    def _on_calibration_status_changed(self, valid: bool, message: str) -> None:
        self._set_commissioning_step("calibration_approved", valid)
        if valid:
            self.clear_alarm(code="CALIBRATION_INVALID")
            self.statusBar().showMessage(f"Calibração pronta: {message}", 5000)
            return
        self.emit_alarm(
            message or "Calibração da célula inválida.",
            severity="warning",
            origin="Calibração",
            code="CALIBRATION_INVALID",
            recommended_action=(
                "Abra Visão → Configuração avançada, valide os pontos e "
                "aprove uma revisão compatível."
            ),
        )
        if not self._authorization.production_mode:
            return
        # A changed/tampered/expired profile invalidates the production lease
        # at the GUI level and blocks every subsequent dispatch in MotionTab.
        self._production_check.setChecked(False)
        self.statusBar().showMessage(
            "Modo Produção encerrado: calibração deixou de ser válida.", 7000
        )

    def _on_operator_changed(self) -> None:
        operator = self._operator_edit.text().strip()
        self._settings.operator_id = operator
        # Keep the equivalent field on the Settings page synchronized.
        settings_edit = getattr(self._tab_settings, "_operator_id", None)
        if settings_edit is not None and settings_edit.text() != operator:
            settings_edit.setText(operator)
        try:
            save_app_settings(self._settings)
        except OSError as exc:
            self.statusBar().showMessage(f"Falha ao salvar operador: {exc}", 5000)
            return
        shown = operator or "não identificado"
        self.statusBar().showMessage(f"Operador da auditoria: {shown}", 4000)

    def _sync_operator_from_settings(self) -> None:
        operator = self._settings.operator_id.strip()
        if self._operator_edit.text() != operator:
            self._operator_edit.setText(operator)

    def _apply_profile(self, profile: str, *, persist: bool) -> None:
        if profile not in VALID_PROFILES:
            profile = "supervisor"
        self._settings.profile = profile
        hidden = set(PROFILE_HIDDEN_PAGES.get(profile, set()))
        if self._authorization.production_mode:
            hidden.update(PRODUCTION_HIDDEN_PAGES)

        combo = getattr(self, "_profile_combo", None)
        if combo is not None and combo.currentData() != profile:
            combo.blockSignals(True)
            combo.setCurrentIndex(VALID_PROFILES.index(profile))
            combo.blockSignals(False)

        for key, button in self._nav_buttons.items():
            button.setVisible(key not in hidden)
        for key, action in self._advanced_actions.items():
            action.setVisible(key not in hidden)
        advanced_visible = any(
            action.isVisible() for action in self._advanced_actions.values()
        )
        self._advanced_button.setVisible(advanced_visible)
        sidebar_settings = getattr(self, "_btn_sidebar_settings", None)
        if sidebar_settings is not None:
            sidebar_settings.setVisible("settings" not in hidden)
        self._tab_safety.set_interface_mode(profile)
        self._tab_vision.set_interface_mode(profile)
        self._tab_robot3d.set_interface_mode(
            profile,
            production=self._authorization.production_mode,
        )

        # If the current page is now gated, fall back to home.
        current_key = next(
            (k for k, idx in self._page_indexes.items() if idx == self._stack.currentIndex()),
            "home",
        )
        if current_key in hidden:
            self.navigate("home")

        if persist:
            try:
                save_app_settings(self._settings)
            except OSError:
                pass
            label = PROFILE_LABELS[profile]
            self.statusBar().showMessage(f"Perfil ativo: {label}", 4000)
            self.show_toast(f"Perfil aplicado: {label}", kind="success")

    def _load_generated_program(self, program) -> None:
        self._tab_motion.load_program(program, "Programa gerado pela aba Pegas.")
        self.navigate("motion")

    # ------------------------------------------------------------------
    # persistent commissioning guide
    # ------------------------------------------------------------------
    def _render_commissioning_checklist(self) -> None:
        checklist = self._settings.commissioning_checklist
        completed = 0
        for key in COMMISSIONING_CHECKLIST_KEYS:
            done = bool(checklist.get(key, False))
            completed += int(done)
            led = self._commissioning_leds.get(key)
            state = self._commissioning_states.get(key)
            if led is not None:
                led.set_color("green" if done else "amber")
                led.setAccessibleDescription(
                    "Concluído" if done else "Pendente"
                )
            if state is not None:
                state.setText("Concluído" if done else "Pendente")
                state.setProperty("status", "ok" if done else "warn")
                state.style().unpolish(state)
                state.style().polish(state)
        total = len(COMMISSIONING_CHECKLIST_KEYS)
        self._commissioning_summary.setText(
            f"{completed} de {total} etapas concluídas"
        )
        self._commissioning_summary.setProperty(
            "status", "ok" if completed == total else "warn"
        )
        self._commissioning_summary.style().unpolish(
            self._commissioning_summary
        )
        self._commissioning_summary.style().polish(
            self._commissioning_summary
        )

    def _set_commissioning_step(
        self,
        key: str,
        completed: bool,
        *,
        persist: bool = True,
    ) -> None:
        if key not in COMMISSIONING_CHECKLIST_KEYS:
            raise KeyError(key)
        completed = bool(completed)
        if self._settings.commissioning_checklist.get(key) == completed:
            return
        self._settings.commissioning_checklist[key] = completed
        self._render_commissioning_checklist()
        if not persist:
            return
        try:
            save_app_settings(self._settings)
        except OSError as exc:
            self.statusBar().showMessage(
                f"Falha ao salvar checklist: {exc}", 5000
            )

    def _sync_commissioning_prerequisites(self) -> None:
        """Refresh authoritative prerequisites without discarding other progress."""
        calibration_ok, _reason, _reference = (
            self._tab_vision.production_calibration_status()
        )
        self._set_commissioning_step(
            "calibration_approved", calibration_ok, persist=False
        )
        # MoveIt's acknowledgement is session-scoped. A new positive/negative
        # signal will update it; otherwise retain the last persisted completion.
        if self._cell_scene_store.is_applied:
            self._set_commissioning_step(
                "cell_scene_applied", True, persist=False
            )

    def _reset_commissioning_checklist(self) -> None:
        self._settings.commissioning_checklist = {
            key: False for key in COMMISSIONING_CHECKLIST_KEYS
        }
        self._render_commissioning_checklist()
        # Authoritative prerequisites already valid in this session remain
        # completed; the operator still needs to validate vision and program.
        calibration_ok, _reason, _reference = (
            self._tab_vision.production_calibration_status()
        )
        self._settings.commissioning_checklist["calibration_approved"] = (
            calibration_ok
        )
        self._settings.commissioning_checklist["cell_scene_applied"] = bool(
            self._cell_scene_store.is_applied
        )
        self._render_commissioning_checklist()
        try:
            save_app_settings(self._settings)
        except OSError as exc:
            self.statusBar().showMessage(
                f"Falha ao reiniciar checklist: {exc}", 5000
            )

    def _on_continue_to_grasp(self) -> None:
        self._set_commissioning_step("vision_validated", True)
        self.navigate("grasp")

    def _on_cell_scene_applied(self, applied: bool) -> None:
        self._set_commissioning_step("cell_scene_applied", applied)

    # ------------------------------------------------------------------
    # Bridge signal handlers
    # ------------------------------------------------------------------
    def _on_connected(self, ok: bool) -> None:
        self._led.set_color("amber")
        self._lbl_ros_status.setText(
            "ROS conectado — aguardando braço" if ok else "Aguardando ROS"
        )
        self._header_ros.setProperty("connected", ok)
        self._header_ros.style().unpolish(self._header_ros)
        self._header_ros.style().polish(self._header_ros)
        self._refresh_command_availability()
        self._append_log("Conectado ao ROS 2." if ok else "Aguardando ROS 2...")

    def _on_arm_alive(self, alive: bool) -> None:
        """Drive the status LED from the arm_status heartbeat, not rclpy.init."""
        self._arm_alive = alive
        self._led.set_color("green" if alive else "amber")
        self._lbl_ros_status.setText(
            "Braço ativo" if alive else "Braço sem sinal (arm_status parou)"
        )
        announce(
            self._lbl_ros_status,
            self._lbl_ros_status.text(),
            assertive=not alive,
        )
        self._append_log(
            "Controlador respondendo (arm_status)." if alive
            else "Controlador não publica arm_status — verifique o hardware."
        )
        if not alive:
            self._header_arm.setText("BRAÇO SEM SINAL")
            self._header_arm.setObjectName("StatusChipWarning")
            self._header_arm.style().unpolish(self._header_arm)
            self._header_arm.style().polish(self._header_arm)
            self._header_mode.setText("MODO --")
            self._lbl_footer_arm.setText("Braço: sem sinal")
            self._dash_arm.setText("SEM SINAL")
            self._dash_mode.setText("--")
            self._dash_last_seen.setText(
                f"último sinal {self._arm_last_seen}" if self._arm_last_seen != "--" else "--"
            )
            self._set_card_status(self._card_arm, "error")
            self._tab_motion.handle_robot_unavailable(
                "Execução interrompida: heartbeat de arm_status perdido."
            )
            self.emit_alarm(
                "Controlador sem heartbeat. Comandos de movimento foram bloqueados e "
                "qualquer execução ativa recebeu solicitação de cancelamento.",
                severity="warning",
                origin="Heartbeat arm_status",
            )
        self._refresh_command_availability()

    def _on_joint_state(self, msg) -> None:
        self._tab_joints.update_joint_state(msg)
        self._tab_telemetry.update_joint_state(msg)
        self._tab_mit.update_joint_state(msg)
        self._tab_pose.update_joint_state(msg)
        self._tab_safety.update_joint_state(msg)
        self._tab_robot3d.update_joint_state(msg)
        if not self._joint_limits_applied:
            limits = self._bridge.joint_limits()
            if limits:
                self._tab_joints.apply_joint_limits()
                self._joint_limits_applied = True

    def _on_arm_status(self, msg) -> None:
        self._tab_system.update_arm_status(msg)
        self._tab_safety.update_arm_status(msg)
        enabled = bool(getattr(msg, "enabled", False))
        mode = str(getattr(msg, "mode", "--"))
        state = "ATIVO" if enabled else "DESABILITADO"
        self._header_arm.setText(f"BRAÇO {state}")
        self._header_arm.setObjectName("StatusChipOk" if enabled else "StatusChipWarning")
        self._header_arm.style().unpolish(self._header_arm)
        self._header_arm.style().polish(self._header_arm)
        self._header_mode.setText(f"MODO {mode.upper()}")
        self._lbl_footer_arm.setText(f"Braço: {state.lower()} · modo {mode}")
        self._dash_arm.setText(state)
        self._dash_mode.setText(mode.upper())
        self._arm_last_seen = datetime.now().strftime("%H:%M:%S")
        self._dash_last_seen.setText(self._arm_last_seen)
        self._set_card_status(self._card_arm, "ok" if enabled else "warn")
        running = bool(getattr(self._tab_motion, "_running", False))
        program = getattr(self._tab_motion, "_program", None)
        n_steps = len(program.steps) if program is not None else 0
        program_name = getattr(program, "name", "") if program is not None else ""
        self._dash_program.setText(
            "EM EXECUÇÃO" if running else (f"{n_steps} etapas" if n_steps else "vazio")
        )
        self._card_program.setToolTip(program_name or "Sem programa carregado")
        self._refresh_command_availability()

    def _refresh_command_availability(self) -> None:
        """Apply one freshness/enable policy to every command surface."""
        node = self._bridge.node
        status = node.arm_status if node is not None else None
        enabled = bool(getattr(status, "enabled", False)) if self._arm_alive else False
        ready = bool(self._arm_alive and enabled)
        self._btn_enable.setEnabled(bool(self._arm_alive and not enabled))
        self._btn_disable.setEnabled(ready)
        self._btn_home.setEnabled(ready)
        self._tab_system.set_robot_state(self._arm_alive, enabled)
        self._tab_joints.set_robot_ready(ready)
        self._tab_pose.set_robot_ready(ready)
        self._tab_gripper.set_robot_ready(ready)
        self._tab_mit.set_robot_ready(ready)
        self._tab_motion.set_robot_ready(ready)

    def _on_gripper_state(self, msg) -> None:
        self._tab_gripper.update_gripper_state(msg)

    def _on_vision_objects(self, msg) -> None:
        objects = getattr(msg, "objects", None) or []
        self._dash_vision.setText(str(len(objects)))

    def _on_joint_detail(self, joint_name: str, msg) -> None:
        self._tab_telemetry.update_joint_detail(joint_name, msg)

    def _on_service_result(self, name: str, ok: bool, message: str) -> None:
        tag = "OK" if ok else "ERRO"
        text = f"[svc:{name}] {tag}: {message}"
        operation = ASYNC_SERVICE_OPERATIONS.get(name)
        handled = bool(
            operation
            and self._finish_async_operation(operation, ok, message)
        )
        if not handled:
            self._append_log(text)
            self.statusBar().showMessage(text, 5000)
        if not ok:
            self._record_failure(name, message)
            self.emit_alarm(
                f"Falha no serviço {name}: {message}",
                severity="error",
                origin=f"Serviço {name}",
            )

    def _on_action_result(self, name: str, ok: bool, message: str) -> None:
        tag = "OK" if ok else "ERRO"
        text = f"[action:{name}] {tag}: {message}"
        self._append_log(text)
        self.statusBar().showMessage(text, 5000)
        if not ok:
            self._record_failure(name, message)
            self.emit_alarm(
                f"Falha na ação {name}: {message}",
                severity="error",
                origin=f"Action {name}",
            )

    def _record_failure(self, name: str, message: str) -> None:
        self._tab_diagnostics.record_error(str(name), str(message))
        summary = f"{name}: {message}"
        if len(summary) > 60:
            summary = summary[:57] + "…"
        self._dash_last_error.setText(summary)
        self._dash_last_error.setToolTip(f"{name}: {message}")
        self._set_card_status(self._card_error, "error")

    def _on_log_message(self, level: str, text: str) -> None:
        self._append_log(f"[{level}] {text}")
        if str(level).lower() == "error":
            self._tab_diagnostics.record_error("Bridge ROS", text)

    def _on_safety_alarm(self, severity: str, message: str) -> None:
        banner_severity = "error" if str(severity).upper() == "FAIL" else "warning"
        self.emit_alarm(
            f"Alarme de segurança {severity}: {message}",
            severity=banner_severity,
            origin="Segurança",
        )

    def _append_log(self, text: str) -> None:
        self._event_log_panel.append(text)

    @staticmethod
    def _stamp_icon(button: QPushButton, name: str) -> None:
        """Attach an SVG icon to ``button`` only when the asset is available."""
        icon = load_icon(name)
        if not icon.isNull():
            button.setIcon(icon)
            button.setIconSize(QSize(16, 16))

    # ------------------------------------------------------------------
    # Session persistence (window geometry, dock layout, last page)
    # ------------------------------------------------------------------
    def _session_settings(self):
        from PySide6.QtCore import QSettings

        return QSettings("reBotArm", "rebotarm_gui")

    def _current_page_key(self) -> str:
        return next(
            (k for k, idx in self._page_indexes.items() if idx == self._stack.currentIndex()),
            "home",
        )

    def apply_saved_session(self) -> bool:
        """Restore geometry, dock layout and last page. Returns False if none.

        Called from the real entry point only; tests skip it so they always
        start deterministically on Início at the default geometry.
        """
        if not self._restore_session_enabled:
            return False
        s = self._session_settings()
        geom = s.value("session/geometry")
        restored = False
        if geom is not None:
            try:
                restored = self.restoreGeometry(QByteArray(geom))
            except Exception:
                restored = False
        state = s.value("session/windowState")
        if state is not None:
            try:
                self.restoreState(QByteArray(state))
            except Exception:
                pass
        last_page = str(s.value("session/lastPage", "") or "")
        profile = self._settings.profile
        if last_page in self._page_indexes and last_page not in PROFILE_HIDDEN_PAGES.get(
            profile, set()
        ):
            self.navigate(last_page)
        else:
            self.navigate("home")
        return restored

    def _save_session(self) -> None:
        s = self._session_settings()
        try:
            s.setValue("session/geometry", bytes(self.saveGeometry()))
            s.setValue("session/windowState", bytes(self.saveState()))
            s.setValue("session/lastPage", self._current_page_key())
        except Exception:
            pass

    def resizeEvent(self, event) -> None:  # noqa: N802 (Qt naming)
        super().resizeEvent(event)
        self._reflow_top_bar()
        toast = getattr(self, "_toast", None)
        if toast is not None:
            toast._relayout()

    def closeEvent(self, event) -> None:  # noqa: N802 (Qt naming)
        if not self._tab_motion.confirm_close():
            event.ignore()
            return
        self._save_session()
        if self._speed_save_timer.isActive():
            self._speed_save_timer.stop()
            self._persist_speed_override()
        try:
            if self._bridge_started:
                self._bridge.shutdown()
                self._bridge_started = False
        finally:
            super().closeEvent(event)


def load_stylesheet(
    theme: str = "dark",
    font_scale_percent: int = 100,
) -> str:
    """Return the composed stylesheet for ``theme`` (``dark`` or ``light``).

    The tuned base QSS is augmented with a generated semantic-status fragment
    so status/severity colours follow the active theme from a single source of
    truth (see :mod:`rebotarm_gui.styles.theme`).
    """
    from .styles.theme import build_stylesheet

    return build_stylesheet(theme, font_scale_percent)


def _apply_fonts(
    app: QApplication,
    font_scale_percent: int = 100,
) -> None:
    """Load the Inter font when available and use it as the application default.

    The font is optional: if it is not present (the common case on stock
    installs) the existing stylesheet fallback chain (Segoe UI / Roboto / Noto
    Sans) still applies. This keeps typography consistent on machines that do
    ship Inter without breaking the others.
    """
    candidates = [
        Path(__file__).resolve().parent / "styles" / "fonts" / "Inter.ttf",
        Path.home() / ".local" / "share" / "fonts" / "Inter.ttf",
    ]
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("rebotarm_gui")) / "styles" / "fonts" / "Inter.ttf"
        )
    except Exception:
        pass
    family = app.font().family()
    for candidate in candidates:
        if candidate.is_file():
            font_id = QFontDatabase.addApplicationFont(str(candidate))
            if font_id != -1:
                families = QFontDatabase.applicationFontFamilies(font_id)
                if families:
                    family = families[0]
                    break
    point_size = 10.0 * clamp_font_scale(font_scale_percent) / 100.0
    font = QFont(family)
    font.setPointSizeF(point_size)
    app.setFont(font)


def _locale_candidates(language: str) -> list[Path]:
    """Resolve ``locale/<language>.json`` from the package and the install share."""
    here = Path(__file__).resolve().parent
    candidates = [here / "locale" / f"{language}.json"]
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("rebotarm_gui")) / "locale" / f"{language}.json"
        )
    except ImportError:
        pass
    return candidates


def _apply_translations(language: str) -> None:
    """Load the translation table for ``language`` if one is shipped.

    Portuguese is the in-code default, so ``pt`` (or an unknown language) loads
    nothing and ``tr()`` keeps returning the Portuguese fallback — existing
    tests that assert on Portuguese strings stay green. A non-default language
    (e.g. ``en``) loads its JSON table once at startup.
    """
    from .i18n import load_translations

    if not language or language == "pt":
        return
    for candidate in _locale_candidates(language):
        if candidate.is_file():
            load_translations(candidate)
            return


def run(namespace: str = "rebotarm", *, usability_test: bool = False) -> int:
    """Application entry point."""
    # Rounded fractional scale factors keep text crisp on mixed-DPI setups.
    try:
        QApplication.setHighDpiScaleFactorRoundingPolicy(
            QApplication.HighDpiScaleFactorRoundingPolicy.PassThrough
        )
    except Exception:
        pass
    app = QApplication.instance() or QApplication([])
    app.setApplicationName("rebotarm_gui")
    settings = load_app_settings()
    _apply_fonts(app, settings.font_scale_percent)
    _apply_translations(settings.language)
    style = load_stylesheet(settings.theme, settings.font_scale_percent)
    if style:
        app.setStyleSheet(style)
        app.setProperty("rebotarm_theme", settings.theme)
        app.setProperty(
            "rebotarm_font_scale",
            clamp_font_scale(settings.font_scale_percent),
        )
    win = MainWindow(namespace=namespace, usability_test=usability_test, restore_session=True)
    if not win.apply_saved_session():
        win.showMaximized()
    else:
        win.show()
    return app.exec()
