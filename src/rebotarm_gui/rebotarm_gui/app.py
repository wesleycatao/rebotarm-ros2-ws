"""Main application window for the reBotArm GUI."""

from __future__ import annotations

from collections import deque
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QAction, QFont, QFontDatabase, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QComboBox,
    QDockWidget,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMenu,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QStackedWidget,
    QStatusBar,
    QTextEdit,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from .accessibility import announce, configure_accessibility, first_focusable, focus_first
from .i18n import tr
from .ros_bridge import RosBridge
from .settings import VALID_PROFILES, load_app_settings, save_app_settings
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
from .widgets import EventLogPanel, OperatorTestPanel, make_button
from .widgets.alarm import Alarm, AlarmBanner, AlarmCenterPanel
from .widgets.status_led import StatusLed


PAGE_META = {
    "home": ("Início", "Acesso rápido ao fluxo diário e ao controle manual."),
    "vision": ("Visão", "Detecte objetos e valide a área de trabalho."),
    "grasp": ("Pegas", "Revise estratégias e gere o programa de coleta."),
    "motion": ("Programa", "Edite, simule e execute sequências de movimento."),
    "robot3d": ("Robô 3D", "Visualização em tempo real da postura do braço."),
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

# Bound the active-alarm queue and coalesce identical alarms within this window
# so a repeating failure (e.g. an arm fault rejecting each dispatched move)
# cannot flood the banner or rebuild the history table unbounded.
ALERT_QUEUE_MAX = 50
ALERT_DEDUP_WINDOW_S = 5.0


class MainWindow(QMainWindow):
    """Top-level window with task navigation and persistent robot status."""

    def __init__(
        self,
        namespace: str = "rebotarm",
        *,
        start_bridge: bool = True,
        usability_test: bool = False,
    ) -> None:
        super().__init__()
        self.setWindowTitle("reBotArm Controller")
        self.setMinimumSize(1360, 720)
        self.resize(1440, 860)

        self._bridge = RosBridge(namespace=namespace)
        self._settings = load_app_settings()
        self._shortcuts: list[QShortcut] = []
        self._nav_buttons: dict[str, QPushButton] = {}
        self._page_indexes: dict[str, int] = {}
        self._advanced_actions: dict[str, QAction] = {}
        self._bridge_started = False
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

        self._build_pages()
        self._build_shell(namespace)
        self._build_event_dock()
        self._build_alarm_center_dock()
        self._operator_test_dock: QDockWidget | None = None
        self._operator_test_panel: OperatorTestPanel | None = None
        if usability_test:
            self._build_operator_test_dock()
        self._wire_signals()
        self._install_shortcuts()
        self._configure_accessibility()
        self._apply_profile(self._settings.profile, persist=False)
        self._refresh_command_availability()
        self.navigate("home")

        if start_bridge:
            self._bridge.start()
            self._bridge_started = True
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
        )
        self._tab_robot3d = Robot3dTab(self._bridge)
        self._tab_gripper = GripperTab(self._bridge)
        self._tab_grasp = GraspTab(self._bridge)
        self._tab_safety = SafetyTab(self._bridge)
        self._tab_telemetry = TelemetryTab(self._bridge)
        self._tab_diagnostics = DiagnosticsTab(self._bridge)
        self._tab_vision = VisionTab(
            self._bridge,
            ros_log=self._log,
            settings=self._settings,
            show_ros_log=False,
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
        sidebar.setFixedWidth(214)
        self._sidebar = sidebar
        side_lay = QVBoxLayout(sidebar)
        side_lay.setContentsMargins(12, 16, 12, 14)
        side_lay.setSpacing(4)

        brand = QLabel("reBotArm")
        brand.setObjectName("BrandLabel")
        side_lay.addWidget(brand)
        product = QLabel("CONTROLE ROBÓTICO")
        product.setObjectName("SidebarCaption")
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
        side_lay.addSpacing(8)

        # This selector adapts the interface; it is deliberately not presented
        # as authentication or as a safety/security boundary.
        self._add_section_label(side_lay, "MODO DA INTERFACE")
        self._profile_combo = QComboBox()
        for profile in VALID_PROFILES:
            self._profile_combo.addItem(PROFILE_LABELS[profile], profile)
        current = self._settings.profile if self._settings.profile in VALID_PROFILES else "supervisor"
        self._profile_combo.setCurrentIndex(VALID_PROFILES.index(current))
        self._profile_combo.setToolTip(
            "Adapta os controles exibidos. Não autentica o usuário: "
            "Operador mostra o fluxo diário; Supervisor adiciona ferramentas; "
            "Engenharia adiciona comandos de baixo nível."
        )
        self._profile_combo.setAccessibleName("Modo da interface")
        self._profile_combo.currentIndexChanged.connect(self._on_profile_changed)
        side_lay.addWidget(self._profile_combo)
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
        title_col.setSpacing(0)
        self._page_title = QLabel("Início")
        self._page_title.setObjectName("PageTitle")
        self._page_subtitle = QLabel("")
        self._page_subtitle.setObjectName("MutedLabel")
        title_col.addWidget(self._page_title)
        title_col.addWidget(self._page_subtitle)
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

    def _add_nav_button(
        self, layout: QVBoxLayout, key: str, text: str, shortcut: str
    ) -> None:
        button = QPushButton(text)
        button.setObjectName("NavButton")
        button.setCheckable(True)
        button.setCursor(Qt.PointingHandCursor)
        button.setToolTip(f"{text} ({shortcut})")
        button.clicked.connect(lambda _checked=False, page=key: self.navigate(page))
        self._nav_group.addButton(button)
        self._nav_buttons[key] = button
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
        self._alert_queue.clear()
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
        self._tab_vision.continue_to_grasp.connect(lambda: self.navigate("grasp"))
        self._tab_grasp.motion_program_generated.connect(
            lambda program: self._load_generated_program(program)
        )
        self._tab_settings._btn_save.clicked.connect(
            self._sync_operator_from_settings
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
            "Ctrl+L: Eventos ROS\n"
            "Ctrl+Shift+A: Alarmes\n"
            "Ctrl+,: Configurações\n"
            "F1: esta ajuda\n\n"
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
                last.count += 1
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
            return
        head = self._alert_queue[0]
        pending = len(self._alert_queue) - 1
        self._alert_banner.render_alarm(head, pending)

    def _dismiss_alert(self) -> None:
        if self._alert_queue:
            head = self._alert_queue.popleft()
            head.acknowledged = True
        self._render_alert_head()

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

    # ------------------------------------------------------------------
    # user profiles
    # ------------------------------------------------------------------
    def _on_profile_changed(self, index: int) -> None:
        profile = self._profile_combo.itemData(index)
        self._apply_profile(profile, persist=True)

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
        hidden = PROFILE_HIDDEN_PAGES.get(profile, set())

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
            self.statusBar().showMessage(
                f"Perfil ativo: {PROFILE_LABELS[profile]}", 4000
            )

    def _load_generated_program(self, program) -> None:
        self._tab_motion.load_program(program, "Programa gerado pela aba Pegas.")
        self.navigate("motion")

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

    def resizeEvent(self, event) -> None:  # noqa: N802 (Qt naming)
        super().resizeEvent(event)
        self._reflow_top_bar()

    def closeEvent(self, event) -> None:  # noqa: N802 (Qt naming)
        if not self._tab_motion.confirm_close():
            event.ignore()
            return
        try:
            if self._bridge_started:
                self._bridge.shutdown()
                self._bridge_started = False
        finally:
            super().closeEvent(event)


def load_stylesheet() -> str:
    """Load the dark QSS installed alongside the package."""
    here = Path(__file__).resolve().parent
    candidates = [here / "styles" / "dark.qss"]
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("rebotarm_gui"))
        candidates.append(share / "styles" / "dark.qss")
    except ImportError:
        pass
    for qss in candidates:
        if qss.is_file():
            return qss.read_text(encoding="utf-8")
    return ""


def _apply_fonts(app: QApplication) -> None:
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
    for candidate in candidates:
        if candidate.is_file():
            font_id = QFontDatabase.addApplicationFont(str(candidate))
            if font_id != -1:
                families = QFontDatabase.applicationFontFamilies(font_id)
                if families:
                    app.setFont(QFont(families[0], 10))
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
    _apply_fonts(app)
    style = load_stylesheet()
    if style:
        app.setStyleSheet(style)
    win = MainWindow(namespace=namespace, usability_test=usability_test)
    win.showMaximized()
    return app.exec()
