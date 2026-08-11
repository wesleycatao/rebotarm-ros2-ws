"""General GUI settings tab."""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ..settings import (
    AppSettings,
    FONT_SCALE_MAX,
    FONT_SCALE_MIN,
    VALID_LANGUAGES,
    VALID_THEMES,
    clamp_font_scale,
    default_settings_path,
    save_app_settings,
)
from ..security import CredentialStore, ROLE_ENGINEERING, ROLE_SUPERVISOR
from ..widgets.layout_utils import fit_button_text, shrink_line_edit


class SettingsTab(QWidget):
    """Application-wide settings."""

    credential_saved = Signal()

    def __init__(self, settings: AppSettings, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._settings = settings
        self._credentials = CredentialStore()

        self._operator_id = QLineEdit(settings.operator_id)
        self._operator_id.setPlaceholderText("Identificação usada no registro de auditoria")
        self._operator_id.setAccessibleName("Identificação do operador")
        shrink_line_edit(self._operator_id, max_width=420)

        # Color theme selector: a light theme improves legibility on a factory
        # floor with high ambient light where the dark theme is hard to read.
        self._theme_combo = QComboBox()
        self._theme_combo.addItem("Escuro (padrão)", "dark")
        self._theme_combo.addItem("Claro (alta luminosidade)", "light")
        current_theme = settings.theme if settings.theme in VALID_THEMES else "dark"
        idx = self._theme_combo.findData(current_theme)
        if idx >= 0:
            self._theme_combo.setCurrentIndex(idx)
        self._theme_combo.setAccessibleName("Tema da interface")
        self._theme_combo.setToolTip(
            "Tema escuro para baixa luminosidade; tema claro para ambientes "
            "com muita luz ambiente (chão de fábrica). A escolha é persistida."
        )
        # Interface language. Translations load at startup, so a change takes
        # effect on the next launch (Portuguese remains the in-code default).
        self._language_combo = QComboBox()
        self._language_combo.addItem("Português (Brasil)", "pt")
        self._language_combo.addItem("English", "en")
        current_lang = settings.language if settings.language in VALID_LANGUAGES else "pt"
        lang_idx = self._language_combo.findData(current_lang)
        if lang_idx >= 0:
            self._language_combo.setCurrentIndex(lang_idx)
        self._language_combo.setAccessibleName("Idioma da interface")
        self._language_combo.setToolTip(
            "Idioma dos rótulos da interface. A mudança é aplicada na próxima "
            "abertura da GUI."
        )
        self._font_scale = QSpinBox()
        self._font_scale.setRange(FONT_SCALE_MIN, FONT_SCALE_MAX)
        self._font_scale.setSingleStep(10)
        self._font_scale.setSuffix("%")
        self._font_scale.setValue(clamp_font_scale(settings.font_scale_percent))
        self._font_scale.setAccessibleName("Escala das fontes da interface")
        self._font_scale.setToolTip(
            "Dimensiona proporcionalmente todos os textos da GUI. "
            "A alteração é aplicada ao salvar."
        )
        self._collision_planning = QCheckBox(
            "Exigir planejamento MoveIt com verificação de colisão"
        )
        self._collision_planning.setChecked(
            bool(settings.collision_planning_required)
        )

        self._status = QLabel(f"Config: {default_settings_path()}")
        self._status.setObjectName("MutedLabel")
        self._status.setWordWrap(True)

        self._btn_save = QPushButton("Salvar configurações gerais")
        self._btn_save.setObjectName("PrimaryButton")
        self._btn_save.clicked.connect(self._save)
        fit_button_text(self._btn_save)

        self._auth_user = QLineEdit()
        self._auth_user.setPlaceholderText("Nome do usuário")
        self._auth_role = QComboBox()
        self._auth_role.addItem("Supervisor", ROLE_SUPERVISOR)
        self._auth_role.addItem("Engenharia", ROLE_ENGINEERING)
        self._auth_pin = QLineEdit()
        self._auth_pin.setEchoMode(QLineEdit.EchoMode.Password)
        self._auth_pin.setPlaceholderText("PIN com no mínimo 6 caracteres")
        self._btn_auth_save = QPushButton("Cadastrar/atualizar credencial")
        self._btn_auth_save.clicked.connect(self._save_credential)

        security_box = QGroupBox("Segurança e permissões")
        security_grid = QGridLayout(security_box)
        security_grid.addWidget(self._label("Usuário"), 0, 0)
        security_grid.addWidget(self._auth_user, 0, 1)
        security_grid.addWidget(self._label("Perfil"), 1, 0)
        security_grid.addWidget(self._auth_role, 1, 1)
        security_grid.addWidget(self._label("PIN"), 2, 0)
        security_grid.addWidget(self._auth_pin, 2, 1)
        security_grid.addWidget(self._btn_auth_save, 3, 1)
        security_note = QLabel(
            "O PIN é armazenado como PBKDF2, nunca em texto puro. Ele é exigido "
            "para ativar o Modo Produção e elevar permissões."
        )
        security_note.setObjectName("MutedLabel")
        security_note.setWordWrap(True)
        security_grid.addWidget(security_note, 4, 0, 1, 2)

        audit_box = QGroupBox("Operação e auditoria")
        audit_grid = QGridLayout(audit_box)
        audit_grid.addWidget(self._label("Operador"), 0, 0)
        audit_grid.addWidget(self._operator_id, 0, 1)
        audit_grid.addWidget(self._label("Tema"), 1, 0)
        audit_grid.addWidget(self._theme_combo, 1, 1)
        audit_grid.addWidget(self._label("Idioma"), 2, 0)
        audit_grid.addWidget(self._language_combo, 2, 1)
        audit_grid.addWidget(self._label("Escala das fontes"), 3, 0)
        audit_grid.addWidget(self._font_scale, 3, 1)
        audit_grid.addWidget(self._collision_planning, 4, 1)
        audit_note = QLabel(
            "Esta identificação acompanha execuções, etapas, cancelamentos e falhas "
            "no registro persistente. A escala entre 90% e 130% atende desde "
            "painéis compactos até monitores 4K."
        )
        audit_note.setObjectName("MutedLabel")
        audit_note.setWordWrap(True)
        audit_grid.addWidget(audit_note, 5, 0, 1, 2)

        buttons = QHBoxLayout()
        buttons.addWidget(self._btn_save)
        buttons.addStretch(1)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(14)
        lay.addWidget(audit_box)
        lay.addWidget(security_box)
        lay.addLayout(buttons)
        lay.addWidget(self._status)
        lay.addStretch(1)

    def _label(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("FieldLabel")
        label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        return label

    def _save_credential(self) -> None:
        try:
            self._credentials.set_user(
                self._auth_user.text(),
                str(self._auth_role.currentData()),
                self._auth_pin.text(),
            )
        except (OSError, ValueError) as exc:
            self._status.setText(f"Credencial não salva: {exc}")
            return
        self._auth_pin.clear()
        self._status.setText(
            f"Credencial de {self._auth_user.text().strip()} salva em "
            f"{self._credentials.path}"
        )
        self.credential_saved.emit()

    def _save(self) -> None:
        self._settings.operator_id = self._operator_id.text().strip()
        chosen_theme = str(self._theme_combo.currentData())
        theme_changed = chosen_theme != self._settings.theme
        self._settings.theme = chosen_theme
        chosen_language = str(self._language_combo.currentData())
        language_changed = chosen_language != self._settings.language
        self._settings.language = chosen_language
        chosen_font_scale = clamp_font_scale(self._font_scale.value())
        font_scale_changed = (
            chosen_font_scale != self._settings.font_scale_percent
        )
        self._settings.font_scale_percent = chosen_font_scale
        self._settings.collision_planning_required = bool(
            self._collision_planning.isChecked()
        )
        path = save_app_settings(self._settings)
        if theme_changed:
            # Live-apply the new theme so the operator sees it immediately.
            parent = self.window()
            applier = getattr(parent, "apply_theme", None)
            if callable(applier):
                applier(chosen_theme)
        if font_scale_changed:
            parent = self.window()
            applier = getattr(parent, "apply_font_scale", None)
            if callable(applier):
                applier(chosen_font_scale)
        parent = self.window()
        motion_tab = getattr(parent, "_tab_motion", None)
        if motion_tab is not None:
            motion_tab.set_operating_mode(
                production=bool(self._settings.production_mode),
                collision_planning_required=bool(
                    self._settings.collision_planning_required
                ),
            )
        restart_hint = " Reinicie a GUI para aplicar o novo idioma." if language_changed else ""
        self._status.setText(f"Configurações salvas em {path}.{restart_hint}")
