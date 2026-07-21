"""General GUI settings tab."""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
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

from ..settings import AppSettings, default_settings_path, save_app_settings
from ..widgets.layout_utils import fit_button_text, shrink_line_edit


class SettingsTab(QWidget):
    """Application-wide settings."""

    def __init__(self, settings: AppSettings, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._settings = settings

        self._operator_id = QLineEdit(settings.operator_id)
        self._operator_id.setPlaceholderText("Identificação usada no registro de auditoria")
        self._operator_id.setAccessibleName("Identificação do operador")
        shrink_line_edit(self._operator_id, max_width=420)

        self._api_key = QLineEdit()
        self._api_key.setEchoMode(QLineEdit.EchoMode.Password)
        self._api_key.setPlaceholderText("OPENAI_API_KEY ou chave salva localmente")
        self._api_key.setText(settings.openai.api_key)
        shrink_line_edit(self._api_key, max_width=520)

        self._show_key = QCheckBox("Mostrar chave")
        self._show_key.toggled.connect(self._toggle_key_visibility)

        self._save_key = QCheckBox("Salvar API key neste computador")
        self._save_key.setChecked(bool(settings.openai.save_api_key))

        self._model = QLineEdit(settings.openai.model)
        self._model.setPlaceholderText("gpt-5-mini")
        shrink_line_edit(self._model, max_width=220)

        self._base_url = QLineEdit(settings.openai.base_url)
        self._base_url.setPlaceholderText("https://api.openai.com/v1")
        shrink_line_edit(self._base_url, max_width=420)

        self._timeout = QSpinBox()
        self._timeout.setRange(5, 180)
        self._timeout.setSuffix(" s")
        self._timeout.setValue(int(settings.openai.timeout_s))
        self._timeout.setAccessibleName("Tempo limite da API em segundos")

        self._status = QLabel(f"Config: {default_settings_path()}")
        self._status.setObjectName("MutedLabel")
        self._status.setWordWrap(True)

        self._btn_save = QPushButton("Salvar configurações gerais")
        self._btn_save.setObjectName("PrimaryButton")
        self._btn_save.clicked.connect(self._save)
        fit_button_text(self._btn_save)

        api_box = QGroupBox("OpenAI API")
        grid = QGridLayout(api_box)
        grid.addWidget(self._label("API key"), 0, 0)
        grid.addWidget(self._api_key, 0, 1, 1, 3)
        grid.addWidget(self._show_key, 1, 1)
        grid.addWidget(self._save_key, 1, 2, 1, 2)
        grid.addWidget(self._label("Modelo"), 2, 0)
        grid.addWidget(self._model, 2, 1)
        grid.addWidget(self._label("Tempo limite"), 2, 2)
        grid.addWidget(self._timeout, 2, 3)
        grid.addWidget(self._label("Base URL"), 3, 0)
        grid.addWidget(self._base_url, 3, 1, 1, 3)
        note = QLabel(
            "A aba Visão usa estes dados apenas quando o botão Auto-tune IA é pressionado. "
            "O processamento normal continua local por OpenCV."
        )
        note.setObjectName("MutedLabel")
        note.setWordWrap(True)
        grid.addWidget(note, 4, 0, 1, 4)

        audit_box = QGroupBox("Operação e auditoria")
        audit_grid = QGridLayout(audit_box)
        audit_grid.addWidget(self._label("Operador"), 0, 0)
        audit_grid.addWidget(self._operator_id, 0, 1)
        audit_note = QLabel(
            "Esta identificação acompanha execuções, etapas, cancelamentos e falhas "
            "no registro persistente. Ajuste-a ao trocar de operador."
        )
        audit_note.setObjectName("MutedLabel")
        audit_note.setWordWrap(True)
        audit_grid.addWidget(audit_note, 1, 0, 1, 2)

        buttons = QHBoxLayout()
        buttons.addWidget(self._btn_save)
        buttons.addStretch(1)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(14)
        lay.addWidget(audit_box)
        lay.addWidget(api_box)
        lay.addLayout(buttons)
        lay.addWidget(self._status)
        lay.addStretch(1)

    def _label(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("FieldLabel")
        label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        return label

    def _toggle_key_visibility(self, checked: bool) -> None:
        mode = QLineEdit.EchoMode.Normal if checked else QLineEdit.EchoMode.Password
        self._api_key.setEchoMode(mode)

    def _save(self) -> None:
        self._settings.operator_id = self._operator_id.text().strip()
        self._settings.openai.api_key = self._api_key.text().strip()
        self._settings.openai.save_api_key = bool(self._save_key.isChecked())
        self._settings.openai.model = self._model.text().strip() or "gpt-5-mini"
        self._settings.openai.base_url = self._base_url.text().strip() or "https://api.openai.com/v1"
        self._settings.openai.timeout_s = int(self._timeout.value())
        path = save_app_settings(self._settings)
        if self._settings.openai.save_api_key:
            self._status.setText(f"Configurações salvas em {path}")
        else:
            self._status.setText(f"Configurações salvas em {path}; API key não foi persistida.")
