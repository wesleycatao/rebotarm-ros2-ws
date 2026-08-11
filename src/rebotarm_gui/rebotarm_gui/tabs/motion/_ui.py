"""MotionTab ui mixin — extracted from motion_tab.py."""

# ruff: noqa: F401,F403,F405
from __future__ import annotations

from ._common import *  # noqa: F403
from ._common import (  # noqa: F401
    _config_dir,
    _dict_to_pose,
    _INVALID_GENERIC_STEP_IDS,
    _pose_msg_to_dict,
    _preload_path,
)


class MotionTabUiMixin:
    """Ui concern methods for MotionTab."""

    def _build_ui(self) -> None:
        self._btn_run = QPushButton("Executar")
        self._btn_run.setObjectName("PrimaryButton")
        self._btn_run.setMinimumHeight(40)
        self._btn_run.clicked.connect(self._run)

        self._btn_run_selected = QPushButton("Executar da etapa")
        self._btn_run_selected.setMinimumHeight(40)
        self._btn_run_selected.setToolTip(
            "Executa a etapa selecionada e todas as seguintes (Ctrl+Shift+Enter)."
        )
        self._btn_run_selected.clicked.connect(self._run_from_selected)

        self._btn_stop = QPushButton("Parar")
        self._btn_stop.setObjectName("DangerButton")
        self._btn_stop.setMinimumHeight(40)
        self._btn_stop.setEnabled(False)
        self._btn_stop.clicked.connect(self._stop)

        self._btn_pause = QPushButton("Pausar")
        self._btn_pause.setMinimumHeight(40)
        self._btn_pause.setEnabled(False)
        self._btn_pause.setToolTip(
            "Cancela o comando ativo e preserva o checkpoint para retomada planejada."
        )
        self._btn_pause.clicked.connect(self._pause)

        self._btn_resume = QPushButton("Retomar")
        self._btn_resume.setMinimumHeight(40)
        self._btn_resume.setEnabled(False)
        self._btn_resume.setToolTip(
            "Revalida o estado e replaneja a etapa interrompida a partir da posição atual."
        )
        if self._persisted_checkpoint is not None:
            self._btn_resume.setText("Recuperar")
        self._btn_resume.clicked.connect(self._resume)

        self._btn_save = QPushButton("Salvar...")
        self._btn_save.clicked.connect(self._save_dialog)

        # "Carregar" is a split tool button: the main part opens the file
        # dialog, the side arrow lists recently used programs (one click to
        # reopen). It looks and sizes like the neighbouring toolbar buttons.
        self._btn_load = QToolButton()
        self._btn_load.setObjectName("LoadButton")
        self._btn_load.setText("Carregar...")
        self._btn_load.setToolButtonStyle(Qt.ToolButtonTextOnly)
        self._btn_load.setPopupMode(QToolButton.MenuButtonPopup)
        self._load_menu = QMenu(self._btn_load)
        self._act_load_dialog = self._load_menu.addAction("Carregar...")
        self._act_load_dialog.triggered.connect(self._load_dialog)
        self._recent_menu = self._load_menu.addMenu("Recentes")
        self._btn_load.setMenu(self._load_menu)
        self._btn_load.clicked.connect(self._load_dialog)

        self._btn_preload = QPushButton("Definir como pré-carga")
        self._btn_preload.setToolTip(
            f"Salva o programa atual em {_preload_path()} para abrir automaticamente."
        )
        self._btn_preload.clicked.connect(self._save_preload)
        self._btn_new = QPushButton("Novo")
        self._btn_new.clicked.connect(self._new_program)
        self._btn_validate = QPushButton("Validar")
        self._btn_validate.setObjectName("PrimaryButton")
        self._btn_validate.setToolTip(
            "Executa a validação pré-execução (preflight): limites, alcance,\n"
            "velocidades, action servers, condições e gripper. Não move o braço."
        )
        self._btn_validate.clicked.connect(self._open_preflight_dialog)
        self._btn_preview = QPushButton("Pré-visualizar")
        self._btn_preview.setToolTip(
            "Mostra em 3D os waypoints cartesianos compilados, sem mover o braço."
        )
        self._btn_preview.clicked.connect(self._preview_trajectory)
        self._btn_release = QPushButton("Liberar receita")
        self._btn_release.setToolTip(
            "Congela a revisão atual com operador, horário e hash de integridade."
        )
        self._btn_release.clicked.connect(self._release_recipe)
        self._btn_new_revision = QPushButton("Nova revisão")
        self._btn_new_revision.setToolTip(
            "Cria uma revisão editável a partir da receita liberada."
        )
        self._btn_new_revision.clicked.connect(self._start_new_revision)
        for b in (self._btn_save, self._btn_load, self._btn_preload, self._btn_new,
                  self._btn_validate, self._btn_preview, self._btn_release,
                  self._btn_new_revision):
            b.setMinimumHeight(40)

        self._lbl_status = QLabel("Pronto.")
        self._lbl_status.setObjectName("MutedLabel")
        self._lbl_progress = QLabel("")
        self._lbl_progress.setObjectName("MetricValue")
        self._lbl_progress.setAlignment(Qt.AlignRight)

        self._chk_dry_run = QCheckBox("Simulação")
        self._chk_dry_run.setToolTip("Executa sem enviar comandos ao braço (dry-run).")
        self._chk_dry_run.toggled.connect(self._on_dry_run_toggled)

        self._chk_step = QCheckBox("Passo a passo")
        self._chk_step.setToolTip("Pausa antes de cada ação; clique em Avançar para continuar.")
        self._chk_step.toggled.connect(self._on_step_by_step_toggled)

        self._chk_record = QCheckBox("Gravar juntas")
        self._chk_record.setToolTip("Grava os ângulos das juntas durante a execução para plot posterior.")

        self._btn_plot_joints = QPushButton("Plotar juntas")
        self._btn_plot_joints.setToolTip("Exibe gráfico temporal dos ângulos gravados na última execução.")
        self._btn_plot_joints.setEnabled(False)
        self._btn_plot_joints.clicked.connect(self._plot_joint_history)

        self._btn_advance = QPushButton("Avançar")
        self._btn_advance.setMinimumWidth(96)
        self._btn_advance.setToolTip("Avança para a próxima ação no modo passo a passo.")
        self._btn_advance.setEnabled(False)
        self._btn_advance.clicked.connect(self._on_advance_clicked)

        self._btn_undo = QPushButton("Desfazer")
        self._btn_undo.setToolTip("Desfazer última alteração (Ctrl+Z).")
        self._btn_undo.clicked.connect(self._undo)
        self._btn_redo = QPushButton("Refazer")
        self._btn_redo.setToolTip("Refazer alteração desfeita (Ctrl+Y).")
        self._btn_redo.clicked.connect(self._redo)
        for b in (self._btn_undo, self._btn_redo):
            b.setMinimumHeight(40)

        # ---- teach controls ----
        self._btn_freedrive = QPushButton("Freedrive")
        self._btn_freedrive.setObjectName("DrawModeButton")
        self._btn_freedrive.setCheckable(True)
        self._btn_freedrive.setToolTip(
            "Libera o braço em compensação de gravidade para movê-lo com a mão\n"
            "enquanto ensina pontos (clique novamente para travar)."
        )
        self._btn_freedrive.setMinimumHeight(40)
        self._btn_freedrive.toggled.connect(self._on_freedrive_toggled)

        self._btn_teach = QPushButton("Ensinar ponto atual")
        self._btn_teach.setToolTip(
            "Captura a pose atual do TCP e adiciona um passo 'Ir ao ponto'\n"
            "ao final do programa."
        )
        self._btn_teach.setMinimumHeight(40)
        self._btn_teach.clicked.connect(self._teach_current_point)

        # ---- view toggle: list editor vs. flowchart ----
        self._btn_view_list = QPushButton("Lista")
        self._btn_view_list.setCheckable(True)
        self._btn_view_list.setChecked(True)
        self._btn_view_list.setToolTip("Editor de etapas em lista.")
        self._btn_view_flow = QPushButton("Fluxo")
        self._btn_view_flow.setCheckable(True)
        self._btn_view_flow.setToolTip(
            "Visualização e edição por diagrama de blocos: clique para selecionar, "
            "arraste para reordenar etapas, setas para navegar, +/- para zoom."
        )
        self._view_group = QButtonGroup(self)
        self._view_group.setExclusive(True)
        self._view_group.addButton(self._btn_view_list, 0)
        self._view_group.addButton(self._btn_view_flow, 1)
        self._view_group.idClicked.connect(self._on_view_toggled)

        # Low-frequency diagnostics and preload options live in an overflow
        # menu so the editor remains usable at the supported 1360 px width.
        self._more_menu = QMenu(self)
        self._act_preload = self._more_menu.addAction("Definir como pré-carga")
        self._act_preload.setToolTip(self._btn_preload.toolTip())
        self._act_preload.triggered.connect(self._save_preload)
        self._more_menu.addSeparator()
        self._act_undo = self._more_menu.addAction("Desfazer (Ctrl+Z)")
        self._act_undo.triggered.connect(self._undo)
        self._act_redo = self._more_menu.addAction("Refazer (Ctrl+Y)")
        self._act_redo.triggered.connect(self._redo)
        self._more_menu.addSeparator()
        self._act_record_joints = self._more_menu.addAction("Gravar juntas na execução")
        self._act_record_joints.setCheckable(True)
        self._act_record_joints.toggled.connect(self._chk_record.setChecked)
        self._act_plot_joints = self._more_menu.addAction("Plotar juntas gravadas")
        self._act_plot_joints.setEnabled(False)
        self._act_plot_joints.triggered.connect(self._plot_joint_history)
        self._more_menu.addSeparator()
        self._act_export_audit = self._more_menu.addAction("Exportar auditoria CSV...")
        self._act_export_audit.setToolTip(
            f"Exporta o registro JSONL persistente em {self._audit_log.path}."
        )
        self._act_export_audit.triggered.connect(self._export_audit_csv)
        self._btn_more_actions = QToolButton()
        self._btn_more_actions.setObjectName("AdvancedButton")
        self._btn_more_actions.setText("Mais ações")
        self._btn_more_actions.setPopupMode(QToolButton.InstantPopup)
        self._btn_more_actions.setMenu(self._more_menu)
        self._btn_more_actions.setToolTip(
            "Pré-carga, diagnóstico e exportação da auditoria de execução."
        )
        self._btn_more_actions.setAccessibleName("Mais ações do programa")

        for b in (
            self._btn_run,
            self._btn_run_selected,
            self._btn_stop,
            self._btn_pause,
            self._btn_resume,
            self._btn_advance,
            self._btn_save,
            self._btn_load,
            self._btn_preload,
            self._btn_new,
            self._btn_undo,
            self._btn_redo,
            self._btn_plot_joints,
            self._btn_freedrive,
            self._btn_teach,
            self._btn_preview,
        ):
            fit_button_text(b)

        # Optional SVG icons on the most-used action buttons. Missing assets
        # leave the text-only button untouched.
        for btn, icon_name in (
            (self._btn_run, "play"),
            (self._btn_stop, "stop"),
            (self._btn_save, "save"),
            (self._btn_load, "open"),
            (self._btn_undo, "undo"),
            (self._btn_redo, "redo"),
        ):
            icon = load_icon(icon_name)
            if not icon.isNull():
                btn.setIcon(icon)
                btn.setIconSize(QSize(16, 16))

        primary_toolbar = QHBoxLayout()
        primary_toolbar.setSpacing(8)
        primary_toolbar.addWidget(self._btn_run)
        primary_toolbar.addWidget(self._btn_run_selected)
        primary_toolbar.addWidget(self._btn_preview)
        primary_toolbar.addWidget(self._btn_stop)
        primary_toolbar.addWidget(self._btn_pause)
        primary_toolbar.addWidget(self._btn_resume)
        primary_toolbar.addWidget(self._btn_advance)
        primary_toolbar.addStretch(1)
        primary_toolbar.addWidget(self._chk_dry_run)
        primary_toolbar.addWidget(self._chk_step)

        secondary_toolbar = QHBoxLayout()
        secondary_toolbar.setSpacing(8)
        secondary_toolbar.addWidget(self._btn_save)
        secondary_toolbar.addWidget(self._btn_load)
        secondary_toolbar.addWidget(self._btn_new)
        secondary_toolbar.addWidget(self._btn_validate)
        secondary_toolbar.addWidget(self._btn_release)
        secondary_toolbar.addWidget(self._btn_new_revision)
        secondary_toolbar.addSpacing(12)
        # Undo/Redo are exposed as visible toolbar buttons (mirroring Ctrl+Z/Y)
        # so mouse/touchpad operators can discover history navigation.
        secondary_toolbar.addWidget(self._btn_undo)
        secondary_toolbar.addWidget(self._btn_redo)
        secondary_toolbar.addSpacing(12)
        secondary_toolbar.addWidget(self._btn_freedrive)
        secondary_toolbar.addWidget(self._btn_teach)
        secondary_toolbar.addStretch(1)
        secondary_toolbar.addWidget(self._btn_view_list)
        secondary_toolbar.addWidget(self._btn_view_flow)
        secondary_toolbar.addWidget(self._btn_more_actions)

        self._progress = QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setTextVisible(True)
        self._progress.setFormat("%p%")
        self._progress.setVisible(False)

        # Program-authored scale. The always-visible shell owns an additional
        # safety-oriented global override (5–100%); the two are multiplied.
        self._name_edit = QLineEdit(self._program.name)
        self._name_edit.setPlaceholderText("Nome do programa")
        self._name_edit.setMaximumWidth(260)
        self._name_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._name_edit.editingFinished.connect(self._on_name_changed)

        self._speed_spinbox = QDoubleSpinBox()
        self._speed_spinbox.setMaximumWidth(120)
        self._speed_spinbox.setRange(0.05, 4.0)
        self._speed_spinbox.setSingleStep(0.05)
        self._speed_spinbox.setValue(1.0)
        self._speed_spinbox.setSuffix("x")
        self._speed_spinbox.setDecimals(2)
        self._speed_spinbox.setFixedWidth(110)
        self._speed_spinbox.setToolTip(
            "Multiplicador salvo neste programa e aplicado a todos os movimentos.\n"
            ">1: mais rápido (a duração de cada segmento é limitada ao mínimo do planejador).\n"
            "<1: mais lento. O controle global do cabeçalho é aplicado em seguida."
        )
        self._speed_spinbox.valueChanged.connect(self._on_speed_scale_changed)
        self._effective_speed_label = QLabel("")
        self._effective_speed_label.setObjectName("MutedLabel")
        self._effective_speed_label.setToolTip(
            "Velocidade efetiva = escala salva no programa × controle global do cabeçalho."
        )

        name_row = QHBoxLayout()
        name_row.setSpacing(8)
        name_row.addWidget(QLabel("Programa:"))
        name_row.addWidget(self._name_edit)
        name_row.addSpacing(16)
        name_row.addWidget(QLabel("Escala do programa:"))
        name_row.addWidget(self._speed_spinbox)
        name_row.addWidget(self._effective_speed_label)
        self._recipe_status_label = QLabel("")
        self._recipe_status_label.setObjectName("MutedLabel")
        name_row.addWidget(self._recipe_status_label)
        name_row.addSpacing(16)
        name_row.addWidget(self._lbl_progress)
        name_row.addStretch(1)
        name_row.addWidget(self._lbl_status)

        # ---- step editor (top level) ----
        self._editor = StepListEditor(self._bridge)
        self._editor.set_joint_names_cb(self._bridge.joint_names)
        self._editor.set_input_ids_cb(self._bridge.dio_input_ids)
        self._editor.set_steps(self._program.steps)
        self._editor.changed.connect(self._on_program_changed)
        self._editor.step_changed.connect(self._on_step_edited)
        self._editor.about_to_change.connect(self._push_undo_snapshot)
        self._editor.selection_changed.connect(self._on_list_selection_changed)

        # flowchart view (editable), stacked with the list editor
        self._flow_widget = ProgramFlowWidget()
        self._flow_widget.step_selected.connect(self._editor.set_selected_index)
        self._flow_widget.reorder_requested.connect(self._on_flow_reorder)
        self._view_stack = QStackedWidget()
        self._view_stack.addWidget(self._editor)       # index 0: Lista
        self._view_stack.addWidget(self._flow_widget)  # index 1: Fluxo
        self._view_stack.setCurrentIndex(0)

        self._editor_box = QGroupBox(self._editor_group_title)
        eb = QVBoxLayout(self._editor_box)
        eb.setContentsMargins(10, 14, 10, 10)
        eb.addWidget(self._view_stack)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(18, 18, 18, 18)
        lay.setSpacing(12)
        lay.addLayout(primary_toolbar)
        lay.addLayout(secondary_toolbar)
        lay.addWidget(self._progress)
        lay.addLayout(name_row)
        lay.addWidget(self._editor_box, 1)

        self._refresh_after_load()
        # Toolbar undo/redo start disabled until the first snapshot exists.
        self._update_undo_buttons()

        # keyboard shortcuts
        QShortcut(QKeySequence.StandardKey.Undo, self, self._undo)
        QShortcut(QKeySequence("Ctrl+Y"), self, self._redo)
        QShortcut(QKeySequence("Ctrl+Shift+Return"), self, self._run_from_selected)


    def _on_list_selection_changed(self, row: int) -> None:
        """Synchronize list selection with the flowchart selection."""
        self._flow_widget.set_selected_index(row if row >= 0 else None)
        self._update_run_availability()


    def _on_flow_reorder(self, source: int, target: int) -> None:
        """Apply a drag-and-drop reorder request from the flowchart."""
        if not (0 <= source < len(self._program.steps) and 0 <= target < len(self._program.steps)):
            return
        if source == target:
            return
        self._push_undo_snapshot()
        step = self._program.steps.pop(source)
        self._program.steps.insert(target, step)
        self._editor.set_steps(self._program.steps)
        self._editor.set_selected_index(target)
        self._flow_widget.set_program(self._program)
        self._flow_widget.set_selected_index(target)
        self._on_program_changed()


    def _navigate_to_step(self, index: int) -> None:
        """Bring both editors to a given root step."""
        if not (0 <= index < len(self._program.steps)):
            return
        self._editor.set_selected_index(index)
        self._flow_widget.set_selected_index(index)
        self._view_stack.setCurrentIndex(0)


    def _on_view_toggled(self, view_id: int) -> None:
        self._view_stack.setCurrentIndex(view_id)
        if view_id == 1:
            # make sure the flowchart reflects the latest program state
            self._flow_widget.set_program(self._program)


    def _on_step_edited(self) -> None:
        """Continuous step parameter edits are debounced before creating an undo snapshot."""
        self._on_program_changed()
        self._undo_debounce_timer.start(1500)


    def _on_name_changed(self) -> None:
        self._ensure_editable_revision()
        self._invalidate_dynamic_analysis()
        self._push_undo_snapshot()
        self._program.name = self._name_edit.text().strip() or "programa"
        self._set_modified(True)


    def _on_speed_scale_changed(self, value: float) -> None:
        self._ensure_editable_revision()
        self._invalidate_dynamic_analysis()
        self._program.speed_scale = float(value)
        self._refresh_effective_speed_label()
        self._set_modified(True)
        self._undo_debounce_timer.start(1500)


    def _on_global_speed_changed(self, _value: float) -> None:
        self._refresh_effective_speed_label()


    def _on_dry_run_toggled(self, checked: bool) -> None:
        self._dry_run = bool(checked)
        if self._dry_run:
            self._lbl_status.setText("Modo simulação ativado.")
        self._update_run_availability()


    def _on_step_by_step_toggled(self, checked: bool) -> None:
        self._step_by_step = bool(checked)
        if self._step_by_step:
            self._lbl_status.setText("Modo passo a passo ativado.")


    def _refresh_after_load(self) -> None:
        self._runtime_dynamic_ceiling = 1.0
        self._name_edit.setText(self._program.name)
        self._speed_spinbox.blockSignals(True)
        self._speed_spinbox.setValue(self._program.speed_scale)
        self._speed_spinbox.blockSignals(False)
        self._refresh_effective_speed_label()
        self._editor.set_steps(self._program.steps)
        self._flow_widget.set_program(self._program)
        self._update_flow_invalid_indices()
        self._update_progress_meta()
        self._refresh_recipe_state()
        self._set_modified(False)


    def _ensure_editable_revision(self) -> None:
        if self._program.recipe_status == RECIPE_RELEASED:
            self._program.start_new_revision()
            self._refresh_recipe_state()


    def _refresh_recipe_state(self) -> None:
        released, _reason = self._program.verify_release()
        state = "LIBERADA" if released else "RASCUNHO"
        self._recipe_status_label.setText(
            f"Receita r{self._program.revision} · {state}"
        )
        locked = bool(self._production_mode)
        editable = not locked and not self._running
        self._editor.set_edit_enabled(editable)
        self._flow_widget.set_edit_enabled(editable)
        self._name_edit.setReadOnly(locked)
        self._speed_spinbox.setEnabled(not locked)
        self._btn_save.setEnabled(not locked)
        self._btn_new.setEnabled(not locked)
        self._btn_release.setEnabled(not locked and not released)
        self._btn_new_revision.setEnabled(not locked and released)


    def set_operating_mode(
        self, *, production: bool, collision_planning_required: bool
    ) -> None:
        self._production_mode = bool(production)
        self._collision_planning_required = bool(collision_planning_required)
        self._refresh_recipe_state()
        self._update_run_availability()


    def _production_prerequisite_status(
        self,
    ) -> tuple[bool, str, dict[str, object]]:
        provider = self._production_prerequisite_provider
        if provider is None:
            return True, "", {}
        try:
            valid, reason, reference = provider()
        except Exception as exc:
            return False, f"validação da calibração indisponível: {exc}", {}
        return bool(valid), str(reason), (
            dict(reference) if isinstance(reference, dict) else {}
        )


    def _refresh_effective_speed_label(self) -> None:
        program_scale = float(self._speed_spinbox.value())
        global_override = float(self._bridge.speed_override)
        ceiling = self._dynamic_override_ceiling()
        physical_override = global_override * ceiling
        ceiling_note = (
            f" · teto receita {ceiling * 100:.0f}%"
            if ceiling < 1.0 - 1e-9
            else ""
        )
        self._effective_speed_label.setText(
            f"Global {global_override * 100:.0f}%{ceiling_note} · "
            f"efetiva {program_scale * physical_override:.2f}x"
        )


    def _dynamic_override_ceiling(self) -> float:
        analysis = self._program.parameters.get("_dynamic_analysis", {})
        try:
            value = float(analysis.get("override_ceiling", 1.0))
        except (AttributeError, TypeError, ValueError):
            value = 1.0
        stored = max(0.05, min(1.0, value))
        runtime = max(0.05, min(1.0, self._runtime_dynamic_ceiling))
        return min(stored, runtime)


    def _invalidate_dynamic_analysis(self) -> None:
        self._program.parameters.pop("_dynamic_analysis", None)
        self._program.parameters.pop("_cell_scene_analysis", None)
        self._program.parameters.pop(VISION_ENVELOPE_ANALYSIS_KEY, None)
        self._runtime_dynamic_ceiling = 1.0
        self._refresh_effective_speed_label()


    def _update_progress_meta(self) -> None:
        if not self._running:
            n_steps = self._program.estimated_steps()
            n_cmds = self._program.estimated_commands()
            self._lbl_progress.setText(f"{n_steps} etapas / {n_cmds} comandos")


    def _set_modified(self, modified: bool) -> None:
        if self._is_modified == modified:
            return
        self._is_modified = modified
        self._update_modified_indicator()


    def confirm_close(self) -> bool:
        """Return whether the application may close without losing work."""
        if self._running:
            QMessageBox.warning(
                self,
                "Programa em execução",
                "Pare a execução do programa antes de fechar a interface.",
            )
            return False
        if not self._is_modified:
            return True

        box = QMessageBox(self)
        box.setIcon(QMessageBox.Warning)
        box.setWindowTitle("Programa não salvo")
        box.setText("O programa possui alterações não salvas.")
        box.setInformativeText("Deseja salvar antes de fechar?")
        save_button = box.addButton("Salvar...", QMessageBox.AcceptRole)
        discard_button = box.addButton("Descartar", QMessageBox.DestructiveRole)
        box.addButton(QMessageBox.Cancel)
        box.setDefaultButton(save_button)
        box.exec()
        clicked = box.clickedButton()
        if clicked is save_button:
            self._save_dialog()
            return not self._is_modified
        return clicked is discard_button


    def _update_modified_indicator(self) -> None:
        title = self._editor_group_title
        if self._is_modified:
            title += " *"
        self._editor_box.setTitle(title)


    def _current_snapshot(self) -> dict[str, Any]:
        return {
            "program": self._program.to_dict(),
            "selected_index": self._editor._list.currentRow(),
        }


    def _apply_snapshot(self, snapshot: dict[str, Any], mark_modified: bool = True) -> None:
        self._program = Program.from_dict(snapshot["program"])
        self._refresh_after_load()
        selected = snapshot.get("selected_index", -1)
        self._editor.set_selected_index(selected)
        if mark_modified:
            self._set_modified(True)


    def _push_undo_snapshot(self) -> None:
        if self._undo_saving:
            return
        snapshot = self._current_snapshot()
        # avoid stacking duplicate consecutive states
        if self._undo_stack and self._undo_stack[-1]["program"] == snapshot["program"]:
            return
        self._undo_stack.append(snapshot)
        if len(self._undo_stack) > self._undo_limit:
            self._undo_stack.pop(0)
        self._redo_stack.clear()
        self._update_undo_buttons()


    def _undo(self) -> None:
        if self._running or not self._undo_stack:
            return
        current = self._current_snapshot()
        snapshot = self._undo_stack.pop()
        self._redo_stack.append(current)
        self._undo_saving = True
        try:
            self._apply_snapshot(snapshot)
        finally:
            self._undo_saving = False
        self._update_undo_buttons()


    def _redo(self) -> None:
        if self._running or not self._redo_stack:
            return
        current = self._current_snapshot()
        snapshot = self._redo_stack.pop()
        self._undo_stack.append(current)
        self._undo_saving = True
        try:
            self._apply_snapshot(snapshot)
        finally:
            self._undo_saving = False
        self._update_undo_buttons()


    def _update_undo_buttons(self) -> None:
        can_undo = bool(self._undo_stack) and not self._running
        can_redo = bool(self._redo_stack) and not self._running
        self._btn_undo.setEnabled(can_undo)
        self._btn_redo.setEnabled(can_redo)
        self._act_undo.setEnabled(can_undo)
        self._act_redo.setEnabled(can_redo)


    def _on_program_changed(self) -> None:
        self._ensure_editable_revision()
        self._invalidate_dynamic_analysis()
        self._update_progress_meta()
        self._set_modified(True)
        self._flow_widget.set_program(self._program)
        self._update_flow_invalid_indices()
        self._update_run_availability()


    def _update_flow_invalid_indices(self) -> None:
        """Lightweight structural invalid-step markers for the flowchart."""
        invalid: set[int] = set()
        if not self._program:
            self._flow_widget.set_invalid_indices(invalid)
            return
        step_ids: dict[str, int] = {}
        for i, step in enumerate(self._program.steps):
            sid = getattr(step, "label", "") or ""
            if not sid or sid in _INVALID_GENERIC_STEP_IDS:
                invalid.add(i)
            elif sid in step_ids:
                invalid.add(i)
                invalid.add(step_ids[sid])
            else:
                step_ids[sid] = i
            if getattr(step, "kind", "") == KIND_IF:
                cond = getattr(step, "condition", None)
                if cond is None or not getattr(cond, "terms", []):
                    invalid.add(i)
        self._flow_widget.set_invalid_indices(invalid)


