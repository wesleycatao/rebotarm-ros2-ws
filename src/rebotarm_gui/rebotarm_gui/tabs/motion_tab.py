"""Motion-planning tab.

Lets the user compose a *program* of motion steps (including control flow:
``LOOP`` and ``IF-ELSE``), parameterize each one, run it against the
``MoveToPose`` action, and save/load the program as a JSON file (with an
optional preload default).

Supported step kinds:
  - Ir ao ponto (move)        : single Cartesian target + duration
  - Linha reta (line)         : start -> end, fixed orientation, speed
  - Arco circular (arc)       : circle through 3 points; tool may aim at center
  - Paletização 3D (pallet)   : 3D grid with approach/retract
  - WAIT (tempo)              : wait N seconds
  - WAIT_FOR_STATE_MACHINE    : wait for the arm state machine to reach a state
  - WAIT_FOR_SIGNAL           : wait for a Raspberry Pi digital input (GPIO)
  - LOOP (repetir)            : repeat child steps N times (or forever)
  - IF-ELSE (condicional)     : branch on sensor / state / vision conditions

Execution is a small program-counter VM over a flattened opcode list (see
:mod:`rebotarm_gui.motion.program`), so loops and conditionals are honoured.
A global ``speed_scale`` scales every move's duration homogeneously.

Teach features (UR/FANUC-style): a *Freedrive* toggle puts the arm in
gravity-compensation mode so the user can move it by hand, and "Ensinar
ponto atual" captures the current TCP pose as a new ``MoveStep``. The
program can also be inspected as a read-only flowchart
(:class:`ProgramFlowWidget`) via the Lista/Fluxo view toggle.
"""

from __future__ import annotations

import math
import os
import time
import hashlib
import json
from pathlib import Path
from typing import Any, Callable, Optional

import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMenu,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from ..audit import ExecutionAuditLog, new_run_id, utc_now_iso
from ..motion import (
    ACTION_OPS,
    EDGE_FALLING,
    EDGE_LEVEL,
    EDGE_RISING,
    OP_BRANCH,
    OP_GOTO,
    OP_GRIPPER,
    OP_LABEL,
    OP_LOOP_BACK,
    OP_LOOP_INIT,
    OP_MOVE,
    OP_OUTPUT,
    OP_WAIT,
    OP_WAIT_SIG,
    OP_WAIT_SM,
    OUTPUT_OFF,
    OUTPUT_TOGGLE,
    MoveStep,
    Program,
    build_label_index,
    evaluate,
    quat_to_rpy,
    rpy_to_quat,
    scale_duration,
)
from ..motion.preflight import (
    REACHABILITY_DEFAULT_TIMEOUT_MS,
    PreflightReport,
    run_preflight,
)
from ..ros_bridge import make_pose
from ..widgets.layout_utils import fit_button_text
from ..widgets.program_flow_widget import ProgramFlowWidget
from ..widgets.step_list_editor import StepListEditor
from ..widgets.preflight_dialog import PreflightDialog
from ..widgets.trajectory_preview_dialog import TrajectoryPreviewDialog


VM_ITER_CAP = 200_000     # safety bound for a single synchronous dispatch
POLL_INTERVAL_MS = 100    # wait-for-state / wait-for-signal polling


def _config_dir() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME")
    root = Path(base) if base else Path.home() / ".config"
    return root / "rebotarm_gui"


def _preload_path() -> Path:
    return _config_dir() / "motion_program.json"


def _dict_to_pose(d: dict):
    qx, qy, qz, qw = rpy_to_quat(float(d["roll"]), float(d["pitch"]), float(d["yaw"]))
    return make_pose(d["x"], d["y"], d["z"], qx, qy, qz, qw)


def _pose_msg_to_dict(msg) -> Optional[dict]:
    """Convert a geometry_msgs/Pose to a PoseDict (x, y, z, roll, pitch, yaw)."""
    if msg is None:
        return None
    try:
        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        qw = float(msg.orientation.w)
        roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)
        return {
            "x": float(msg.position.x),
            "y": float(msg.position.y),
            "z": float(msg.position.z),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }
    except Exception:
        return None


class MotionTab(QWidget):
    """Tab to design, run and persist motion programs."""

    program_loaded = Signal()

    def __init__(
        self,
        bridge,
        parent: QWidget | None = None,
        *,
        audit_log: ExecutionAuditLog | None = None,
        operator_provider: Callable[[], str] | None = None,
    ) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._audit_log = audit_log or ExecutionAuditLog()
        self._operator_provider = operator_provider or self._default_operator
        self._program = Program(name="programa")
        self._robot_ready = False

        # ---- VM state ----
        self._running = False
        self._vm_ops: list = []
        self._vm_labels: dict[str, int] = {}
        self._vm_pc = 0
        self._vm_loop_counters: dict[str, int] = {}
        self._vm_seq = 0
        self._vm_pending_tag: Optional[str] = None
        self._vm_pending_gripper = False
        self._vm_meta: dict[str, Any] = {}
        self._vm_action_count = 0
        self._vm_action_seen = 0
        self._speed_scale = 1.0
        self._is_modified = False
        self._last_preflight: Optional[PreflightReport] = None
        self._preflight_in_progress = False

        # ---- persistent execution audit ----
        self._audit_run: dict[str, Any] | None = None
        self._audit_active_step: dict[str, Any] | None = None
        self._audit_step_sequence = 0

        # ---- joint recording ----
        self._record_joints = False
        self._joint_history: list[tuple[float, list[str], list[float]]] = []
        self._joint_history_markers: list[tuple[float, str]] = []
        self._joint_recording_active = False
        self._joint_recording_start = 0.0
        self._dry_run_record_timer = QTimer(self)
        self._dry_run_record_timer.timeout.connect(self._on_dry_run_record_tick)
        self._dry_run_record_interval_ms = 100
        self._editor_group_title = "Programa de movimento"

        # ---- run modes ----
        self._dry_run = False
        self._step_by_step = False
        self._step_waiting = False
        self._step_advance_pending = False

        # ---- undo/redo ----
        self._undo_stack: list[dict[str, Any]] = []
        self._redo_stack: list[dict[str, Any]] = []
        self._undo_limit = 50
        self._undo_saving = False
        self._undo_debounce_timer = QTimer(self)
        self._undo_debounce_timer.setSingleShot(True)
        self._undo_debounce_timer.timeout.connect(self._push_undo_snapshot)

        # ---- async-wait state ----
        self._wait_timer = QTimer(self)
        self._wait_timer.timeout.connect(self._on_wait_poll)
        self._wait_kind = ""
        self._wait_deadline: Optional[float] = None
        self._wait_target_state = ""
        self._wait_io_id = ""
        self._wait_value = 0
        self._wait_edge = EDGE_LEVEL
        self._wait_edge_last = 0

        self._build_ui()
        self._wire_bridge()

        preload = _preload_path()
        if preload.is_file():
            try:
                self._load_from_file(preload)
            except Exception as exc:
                self._lbl_status.setText(f"Erro ao carregar pré-carga: {exc}")

    # ==================================================================
    # UI construction
    # ==================================================================
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

        self._btn_save = QPushButton("Salvar...")
        self._btn_save.clicked.connect(self._save_dialog)
        self._btn_load = QPushButton("Carregar...")
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
        for b in (self._btn_save, self._btn_load, self._btn_preload, self._btn_new,
                  self._btn_validate, self._btn_preview):
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
        self._btn_view_flow.setToolTip("Visualização do programa em fluxograma (somente leitura).")
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

        primary_toolbar = QHBoxLayout()
        primary_toolbar.setSpacing(8)
        primary_toolbar.addWidget(self._btn_run)
        primary_toolbar.addWidget(self._btn_run_selected)
        primary_toolbar.addWidget(self._btn_preview)
        primary_toolbar.addWidget(self._btn_stop)
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

        # program name + global speed scale
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
            "Multiplicador global de velocidade aplicado a todos os movimentos.\n"
            ">1: mais rápido (a duração de cada segmento é limitada ao mínimo do planejador).\n"
            "<1: mais lento. Afeta a rotina inteira de forma homogênea."
        )
        self._speed_spinbox.valueChanged.connect(self._on_speed_scale_changed)

        name_row = QHBoxLayout()
        name_row.setSpacing(8)
        name_row.addWidget(QLabel("Programa:"))
        name_row.addWidget(self._name_edit)
        name_row.addSpacing(16)
        name_row.addWidget(QLabel("Escala de velocidade:"))
        name_row.addWidget(self._speed_spinbox)
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
        self._editor.selection_changed.connect(
            lambda _row: self._update_run_availability()
        )

        # flowchart view (read-only), stacked with the list editor
        self._flow_widget = ProgramFlowWidget()
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

        # keyboard shortcuts
        QShortcut(QKeySequence.StandardKey.Undo, self, self._undo)
        QShortcut(QKeySequence("Ctrl+Y"), self, self._redo)
        QShortcut(QKeySequence("Ctrl+Shift+Return"), self, self._run_from_selected)

    # ==================================================================
    # Bridge wiring
    # ==================================================================
    def _wire_bridge(self) -> None:
        self._bridge.pose_action_result.connect(self._on_pose_result)
        self._bridge.action_result.connect(self._on_action_result)
        self._bridge.joint_state_changed.connect(self._on_joint_state)
        self._bridge.service_result.connect(self._on_teach_service_result)

    # ==================================================================
    # Program-change handlers
    # ==================================================================
    def _on_program_changed(self) -> None:
        self._update_progress_meta()
        self._set_modified(True)
        self._flow_widget.set_program(self._program)
        self._update_run_availability()

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
        self._push_undo_snapshot()
        self._program.name = self._name_edit.text().strip() or "programa"
        self._set_modified(True)

    def _on_speed_scale_changed(self, value: float) -> None:
        self._program.speed_scale = float(value)
        self._set_modified(True)
        self._undo_debounce_timer.start(1500)

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
        self._name_edit.setText(self._program.name)
        self._speed_spinbox.blockSignals(True)
        self._speed_spinbox.setValue(self._program.speed_scale)
        self._speed_spinbox.blockSignals(False)
        self._editor.set_steps(self._program.steps)
        self._flow_widget.set_program(self._program)
        self._update_progress_meta()
        self._set_modified(False)

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

    # ------------------------------------------------------------------
    # Undo / redo
    # ------------------------------------------------------------------
    def _current_snapshot(self) -> dict[str, Any]:
        return {
            "program": self._program.to_dict(),
            "selected_index": self._editor._list.currentRow(),
        }

    def _apply_snapshot(self, snapshot: dict[str, Any], mark_modified: bool = True) -> None:
        self._program = Program.from_dict(snapshot["program"])
        self._refresh_after_load()
        selected = snapshot.get("selected_index", -1)
        if 0 <= selected < self._editor._list.count():
            self._editor._list.setCurrentRow(selected)
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

    # ==================================================================
    # Teach (freedrive + point capture)
    # ==================================================================
    def _set_freedrive_checked(self, checked: bool) -> None:
        self._btn_freedrive.blockSignals(True)
        self._btn_freedrive.setChecked(checked)
        self._btn_freedrive.blockSignals(False)

    def _on_freedrive_toggled(self, checked: bool) -> None:
        if self._running:
            self._set_freedrive_checked(False)
            self._lbl_status.setText("Freedrive indisponível durante a execução.")
            return
        if not self._robot_ready:
            self._set_freedrive_checked(False)
            self._lbl_status.setText("Freedrive indisponível: braço sem prontidão operacional.")
            return
        if checked:
            self._lbl_status.setText("Ativando freedrive...")
            self._bridge.gravity_start()
        else:
            self._bridge.gravity_stop()

    def _on_teach_service_result(self, label: str, success: bool, message: str) -> None:
        if label not in ("gravity_start", "gravity_stop"):
            return
        if success:
            if label == "gravity_start":
                self._lbl_status.setText("Freedrive ativo — mova o braço com a mão.")
            else:
                self._lbl_status.setText("Freedrive desativado.")
            return
        # service failed: undo the toggle to reflect the real arm state
        self._set_freedrive_checked(label == "gravity_stop")
        self._lbl_status.setText(f"Falha no freedrive: {message}")

    def _teach_current_point(self) -> None:
        if self._running:
            return
        pose_msg = self._bridge.tcp_pose_fk() or self._bridge.last_reached_pose
        pose_dict = _pose_msg_to_dict(pose_msg)
        if pose_dict is None:
            QMessageBox.information(
                self,
                "Pose indisponível",
                "Não foi possível obter a pose atual do TCP.\n"
                "Verifique se o braço está conectado e publicando joint_states.",
            )
            return
        n = sum(1 for s in self._program.steps if s.label.startswith("Ponto ")) + 1
        step = MoveStep(pose=pose_dict, duration=2.0, label=f"Ponto {n}")
        # same mutation pattern as the editor's add-step: undo snapshot,
        # in-place append, refresh, changed
        self._push_undo_snapshot()
        self._program.steps.append(step)
        self._editor.set_steps(self._program.steps)
        self._on_program_changed()
        self._lbl_status.setText(
            f"Ponto {n} ensinado: ({pose_dict['x']:.3f}, {pose_dict['y']:.3f}, "
            f"{pose_dict['z']:.3f})"
        )

    def _check_execution_preconditions(self) -> tuple[bool, str]:
        """Return (ok, message) verifying the arm is ready to run a program."""
        if self._dry_run:
            # Dry-run only needs the bridge for condition/signal evaluation.
            if not self._bridge.is_connected:
                return False, "Não conectado ao ROS 2."
            return True, ""
        return self._bridge.motion_readiness("move_to_pose")

    # ==================================================================
    # Pre-flight validation
    # ==================================================================
    def _open_preflight_dialog(self) -> None:
        """Open the preflight report dialog and run validation immediately."""
        if self._running:
            return
        # Commit any pending editor values so the dialog sees the latest program.
        self._sync_editors_to_program()
        dialog = PreflightDialog(
            self._bridge,
            get_program_cb=lambda: (self._program, not self._dry_run),
            parent=self,
        )
        dialog.revalidate()
        dialog.exec()

    def _preview_trajectory(self) -> None:
        """Compile and display the route without contacting the controller."""
        if self._running:
            return
        self._sync_editors_to_program()
        if not self._program.steps:
            QMessageBox.information(
                self, "Programa vazio", "Adicione etapas antes de gerar a prévia."
            )
            return
        dialog = TrajectoryPreviewDialog(self._program, parent=self)
        dialog.exec()

    def _preflight_gate(self, program: Program | None = None) -> bool:
        """Run preflight before execution; return whether the run may proceed.

        Real executions block on any error (including unreachable poses, with
        a bounded wait for the IK service). Dry-runs run the synchronous
        checks only and never block on reachability.
        """
        if self._preflight_in_progress:
            # A second click on Executar while the gate's nested event loop is
            # still running must not start a second probe/run.
            return False
        self._preflight_in_progress = True
        self._btn_run.setEnabled(False)
        self._btn_run_selected.setEnabled(False)
        try:
            return self._run_preflight_inner(not self._dry_run, program)
        finally:
            self._preflight_in_progress = False
            self._update_run_availability()

    def _run_preflight_inner(
        self,
        for_real_run: bool,
        program: Program | None = None,
    ) -> bool:
        try:
            report = run_preflight(
                program or self._program,
                self._bridge,
                for_real_run=for_real_run,
                with_reachability=for_real_run,
                reachability_timeout_ms=REACHABILITY_DEFAULT_TIMEOUT_MS,
            )
        except Exception as exc:  # defensive: never let preflight crash a run
            QMessageBox.critical(
                self, "Validação indisponível",
                f"Não foi possível executar o preflight: {exc}",
            )
            return False
        self._last_preflight = report
        if report.ok and not report.warnings:
            return True
        if not report.ok:
            self._show_preflight_block(report)
            return False
        # Warnings only: inform once and let the operator decide via the
        # confirm dialog that follows.
        QMessageBox.information(
            self, "Preflight OK com avisos",
            f"{report.summary()}\n\nA execução pode prosseguir; revise os avisos "
            "no botão \"Validar\" se necessário.",
        )
        return True

    def _show_preflight_block(self, report: PreflightReport) -> None:
        errors = report.errors[:12]
        lines = "\n".join(f"• [{e.category}] {e.message}" for e in errors)
        more = "" if len(report.errors) <= len(errors) else f"\n… (+{len(report.errors) - len(errors)} erro(s))"
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Critical)
        box.setWindowTitle("Execução bloqueada pelo preflight")
        box.setText(
            f"{len(report.errors)} erro(s) impedem a execução:\n\n{lines}{more}"
        )
        open_btn = box.addButton("Abrir validação", QMessageBox.AcceptRole)
        box.addButton(QMessageBox.Cancel)
        box.setDefaultButton(open_btn)
        box.exec()
        if box.clickedButton() is open_btn:
            self._open_preflight_dialog()


    # ==================================================================
    # Run / Stop (PC-based VM)
    # ==================================================================
    def _run(self) -> None:
        self._start_run(0)

    def _run_from_selected(self) -> None:
        start_index = self._editor.selected_index()
        if start_index < 0:
            QMessageBox.information(
                self,
                "Nenhuma etapa selecionada",
                "Selecione uma etapa na lista antes de executar a partir dela.",
            )
            return
        self._start_run(start_index)

    def _execution_program(self, start_index: int) -> Program:
        """Return the top-level suffix selected for this execution."""
        if not (0 <= start_index < len(self._program.steps)):
            raise IndexError("etapa inicial fora do programa")
        return Program(
            name=self._program.name,
            version=self._program.version,
            speed_scale=float(self._speed_spinbox.value()),
            steps=list(self._program.steps[start_index:]),
        )

    def _compile_execution(self, start_index: int):
        """Compile a suffix and map its highlights back to the full editor."""
        program = self._execution_program(start_index)
        ops, raw_meta = program.flatten_with_meta()
        meta = dict(raw_meta)
        meta["pc_to_root_index"] = {
            int(pc): int(root_index) + start_index
            for pc, root_index in raw_meta.get("pc_to_root_index", {}).items()
        }
        return program, ops, meta, build_label_index(ops)

    @staticmethod
    def _default_operator() -> str:
        return os.environ.get("USER", "").strip() or "não identificado"

    def _audit_common_fields(self) -> dict[str, Any]:
        run = self._audit_run or {}
        return {
            "operator": run.get("operator", "não identificado"),
            "program": run.get("program", self._program.name),
            "program_version": run.get("program_version", self._program.version),
            "program_hash": run.get("program_hash", ""),
            "mode": run.get("mode", "simulation" if self._dry_run else "real"),
            "start_index": run.get("start_index", 0),
        }

    def _begin_audit_run(self, program: Program, start_index: int) -> bool:
        """Create the run context and durably record it before motion starts."""
        try:
            program_data = program.to_dict()
            canonical = json.dumps(
                program_data,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8")
        except (TypeError, ValueError) as exc:
            self._audit_log.last_error = f"programa não serializável: {exc}"
            return False
        operator = str(self._operator_provider() or "").strip() or "não identificado"
        started_at = utc_now_iso()
        run_id = new_run_id()
        self._audit_run = {
            "run_id": run_id,
            "operator": operator,
            "program": program.name,
            "program_version": program.version,
            "program_hash": hashlib.sha256(canonical).hexdigest(),
            "mode": "simulation" if self._dry_run else "real",
            "start_index": int(start_index),
            "started_at_utc": started_at,
            "started_monotonic": time.monotonic(),
            "write_failed": False,
        }
        self._audit_active_step = None
        self._audit_step_sequence = 0
        ok = self._append_audit(
            "run_start",
            run_id=run_id,
            **self._audit_common_fields(),
            started_at_utc=started_at,
            parameters={
                "speed_scale": float(self._speed_spinbox.value()),
                "step_by_step": bool(self._step_by_step),
                "record_joints": bool(self._chk_record.isChecked()),
                "estimated_steps": program.estimated_steps(),
                "estimated_commands": program.estimated_commands(),
            },
            result="started",
        )
        if not ok:
            self._audit_run = None
        return ok

    def _append_audit(self, event: str, **fields: Any) -> bool:
        ok = self._audit_log.append(event, **fields)
        if not ok and self._audit_run is not None:
            self._audit_run["write_failed"] = True
        return ok

    def _audit_step_details(self, op: tuple) -> tuple[str, str, dict[str, Any]]:
        tag = str(op[0]) if op else "unknown"
        if tag == OP_MOVE:
            raw_duration = float(op[2])
            return tag, str(op[3]), {
                "pose": dict(op[1]),
                "duration_s": raw_duration,
                "scaled_duration_s": scale_duration(raw_duration, self._speed_scale),
            }
        if tag == OP_GRIPPER:
            return tag, str(op[4]), {
                "position_mm": float(op[1]),
                "max_effort": float(op[2]),
                "timeout_s": float(op[3]),
            }
        if tag == OP_WAIT:
            return tag, str(op[2]), {"duration_s": float(op[1])}
        if tag == OP_WAIT_SM:
            return tag, str(op[3]), {"state": str(op[1]), "timeout_s": float(op[2])}
        if tag == OP_WAIT_SIG:
            return tag, str(op[5]), {
                "io_id": str(op[1]),
                "value": int(op[2]),
                "edge": str(op[3]),
                "timeout_s": float(op[4]),
            }
        if tag == OP_OUTPUT:
            return tag, str(op[3]), {"io_id": str(op[1]), "action": str(op[2])}
        return tag, "", {"opcode": list(op[1:])}

    def _audit_begin_step(self, op: tuple) -> bool:
        if self._audit_run is None:
            return False
        step_type, step_id, parameters = self._audit_step_details(op)
        self._audit_step_sequence += 1
        started_at = utc_now_iso()
        self._audit_active_step = {
            "step_sequence": self._audit_step_sequence,
            "step_index": self._vm_meta.get("pc_to_root_index", {}).get(self._vm_pc),
            "step_id": step_id,
            "step_type": step_type,
            "parameters": parameters,
            "started_at_utc": started_at,
            "started_monotonic": time.monotonic(),
        }
        fields = dict(self._audit_active_step)
        fields.pop("started_monotonic", None)
        return self._append_audit(
            "step_start",
            run_id=self._audit_run["run_id"],
            **self._audit_common_fields(),
            **fields,
            result="started",
        )

    def _audit_finish_step(self, result: str, message: str = "") -> bool:
        if self._audit_run is None or self._audit_active_step is None:
            return True
        active = self._audit_active_step
        self._audit_active_step = None
        finished_at = utc_now_iso()
        fields = {
            key: value for key, value in active.items() if key != "started_monotonic"
        }
        fields.update(
            finished_at_utc=finished_at,
            duration_ms=round(
                max(0.0, time.monotonic() - active["started_monotonic"]) * 1000.0,
                3,
            ),
            result=result,
            message=str(message),
        )
        return self._append_audit(
            "step_end",
            run_id=self._audit_run["run_id"],
            **self._audit_common_fields(),
            **fields,
        )

    def _audit_cancel(self, source: str, message: str) -> None:
        if self._audit_run is None:
            return
        active = self._audit_active_step or {}
        self._append_audit(
            "cancel_requested",
            run_id=self._audit_run["run_id"],
            **self._audit_common_fields(),
            step_sequence=active.get("step_sequence", ""),
            step_index=active.get("step_index", ""),
            step_id=active.get("step_id", ""),
            step_type=active.get("step_type", ""),
            parameters=active.get("parameters", {}),
            result="requested",
            message=message,
            cancel_source=source,
        )

    def _audit_finish_run(self, result: str, message: str) -> bool:
        if self._audit_run is None:
            return True
        run = self._audit_run
        ok = not bool(run.get("write_failed"))
        if result == "failure":
            active = self._audit_active_step or {}
            ok = self._append_audit(
                "failure",
                run_id=run["run_id"],
                **self._audit_common_fields(),
                step_sequence=active.get("step_sequence", ""),
                step_index=active.get("step_index", ""),
                step_id=active.get("step_id", ""),
                step_type=active.get("step_type", ""),
                parameters=active.get("parameters", {}),
                result="failure",
                message=message,
            ) and ok
        ok = self._audit_finish_step(result, message) and ok
        ok = self._append_audit(
            "run_end",
            run_id=run["run_id"],
            **self._audit_common_fields(),
            started_at_utc=run["started_at_utc"],
            finished_at_utc=utc_now_iso(),
            duration_ms=round(
                max(0.0, time.monotonic() - run["started_monotonic"]) * 1000.0,
                3,
            ),
            result=result,
            message=message,
        ) and ok
        self._audit_run = None
        self._audit_active_step = None
        return ok

    def _export_audit_csv(self) -> None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        suggested = self._audit_log.path.parent / f"execution_audit_{stamp}.csv"
        path, _ = QFileDialog.getSaveFileName(
            self, "Exportar auditoria CSV", str(suggested), "CSV (*.csv)"
        )
        if not path:
            return
        try:
            count = self._audit_log.export_csv(path)
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, "Erro ao exportar auditoria", str(exc))
            return
        self._lbl_status.setText(f"Auditoria exportada: {count} evento(s).")
        QMessageBox.information(
            self,
            "Auditoria exportada",
            f"{count} evento(s) salvos em:\n{path}",
        )

    def _start_run(self, start_index: int) -> None:
        if self._running:
            return
        self._sync_editors_to_program()
        if not self._program.steps:
            QMessageBox.information(self, "Programa vazio", "Adicione etapas antes de executar.")
            return
        try:
            run_program, ops, meta, labels = self._compile_execution(start_index)
        except Exception as exc:
            QMessageBox.critical(self, "Programa inválido", str(exc))
            return
        if not any(op and op[0] in ACTION_OPS for op in ops):
            QMessageBox.information(self, "Programa vazio", "O programa não gera nenhum movimento.")
            return

        if not self._preflight_gate(run_program):
            return

        ok, reason = self._check_execution_preconditions()
        if not ok:
            QMessageBox.warning(self, "Não é possível executar", reason)
            return

        n_steps = run_program.estimated_steps()
        n_cmds = run_program.estimated_commands()
        speed = self._speed_spinbox.value()
        state = self._bridge.state_machine or "desconhecido"
        mode_text = []
        if self._dry_run:
            mode_text.append("modo SIMULAÇÃO")
        if self._step_by_step:
            mode_text.append("modo PASSO A PASSO")
        if start_index > 0:
            mode_text.append(f"a partir da etapa {start_index + 1}")
        mode_line = f"<br>• Modo: {', '.join(mode_text)}" if mode_text else ""
        reply = QMessageBox.question(
            self,
            "Confirmar execução",
            f"<b>{self._program.name}</b><br>"
            f"• {n_steps} etapa(s)<br>"
            f"• {n_cmds} comando(s) de movimento/gripper/I/O<br>"
            f"• Escala de velocidade: {speed:.2f}x<br>"
            f"• Estado do braço: <b>{state}</b>"
            f"{mode_line}<br><br>"
            "Deseja executar o programa?",
        )
        if reply != QMessageBox.Yes:
            return

        if not self._begin_audit_run(run_program, start_index):
            QMessageBox.critical(
                self,
                "Auditoria indisponível",
                "A execução não foi iniciada porque o registro de auditoria não pôde "
                f"ser salvo em:\n{self._audit_log.path}\n\n{self._audit_log.last_error}",
            )
            return

        self._vm_ops = ops
        self._vm_labels = labels
        self._vm_meta = meta
        self._vm_action_count = meta.get("action_count", n_cmds)
        self._vm_action_seen = 0
        self._vm_pc = 0
        self._vm_loop_counters = {}
        self._vm_seq = 0
        self._vm_pending_tag = None
        self._vm_pending_gripper = False
        self._step_waiting = False
        self._step_advance_pending = False
        self._speed_scale = float(self._speed_spinbox.value())
        self._running = True
        self._record_joints = self._chk_record.isChecked()
        self._joint_history.clear()
        self._joint_history_markers.clear()
        self._joint_recording_active = self._record_joints
        self._joint_recording_start = time.monotonic()
        if self._dry_run and self._record_joints:
            self._dry_run_record_timer.start(self._dry_run_record_interval_ms)
        self._btn_run.setEnabled(False)
        self._btn_run_selected.setEnabled(False)
        self._btn_preview.setEnabled(False)
        self._btn_stop.setEnabled(True)
        self._btn_advance.setEnabled(False)
        self._btn_freedrive.setEnabled(False)
        self._btn_teach.setEnabled(False)
        self._editor.set_edit_enabled(False)
        self._flow_widget.set_edit_enabled(False)
        self._progress.setValue(0)
        self._progress.setVisible(True)
        self._chk_dry_run.setEnabled(False)
        self._chk_step.setEnabled(False)
        self._act_record_joints.setEnabled(False)
        self._act_preload.setEnabled(False)
        self._update_undo_buttons()
        self._vm_step()

    def _vm_step(self) -> None:
        """Dispatch synchronous control opcodes until an async one is hit."""
        if not self._running:
            return
        ops = self._vm_ops
        total = len(ops)
        iters = 0
        while 0 <= self._vm_pc < len(ops):
            iters += 1
            if iters > VM_ITER_CAP:
                self._finish(ok=False, message="Limite de iterações excedido (possível loop infinito).")
                return
            op = ops[self._vm_pc]
            tag = op[0] if op else ""

            if tag == OP_LABEL:
                self._vm_pc += 1
                continue
            if tag == OP_GOTO:
                target = self._vm_labels.get(op[1])
                if target is None:
                    self._finish(ok=False, message=f"Rótulo desconhecido: {op[1]}")
                    return
                self._vm_pc = target
                continue
            if tag == OP_BRANCH:
                cond, name, polarity = op[1], op[2], op[3]
                try:
                    res = evaluate(cond, self._bridge)
                except Exception as exc:
                    self._finish(ok=False, message=f"Erro ao avaliar condição: {exc}")
                    return
                if bool(res) == bool(polarity):
                    target = self._vm_labels.get(name)
                    if target is None:
                        self._finish(ok=False, message=f"Rótulo desconhecido: {name}")
                        return
                    self._vm_pc = target
                else:
                    self._vm_pc += 1
                continue
            if tag == OP_LOOP_INIT:
                self._vm_loop_counters[op[1]] = 0
                self._vm_pc += 1
                continue
            if tag == OP_LOOP_BACK:
                loop_id, count, name = op[1], op[2], op[3]
                self._vm_loop_counters[loop_id] = self._vm_loop_counters.get(loop_id, 0) + 1
                c = self._vm_loop_counters[loop_id]
                if count <= 0 or c < count:
                    target = self._vm_labels.get(name, self._vm_pc)
                    self._vm_pc = target
                else:
                    self._vm_pc += 1
                continue

            # digital output: fire-and-forget, advance immediately
            if tag == OP_OUTPUT:
                io_id, action, step_id = op[1], op[2], op[3]
                self._record_step_marker(step_id)
                self._vm_action_seen += 1
                self._update_progress_bar()
                self._highlight_current_step()
                self._bridge.safety_set_step(step_id)
                if not self._audit_begin_step(op):
                    self._finish(
                        ok=False,
                        message=f"Falha ao registrar início da etapa: {self._audit_log.last_error}",
                        result="failure",
                    )
                    return
                self._dispatch_output(io_id, action)
                if not self._audit_finish_step(
                    "completed" if self._dry_run else "dispatched"
                ):
                    self._finish(
                        ok=False,
                        message=f"Falha ao registrar fim da etapa: {self._audit_log.last_error}",
                        result="failure",
                    )
                    return
                self._vm_pc += 1
                continue

            # async opcodes: highlight, progress, maybe step-by-step pause, then dispatch
            if tag in (OP_MOVE, OP_GRIPPER, OP_WAIT, OP_WAIT_SM, OP_WAIT_SIG):
                self._record_step_marker(op[-1])
                self._vm_action_seen += 1
                self._update_progress_bar()
                self._highlight_current_step()
                if self._step_by_step and not self._step_advance_pending:
                    self._pause_for_step(tag)
                    return
                self._step_advance_pending = False
                self._step_waiting = False
                if not self._audit_begin_step(op):
                    self._finish(
                        ok=False,
                        message=f"Falha ao registrar início da etapa: {self._audit_log.last_error}",
                        result="failure",
                    )
                    return
                if tag == OP_MOVE:
                    self._dispatch_move(op, total)
                elif tag == OP_GRIPPER:
                    self._dispatch_gripper(op, total)
                elif tag == OP_WAIT:
                    self._dispatch_wait_time(op, total)
                elif tag == OP_WAIT_SM:
                    self._dispatch_wait_sm(op, total)
                elif tag == OP_WAIT_SIG:
                    self._dispatch_wait_sig(op, total)
                return

            # unknown opcode: skip defensively
            self._vm_pc += 1

        # reached the end
        self._finish(ok=True, message="Programa concluído.")

    def _highlight_current_step(self) -> None:
        root_index = self._vm_meta.get("pc_to_root_index", {}).get(self._vm_pc)
        self._editor.highlight_row(root_index)
        self._flow_widget.set_active_root_index(root_index)

    def _update_progress_bar(self) -> None:
        total = max(1, self._vm_action_count)
        pct = int(100 * self._vm_action_seen / total)
        self._progress.setValue(min(pct, 100))

    def _pause_for_step(self, tag: str) -> None:
        self._step_waiting = True
        self._step_advance_pending = False
        self._btn_advance.setEnabled(True)
        kind_label = {
            OP_MOVE: "movimento",
            OP_GRIPPER: "gripper",
            OP_WAIT: "WAIT",
            OP_WAIT_SM: "WAIT estado",
            OP_WAIT_SIG: "WAIT sinal",
        }.get(tag, tag)
        self._lbl_status.setText(f"Pausado no {kind_label}. Clique em Avançar.")

    def _on_advance_clicked(self) -> None:
        if not self._running or not self._step_waiting:
            return
        self._step_waiting = False
        self._step_advance_pending = True
        self._btn_advance.setEnabled(False)
        self._vm_step()

    def _vm_advance(self) -> None:
        if not self._audit_finish_step("success"):
            self._finish(
                ok=False,
                message=f"Falha ao registrar fim da etapa: {self._audit_log.last_error}",
                result="failure",
            )
            return
        self._vm_pc += 1
        self._vm_step()

    # ---- async dispatchers ----
    def _dispatch_move(self, op, total: int) -> None:
        pose_dict, raw_dt, step_id = op[1], op[2], op[3]
        dt = scale_duration(raw_dt, self._speed_scale)
        self._bridge.safety_set_step(step_id)
        self._vm_seq += 1
        tag = f"wp{self._vm_seq}"
        self._vm_pending_tag = tag
        self._lbl_status.setText(f"Movendo (passo {self._vm_pc + 1}/{total})")
        self._lbl_progress.setText(f"{self._vm_pc + 1}/{total}")
        if self._dry_run:
            delay_ms = int(min(max(0.05, dt), 1.0) * 1000)
            self._lbl_status.setText(f"[SIMULAÇÃO] Movendo (passo {self._vm_pc + 1}/{total})")
            QTimer.singleShot(delay_ms, self._vm_advance)
            return
        if not self._bridge.move_to_pose_tagged(tag, _dict_to_pose(pose_dict), dt):
            # The bridge already emitted a failure result synchronously, which
            # routes through _on_pose_result and finishes the run; guard anyway.
            if self._running:
                self._finish(ok=False, message="Action server indisponível.")

    def _dispatch_gripper(self, op, total: int) -> None:
        position_mm, max_effort, _timeout, step_id = (
            float(op[1]),
            float(op[2]),
            float(op[3]),
            op[4],
        )
        self._bridge.safety_set_step(step_id)
        self._vm_pending_gripper = True
        self._wait_kind = "gripper"
        self._wait_deadline = (time.monotonic() + _timeout) if _timeout > 0 else None
        self._lbl_status.setText(f"Gripper {position_mm:.1f} mm (passo {self._vm_pc + 1}/{total})")
        self._lbl_progress.setText(f"{self._vm_pc + 1}/{total}")
        if self._dry_run:
            self._lbl_status.setText(f"[SIMULAÇÃO] Gripper {position_mm:.1f} mm (passo {self._vm_pc + 1}/{total})")
            QTimer.singleShot(200, self._vm_advance)
            return
        if not self._bridge.gripper_command(position_mm, max_effort):
            if self._running:
                self._vm_pending_gripper = False
                self._finish(ok=False, message="Action server do gripper indisponível.")
        elif self._wait_deadline is not None:
            self._wait_timer.start(POLL_INTERVAL_MS)

    def _dispatch_wait_time(self, op, total: int) -> None:
        secs, step_id = float(op[1]), op[2]
        self._bridge.safety_set_step(step_id)
        self._wait_kind = "time"
        self._lbl_status.setText(f"WAIT {secs:.2f}s (passo {self._vm_pc + 1}/{total})")
        self._lbl_progress.setText(f"{self._vm_pc + 1}/{total}")
        if self._dry_run:
            self._lbl_status.setText(f"[SIMULAÇÃO] WAIT {secs:.2f}s (passo {self._vm_pc + 1}/{total})")
            delay_ms = int(min(max(0.0, secs), 1.0) * 1000)
            QTimer.singleShot(delay_ms, self._vm_advance)
            return
        self._wait_timer.start(int(max(0.0, secs) * 1000))

    def _dispatch_wait_sm(self, op, total: int) -> None:
        state, timeout, step_id = op[1], float(op[2]), op[3]
        self._bridge.safety_set_step(step_id)
        self._wait_kind = "sm"
        self._wait_target_state = state
        self._wait_deadline = (time.monotonic() + timeout) if timeout > 0 else None
        self._lbl_status.setText(f"WAIT estado = {state} (passo {self._vm_pc + 1}/{total})")
        self._lbl_progress.setText(f"{self._vm_pc + 1}/{total}")
        if self._dry_run:
            self._lbl_status.setText(f"[SIMULAÇÃO] WAIT estado = {state} (passo {self._vm_pc + 1}/{total})")
            QTimer.singleShot(300, self._vm_advance)
            return
        self._wait_timer.start(POLL_INTERVAL_MS)

    def _dispatch_output(self, io_id: str, action: str) -> None:
        action_label = {"on": "LIGAR", "off": "DESLIGAR", "toggle": "ALTERNAR"}.get(action, action.upper())
        if self._dry_run:
            self._lbl_status.setText(f"[SIMULAÇÃO] OUTPUT {action_label} {io_id}")
            return
        if action == OUTPUT_TOGGLE:
            ios = self._bridge.dio_ios_snapshot()
            io = ios.get(io_id)
            cur = bool(getattr(io, "value", False)) if io is not None else False
            self._bridge.set_dio_output(io_id, not cur)
        elif action == OUTPUT_OFF:
            self._bridge.set_dio_output(io_id, False)
        else:
            self._bridge.set_dio_output(io_id, True)

    def _dispatch_wait_sig(self, op, total: int) -> None:
        io_id, value, edge, timeout, step_id = (
            str(op[1]), int(op[2]), str(op[3]), float(op[4]), op[5]
        )
        self._bridge.safety_set_step(step_id)
        self._wait_kind = "sig"
        self._wait_io_id = io_id
        self._wait_value = value
        self._wait_edge = edge
        self._wait_edge_last = self._bridge.signal_by_id(io_id)
        self._wait_deadline = (time.monotonic() + timeout) if timeout > 0 else None
        edge_txt = {"level": "nível", "rising": "subida", "falling": "descida"}.get(edge, edge)
        self._lbl_status.setText(
            f"WAIT {io_id} (borda {edge_txt}) (passo {self._vm_pc + 1}/{total})"
        )
        self._lbl_progress.setText(f"{self._vm_pc + 1}/{total}")
        if self._dry_run:
            self._lbl_status.setText(
                f"[SIMULAÇÃO] WAIT {io_id} (borda {edge_txt}) (passo {self._vm_pc + 1}/{total})"
            )
            QTimer.singleShot(300, self._vm_advance)
            return
        self._wait_timer.start(POLL_INTERVAL_MS)

    # ---- async completion ----
    def _on_pose_result(self, tag: str, success: bool, message: str) -> None:
        if tag != self._vm_pending_tag:
            return  # stale / unrelated
        self._vm_pending_tag = None
        if not self._running:
            return
        if success:
            self._vm_advance()
        else:
            self._finish(ok=False, message=f"Falha no passo {self._vm_pc + 1}: {message}")

    def _on_action_result(self, name: str, success: bool, message: str) -> None:
        if not self._running or not self._vm_pending_gripper or name != "gripper_command":
            return
        self._vm_pending_gripper = False
        if self._wait_timer.isActive() and self._wait_kind == "gripper":
            self._wait_timer.stop()
        if success:
            self._vm_advance()
        else:
            self._finish(ok=False, message=f"Falha no gripper no passo {self._vm_pc + 1}: {message}")

    def _on_joint_state(self, msg) -> None:
        """Cache latest joint state; record if a program is running.

        In dry-run mode we use synthetic samples instead, because the real arm
        (or mock) does not move and would produce a flat line in the plot.
        """
        if self._dry_run or not self._joint_recording_active or not msg.name:
            return
        # Use monotonic time relative to the first recorded sample.
        now = time.monotonic()
        if not self._joint_history:
            self._joint_recording_start = now
        t = now - getattr(self, "_joint_recording_start", now)
        self._joint_history.append((t, list(msg.name), list(msg.position)))

    def _on_dry_run_record_tick(self) -> None:
        """Generate dummy joint samples during dry-run so the plot works offline."""
        if not self._running or not self._dry_run or not self._joint_recording_active:
            return
        if self._step_waiting:
            return
        self._joint_history.append(self._make_dry_run_joint_sample())

    def _record_step_marker(self, step_id: str) -> None:
        """Store the start time of a program step for later plotting."""
        if not self._joint_recording_active:
            return
        t = time.monotonic() - self._joint_recording_start
        # Avoid duplicate markers at the exact same timestamp.
        if self._joint_history_markers and self._joint_history_markers[-1][0] == t:
            return
        self._joint_history_markers.append((t, step_id))

    def _make_dry_run_joint_sample(self) -> tuple[float, list[str], list[float]]:
        """Create a plausible synthetic joint sample for dry-run plotting."""
        now = time.monotonic()
        t = now - self._joint_recording_start
        names = [f"joint{i}" for i in range(1, 7)]
        # Use smooth sinusoidal trajectories so the plot looks like motion.
        # Higher frequencies and distinct offsets make each joint clearly
        # distinguishable and avoid flat-looking segments.
        positions = [
            0.55 * math.sin(1.5 * t + 0.0) + 0.10,
            0.65 * math.sin(1.8 * t + 0.7) - 0.15,
            0.50 * math.sin(2.1 * t + 1.4) + 0.25,
            0.75 * math.sin(1.6 * t + 2.1) - 0.10,
            0.40 * math.sin(2.4 * t + 2.8) + 0.20,
            0.85 * math.sin(1.9 * t + 3.5) - 0.25,
        ]
        return (t, names, positions)

    def _plot_joint_history(self) -> None:
        """Open a matplotlib window with joint angles over time."""
        if not self._joint_history:
            QMessageBox.information(self, "Sem dados", "Nenhuma amostra de juntas foi gravada.")
            return

        times = []
        series: dict[str, list[float]] = {}
        for t, names, positions in self._joint_history:
            times.append(t)
            for name, pos in zip(names, positions):
                if name == "gripper":
                    continue
                series.setdefault(name, []).append(float(pos))

        if not series:
            QMessageBox.information(self, "Sem dados", "Nenhuma junta do braço foi encontrada nas amostras.")
            return

        fig, ax = plt.subplots(figsize=(12, 6))
        for name, values in series.items():
            # Ensure all series have the same length as times (gripper may be absent in some samples).
            if len(values) == len(times):
                ax.plot(times, values, label=name, lw=1.5)

        # Vertical markers for step starts.
        if self._joint_history_markers:
            y_min, y_max = ax.get_ylim()
            y_range = y_max - y_min if y_max != y_min else 1.0
            for t, step_id in self._joint_history_markers:
                ax.axvline(t, color="gray", linestyle="--", linewidth=0.8, alpha=0.6)
                # Place label near the top, rotated for readability.
                ax.text(
                    t,
                    y_max - 0.02 * y_range,
                    step_id,
                    rotation=90,
                    va="top",
                    ha="right",
                    fontsize=7,
                    alpha=0.75,
                    color="#444444",
                )

        ax.set_xlabel("Tempo (s)")
        ax.set_ylabel("Posição (rad)")
        ax.set_title(f"Ângulos das juntas — {self._program.name}")
        ax.legend(loc="upper right")
        ax.grid(True)
        plt.tight_layout()
        plt.show()

    def _on_wait_poll(self) -> None:
        if not self._running:
            return

        if self._wait_kind == "time":
            # single-shot timer reached: handled here for uniformity
            self._wait_timer.stop()
            self._vm_advance()
            return

        if self._wait_kind == "sm":
            if self._bridge.state_machine == self._wait_target_state:
                self._wait_timer.stop()
                self._vm_advance()
                return
        elif self._wait_kind == "sig":
            cur = self._bridge.signal_by_id(self._wait_io_id)
            matched = False
            if self._wait_edge == EDGE_LEVEL:
                matched = cur == self._wait_value
            elif self._wait_edge == EDGE_RISING:
                matched = self._wait_edge_last == 0 and cur == 1
            elif self._wait_edge == EDGE_FALLING:
                matched = self._wait_edge_last == 1 and cur == 0
            self._wait_edge_last = cur
            if matched:
                self._wait_timer.stop()
                self._vm_advance()
                return

        # not yet satisfied: check timeout
        if self._wait_deadline is not None and time.monotonic() >= self._wait_deadline:
            self._wait_timer.stop()
            label = {
                "sm": f"WAIT_FOR_STATE_MACHINE ({self._wait_target_state})",
                "sig": f"WAIT_FOR_SIGNAL ({self._wait_io_id})",
                "gripper": "GRIPPER",
            }.get(self._wait_kind, "WAIT")
            self._finish(ok=False, message=f"Timeout em {label}.")

    def _stop(
        self,
        _checked: bool = False,
        *,
        reason: str = "Execução interrompida.",
        source: str = "operator",
    ) -> None:
        if not self._running:
            return
        self._audit_cancel(source, reason)
        self._running = False
        if self._wait_timer.isActive():
            self._wait_timer.stop()
        if not self._dry_run:
            self._bridge.cancel_active_actions({"move_to_pose", "gripper_command"})
        self._finish(ok=False, message=reason, result="canceled")

    def _finish(self, ok: bool, message: str, result: str | None = None) -> None:
        audit_result = result or ("success" if ok else "failure")
        audit_ok = self._audit_finish_run(audit_result, message)
        self._running = False
        self._vm_pending_tag = None
        self._vm_pending_gripper = False
        self._wait_timer.stop()
        self._step_waiting = False
        self._step_advance_pending = False
        self._joint_recording_active = False
        self._dry_run_record_timer.stop()
        self._btn_run.setEnabled(True)
        self._btn_stop.setEnabled(False)
        self._btn_advance.setEnabled(False)
        self._editor.set_edit_enabled(True)
        self._editor.clear_highlight()
        self._flow_widget.set_edit_enabled(True)
        self._flow_widget.set_active_root_index(None)
        self._progress.setVisible(False)
        self._progress.setValue(0)
        self._chk_dry_run.setEnabled(True)
        self._chk_step.setEnabled(True)
        self._act_record_joints.setEnabled(True)
        self._act_preload.setEnabled(True)
        self._update_undo_buttons()
        if audit_ok:
            self._lbl_status.setText(message)
            self._lbl_progress.setText("OK" if ok else "PARADO")
        else:
            self._lbl_status.setText(
                f"{message} ALERTA: registro de auditoria incompleto "
                f"({self._audit_log.last_error})."
            )
            self._lbl_progress.setText("AUDITORIA!")
        self._btn_plot_joints.setEnabled(bool(self._joint_history))
        self._act_plot_joints.setEnabled(bool(self._joint_history))
        self.set_robot_ready(self._robot_ready)

    def _update_run_availability(self) -> None:
        if self._running:
            self._btn_run.setEnabled(False)
            self._btn_run_selected.setEnabled(False)
            self._btn_preview.setEnabled(False)
            return
        dry_run_available = bool(self._dry_run and self._bridge.is_connected)
        available = bool(self._robot_ready or dry_run_available)
        self._btn_run.setEnabled(available)
        self._btn_run_selected.setEnabled(
            available and self._editor.selected_index() >= 0
        )
        self._btn_preview.setEnabled(bool(self._program.steps))

    def set_robot_ready(self, ready: bool) -> None:
        """Update motion/teach controls from the shared readiness policy."""
        self._robot_ready = bool(ready)
        self._btn_freedrive.setEnabled(self._robot_ready and not self._running)
        self._btn_teach.setEnabled(self._robot_ready and not self._running)
        self._update_run_availability()

    def handle_robot_unavailable(self, message: str) -> None:
        """Fail closed when a real execution loses its controller heartbeat."""
        self.set_robot_ready(False)
        self._set_freedrive_checked(False)
        if self._running and not self._dry_run:
            self._stop(reason=str(message), source="heartbeat")

    def stop_for_software_stop(self) -> None:
        """Stop an active program and identify the global software-stop source."""
        self._stop(
            reason="Execução interrompida pela parada de software.",
            source="software_stop",
        )

    # ==================================================================
    # Persistence
    # ==================================================================
    def _new_program(self) -> None:
        if self._running:
            return
        if self._program.steps and not self._confirm("Limpar o programa atual?"):
            return
        self._program = Program(name="programa")
        self._undo_stack.clear()
        self._redo_stack.clear()
        self._refresh_after_load()
        self._update_undo_buttons()
        self._lbl_status.setText("Novo programa.")
        self._set_modified(False)

    def load_program(self, program: Program, status: str = "Programa carregado.") -> None:
        if self._running:
            return
        self._program = program
        self._undo_stack.clear()
        self._redo_stack.clear()
        self._refresh_after_load()
        self._update_undo_buttons()
        self._lbl_status.setText(status)
        self.program_loaded.emit()
        self._set_modified(False)

    def _save_dialog(self) -> None:
        if self._running:
            return
        start_dir = self._last_dir()
        path, _ = QFileDialog.getSaveFileName(
            self, "Salvar programa de movimento", start_dir, "Programas JSON (*.json)"
        )
        if not path:
            return
        path = str(Path(path).with_suffix(".json"))
        try:
            self._save_to_file(path)
        except Exception as exc:
            QMessageBox.critical(self, "Erro ao salvar", str(exc))

    def _load_dialog(self) -> None:
        if self._running:
            return
        if self._is_modified and not self._confirm("Há alterações não salvas. Deseja descartá-las?"):
            return
        start_dir = self._last_dir()
        path, _ = QFileDialog.getOpenFileName(
            self, "Carregar programa de movimento", start_dir, "Programas JSON (*.json)"
        )
        if not path:
            return
        try:
            self._load_from_file(path)
            self._remember_dir(path)
        except Exception as exc:
            QMessageBox.critical(self, "Erro ao carregar", str(exc))

    def _save_preload(self) -> None:
        if self._running:
            return
        try:
            self._config_dir_ensure()
            self._save_to_file(str(_preload_path()))
            self._lbl_status.setText(f"Pré-carga definida: {_preload_path()}")
        except Exception as exc:
            QMessageBox.critical(self, "Erro ao definir pré-carga", str(exc))

    def _config_dir_ensure(self) -> None:
        d = _config_dir()
        d.mkdir(parents=True, exist_ok=True)

    def _save_to_file(self, path: str) -> None:
        # make sure editors committed their latest values
        self._sync_editors_to_program()
        Path(path).write_text(self._program.to_json(), encoding="utf-8")
        self._lbl_status.setText(f"Salvo em {path}")
        self._set_modified(False)

    def _sync_editors_to_program(self) -> None:
        # The StepListEditor mutates program.steps in place; speed_scale/name
        # are kept in sync via their change handlers. Nothing extra to do, but
        # this hook documents the intent and is a safe place to extend.
        self._program.name = self._name_edit.text().strip() or "programa"
        self._program.speed_scale = float(self._speed_spinbox.value())

    def _load_from_file(self, path) -> None:
        text = Path(path).read_text(encoding="utf-8")
        program = Program.from_json(text)
        self._program = program
        self._undo_stack.clear()
        self._redo_stack.clear()
        self._refresh_after_load()
        self._update_undo_buttons()
        self._lbl_status.setText(f"Carregado: {path}")
        self.program_loaded.emit()
        self._set_modified(False)

    def _confirm(self, text: str) -> bool:
        return QMessageBox.question(self, "Confirmar", text) == QMessageBox.Yes

    # ---- remember last used directory via QSettings ----
    def _settings(self):
        from PySide6.QtCore import QSettings
        return QSettings("reBotArm", "rebotarm_gui")

    def _last_dir(self) -> str:
        return self._settings().value("motion/lastDir", str(Path.home()))

    def _remember_dir(self, path: str) -> None:
        self._settings().setValue("motion/lastDir", str(Path(path).parent))
