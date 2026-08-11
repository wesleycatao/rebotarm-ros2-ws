"""Composed MotionTab — brings together the editor, VM, preflight and teach mixins.

This module is the single entry point for the motion tab.  The bulk of the
logic lives in dedicated mixin modules (``_ui``, ``_run``, ``_preflight``,
``_teach``); here we only define the class skeleton, signals, ``__init__``
and bridge wiring.
"""

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
from ._ui import MotionTabUiMixin
from ._run import MotionTabRunMixin
from ._preflight import MotionTabPreflightMixin
from ._teach import MotionTabTeachMixin


class MotionTab(
    QWidget,
    MotionTabUiMixin,
    MotionTabRunMixin,
    MotionTabPreflightMixin,
    MotionTabTeachMixin,
):
    """Tab to design, run and persist motion programs."""

    program_loaded = Signal()
    # (message, kind) transient confirmation surfaced by the shell's toast.
    toast_requested = Signal(str, str)

    def __init__(
        self,
        bridge,
        parent: QWidget | None = None,
        *,
        audit_log: ExecutionAuditLog | None = None,
        operator_provider: Callable[[], str] | None = None,
        recovery_store: RecoveryStore | None = None,
        cell_scene_store: CellSceneStore | None = None,
        production_prerequisite_provider: (
            Callable[[], tuple[bool, str, dict[str, object]]] | None
        ) = None,
    ) -> None:
        super().__init__(parent)
        self._bridge = bridge
        self._audit_log = audit_log or ExecutionAuditLog()
        self._operator_provider = operator_provider or self._default_operator
        self._recovery_store = recovery_store or RecoveryStore()
        self._cell_scene_store = cell_scene_store
        self._production_prerequisite_provider = (
            production_prerequisite_provider
        )
        self._program = Program(name="programa")
        self._runtime_dynamic_ceiling = 1.0
        self._current_path = ""
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
        self._pending_safety_dispatch: tuple[tuple, int, str] | None = None
        self._vm_meta: dict[str, Any] = {}
        self._vm_action_count = 0
        self._vm_action_seen = 0
        self._speed_scale = 1.0
        self._is_modified = False
        self._last_preflight: Optional[PreflightReport] = None
        self._preflight_in_progress = False
        self._production_mode = False
        self._collision_planning_required = False
        self._paused = False
        self._pause_requested = False
        self._recovery_required = False
        self._recovery_reason = ""
        self._paused_wait_remaining: float | None = None
        self._persisted_checkpoint = self._recovery_store.load()

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
        self._refresh_recent_menu()

        preload = _preload_path()
        if preload.is_file():
            try:
                self._load_from_file(preload)
            except Exception as exc:
                self._lbl_status.setText(f"Erro ao carregar pré-carga: {exc}")

    def _wire_bridge(self) -> None:
        self._bridge.pose_action_result.connect(self._on_pose_result)
        self._bridge.action_result.connect(self._on_action_result)
        self._bridge.joint_state_changed.connect(self._on_joint_state)
        self._bridge.service_result.connect(self._on_teach_service_result)
        self._bridge.service_result.connect(self._on_safety_step_result)
        self._bridge.speed_override_changed.connect(self._on_global_speed_changed)
