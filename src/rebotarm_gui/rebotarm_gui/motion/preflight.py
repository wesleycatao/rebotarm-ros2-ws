"""Pre-flight validation for motion programs.

Runs a battery of checks over a :class:`~rebotarm_gui.motion.program.Program`
and the live ROS state *before* execution, so the operator gets a single
diagnostic view instead of discovering problems mid-run.

Checks are split into two groups:

* **Synchronous checks** (instant, run in the UI thread): program structure,
  bridge/arm readiness, action-server discovery, velocities, condition
  well-formedness, gripper ranges, I/O references and wait-for-state names.
* **Reachability** (asynchronous, via the ``move_to_pose_ik`` service): each
  unique Cartesian target is solved and, when a solution is returned, the
  joint angles are checked against the URDF position limits.

A report is a list of :class:`CheckItem`. Errors block real execution;
warnings do not. :func:`run_preflight` returns the full report and, when
``with_reachability`` is set, waits on the IK results through a nested Qt
event loop bounded by an overall timeout (so a missing/slow IK service never
hangs the UI).

This is a client-side diagnostic aid only: it does not replace the
controller-side safety validation or a risk assessment of the cell.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, List, Optional

from PySide6.QtCore import QObject, QTimer, Signal

from .conditions import (
    COMBINER_ALL,
    COMBINER_ANY,
    OPS_NUMERIC,
    OPS_STRING,
    SRC_GRIPPER_POSITION,
    SRC_JOINT_POSITION,
    SRC_SIGNAL,
    SRC_STATE_MACHINE,
    SRC_VISION_DETECTED,
    SRC_VISION_POSE,
    VALID_STATE_MACHINES,
    ConditionExpr,
)
from .program import (
    ACTION_OPS,
    OP_GOTO,
    OP_MOVE,
    ArcStep,
    GripperStep,
    IfStep,
    LineStep,
    LoopStep,
    MoveStep,
    OutputStep,
    PalletStep,
    Program,
    Step,
    WaitForSignalStep,
    WaitForStateMachineStep,
    build_label_index,
)
from .vision_envelope import envelope_spec, envelope_variants

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ..ros_bridge import RosBridge


# Severities ------------------------------------------------------------------
SEV_ERROR = "error"
SEV_WARNING = "warning"
SEV_INFO = "info"

# Statuses --------------------------------------------------------------------
STATUS_PASS = "pass"
STATUS_FAIL = "fail"
STATUS_SKIP = "skip"
STATUS_INFO = "info"

# Category labels (PT) --------------------------------------------------------
CAT_STRUCTURE = "Estrutura"
CAT_ARM = "Braço"
CAT_ACTION_SERVERS = "Action servers"
CAT_VELOCITIES = "Velocidades"
CAT_CONDITIONS = "Condições"
CAT_GRIPPER = "Gripper"
CAT_IO = "I/O digital"
CAT_WAIT_STATE = "Espera de estado"
CAT_REACHABILITY = "Alcance"
CAT_VISION_ENVELOPE = "Envelope da visão"

# Sanity ranges ---------------------------------------------------------------
SPEED_SCALE_MIN = 0.05
SPEED_SCALE_MAX = 4.0
GRIPPER_POS_MIN_MM = 0.0
GRIPPER_POS_MAX_MM = 85.0
GRIPPER_EFFORT_MAX = 100.0
MOVE_DURATION_MIN_S = 0.05
LINE_SPEED_MIN_MS = 0.001
# Cap the number of IK probes so very large pallet programs stay responsive.
REACHABILITY_MAX_PROBES = 120
REACHABILITY_DEFAULT_TIMEOUT_MS = 4000

# Sources that require specific params to be present.
_COND_REQUIRED_PARAMS = {
    SRC_SIGNAL: ("io_id",),
    SRC_JOINT_POSITION: ("joint",),
    SRC_VISION_POSE: ("axis",),
}
_VALID_SOURCES = {
    SRC_SIGNAL,
    SRC_STATE_MACHINE,
    SRC_JOINT_POSITION,
    SRC_GRIPPER_POSITION,
    SRC_VISION_DETECTED,
    SRC_VISION_POSE,
}
_VALID_EDGES = {"level", "rising", "falling"}

# Conservative Cartesian workspace envelope used as a quick sanity filter
# before the IK service is consulted. Poses outside this box are flagged as
# warnings (not hard errors): the real authority is the IK solver.
_WORKSPACE_X = (0.0, 1.0)
_WORKSPACE_Y = (-0.8, 0.8)
_WORKSPACE_Z = (-0.4, 1.0)


@dataclass
class CheckItem:
    """One preflight result row."""

    category: str
    name: str
    severity: str  # one of SEV_*
    status: str    # one of STATUS_*
    message: str
    detail: str = ""
    step_index: Optional[int] = None

    @property
    def is_error(self) -> bool:
        return self.status == STATUS_FAIL and self.severity == SEV_ERROR

    @property
    def is_warning(self) -> bool:
        return self.severity == SEV_WARNING and self.status in (STATUS_FAIL, STATUS_INFO)


@dataclass
class PreflightReport:
    """Aggregate preflight result."""

    items: List[CheckItem] = field(default_factory=list)
    reachability_done: bool = False
    reachability_skipped_reason: str = ""

    @property
    def errors(self) -> List[CheckItem]:
        return [i for i in self.items if i.is_error]

    @property
    def warnings(self) -> List[CheckItem]:
        return [i for i in self.items if i.is_warning]

    @property
    def ok(self) -> bool:
        """True when there are no blocking errors."""
        return not self.errors

    def add(self, item: CheckItem) -> None:
        self.items.append(item)

    def summary(self) -> str:
        n_err = len(self.errors)
        n_warn = len(self.warnings)
        if n_err:
            return f"{n_err} erro(s), {n_warn} aviso(s) — execução real bloqueada"
        if n_warn:
            return f"OK com {n_warn} aviso(s)"
        return "OK — nenhuma pendência"


# ----------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------
def _walk_steps(steps: List[Step]):
    """Yield ``(index, step)`` for every step, recursing into compound steps."""
    for idx, step in enumerate(steps):
        yield idx, step
        if isinstance(step, LoopStep):
            yield from _walk_steps(step.steps)
        elif isinstance(step, IfStep):
            yield from _walk_steps(step.then_steps)
            yield from _walk_steps(step.else_steps)


def _round_pose(pose_dict: dict, ndigits: int = 3) -> tuple:
    """Hashable rounded pose key for de-duplication."""
    return tuple(round(float(pose_dict.get(k, 0.0)), ndigits) for k in ("x", "y", "z", "roll", "pitch", "yaw"))


def _pose_in_workspace(pose_dict: dict) -> bool:
    x = float(pose_dict.get("x", 0.0))
    y = float(pose_dict.get("y", 0.0))
    z = float(pose_dict.get("z", 0.0))
    return (_WORKSPACE_X[0] <= x <= _WORKSPACE_X[1]
            and _WORKSPACE_Y[0] <= y <= _WORKSPACE_Y[1]
            and _WORKSPACE_Z[0] <= z <= _WORKSPACE_Z[1])


def _collect_move_poses(program: Program) -> List[tuple]:
    """Unique targets across nominal or vision-envelope program variants."""
    try:
        variants = envelope_variants(program)
    except Exception:
        return []
    seen: set[tuple] = set()
    ordered: list[tuple] = []
    for _label, variant in variants:
        for op in variant.flatten():
            if op and op[0] == OP_MOVE:
                key = _round_pose(op[1])
                if key not in seen:
                    seen.add(key)
                    ordered.append(key)
    return ordered


def _check_vision_envelope(
    report: PreflightReport,
    program: Program,
    *,
    require_certified: bool,
) -> None:
    guided = bool(program.parameters.get("_vision_guided", False))
    try:
        spec = envelope_spec(program)
        if spec is None:
            if guided:
                raise ValueError("receita guiada por visão sem envelope declarado")
            report.add(
                CheckItem(
                    CAT_VISION_ENVELOPE,
                    "Aplicabilidade",
                    SEV_INFO,
                    STATUS_SKIP,
                    "Receita sem alvos variáveis pela visão.",
                )
            )
            return
        if require_certified:
            spec.validate_for_production()
        samples = spec.sample_positions()
        if not samples:
            raise ValueError("envelope não produziu amostras")
        # Building the first variant also proves that every declared label
        # still resolves to a Cartesian step after recipe edits.
        envelope_variants(program)
    except (TypeError, ValueError) as exc:
        severity = SEV_ERROR if require_certified else SEV_WARNING
        report.add(
            CheckItem(
                CAT_VISION_ENVELOPE,
                "Contrato",
                severity,
                STATUS_FAIL,
                f"Envelope da visão inválido: {exc}.",
            )
        )
        return
    ux, uy, uz = spec.effective_uncertainty_m
    report.add(
        CheckItem(
            CAT_VISION_ENVELOPE,
            "Amostragem robusta",
            SEV_INFO,
            STATUS_PASS,
            f"{len(samples)} posição(ões) críticas serão verificadas; "
            f"incerteza efetiva ±({ux * 1000.0:.1f}, "
            f"{uy * 1000.0:.1f}, {uz * 1000.0:.1f}) mm.",
            detail=f"assinatura {spec.signature()[:12]}",
        )
    )


# ----------------------------------------------------------------------
# synchronous checks
# ----------------------------------------------------------------------
def _check_structure(report: PreflightReport, program: Program) -> None:
    if not program.steps:
        report.add(CheckItem(
            CAT_STRUCTURE, "Etapas", SEV_ERROR, STATUS_FAIL,
            "Programa vazio: adicione etapas antes de executar.",
        ))
        return
    try:
        ops = program.flatten()
    except Exception as exc:
        report.add(CheckItem(
            CAT_STRUCTURE, "Montagem", SEV_ERROR, STATUS_FAIL,
            f"Falha ao montar opcodes: {exc}",
        ))
        return
    report.add(CheckItem(
        CAT_STRUCTURE, "Montagem", SEV_INFO, STATUS_PASS,
        f"Programa monta em {len(ops)} opcode(s).",
    ))
    action_count = sum(1 for op in ops if op and op[0] in ACTION_OPS)
    if action_count == 0:
        report.add(CheckItem(
            CAT_STRUCTURE, "Ações", SEV_ERROR, STATUS_FAIL,
            "O programa não gera nenhum comando de movimento/gripper/I/O.",
        ))
    else:
        report.add(CheckItem(
            CAT_STRUCTURE, "Ações", SEV_INFO, STATUS_PASS,
            f"{action_count} comando(s) de ação serão executados.",
        ))
    # label/goto integrity
    try:
        labels = build_label_index(ops)
    except Exception as exc:
        report.add(CheckItem(
            CAT_STRUCTURE, "Rótulos", SEV_ERROR, STATUS_FAIL,
            f"Erro ao indexar rótulos: {exc}",
        ))
        labels = {}
    missing = [op[1] for op in ops if op and op[0] == OP_GOTO and op[1] not in labels]
    if missing:
        report.add(CheckItem(
            CAT_STRUCTURE, "Rótulos", SEV_ERROR, STATUS_FAIL,
            f"Goto sem alvo: {', '.join(sorted(set(missing)))}",
        ))


def _check_arm_state(report: PreflightReport, bridge: "RosBridge", for_real_run: bool) -> None:
    if not for_real_run:
        report.add(CheckItem(
            CAT_ARM, "Modo simulação", SEV_INFO, STATUS_INFO,
            "Modo simulação: prontidão do braço não é exigida.",
        ))
        if not bridge.is_connected:
            report.add(CheckItem(
                CAT_ARM, "Conexão ROS", SEV_ERROR, STATUS_FAIL,
                "Simulação ainda exige conexão com o ROS 2 (avaliação de condições/sinais).",
            ))
        return
    ok, reason = bridge.motion_readiness("move_to_pose")
    if ok:
        report.add(CheckItem(
            CAT_ARM, "Prontidão", SEV_INFO, STATUS_PASS,
            "Braço conectado, habilitado e fora de estado protetivo.",
        ))
    else:
        report.add(CheckItem(CAT_ARM, "Prontidão", SEV_ERROR, STATUS_FAIL, reason))


def _check_action_servers(
    report: PreflightReport, program: Program, bridge: "RosBridge", for_real_run: bool
) -> None:
    try:
        ops = program.flatten()
    except Exception:
        return
    kinds = {op[0] for op in ops if op}
    needs_move = OP_MOVE in kinds
    # Gripper opcodes use the OP tag "gripper"
    needs_gripper = any(op and op[0] == "gripper" for op in ops)
    if not (needs_move or needs_gripper):
        return
    severity = SEV_ERROR if for_real_run else SEV_WARNING
    if needs_move:
        ready = bridge.action_server_ready("move_to_pose")
        report.add(CheckItem(
            CAT_ACTION_SERVERS, "move_to_pose", severity,
            STATUS_PASS if ready else STATUS_FAIL,
            "Action server move_to_pose descoberto." if ready
            else "Action server move_to_pose indisponível.",
        ))
    if needs_gripper:
        ready = bridge.action_server_ready("gripper_command")
        report.add(CheckItem(
            CAT_ACTION_SERVERS, "gripper_command", severity,
            STATUS_PASS if ready else STATUS_FAIL,
            "Action server gripper_command descoberto." if ready
            else "Action server gripper_command indisponível.",
        ))


def _check_velocities(report: PreflightReport, program: Program) -> None:
    scale = float(getattr(program, "speed_scale", 1.0))
    if scale <= 0.0:
        report.add(CheckItem(
            CAT_VELOCITIES, "Escala global", SEV_ERROR, STATUS_FAIL,
            f"Escala de velocidade inválida ({scale}). Deve ser > 0.",
        ))
    elif scale < SPEED_SCALE_MIN or scale > SPEED_SCALE_MAX:
        report.add(CheckItem(
            CAT_VELOCITIES, "Escala global", SEV_WARNING, STATUS_FAIL,
            f"Escala de velocidade {scale:.2f}x fora da faixa recomendada "
            f"({SPEED_SCALE_MIN}–{SPEED_SCALE_MAX}x).",
        ))
    else:
        report.add(CheckItem(
            CAT_VELOCITIES, "Escala global", SEV_INFO, STATUS_PASS,
            f"Escala de velocidade {scale:.2f}x.",
        ))

    for idx, step in _walk_steps(program.steps):
        if isinstance(step, MoveStep):
            if float(step.duration) < MOVE_DURATION_MIN_S:
                report.add(CheckItem(
                    CAT_VELOCITIES, "Duração do movimento", SEV_ERROR, STATUS_FAIL,
                    f"Duração {step.duration:.3f}s abaixo do mínimo ({MOVE_DURATION_MIN_S}s).",
                    step_index=idx,
                ))
        elif isinstance(step, LineStep):
            if float(step.speed) < LINE_SPEED_MIN_MS:
                report.add(CheckItem(
                    CAT_VELOCITIES, "Velocidade de linha", SEV_ERROR, STATUS_FAIL,
                    f"Velocidade {step.speed:.4f} m/s muito baixa.",
                    step_index=idx,
                ))
        elif isinstance(step, ArcStep):
            if float(step.speed) < LINE_SPEED_MIN_MS:
                report.add(CheckItem(
                    CAT_VELOCITIES, "Velocidade do arco", SEV_ERROR, STATUS_FAIL,
                    f"Velocidade {step.speed:.4f} m/s muito baixa.",
                    step_index=idx,
                ))
        elif isinstance(step, PalletStep):
            if float(step.speed) < LINE_SPEED_MIN_MS:
                report.add(CheckItem(
                    CAT_VELOCITIES, "Velocidade da paletização", SEV_ERROR, STATUS_FAIL,
                    f"Velocidade {step.speed:.4f} m/s muito baixa.",
                    step_index=idx,
                ))


def _validate_condition(report: PreflightReport, expr: ConditionExpr, idx: int) -> None:
    if expr.combiner not in (COMBINER_ALL, COMBINER_ANY):
        report.add(CheckItem(
            CAT_CONDITIONS, "Combinador", SEV_ERROR, STATUS_FAIL,
            f"Combinador inválido: {expr.combiner!r}.",
            step_index=idx,
        ))
    if not expr.items:
        return  # empty condition is treated as always-true; not malformed
    for pos, cond in enumerate(expr.items):
        prefix = f"Condição {pos + 1}"
        if cond.source not in _VALID_SOURCES:
            report.add(CheckItem(
                CAT_CONDITIONS, prefix, SEV_ERROR, STATUS_FAIL,
                f"Fonte desconhecida: {cond.source!r}.",
                step_index=idx,
            ))
            continue
        required = _COND_REQUIRED_PARAMS.get(cond.source, ())
        for param in required:
            val = cond.params.get(param)
            if val in (None, ""):
                report.add(CheckItem(
                    CAT_CONDITIONS, prefix, SEV_ERROR, STATUS_FAIL,
                    f"Fonte {cond.source!r} exige parâmetro {param!r}.",
                    step_index=idx,
                ))
        allowed_ops = OPS_STRING if cond.value_type == "string" else OPS_NUMERIC
        if cond.op not in allowed_ops:
            report.add(CheckItem(
                CAT_CONDITIONS, prefix, SEV_ERROR, STATUS_FAIL,
                f"Operador {cond.op!r} inválido para valor {cond.value_type!r}.",
                step_index=idx,
            ))
        if cond.value is None:
            report.add(CheckItem(
                CAT_CONDITIONS, prefix, SEV_ERROR, STATUS_FAIL,
                "Condição sem valor de comparação.",
                step_index=idx,
            ))


def _check_conditions(report: PreflightReport, program: Program) -> None:
    has_if = False
    for idx, step in _walk_steps(program.steps):
        if isinstance(step, IfStep):
            has_if = True
            _validate_condition(report, step.condition, idx)
    if has_if:
        all_ok = not any(i.category == CAT_CONDITIONS and i.is_error for i in report.items)
        if all_ok:
            report.add(CheckItem(
                CAT_CONDITIONS, "Estrutura", SEV_INFO, STATUS_PASS,
                "Todas as condições IF estão bem formadas.",
            ))


def _check_gripper(report: PreflightReport, program: Program) -> None:
    found = False
    for idx, step in _walk_steps(program.steps):
        if not isinstance(step, GripperStep):
            continue
        found = True
        pos = float(step.position_mm)
        if pos < GRIPPER_POS_MIN_MM or pos > GRIPPER_POS_MAX_MM:
            report.add(CheckItem(
                CAT_GRIPPER, "Posição", SEV_WARNING, STATUS_FAIL,
                f"Posição {pos:.1f} mm fora da faixa "
                f"{GRIPPER_POS_MIN_MM}–{GRIPPER_POS_MAX_MM} mm.",
                step_index=idx,
            ))
        eff = float(step.max_effort)
        if eff < 0.0 or eff > GRIPPER_EFFORT_MAX:
            report.add(CheckItem(
                CAT_GRIPPER, "Esforço", SEV_WARNING, STATUS_FAIL,
                f"Esforço {eff:.1f} fora da faixa 0–{GRIPPER_EFFORT_MAX}.",
                step_index=idx,
            ))
        if float(step.timeout) < 0.0:
            report.add(CheckItem(
                CAT_GRIPPER, "Timeout", SEV_ERROR, STATUS_FAIL,
                f"Timeout {step.timeout:.2f}s negativo.",
                step_index=idx,
            ))
    if found and not any(i.category == CAT_GRIPPER for i in report.items if i.is_error or i.is_warning):
        report.add(CheckItem(
            CAT_GRIPPER, "Faixas", SEV_INFO, STATUS_PASS,
            "Comandos de gripper dentro das faixas esperadas.",
        ))


def _check_io(report: PreflightReport, program: Program, bridge: "RosBridge") -> None:
    input_ids = set(bridge.dio_input_ids())
    output_ids = set(bridge.dio_output_ids())
    found = False
    for idx, step in _walk_steps(program.steps):
        if isinstance(step, OutputStep):
            found = True
            if output_ids and step.io_id not in output_ids:
                report.add(CheckItem(
                    CAT_IO, f"Saída {step.io_id}", SEV_WARNING, STATUS_FAIL,
                    f"Saída {step.io_id} não está entre as conhecidas: "
                    f"{', '.join(sorted(output_ids))}.",
                    step_index=idx,
                ))
            if step.action not in ("on", "off", "toggle"):
                report.add(CheckItem(
                    CAT_IO, f"Ação {step.io_id}", SEV_ERROR, STATUS_FAIL,
                    f"Ação de saída inválida: {step.action!r}.",
                    step_index=idx,
                ))
        elif isinstance(step, WaitForSignalStep):
            found = True
            if input_ids and step.io_id not in input_ids:
                report.add(CheckItem(
                    CAT_IO, f"Entrada {step.io_id}", SEV_WARNING, STATUS_FAIL,
                    f"Entrada {step.io_id} não está entre as conhecidas: "
                    f"{', '.join(sorted(input_ids))}.",
                    step_index=idx,
                ))
            if step.edge not in _VALID_EDGES:
                report.add(CheckItem(
                    CAT_IO, f"Borda {step.io_id}", SEV_ERROR, STATUS_FAIL,
                    f"Borda inválida: {step.edge!r}.",
                    step_index=idx,
                ))
            if int(step.value) not in (0, 1):
                report.add(CheckItem(
                    CAT_IO, f"Valor {step.io_id}", SEV_ERROR, STATUS_FAIL,
                    f"Valor de entrada inválido: {step.value}.",
                    step_index=idx,
                ))
    if found and not any(i.category == CAT_IO for i in report.items if i.is_error or i.is_warning):
        report.add(CheckItem(
            CAT_IO, "Referências", SEV_INFO, STATUS_PASS,
            "Referências de I/O digitais válidas.",
        ))


def _check_wait_state(report: PreflightReport, program: Program) -> None:
    found = False
    for idx, step in _walk_steps(program.steps):
        if isinstance(step, WaitForStateMachineStep):
            found = True
            if step.state.upper() not in {s.upper() for s in VALID_STATE_MACHINES}:
                report.add(CheckItem(
                    CAT_WAIT_STATE, f"Estado {step.state}", SEV_WARNING, STATUS_FAIL,
                    f"Estado {step.state!r} não é um estado de máquina reconhecido "
                    f"({', '.join(VALID_STATE_MACHINES)}).",
                    step_index=idx,
                ))
            if float(step.timeout) < 0.0:
                report.add(CheckItem(
                    CAT_WAIT_STATE, f"Timeout {step.state}", SEV_ERROR, STATUS_FAIL,
                    f"Timeout {step.timeout:.2f}s negativo.",
                    step_index=idx,
                ))
    if found and not any(i.category == CAT_WAIT_STATE for i in report.items if i.is_error or i.is_warning):
        report.add(CheckItem(
            CAT_WAIT_STATE, "Estados", SEV_INFO, STATUS_PASS,
            "Estados de máquina referenciados são válidos.",
        ))


def run_sync_checks(
    program: Program,
    bridge: "RosBridge",
    *,
    for_real_run: bool,
    require_certified_envelope: bool = False,
) -> PreflightReport:
    """Run every synchronous preflight check and return the report.

    ``for_real_run`` tunes severities: a real (non dry-run) execution treats a
    missing action server or an unready arm as a hard error, while dry-run
    downgrades them to warnings so the simulation can still proceed.
    """
    report = PreflightReport()
    _check_structure(report, program)
    if report.errors:
        # Structural failures make the remaining checks misleading.
        return report
    _check_vision_envelope(
        report,
        program,
        require_certified=bool(require_certified_envelope),
    )
    _check_arm_state(report, bridge, for_real_run)
    _check_action_servers(report, program, bridge, for_real_run)
    _check_velocities(report, program)
    _check_conditions(report, program)
    _check_gripper(report, program)
    _check_io(report, program, bridge)
    _check_wait_state(report, program)
    return report


# ----------------------------------------------------------------------
# reachability (async, aggregated)
# ----------------------------------------------------------------------
class ReachabilityProbe(QObject):
    """Probe unique Cartesian targets via ``move_to_pose_ik``.

    Emits :attr:`finished` once every probe has either returned, failed to be
    dispatched or the overall :class:`QTimer` deadline has elapsed. The
    resulting :class:`list` of :class:`CheckItem` is appended to ``report``.
    """

    finished = Signal(object)  # PreflightReport
    progress = Signal(int, int)  # (done, total) reachability probes resolved

    def __init__(
        self,
        program: Program,
        bridge: "RosBridge",
        report: PreflightReport,
        *,
        timeout_ms: int = REACHABILITY_DEFAULT_TIMEOUT_MS,
        max_probes: int = REACHABILITY_MAX_PROBES,
        parent: Optional[QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._program = program
        self._bridge = bridge
        self._report = report
        self._timeout_ms = int(timeout_ms)
        self._max_probes = int(max_probes)
        self._pending: dict[str, tuple] = {}  # key -> rounded pose
        self._results: List[CheckItem] = []
        self._total = 0
        self._deadline_timer = QTimer(self)
        self._deadline_timer.setSingleShot(True)
        self._deadline_timer.timeout.connect(self._on_deadline)
        self._dispatched = False
        self._finished_flag = False

    def start(self) -> None:
        """Fire IK probes and arm the deadline timer."""
        if self._dispatched:
            return
        self._dispatched = True
        keys = _collect_move_poses(self._program)
        if not keys:
            self._report.reachability_done = True
            self._report.add(CheckItem(
                CAT_REACHABILITY, "Alvos", SEV_INFO, STATUS_SKIP,
                "Programa sem alvos cartesianos para verificar.",
            ))
            self.finished.emit(self._report)
            return
        if len(keys) > self._max_probes:
            step = max(1, len(keys) // self._max_probes)
            sampled = keys[::step][: self._max_probes]
            self._report.add(CheckItem(
                CAT_REACHABILITY, "Amostragem", SEV_INFO, STATUS_INFO,
                f"{len(keys)} alvo(s) único(s); amostrando {len(sampled)} para IK.",
            ))
            keys = sampled
        # Workspace sanity pass -> surface obvious outliers as warnings up front.
        for key in keys:
            pose_dict = dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), key))
            if not _pose_in_workspace(pose_dict):
                self._report.add(CheckItem(
                    CAT_REACHABILITY, "Workspace", SEV_WARNING, STATUS_FAIL,
                    "Alvo fora do envelope cartesiano de referência "
                    f"({pose_dict['x']:.2f}, {pose_dict['y']:.2f}, {pose_dict['z']:.2f}).",
                ))
        # Hook the bridge signal. Try to fire probes; if the service is not
        # discoverable at all, bridge.check_pose_reachable returns False for
        # every call and we surface a single warning instead of N errors.
        connected = self._connect_bridge()
        if not connected:
            self._report.reachability_done = True
            self._report.reachability_skipped_reason = "serviço move_to_pose_ik indisponível"
            self._report.add(CheckItem(
                CAT_REACHABILITY, "Serviço IK", SEV_WARNING, STATUS_SKIP,
                "Serviço move_to_pose_ik indisponível — alcance não verificado.",
            ))
            self.finished.emit(self._report)
            return
        for key in keys:
            self._pending[str(key)] = key
            pose_dict = dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), key))
            self._bridge.check_pose_reachable(str(key), pose_dict)
        if not self._pending:
            self._finish()
            return
        self._total = len(self._pending)
        self.progress.emit(0, self._total)
        self._deadline_timer.start(self._timeout_ms)

    def _connect_bridge(self) -> bool:
        """Probe the IK service once; return False if clearly unavailable."""
        try:
            ready = bool(self._bridge.ik_service_ready())
        except Exception:
            ready = False
        if not ready:
            return False
        try:
            self._bridge.ik_pose_checked.connect(self._on_ik_result)
        except Exception:
            return False
        return True

    def _on_ik_result(self, key: str, success: bool, message: str, q_solution: object) -> None:
        if key not in self._pending:
            return  # stale / unexpected
        rounded = self._pending.pop(key)
        pose_dict = dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), rounded))
        coord = f"({pose_dict['x']:.2f}, {pose_dict['y']:.2f}, {pose_dict['z']:.2f})"
        if not success:
            self._report.add(CheckItem(
                CAT_REACHABILITY, "IK", SEV_ERROR, STATUS_FAIL,
                f"Alvo {coord} sem solução IK: {message}.",
            ))
        else:
            joint_note = self._check_joint_limits(q_solution)
            if joint_note:
                self._report.add(CheckItem(
                    CAT_REACHABILITY, "IK", SEV_WARNING, STATUS_FAIL,
                    f"Alvo {coord} alcançável, mas {joint_note}.",
                ))
            else:
                self._report.add(CheckItem(
                    CAT_REACHABILITY, "IK", SEV_INFO, STATUS_PASS,
                    f"Alvo {coord} alcançável.",
                ))
        if not self._pending:
            self._finish()
        else:
            self.progress.emit(self._total - len(self._pending), self._total)

    def _check_joint_limits(self, q_solution: object) -> str:
        """Return an empty string if all joints are within limits, else a note."""
        if not q_solution:
            return ""
        try:
            angles = [float(v) for v in q_solution]
        except (TypeError, ValueError):
            return ""
        limits = self._bridge.joint_limits()
        names = self._bridge.joint_names()
        if not limits or not names:
            return ""
        violations = []
        for name, angle in zip(names, angles):
            lo, hi = limits.get(name, (None, None))
            if lo is None or hi is None:
                continue
            if angle < lo or angle > hi:
                violations.append(f"{name}={angle:.3f} rad fora de [{lo:.3f}, {hi:.3f}]")
        return "; ".join(violations)

    def _on_deadline(self) -> None:
        for key, rounded in list(self._pending.items()):
            pose_dict = dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), rounded))
            coord = f"({pose_dict['x']:.2f}, {pose_dict['y']:.2f}, {pose_dict['z']:.2f})"
            self._report.add(CheckItem(
                CAT_REACHABILITY, "IK", SEV_WARNING, STATUS_FAIL,
                f"Alvo {coord} sem resposta do IK no tempo limite.",
            ))
        self._pending.clear()
        self.progress.emit(self._total, self._total)
        self._finish()

    def _finish(self) -> None:
        if self._finished_flag:
            return
        self._finished_flag = True
        if self._deadline_timer.isActive():
            self._deadline_timer.stop()
        try:
            self._bridge.ik_pose_checked.disconnect(self._on_ik_result)
        except (TypeError, RuntimeError):
            pass
        self._report.reachability_done = True
        self.finished.emit(self._report)


# ----------------------------------------------------------------------
# top-level entry point
# ----------------------------------------------------------------------
def run_preflight(
    program: Program,
    bridge: "RosBridge",
    *,
    for_real_run: bool,
    with_reachability: bool = False,
    reachability_timeout_ms: int = REACHABILITY_DEFAULT_TIMEOUT_MS,
    require_certified_envelope: bool = False,
    on_report: Optional[Callable[[PreflightReport], None]] = None,
) -> PreflightReport:
    """Run sync checks and (optionally) async reachability.

    When ``with_reachability`` is False, returns immediately with the sync
    report. When True, runs sync checks first; if they already have errors,
    reachability is skipped (no point probing an unsound program). Otherwise
    a :class:`ReachabilityProbe` is started. If ``on_report`` is provided the
    probe runs asynchronously and the callback is invoked once with the final
    report (and the partially-filled sync report is returned immediately);
    otherwise the call blocks on a nested Qt event loop until the probe
    finishes or the deadline expires.
    """
    report = run_sync_checks(
        program,
        bridge,
        for_real_run=for_real_run,
        require_certified_envelope=require_certified_envelope,
    )
    if not with_reachability:
        return report
    if report.errors:
        report.reachability_skipped_reason = "erros síncronos impedem o teste de alcance"
        return report

    probe = ReachabilityProbe(
        program, bridge, report, timeout_ms=reachability_timeout_ms
    )
    if on_report is not None:
        probe.finished.connect(on_report)
        probe.start()
        return report

    # Synchronous flavour: drive a nested event loop with a hard deadline.
    # If the probe finishes during start() (e.g. service unavailable), the
    # flag is set before we ever enter the loop and we return right away.
    from PySide6.QtCore import QEventLoop, QTimer

    done = {"flag": False}
    probe.finished.connect(lambda _r: done.__setitem__("flag", True))
    probe.start()
    if not done["flag"]:
        loop = QEventLoop()
        probe.finished.connect(lambda _r: loop.quit())
        hard_stop = QTimer()
        hard_stop.setSingleShot(True)
        hard_stop.setInterval(int(reachability_timeout_ms) + 500)
        hard_stop.timeout.connect(loop.quit)
        hard_stop.start()
        loop.exec()
        hard_stop.stop()
    return report
