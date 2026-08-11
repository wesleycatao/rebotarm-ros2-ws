"""Structural and operational validation for motion programs before saving.

Validation is split into two levels, matching the preflight taxonomy but
focused on what can be decided before the program is persisted:

* :func:`validate_edit` — structural correctness.  Errors here block saving.
* :func:`validate_operational` — best-effort checks that need ROS/MoveIt/URDF
  data.  Warnings allow saving as a draft but block recipe release.

The validator is intentionally independent from the UI so it can be unit
 tested without a running ROS node.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple

from .conditions import ConditionExpr
from .program import (
    ACTION_OPS,
    OP_GOTO,
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
    WaitStep,
    build_label_index,
    flatten_steps,
)

SEV_ERROR = "error"
SEV_WARNING = "warning"
SEV_INFO = "info"

_CAT_STRUCTURE = "Estrutura"
_CAT_DATA = "Dados"
_CAT_OP = "Operacional"

_VALID_IO_RE = "^[A-Za-z0-9_]+"


@dataclass
class ProgramValidationIssue:
    category: str
    severity: str
    message: str
    step_index: Optional[int] = None
    step_id: str = ""


@dataclass
class ProgramValidationResult:
    structural_errors: List[ProgramValidationIssue] = field(default_factory=list)
    operational_warnings: List[ProgramValidationIssue] = field(default_factory=list)
    infos: List[ProgramValidationIssue] = field(default_factory=list)

    @property
    def can_save(self) -> bool:
        return not self.structural_errors

    @property
    def can_release(self) -> bool:
        return self.can_save and not self.operational_warnings

    def summary(self) -> str:
        if self.structural_errors:
            return (
                f"{len(self.structural_errors)} erro(s) estrutural(is) — "
                "salvamento bloqueado"
            )
        if self.operational_warnings:
            return (
                f"{len(self.operational_warnings)} aviso(s) operacional(is) — "
                "salvo como rascunho, não liberável"
            )
        return "Válido — pronto para salvar"


def _is_nan(value: Any) -> bool:
    try:
        return math.isnan(float(value))
    except (TypeError, ValueError):
        return False


def _is_finite(value: Any) -> bool:
    try:
        f = float(value)
        return math.isfinite(f)
    except (TypeError, ValueError):
        return False


def _pose_fields(pose: Dict[str, Any]) -> List[str]:
    return ["x", "y", "z", "roll", "pitch", "yaw"]


def _validate_pose(
    result: ProgramValidationResult,
    pose: Dict[str, Any],
    label: str,
    step_index: int,
    step_id: str,
) -> None:
    for key in _pose_fields(pose):
        value = pose.get(key)
        if value is None:
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    f"{label} não possui o campo '{key}'.",
                    step_index,
                    step_id,
                )
            )
        elif _is_nan(value) or not _is_finite(value):
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    f"{label}['{key}'] não é um número finito.",
                    step_index,
                    step_id,
                )
            )


def _validate_condition(
    result: ProgramValidationResult,
    expr: ConditionExpr,
    step_index: int,
    step_id: str,
) -> None:
    if expr.is_empty():
        result.operational_warnings.append(
            ProgramValidationIssue(
                _CAT_STRUCTURE,
                SEV_WARNING,
                "Condição IF vazia: o bloco será sempre executado.",
                step_index,
                step_id,
            )
        )
        return
    for item in expr.items:
        if item.source == "signal" and not item.params.get("io_id"):
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "Condição de sinal não informa 'io_id'.",
                    step_index,
                    step_id,
                )
            )
        if item.value is None:
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "Condição sem valor de comparação.",
                    step_index,
                    step_id,
                )
            )


def _walk_steps(
    steps: List[Step],
) -> List[Tuple[int, Step]]:
    """Yield (flat_index, step) for top-level steps only."""
    return list(enumerate(steps))


def _collect_ids(steps: List[Step], seen: Optional[Set[str]] = None) -> Set[str]:
    if seen is None:
        seen = set()
    for step in steps:
        if isinstance(step, (LoopStep, IfStep)):
            children = []
            if isinstance(step, LoopStep):
                children = step.steps
            else:
                children = list(step.then_steps) + list(step.else_steps)
            _collect_ids(children, seen)
    return seen


def validate_edit(program: Program) -> ProgramValidationResult:
    """Run structural validation that must pass before saving.

    Errors block saving.  Warnings are structural observations that do not
    prevent persistence but are still surfaced to the operator.
    """
    result = ProgramValidationResult()

    if program.version != 3:
        result.structural_errors.append(
            ProgramValidationIssue(
                _CAT_STRUCTURE,
                SEV_ERROR,
                f"Versão de programa não suportada: {program.version}.",
            )
        )

    if not program.name or not program.name.strip():
        result.structural_errors.append(
            ProgramValidationIssue(
                _CAT_STRUCTURE,
                SEV_ERROR,
                "O programa não possui um nome.",
            )
        )

    if not program.steps:
        result.structural_errors.append(
            ProgramValidationIssue(
                _CAT_STRUCTURE,
                SEV_ERROR,
                "Programa vazio: adicione ao menos uma etapa.",
            )
        )

    if _is_nan(program.speed_scale) or not _is_finite(program.speed_scale):
        result.structural_errors.append(
            ProgramValidationIssue(
                _CAT_STRUCTURE,
                SEV_ERROR,
                "Escala de velocidade global não é um número finito.",
            )
        )
    elif program.speed_scale <= 0.0:
        result.structural_errors.append(
            ProgramValidationIssue(
                _CAT_STRUCTURE,
                SEV_ERROR,
                "Escala de velocidade global deve ser maior que zero.",
            )
        )

    # Step IDs must be unique after flattening (opcodes reference them).
    try:
        opcodes = flatten_steps(program.steps)
    except Exception as exc:
        result.structural_errors.append(
            ProgramValidationIssue(
                _CAT_STRUCTURE,
                SEV_ERROR,
                f"Erro ao montar opcodes: {exc}",
            )
        )
        return result

    try:
        labels = build_label_index(opcodes)
    except Exception as exc:
        result.structural_errors.append(
            ProgramValidationIssue(
                _CAT_STRUCTURE,
                SEV_ERROR,
                f"Erro ao indexar rótulos: {exc}",
            )
        )
        labels = {}

    missing_labels = [
        op[1] for op in opcodes if op and op[0] == OP_GOTO and op[1] not in labels
    ]
    if missing_labels:
        result.structural_errors.append(
            ProgramValidationIssue(
                _CAT_STRUCTURE,
                SEV_ERROR,
                f"Rótulos de desvio não encontrados: {', '.join(sorted(set(missing_labels)))}.",
            )
        )

    action_count = sum(1 for op in opcodes if op and op[0] in ACTION_OPS)
    if action_count == 0:
        result.structural_errors.append(
            ProgramValidationIssue(
                _CAT_STRUCTURE,
                SEV_ERROR,
                "O programa não gera nenhum comando de ação (movimento/gripper/I/O).",
            )
        )
    else:
        result.infos.append(
            ProgramValidationIssue(
                _CAT_STRUCTURE,
                SEV_INFO,
                f"Programa gera {action_count} comando(s) de ação.",
            )
        )

    # Validate each top-level step.  Nested steps are validated recursively.
    for step_index, step in _walk_steps(program.steps):
        _validate_step(result, step, step_index)

    return result


def _validate_step(
    result: ProgramValidationResult, step: Step, step_index: int
) -> None:
    step_id = step.label or f"step_{step_index}"

    if isinstance(step, MoveStep):
        _validate_pose(result, step.pose, "Pose alvo", step_index, step_id)
        if step.duration <= 0.0 or not _is_finite(step.duration):
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "Duração do movimento deve ser um número positivo finito.",
                    step_index,
                    step_id,
                )
            )

    elif isinstance(step, LineStep):
        _validate_pose(result, step.start, "Pose inicial", step_index, step_id)
        _validate_pose(result, step.end, "Pose final", step_index, step_id)
        if step.speed <= 0.0 or not _is_finite(step.speed):
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "Velocidade da linha deve ser um número positivo finito.",
                    step_index,
                    step_id,
                )
            )

    elif isinstance(step, ArcStep):
        for label, pose in (("P1", step.p1), ("P2", step.p2), ("P3", step.p3)):
            _validate_pose(result, pose, f"Arco {label}", step_index, step_id)
        if step.speed <= 0.0 or not _is_finite(step.speed):
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "Velocidade do arco deve ser um número positivo finito.",
                    step_index,
                    step_id,
                )
            )
        if step.samples < 2:
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "Arco deve ter ao menos 2 amostras.",
                    step_index,
                    step_id,
                )
            )

    elif isinstance(step, PalletStep):
        _validate_pose(result, step.corner, "Canto de origem", step_index, step_id)
        for dim, value in (("nx", step.nx), ("ny", step.ny), ("nz", step.nz)):
            if not isinstance(value, int) or value < 1:
                result.structural_errors.append(
                    ProgramValidationIssue(
                        _CAT_STRUCTURE,
                        SEV_ERROR,
                        f"Dimensão {dim} da paletização deve ser um inteiro >= 1.",
                        step_index,
                        step_id,
                    )
                )
        if step.speed <= 0.0 or not _is_finite(step.speed):
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "Velocidade da paletização deve ser um número positivo finito.",
                    step_index,
                    step_id,
                )
            )

    elif isinstance(step, WaitStep):
        if step.time < 0.0 or not _is_finite(step.time):
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "Tempo de espera deve ser um número finito >= 0.",
                    step_index,
                    step_id,
                )
            )

    elif isinstance(step, WaitForStateMachineStep):
        if not step.state:
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "Estado de espera não informado.",
                    step_index,
                    step_id,
                )
            )
        if step.timeout < 0.0 or not _is_finite(step.timeout):
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "Timeout de espera de estado deve ser um número finito >= 0.",
                    step_index,
                    step_id,
                )
            )

    elif isinstance(step, WaitForSignalStep):
        if not step.io_id:
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "ID de entrada digital não informado.",
                    step_index,
                    step_id,
                )
            )
        if step.timeout < 0.0 or not _is_finite(step.timeout):
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "Timeout de espera de sinal deve ser um número finito >= 0.",
                    step_index,
                    step_id,
                )
            )

    elif isinstance(step, OutputStep):
        if not step.io_id:
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "ID de saída digital não informado.",
                    step_index,
                    step_id,
                )
            )
        if step.action not in ("on", "off", "toggle"):
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    f"Ação de saída inválida: {step.action!r}.",
                    step_index,
                    step_id,
                )
            )

    elif isinstance(step, GripperStep):
        if not _is_finite(step.position_mm):
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "Posição do gripper não é um número finito.",
                    step_index,
                    step_id,
                )
            )
        if step.timeout < 0.0 or not _is_finite(step.timeout):
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "Timeout do gripper deve ser um número finito >= 0.",
                    step_index,
                    step_id,
                )
            )

    elif isinstance(step, LoopStep):
        if not isinstance(step.count, int):
            result.structural_errors.append(
                ProgramValidationIssue(
                    _CAT_STRUCTURE,
                    SEV_ERROR,
                    "Contador de LOOP deve ser um inteiro.",
                    step_index,
                    step_id,
                )
            )
        for child in step.steps:
            _validate_step(result, child, step_index)

    elif isinstance(step, IfStep):
        _validate_condition(result, step.condition, step_index, step_id)
        for child in step.then_steps:
            _validate_step(result, child, step_index)
        for child in step.else_steps:
            _validate_step(result, child, step_index)

    else:
        result.structural_errors.append(
            ProgramValidationIssue(
                _CAT_STRUCTURE,
                SEV_ERROR,
                f"Tipo de etapa desconhecido: {step.kind!r}.",
                step_index,
                step_id,
            )
        )


def validate_operational(
    program: Program,
    *,
    bridge=None,
    joint_limits: Optional[Dict[str, Tuple[float, float]]] = None,
) -> ProgramValidationResult:
    """Best-effort operational validation.

    Warnings here allow the file to be saved as a draft but block recipe
    release.  Missing ROS/MoveIt data should not prevent saving a structurally
    valid draft.
    """
    result = ProgramValidationResult()

    if bridge is None or not getattr(bridge, "is_connected", False):
        result.operational_warnings.append(
            ProgramValidationIssue(
                _CAT_OP,
                SEV_WARNING,
                "ROS não conectado: alcance, limites de juntas e cena de colisão não verificados.",
            )
        )
        return result

    # Placeholder for operational checks that need bridge data.
    # Real implementation would probe IK, check joint limits, verify collision
    # scene, payload/tool, and production prerequisites.  Keeping the function
    # small prevents blocking the save path when ROS is unavailable.
    result.infos.append(
        ProgramValidationIssue(
            _CAT_OP,
            SEV_INFO,
            "ROS conectado: validações operacionais adicionais serão executadas no preflight.",
        )
    )
    return result


def validate_program(
    program: Program,
    *,
    bridge=None,
) -> ProgramValidationResult:
    """Run edit and operational validation and merge the results."""
    edit = validate_edit(program)
    if not edit.can_save:
        return edit
    op = validate_operational(program, bridge=bridge)
    return ProgramValidationResult(
        structural_errors=edit.structural_errors,
        operational_warnings=op.operational_warnings,
        infos=edit.infos + op.infos,
    )
