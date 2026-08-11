"""Conservative joint-demand analysis for compiled motion programs.

The controller executes every Cartesian opcode as a synchronized minimum-jerk
joint move.  This module resolves each compiled waypoint through the same IK
service and estimates the analytical peak velocity and acceleration of that
profile.  Limits come from the controller's latched ``trajectory_limits``
topic, so the GUI never certifies a path against duplicated defaults.

The analysis is intentionally fail-closed: missing limits, joint state, IK
solutions, excessive control-flow expansion or a timeout produce an
unverified report instead of an optimistic result.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Iterable

from PySide6.QtCore import QEventLoop, QObject, QTimer, Signal

from .conditions import ConditionExpr
from .program import (
    OP_MOVE,
    IfStep,
    LoopStep,
    Program,
    Step,
)
from .trajectory import scale_duration

if TYPE_CHECKING:  # pragma: no cover
    from ..ros_bridge import RosBridge


MAX_CONTROL_PATHS = 32
MAX_LOOP_ITERATIONS = 100
MAX_MOTION_TARGETS = 600
DYNAMICS_TIMEOUT_MS = 10_000

# Analytical maxima for s(t)=10t³-15t⁴+6t⁵, t in [0, 1].
MINIMUM_JERK_PEAK_VELOCITY = 1.875
MINIMUM_JERK_PEAK_ACCELERATION = 10.0 / math.sqrt(3.0)


@dataclass(frozen=True)
class MotionTarget:
    pose: dict[str, float]
    duration_s: float
    root_index: int
    step_id: str

    def key(self) -> tuple[float, ...]:
        return tuple(
            round(float(self.pose.get(name, 0.0)), 9)
            for name in ("x", "y", "z", "roll", "pitch", "yaw")
        )


@dataclass(frozen=True)
class JointDynamicsMetric:
    joint_name: str
    peak_velocity: float
    velocity_limit: float
    velocity_utilization: float
    peak_acceleration: float
    acceleration_limit: float
    acceleration_utilization: float
    margin_percent: float
    limiting_step_id: str


@dataclass
class DynamicsAnalysisReport:
    verified: bool = False
    within_limits: bool = False
    metrics: tuple[JointDynamicsMetric, ...] = ()
    limiting_joint: str = ""
    limiting_quantity: str = ""
    required_time_scale: float = 0.0
    max_safe_override: float = 0.0
    analyzed_override: float = 1.0
    path_count: int = 0
    target_count: int = 0
    safety_factor: float = 0.0
    messages: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.verified and self.within_limits

    def summary(self) -> str:
        if not self.verified:
            reason = self.messages[0] if self.messages else "dados insuficientes"
            return f"NÃO VERIFICADO — {reason}"
        if not self.within_limits:
            return (
                f"LIMITE EXCEDIDO — {self.limiting_joint} "
                f"({self.limiting_quantity}); override seguro estimado "
                f"{self.max_safe_override * 100:.0f}%"
            )
        return (
            f"APROVADO — limitante {self.limiting_joint or 'nenhuma'}; "
            f"override máximo seguro {self.max_safe_override * 100:.0f}%"
        )


class MotionPathExpansionError(ValueError):
    """The program cannot be expanded safely within the analysis bounds."""


def limits_signature(limits: dict[str, object]) -> str:
    """Stable digest for the controller limit set stored with a release."""

    payload = {
        "joint_names": list(limits.get("joint_names", [])),
        "max_velocity": [float(value) for value in limits.get("max_velocity", [])],
        "max_acceleration": [
            float(value) for value in limits.get("max_acceleration", [])
        ],
        "safety_factor": float(limits.get("safety_factor", 0.0)),
    }
    canonical = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def _leaf_targets(
    step: Step,
    *,
    root_index: int,
    step_id: str,
    program_scale: float,
    speed_override: float,
) -> list[MotionTarget]:
    opcodes: list[tuple] = []
    step.emit(opcodes, step_id, lambda _child: None)
    targets: list[MotionTarget] = []
    for opcode in opcodes:
        if opcode and opcode[0] == OP_MOVE:
            targets.append(
                MotionTarget(
                    pose={
                        name: float(opcode[1].get(name, 0.0))
                        for name in ("x", "y", "z", "roll", "pitch", "yaw")
                    },
                    duration_s=(
                        scale_duration(float(opcode[2]), program_scale)
                        / speed_override
                    ),
                    root_index=root_index,
                    step_id=str(opcode[-1]),
                )
            )
    return targets


def _combine_paths(
    prefixes: list[list[MotionTarget]],
    alternatives: list[list[MotionTarget]],
) -> list[list[MotionTarget]]:
    combined = [prefix + suffix for prefix in prefixes for suffix in alternatives]
    if len(combined) > MAX_CONTROL_PATHS:
        raise MotionPathExpansionError(
            f"mais de {MAX_CONTROL_PATHS} caminhos possíveis em IF/ELSE"
        )
    if any(len(path) > MAX_MOTION_TARGETS for path in combined):
        raise MotionPathExpansionError(
            f"mais de {MAX_MOTION_TARGETS} alvos em um caminho"
        )
    return combined


def _expand_steps(
    steps: Iterable[Step],
    *,
    root_index: int,
    program_scale: float,
    speed_override: float,
    counter: list[int],
    notes: list[str],
) -> list[list[MotionTarget]]:
    paths: list[list[MotionTarget]] = [[]]
    for step in steps:
        counter[0] += 1
        step_id = f"dyn_{counter[0]:03d}_{step.kind}"
        alternatives: list[list[MotionTarget]]
        if isinstance(step, LoopStep):
            if step.count <= 0:
                iterations = 2
                notes.append(
                    f"{step_id}: LOOP infinito analisado em duas iterações "
                    "para incluir a transição de retorno."
                )
            elif step.count > MAX_LOOP_ITERATIONS:
                raise MotionPathExpansionError(
                    f"{step_id}: LOOP com {step.count} iterações excede "
                    f"o limite de análise ({MAX_LOOP_ITERATIONS})"
                )
            else:
                iterations = int(step.count)
            alternatives = [[]]
            for _ in range(iterations):
                child_paths = _expand_steps(
                    step.steps,
                    root_index=root_index,
                    program_scale=program_scale,
                    speed_override=speed_override,
                    counter=counter,
                    notes=notes,
                )
                alternatives = _combine_paths(alternatives, child_paths)
        elif isinstance(step, IfStep):
            then_paths = _expand_steps(
                step.then_steps,
                root_index=root_index,
                program_scale=program_scale,
                speed_override=speed_override,
                counter=counter,
                notes=notes,
            )
            if isinstance(step.condition, ConditionExpr) and step.condition.is_empty():
                alternatives = then_paths
            else:
                else_paths = _expand_steps(
                    step.else_steps,
                    root_index=root_index,
                    program_scale=program_scale,
                    speed_override=speed_override,
                    counter=counter,
                    notes=notes,
                )
                alternatives = then_paths + else_paths
                if len(alternatives) > MAX_CONTROL_PATHS:
                    raise MotionPathExpansionError(
                        f"mais de {MAX_CONTROL_PATHS} caminhos possíveis em IF/ELSE"
                    )
        else:
            display_step_id = str(getattr(step, "label", "")).strip() or step_id
            alternatives = [
                _leaf_targets(
                    step,
                    root_index=root_index,
                    step_id=display_step_id,
                    program_scale=program_scale,
                    speed_override=speed_override,
                )
            ]
        paths = _combine_paths(paths, alternatives)
    return paths


def build_motion_paths(
    program: Program,
    *,
    start_index: int = 0,
    speed_override: float = 1.0,
) -> tuple[list[list[MotionTarget]], list[str]]:
    """Expand deterministic loops and both possible IF branches."""

    start = max(0, min(int(start_index), len(program.steps)))
    override = max(0.05, min(1.0, float(speed_override)))
    notes: list[str] = []
    paths: list[list[MotionTarget]] = [[]]
    counter = [0]
    for root_index, step in enumerate(program.steps[start:], start=start):
        alternatives = _expand_steps(
            [step],
            root_index=root_index,
            program_scale=float(program.speed_scale),
            speed_override=override,
            counter=counter,
            notes=notes,
        )
        paths = _combine_paths(paths, alternatives)

    # Remove duplicate control paths (common when one IF branch has no motion).
    unique: dict[tuple, list[MotionTarget]] = {}
    for path in paths:
        signature = tuple((target.key(), target.duration_s) for target in path)
        unique.setdefault(signature, path)
    return list(unique.values()), notes


def analyze_joint_dynamics(
    paths: list[list[MotionTarget]],
    solutions: dict[tuple[float, ...], list[float]],
    *,
    joint_names: list[str],
    current_positions: list[float],
    max_velocities: list[float],
    max_accelerations: list[float],
    safety_factor: float,
    analyzed_override: float,
    notes: list[str] | None = None,
) -> DynamicsAnalysisReport:
    """Calculate minimum-jerk peak demand for every joint and control path."""

    report = DynamicsAnalysisReport(
        analyzed_override=float(analyzed_override),
        path_count=len(paths),
        target_count=sum(len(path) for path in paths),
        safety_factor=float(safety_factor),
        messages=list(notes or []),
    )
    joint_count = len(joint_names)
    if not paths or not any(paths):
        report.verified = True
        report.within_limits = True
        report.max_safe_override = 1.0
        report.required_time_scale = 1.0
        report.messages.insert(0, "programa sem movimentos cartesianos")
        return report
    vectors = (
        current_positions,
        max_velocities,
        max_accelerations,
    )
    if (
        joint_count == 0
        or any(len(vector) != joint_count for vector in vectors)
        or not 0.0 < safety_factor <= 1.0
    ):
        report.messages.insert(0, "estado ou limites das juntas incompletos")
        return report
    if any(
        not math.isfinite(value)
        for vector in vectors
        for value in vector
    ) or any(value <= 0.0 for value in max_velocities + max_accelerations):
        report.messages.insert(0, "estado ou limites das juntas inválidos")
        return report

    peak_v = [0.0] * joint_count
    peak_a = [0.0] * joint_count
    limiting_step = [""] * joint_count
    for path in paths:
        previous = [float(value) for value in current_positions]
        for target in path:
            solution = solutions.get(target.key())
            if solution is None or len(solution) != joint_count:
                report.messages.insert(
                    0, f"solução IK ausente para {target.step_id}"
                )
                return report
            dt = float(target.duration_s)
            if not math.isfinite(dt) or dt <= 0.0:
                report.messages.insert(0, f"duração inválida em {target.step_id}")
                return report
            for index, (before, after) in enumerate(zip(previous, solution)):
                delta = abs(float(after) - float(before))
                velocity = MINIMUM_JERK_PEAK_VELOCITY * delta / dt
                acceleration = MINIMUM_JERK_PEAK_ACCELERATION * delta / (dt * dt)
                effective_v = max_velocities[index] * safety_factor
                effective_a = max_accelerations[index] * safety_factor
                old_score = max(
                    peak_v[index] / effective_v,
                    math.sqrt(peak_a[index] / effective_a)
                    if peak_a[index] > 0.0 else 0.0,
                )
                new_score = max(
                    velocity / effective_v,
                    math.sqrt(acceleration / effective_a),
                )
                peak_v[index] = max(peak_v[index], velocity)
                peak_a[index] = max(peak_a[index], acceleration)
                if new_score >= old_score:
                    limiting_step[index] = target.step_id
            previous = [float(value) for value in solution]

    metrics: list[JointDynamicsMetric] = []
    overall_score = -1.0
    overall_joint = ""
    overall_quantity = ""
    max_velocity_ratio = 0.0
    max_acceleration_ratio = 0.0
    for index, name in enumerate(joint_names):
        effective_v = max_velocities[index] * safety_factor
        effective_a = max_accelerations[index] * safety_factor
        velocity_ratio = peak_v[index] / effective_v
        acceleration_ratio = peak_a[index] / effective_a
        score_v = velocity_ratio
        score_a = math.sqrt(acceleration_ratio)
        score = max(score_v, score_a)
        quantity = "velocidade" if score_v >= score_a else "aceleração"
        if score > overall_score:
            overall_score = score
            overall_joint = name
            overall_quantity = quantity
        max_velocity_ratio = max(max_velocity_ratio, velocity_ratio)
        max_acceleration_ratio = max(max_acceleration_ratio, acceleration_ratio)
        utilization = max(velocity_ratio, acceleration_ratio)
        metrics.append(
            JointDynamicsMetric(
                joint_name=name,
                peak_velocity=peak_v[index],
                velocity_limit=effective_v,
                velocity_utilization=velocity_ratio,
                peak_acceleration=peak_a[index],
                acceleration_limit=effective_a,
                acceleration_utilization=acceleration_ratio,
                margin_percent=(1.0 - utilization) * 100.0,
                limiting_step_id=limiting_step[index],
            )
        )

    required_scale = max(max_velocity_ratio, math.sqrt(max_acceleration_ratio))
    if required_scale <= 1e-12:
        max_override = 1.0
    else:
        max_override = min(1.0, float(analyzed_override) / required_scale)
    report.metrics = tuple(metrics)
    report.limiting_joint = overall_joint
    report.limiting_quantity = overall_quantity
    report.required_time_scale = max(1.0, required_scale)
    report.max_safe_override = max(0.0, max_override)
    report.verified = True
    report.within_limits = (
        max_velocity_ratio <= 1.0 + 1e-9
        and max_acceleration_ratio <= 1.0 + 1e-9
    )
    return report


class DynamicsProbe(QObject):
    """Resolve every unique target through IK, then calculate joint demand."""

    finished = Signal(object)
    progress = Signal(int, int)

    def __init__(
        self,
        program: Program,
        bridge: "RosBridge",
        *,
        start_index: int = 0,
        speed_override: float = 1.0,
        timeout_ms: int = DYNAMICS_TIMEOUT_MS,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._program = program
        self._bridge = bridge
        self._start_index = int(start_index)
        self._override = max(0.05, min(1.0, float(speed_override)))
        self._timeout_ms = max(100, int(timeout_ms))
        self._paths: list[list[MotionTarget]] = []
        self._notes: list[str] = []
        self._pending: dict[str, tuple[float, ...]] = {}
        self._solutions: dict[tuple[float, ...], list[float]] = {}
        self._failures: list[str] = []
        self._total = 0
        self._done = False
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._on_timeout)

    def start(self) -> None:
        try:
            self._paths, self._notes = build_motion_paths(
                self._program,
                start_index=self._start_index,
                speed_override=self._override,
            )
        except (ValueError, TypeError) as exc:
            self._finish_unverified(str(exc))
            return
        targets = {
            target.key(): target
            for path in self._paths
            for target in path
        }
        if not targets:
            self._emit(
                DynamicsAnalysisReport(
                    verified=True,
                    within_limits=True,
                    required_time_scale=1.0,
                    max_safe_override=1.0,
                    analyzed_override=self._override,
                    path_count=len(self._paths),
                    messages=["programa sem movimentos cartesianos"],
                )
            )
            return
        limits = self._bridge.trajectory_limits()
        current_names = self._bridge.joint_names()
        current_values = self._bridge.current_arm_joint_positions()
        limit_names = list(limits.get("joint_names", [])) if limits else []
        if not limits or not limit_names:
            self._finish_unverified(
                "limites dinâmicos do controlador ainda não foram recebidos"
            )
            return
        if set(current_names) != set(limit_names) or len(current_values) != len(current_names):
            self._finish_unverified("joint_state não corresponde aos limites do controlador")
            return
        if not self._bridge.ik_service_ready():
            self._finish_unverified("serviço move_to_pose_ik indisponível")
            return
        self._bridge.ik_pose_checked.connect(self._on_ik_result)
        requests: list[tuple[str, tuple[float, ...], MotionTarget]] = []
        for index, (key, target) in enumerate(targets.items()):
            request_key = f"dynamics:{id(self)}:{index}"
            self._pending[request_key] = key
            requests.append((request_key, key, target))
        self._total = len(requests)
        self.progress.emit(0, self._total)
        for request_key, _key, target in requests:
            if not self._bridge.check_pose_reachable(request_key, target.pose):
                self._pending.pop(request_key, None)
                self._failures.append(f"{target.step_id}: serviço IK indisponível")
            if self._done:
                return
        if not self._pending:
            if self._failures:
                self._finish_unverified(self._failures[0])
            else:
                self._complete()
            return
        self._timer.start(self._timeout_ms)

    def _on_ik_result(
        self,
        request_key: str,
        success: bool,
        message: str,
        q_solution: object,
    ) -> None:
        key = self._pending.pop(request_key, None)
        if key is None:
            return
        if success:
            try:
                self._solutions[key] = [float(value) for value in q_solution]
            except (TypeError, ValueError):
                self._failures.append("resposta IK contém valores inválidos")
        else:
            self._failures.append(message or f"alvo {key[:3]} sem solução IK")
        self.progress.emit(self._total - len(self._pending), self._total)
        if not self._pending:
            self._complete()

    def _ordered_state_and_limits(
        self,
    ) -> tuple[list[str], list[float], list[float], list[float], float]:
        data = self._bridge.trajectory_limits()
        limit_names = [str(value) for value in data.get("joint_names", [])]
        current_names = self._bridge.joint_names()
        current_values = self._bridge.current_arm_joint_positions()
        current_map = dict(zip(current_names, current_values))
        return (
            limit_names,
            [float(current_map[name]) for name in limit_names],
            [float(value) for value in data.get("max_velocity", [])],
            [float(value) for value in data.get("max_acceleration", [])],
            float(data.get("safety_factor", 0.0)),
        )

    def _complete(self) -> None:
        if self._failures:
            self._finish_unverified(self._failures[0])
            return
        try:
            names, current, velocities, accelerations, factor = (
                self._ordered_state_and_limits()
            )
            report = analyze_joint_dynamics(
                self._paths,
                self._solutions,
                joint_names=names,
                current_positions=current,
                max_velocities=velocities,
                max_accelerations=accelerations,
                safety_factor=factor,
                analyzed_override=self._override,
                notes=self._notes,
            )
        except (KeyError, TypeError, ValueError) as exc:
            self._finish_unverified(str(exc))
            return
        self._emit(report)

    def _on_timeout(self) -> None:
        self._finish_unverified(
            f"{len(self._pending)} alvo(s) sem resposta do IK no tempo limite"
        )

    def _finish_unverified(self, reason: str) -> None:
        report = DynamicsAnalysisReport(
            analyzed_override=self._override,
            path_count=len(self._paths),
            target_count=sum(len(path) for path in self._paths),
            messages=[str(reason), *self._notes],
        )
        self._emit(report)

    def _emit(self, report: DynamicsAnalysisReport) -> None:
        if self._done:
            return
        self._done = True
        if self._timer.isActive():
            self._timer.stop()
        try:
            self._bridge.ik_pose_checked.disconnect(self._on_ik_result)
        except (TypeError, RuntimeError):
            pass
        self.finished.emit(report)


def run_dynamic_analysis(
    program: Program,
    bridge: "RosBridge",
    *,
    start_index: int = 0,
    speed_override: float = 1.0,
    timeout_ms: int = DYNAMICS_TIMEOUT_MS,
) -> DynamicsAnalysisReport:
    """Blocking Qt helper used by release and execution gates."""

    result: list[DynamicsAnalysisReport] = []
    probe = DynamicsProbe(
        program,
        bridge,
        start_index=start_index,
        speed_override=speed_override,
        timeout_ms=timeout_ms,
    )
    probe.finished.connect(result.append)
    probe.start()
    if not result:
        loop = QEventLoop()
        probe.finished.connect(lambda _report: loop.quit())
        hard_stop = QTimer()
        hard_stop.setSingleShot(True)
        hard_stop.setInterval(int(timeout_ms) + 500)
        hard_stop.timeout.connect(loop.quit)
        hard_stop.start()
        loop.exec()
        hard_stop.stop()
    if result:
        return result[0]
    return DynamicsAnalysisReport(
        analyzed_override=float(speed_override),
        messages=["análise dinâmica não concluiu"],
    )
