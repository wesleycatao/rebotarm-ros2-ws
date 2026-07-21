"""Structured conditions for IF-ELSE / loop guards in motion programs.

Conditions are intentionally *structured* (not free-text expressions) so the
GUI can build them from combo boxes and the runtime can evaluate them safely
without ``eval``. A :class:`ConditionExpr` combines one or more
:class:`Condition` atoms with an AND/OR combiner.

A live :class:`ConditionContext` supplies the current sensor/state values read
from the ROS bridge caches (state machine, digital inputs, joint positions,
gripper and - later - computer-vision signals).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Protocol


# Allowed comparison operators ------------------------------------------------
OPS_NUMERIC = ("==", "!=", ">", ">=", "<", "<=")
OPS_STRING = ("==", "!=")

# Source kinds ----------------------------------------------------------------
SRC_SIGNAL = "signal"
SRC_STATE_MACHINE = "state_machine"
SRC_JOINT_POSITION = "joint_position"
SRC_GRIPPER_POSITION = "gripper_position"
SRC_VISION_DETECTED = "vision_detected"
SRC_VISION_POSE = "vision_pose"

SOURCE_LABELS_PT: Dict[str, str] = {
    SRC_SIGNAL: "Entrada digital (DI)",
    SRC_STATE_MACHINE: "Máquina de estados",
    SRC_JOINT_POSITION: "Posição de junta",
    SRC_GRIPPER_POSITION: "Posição do gripper",
    SRC_VISION_DETECTED: "Visão: peça detectada",
    SRC_VISION_POSE: "Visão: coordenada",
}

# Display labels for source params (used by describe / GUI tooltips)
SOURCE_NEEDS_IO_ID = {SRC_SIGNAL}
SOURCE_NEEDS_JOINT = {SRC_JOINT_POSITION}
SOURCE_NEEDS_AXIS = {SRC_VISION_POSE}

COMBINER_ALL = "all"
COMBINER_ANY = "any"
COMBINER_LABELS_PT = {COMBINER_ALL: "Todas (E)", COMBINER_ANY: "Qualquer (OU)"}

VALID_STATE_MACHINES = (
    "IDLE",
    "TRAJ_RUNNING",
    "LOWLEVEL_STREAMING",
    "GRAVITY_COMP",
    "SAFE_HOMING",
    "PROTECTIVE_STOP",
)


class ConditionContext(Protocol):
    """Read-only live values consumed by the evaluator.

    The motion runner implements this against the ROS bridge caches.
    """

    state_machine: str

    def signal_by_id(self, io_id: str) -> int:  # pragma: no cover - protocol
        ...

    def joint_position(self, joint: str) -> Optional[float]:  # pragma: no cover
        ...

    @property
    def gripper_position(self) -> Optional[float]:  # pragma: no cover
        ...

    def vision_detected(self) -> bool:  # pragma: no cover
        ...

    def vision_pose(self, axis: str) -> Optional[float]:  # pragma: no cover
        ...


# ----------------------------------------------------------------------
# dataclasses
# ----------------------------------------------------------------------
@dataclass
class Condition:
    """A single comparison ``<source> <op> <value>``."""

    source: str = SRC_SIGNAL
    params: Dict[str, Any] = field(default_factory=dict)
    op: str = "=="
    value: Any = 1
    value_type: str = "number"  # "number" | "string" | "bool"

    # ---- serialization ----
    def to_dict(self) -> Dict[str, Any]:
        return {
            "source": self.source,
            "params": dict(self.params),
            "op": self.op,
            "value": self.value,
            "value_type": self.value_type,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Condition":
        return cls(
            source=str(data.get("source", SRC_SIGNAL)),
            params=dict(data.get("params", {}) or {}),
            op=str(data.get("op", "==")),
            value=data.get("value", 0),
            value_type=str(data.get("value_type", "number")),
        )

    # ---- helpers ----
    def _read(self, ctx: ConditionContext) -> Any:
        if self.source == SRC_SIGNAL:
            io_id = self.params.get("io_id")
            return int(ctx.signal_by_id(str(io_id or "")))
        if self.source == SRC_STATE_MACHINE:
            return str(ctx.state_machine)
        if self.source == SRC_JOINT_POSITION:
            return ctx.joint_position(str(self.params.get("joint", "")))
        if self.source == SRC_GRIPPER_POSITION:
            return ctx.gripper_position
        if self.source == SRC_VISION_DETECTED:
            return bool(ctx.vision_detected())
        if self.source == SRC_VISION_POSE:
            return ctx.vision_pose(str(self.params.get("axis", "x")))
        return None

    def describe(self) -> str:
        src_label = SOURCE_LABELS_PT.get(self.source, self.source)
        detail = ""
        if self.source in SOURCE_NEEDS_IO_ID:
            io_id = self.params.get("io_id")
            detail = f" {io_id or '?'}"
        elif self.source in SOURCE_NEEDS_JOINT:
            detail = f" {self.params.get('joint', '?')}"
        elif self.source in SOURCE_NEEDS_AXIS:
            detail = f" {self.params.get('axis', 'x')}"
        return f"{src_label}{detail} {self.op} {self._fmt_value()}"

    def _fmt_value(self) -> str:
        if self.value_type == "bool":
            return "verdadeiro" if bool(self.value) else "falso"
        if self.value_type == "string":
            return f'"{self.value}"'
        return str(self.value)


@dataclass
class ConditionExpr:
    """One or more :class:`Condition` atoms combined with AND/OR."""

    items: List[Condition] = field(default_factory=list)
    combiner: str = COMBINER_ALL

    def to_dict(self) -> Dict[str, Any]:
        return {
            "combiner": self.combiner,
            "items": [c.to_dict() for c in self.items],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any] | None) -> "ConditionExpr":
        if not data:
            return cls()
        items = [Condition.from_dict(c) for c in data.get("items", [])]
        return cls(items=items, combiner=str(data.get("combiner", COMBINER_ALL)))

    def is_empty(self) -> bool:
        return not self.items

    def describe(self) -> str:
        if not self.items:
            return "(sem condição)"
        joiner = " E " if self.combiner == COMBINER_ALL else " OU "
        return joiner.join(f"({c.describe()})" for c in self.items)


# ----------------------------------------------------------------------
# evaluator
# ----------------------------------------------------------------------
def _to_number(v: Any) -> Optional[float]:
    try:
        if isinstance(v, bool):
            return 1.0 if v else 0.0
        return float(v)
    except (TypeError, ValueError):
        return None


def evaluate_condition(c: Condition, ctx: ConditionContext) -> bool:
    left = c._read(ctx)
    right = c.value

    if c.value_type == "string" or isinstance(left, str) or isinstance(right, str):
        # string comparison: only == / != are meaningful
        ls = str(left)
        rs = str(right)
        if c.op == "==":
            return ls == rs
        if c.op == "!=":
            return ls != rs
        return False

    # numeric / bool comparison
    ln = _to_number(left)
    rn = _to_number(right)
    if ln is None or rn is None:
        return False
    if c.op == "==":
        return abs(ln - rn) < 1e-9
    if c.op == "!=":
        return abs(ln - rn) >= 1e-9
    if c.op == ">":
        return ln > rn
    if c.op == ">=":
        return ln >= rn
    if c.op == "<":
        return ln < rn
    if c.op == "<=":
        return ln <= rn
    return False


def evaluate(expr: ConditionExpr, ctx: ConditionContext) -> bool:
    if not expr.items:
        return True  # an empty condition is considered "true"
    results = [evaluate_condition(c, ctx) for c in expr.items]
    if expr.combiner == COMBINER_ANY:
        return any(results)
    return all(results)
