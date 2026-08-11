"""Motion-program data model and (de)serialization.

A :class:`Program` is an ordered tree of steps. Leaves are atomic actions
(move, wait, wait-for-state, wait-for-signal); :class:`LoopStep` and
:class:`IfStep` are *compound* steps that contain their own child step lists.

Execution model
---------------
At run time the tree is flattened into a linear list of *opcodes* by
:func:`flatten_steps`. Each opcode is a tuple whose first element is a tag:

``("move", pose_dict, duration, step_id)``
    Go to a Cartesian pose (duration scaled by the program ``speed_scale``).
``("wait", seconds, step_id)``
    Time wait.
``("wait_sm", state, timeout, step_id)``
    Wait for the arm state machine to reach ``state``.
``("wait_sig", io_id, value, edge, timeout, step_id)``
    Wait for a digital input ``io_id`` to reach ``value`` (edge-aware).
``("label", name)`` / ``("goto", name)``
    Unconditional jump target / jump.
``("branch", condition_expr, name, polarity)``
    Jump to ``name`` when ``evaluate(condition_expr) == polarity``.
``("loop_init", loop_id, count)`` / ``("loop_back", loop_id, count, name)``
    Counted / infinite loop bookkeeping.

The motion runner is a small program-counter VM over this opcode list.

Serialization target is JSON (stdlib :mod:`json`) so a saved program is
self-contained and version-controllable.
"""

from __future__ import annotations

import json
import re
import hashlib
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Tuple

from .conditions import ConditionExpr
from .trajectory import (
    PoseDict,
    arc_waypoints,
    line_waypoints,
    pallet_waypoints,
)

PROGRAM_VERSION = 3
RECIPE_DRAFT = "draft"
RECIPE_RELEASED = "released"
RECIPE_OBSOLETE = "obsolete"
RECIPE_STATUSES = (RECIPE_DRAFT, RECIPE_RELEASED, RECIPE_OBSOLETE)

# Opcode tags -----------------------------------------------------------------
OP_MOVE = "move"
OP_GRIPPER = "gripper"
OP_WAIT = "wait"
OP_WAIT_SM = "wait_sm"
OP_WAIT_SIG = "wait_sig"
OP_OUTPUT = "output"
OP_LABEL = "label"
OP_GOTO = "goto"
OP_BRANCH = "branch"
OP_LOOP_INIT = "loop_init"
OP_LOOP_BACK = "loop_back"

ACTION_OPS = (OP_MOVE, OP_GRIPPER, OP_WAIT, OP_WAIT_SM, OP_WAIT_SIG, OP_OUTPUT)

# Edge modes for wait-for-signal ---------------------------------------------
EDGE_LEVEL = "level"          # match current level
EDGE_RISING = "rising"        # match 0 -> 1 transition
EDGE_FALLING = "falling"      # match 1 -> 0 transition
EDGE_LABELS_PT = {
    EDGE_LEVEL: "Nível",
    EDGE_RISING: "Borda de subida",
    EDGE_FALLING: "Borda de descida",
}

# Kinds -----------------------------------------------------------------------
KIND_MOVE = "move"
KIND_GRIPPER = "gripper"
KIND_WAIT = "wait"
KIND_WAIT_SM = "wait_sm"
KIND_WAIT_SIG = "wait_sig"
KIND_OUTPUT = "output"
KIND_LINE = "line"
KIND_ARC = "arc"
KIND_PALLET = "pallet"
KIND_LOOP = "loop"
KIND_IF = "if"

KIND_LABELS_PT = {
    KIND_MOVE: "Ir ao ponto",
    KIND_GRIPPER: "Gripper",
    KIND_WAIT: "WAIT (tempo)",
    KIND_WAIT_SM: "WAIT_FOR_STATE_MACHINE",
    KIND_WAIT_SIG: "WAIT_FOR_SIGNAL",
    KIND_OUTPUT: "Saída digital (OUTPUT)",
    KIND_LINE: "Linha reta",
    KIND_ARC: "Arco circular",
    KIND_PALLET: "Paletização 3D",
    KIND_LOOP: "LOOP (repetir)",
    KIND_IF: "IF-ELSE (condicional)",
}

# Kinds offered in the "add step" menu (order matters for the UI).
KIND_MENU_ORDER = (
    KIND_MOVE,
    KIND_GRIPPER,
    KIND_LINE,
    KIND_ARC,
    KIND_PALLET,
    KIND_WAIT,
    KIND_WAIT_SM,
    KIND_WAIT_SIG,
    KIND_OUTPUT,
    KIND_LOOP,
    KIND_IF,
)

# Output actions ---------------------------------------------------------------
OUTPUT_ON = "on"
OUTPUT_OFF = "off"
OUTPUT_TOGGLE = "toggle"
OUTPUT_ACTION_LABELS_PT = {
    OUTPUT_ON: "Ligar (OUTPUT_ON)",
    OUTPUT_OFF: "Desligar (OUTPUT_OFF)",
    OUTPUT_TOGGLE: "Alternar (OUTPUT_TOGGLE)",
}


# ----------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------
def pose(x: float = 0.0, y: float = 0.0, z: float = 0.3,
         roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> PoseDict:
    return {"x": float(x), "y": float(y), "z": float(z),
            "roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)}


def _pose_to_dict(p: PoseDict) -> PoseDict:
    return {k: float(p.get(k, 0.0)) for k in ("x", "y", "z", "roll", "pitch", "yaw")}


def _sanitize_id(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", text).strip("_")


def _make_step_id(n: int, step: "Step") -> str:
    label_part = _sanitize_id(step.label.strip()) if step.label else ""
    if label_part:
        return f"{label_part}_{n:03d}"
    return f"step{n:03d}_{step.kind}"


# ----------------------------------------------------------------------
# step dataclasses
# ----------------------------------------------------------------------
@dataclass
class Step:
    """Base step: carries a user label and the discriminator ``kind``."""

    kind: str = ""
    label: str = ""

    def short_description(self) -> str:  # pragma: no cover - overridden
        return self.label or KIND_LABELS_PT.get(self.kind, self.kind)

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self) if is_dataclass(self) else {}
        data["kind"] = self.kind
        return data

    def emit(self, out: List[Tuple], step_id: str, recurse: Callable[["Step"], None]) -> None:
        """Append this step's opcodes to ``out``. Override in subclasses."""
        return


@dataclass
class MoveStep(Step):
    kind: str = field(default=KIND_MOVE, init=False, repr=False)
    pose: PoseDict = field(default_factory=lambda: pose())
    duration: float = 2.0

    def short_description(self) -> str:
        return self.label or f"Ponto ({self.pose['x']:.2f}, {self.pose['y']:.2f}, {self.pose['z']:.2f})"

    def emit(self, out, step_id, recurse) -> None:
        out.append((OP_MOVE, _pose_to_dict(self.pose), max(0.1, float(self.duration)), step_id))


@dataclass
class GripperStep(Step):
    kind: str = field(default=KIND_GRIPPER, init=False, repr=False)
    position_mm: float = 0.0
    max_effort: float = 0.0
    timeout: float = 3.0

    def short_description(self) -> str:
        return self.label or f"Gripper {self.position_mm:.1f} mm"

    def emit(self, out, step_id, recurse) -> None:
        out.append((
            OP_GRIPPER,
            float(self.position_mm),
            float(self.max_effort),
            max(0.0, float(self.timeout)),
            step_id,
        ))


@dataclass
class WaitStep(Step):
    kind: str = field(default=KIND_WAIT, init=False, repr=False)
    time: float = 1.0

    def short_description(self) -> str:
        return self.label or f"WAIT {self.time:.2f}s"

    def emit(self, out, step_id, recurse) -> None:
        out.append((OP_WAIT, max(0.0, float(self.time)), step_id))


@dataclass
class WaitForStateMachineStep(Step):
    kind: str = field(default=KIND_WAIT_SM, init=False, repr=False)
    state: str = "IDLE"
    timeout: float = 0.0  # seconds; <= 0 means wait forever

    def short_description(self) -> str:
        t = f", timeout {self.timeout:.0f}s" if self.timeout > 0 else ""
        return self.label or f"WAIT estado = {self.state}{t}"

    def emit(self, out, step_id, recurse) -> None:
        out.append((OP_WAIT_SM, str(self.state), max(0.0, float(self.timeout)), step_id))


@dataclass
class WaitForSignalStep(Step):
    kind: str = field(default=KIND_WAIT_SIG, init=False, repr=False)
    io_id: str = "IN1"
    value: int = 1
    edge: str = EDGE_LEVEL
    timeout: float = 0.0  # seconds; <= 0 means wait forever

    def short_description(self) -> str:
        t = f", timeout {self.timeout:.0f}s" if self.timeout > 0 else ""
        edge = EDGE_LABELS_PT.get(self.edge, self.edge)
        return self.label or f"WAIT {self.io_id}={self.value} ({edge}){t}"

    def emit(self, out, step_id, recurse) -> None:
        out.append((
            OP_WAIT_SIG,
            str(self.io_id),
            int(bool(self.value)),
            str(self.edge),
            max(0.0, float(self.timeout)),
            step_id,
        ))


@dataclass
class OutputStep(Step):
    """Drive a digital relay output: OUTPUT_ON / OUTPUT_OFF / OUTPUT_TOGGLE."""

    kind: str = field(default=KIND_OUTPUT, init=False, repr=False)
    io_id: str = "OUT1"
    action: str = OUTPUT_ON

    def short_description(self) -> str:
        act = OUTPUT_ACTION_LABELS_PT.get(self.action, self.action)
        return self.label or f"{act} {self.io_id}"

    def emit(self, out, step_id, recurse) -> None:
        out.append((OP_OUTPUT, str(self.io_id), str(self.action), step_id))


@dataclass
class LineStep(Step):
    kind: str = field(default=KIND_LINE, init=False, repr=False)
    start: PoseDict = field(default_factory=lambda: pose(0.30, -0.10, 0.30))
    end: PoseDict = field(default_factory=lambda: pose(0.30, 0.10, 0.30))
    speed: float = 0.10  # m/s
    orientation_from: str = "start"  # "start" | "end"

    def short_description(self) -> str:
        e = self.end
        return self.label or f"Linha -> ({e['x']:.2f}, {e['y']:.2f}, {e['z']:.2f}) @ {self.speed:.2f}m/s"

    def emit(self, out, step_id, recurse) -> None:
        wps = line_waypoints(self.start, self.end, self.speed,
                             orientation_from=self.orientation_from)
        for p, dt in wps:
            out.append((OP_MOVE, p, dt, step_id))


@dataclass
class ArcStep(Step):
    kind: str = field(default=KIND_ARC, init=False, repr=False)
    p1: PoseDict = field(default_factory=lambda: pose(0.35, -0.10, 0.25))
    p2: PoseDict = field(default_factory=lambda: pose(0.40, 0.00, 0.30))
    p3: PoseDict = field(default_factory=lambda: pose(0.35, 0.10, 0.25))
    speed: float = 0.08  # m/s
    samples: int = 24
    point_to_center: bool = True  # welding: tool aims at circle center

    def short_description(self) -> str:
        tag = "centro" if self.point_to_center else "fixo"
        return self.label or f"Arco ({self.samples} amostras, orient {tag})"

    def emit(self, out, step_id, recurse) -> None:
        fixed = (self.p1["roll"], self.p1["pitch"], self.p1["yaw"])
        wps = arc_waypoints(self.p1, self.p2, self.p3, self.speed,
                            samples=int(self.samples),
                            point_to_center=bool(self.point_to_center),
                            fixed_rpy=fixed)
        for p, dt in wps:
            out.append((OP_MOVE, p, dt, step_id))


@dataclass
class PalletStep(Step):
    kind: str = field(default=KIND_PALLET, init=False, repr=False)
    corner: PoseDict = field(default_factory=lambda: pose(0.30, -0.10, 0.20))
    nx: int = 3
    ny: int = 3
    nz: int = 1
    dx: float = 0.05
    dy: float = 0.05
    dz: float = 0.04
    speed: float = 0.12  # m/s
    approach_height: float = 0.05
    snake: bool = True

    def short_description(self) -> str:
        return self.label or f"Pallet {self.nx}x{self.ny}x{self.nz}"

    def emit(self, out, step_id, recurse) -> None:
        wps = pallet_waypoints(
            self.corner, int(self.nx), int(self.ny), int(self.nz),
            float(self.dx), float(self.dy), float(self.dz), float(self.speed),
            approach_height=float(self.approach_height), snake=bool(self.snake),
        )
        for p, dt in wps:
            out.append((OP_MOVE, p, dt, step_id))


# ---- compound steps --------------------------------------------------------
@dataclass
class LoopStep(Step):
    """Repeat its child ``steps`` ``count`` times (``count <= 0`` = forever)."""

    kind: str = field(default=KIND_LOOP, init=False, repr=False)
    count: int = 3
    steps: List[Step] = field(default_factory=list)

    def short_description(self) -> str:
        rep = "sempre" if self.count <= 0 else f"{self.count}x"
        n = len(self.steps)
        return self.label or f"LOOP {rep} ({n} etapa{'s' if n != 1 else ''})"

    def emit(self, out, step_id, recurse) -> None:
        loop_id = f"L{step_id}"
        out.append((OP_LOOP_INIT, loop_id, int(self.count)))
        out.append((OP_LABEL, f"{loop_id}_top"))
        for child in self.steps:
            recurse(child)
        out.append((OP_LOOP_BACK, loop_id, int(self.count), f"{loop_id}_top"))


@dataclass
class IfStep(Step):
    """Run ``then_steps`` when ``condition`` holds, else ``else_steps``."""

    kind: str = field(default=KIND_IF, init=False, repr=False)
    condition: ConditionExpr = field(default_factory=ConditionExpr)
    then_steps: List[Step] = field(default_factory=list)
    else_steps: List[Step] = field(default_factory=list)

    def short_description(self) -> str:
        cond = self.condition.describe() if not self.condition.is_empty() else "(sempre)"
        n_then = len(self.then_steps)
        has_else = bool(self.else_steps)
        else_part = f", senão {len(self.else_steps)}" if has_else else ""
        return self.label or f"IF {cond} ({n_then}{else_part})"

    def emit(self, out, step_id, recurse) -> None:
        if_id = f"I{step_id}"
        # If the condition is empty, behave as "always true": skip the branch.
        if not self.condition.is_empty():
            out.append((OP_BRANCH, self.condition, f"{if_id}_else", False))
        for child in self.then_steps:
            recurse(child)
        out.append((OP_GOTO, f"{if_id}_end"))
        out.append((OP_LABEL, f"{if_id}_else"))
        for child in self.else_steps:
            recurse(child)
        out.append((OP_LABEL, f"{if_id}_end"))


# registry: kind -> class
_STEP_REGISTRY: Dict[str, type] = {
    KIND_MOVE: MoveStep,
    KIND_GRIPPER: GripperStep,
    KIND_WAIT: WaitStep,
    KIND_WAIT_SM: WaitForStateMachineStep,
    KIND_WAIT_SIG: WaitForSignalStep,
    KIND_OUTPUT: OutputStep,
    KIND_LINE: LineStep,
    KIND_ARC: ArcStep,
    KIND_PALLET: PalletStep,
    KIND_LOOP: LoopStep,
    KIND_IF: IfStep,
}

# Fields that must NOT be passed through the dataclass constructor when
# rebuilding from a dict (they are rebuilt explicitly to preserve types).
_COMPOUND_LAZY_FIELDS = {
    KIND_LOOP: {"steps"},
    KIND_IF: {"condition", "then_steps", "else_steps"},
}


def step_from_dict(data: Dict[str, Any]) -> Step:
    """Rebuild a step from its JSON dict; unknown / non-init keys are ignored."""
    kind = data.get("kind", KIND_MOVE)
    cls = _STEP_REGISTRY.get(kind, MoveStep)
    init_names = {f.name for f in fields(cls) if f.init}
    skip = _COMPOUND_LAZY_FIELDS.get(kind, set())
    kwargs = {k: v for k, v in data.items() if k in init_names and k not in skip}
    obj = cls(**kwargs)

    if isinstance(obj, LoopStep):
        obj.steps = [step_from_dict(c) for c in data.get("steps", [])]
    elif isinstance(obj, IfStep):
        obj.condition = ConditionExpr.from_dict(data.get("condition", {}))
        obj.then_steps = [step_from_dict(c) for c in data.get("then_steps", [])]
        obj.else_steps = [step_from_dict(c) for c in data.get("else_steps", [])]
    return obj


def make_step(kind: str) -> Step:
    """Create a default step of the given kind."""
    cls = _STEP_REGISTRY.get(kind, MoveStep)
    return cls()


def flatten_steps_with_meta(steps: List[Step]) -> Tuple[List[Tuple], Dict[str, Any]]:
    """Flatten a step tree into a linear opcode list (DFS order).

    Returns ``(opcodes, meta)`` where ``meta`` contains:
      - ``pc_to_root_index``: mapping from opcode index to the index of the
        top-level step that produced it.
      - ``action_count``: number of opcodes that are real actions
        (move/gripper/wait/output/etc.).

    Step ids are assigned in DFS order and embedded into action opcodes; the
    compound steps also emit label/goto/branch/loop opcodes for control flow.
    """
    out: List[Tuple] = []
    pc_to_root: Dict[int, int] = {}
    counter = {"n": 0}

    def walk(step: Step, root_index: int) -> None:
        counter["n"] += 1
        sid = _make_step_id(counter["n"], step)
        before = len(out)
        step.emit(out, sid, lambda child: walk(child, root_index))
        for pc in range(before, len(out)):
            op = out[pc]
            if op and op[0] in ACTION_OPS:
                pc_to_root[pc] = root_index

    for root_index, step in enumerate(steps):
        walk(step, root_index)

    meta = {
        "pc_to_root_index": pc_to_root,
        "action_count": sum(1 for op in out if op and op[0] in ACTION_OPS),
    }
    return out, meta


def flatten_steps(steps: List[Step]) -> List[Tuple]:
    """Flatten a step tree into a linear opcode list (DFS order)."""
    return flatten_steps_with_meta(steps)[0]


def build_label_index(opcodes: List[Tuple]) -> Dict[str, int]:
    """Map every ``("label", name)`` opcode to its index in the list."""
    labels: Dict[str, int] = {}
    for i, op in enumerate(opcodes):
        if op and op[0] == OP_LABEL:
            labels[op[1]] = i
    return labels


# ----------------------------------------------------------------------
# Program
# ----------------------------------------------------------------------
@dataclass
class Program:
    name: str = "programa"
    steps: List[Step] = field(default_factory=list)
    version: int = PROGRAM_VERSION
    speed_scale: float = 1.0  # global velocity multiplier applied to every move
    revision: int = 1
    recipe_status: str = RECIPE_DRAFT
    released_by: str = ""
    released_at_utc: str = ""
    release_hash: str = ""
    parameters: Dict[str, Any] = field(default_factory=dict)

    # ---- queries ----
    def __len__(self) -> int:
        return len(self.steps)

    def __iter__(self):
        return iter(self.steps)

    def flatten(self) -> List[Tuple]:
        return flatten_steps(self.steps)

    def flatten_with_meta(self) -> Tuple[List[Tuple], Dict[str, Any]]:
        return flatten_steps_with_meta(self.steps)

    def estimated_steps(self) -> int:
        """Total number of logical steps (including nested children, DFS)."""
        count = 0

        def walk(step: Step) -> None:
            nonlocal count
            count += 1
            if isinstance(step, LoopStep):
                for c in step.steps:
                    walk(c)
            elif isinstance(step, IfStep):
                for c in step.then_steps:
                    walk(c)
                for c in step.else_steps:
                    walk(c)

        for s in self.steps:
            walk(s)
        return count

    def estimated_commands(self) -> int:
        return sum(1 for op in self.flatten() if op and op[0] in ACTION_OPS)

    # ---- (de)serialization ----
    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "version": self.version,
            "speed_scale": float(self.speed_scale),
            "revision": max(1, int(self.revision)),
            "recipe_status": (
                self.recipe_status if self.recipe_status in RECIPE_STATUSES else RECIPE_DRAFT
            ),
            "released_by": self.released_by,
            "released_at_utc": self.released_at_utc,
            "release_hash": self.release_hash,
            "parameters": dict(self.parameters),
            "steps": [s.to_dict() for s in self.steps],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Program":
        steps = [step_from_dict(s) for s in data.get("steps", [])]
        return cls(
            name=str(data.get("name", "programa")),
            version=int(data.get("version", PROGRAM_VERSION)),
            speed_scale=float(data.get("speed_scale", 1.0)),
            revision=max(1, int(data.get("revision", 1))),
            recipe_status=str(data.get("recipe_status", RECIPE_DRAFT)),
            released_by=str(data.get("released_by", "")),
            released_at_utc=str(data.get("released_at_utc", "")),
            release_hash=str(data.get("release_hash", "")),
            parameters=(
                dict(data.get("parameters", {}))
                if isinstance(data.get("parameters", {}), dict)
                else {}
            ),
            steps=steps,
        )

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)

    @classmethod
    def from_json(cls, text: str) -> "Program":
        return cls.from_dict(json.loads(text))

    # ---- industrial-recipe lifecycle ----
    def release_payload(self) -> Dict[str, Any]:
        """Return the immutable content covered by a recipe release hash."""
        return {
            "name": self.name,
            "version": self.version,
            "speed_scale": float(self.speed_scale),
            "revision": max(1, int(self.revision)),
            "parameters": dict(self.parameters),
            "steps": [step.to_dict() for step in self.steps],
        }

    def compute_release_hash(self) -> str:
        canonical = json.dumps(
            self.release_payload(),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        return hashlib.sha256(canonical).hexdigest()

    def release(self, operator: str) -> str:
        """Release the current revision and return its integrity hash."""
        identity = str(operator).strip()
        if not identity:
            raise ValueError("a released recipe requires an identified operator")
        if not self.steps:
            raise ValueError("an empty recipe cannot be released")
        self.recipe_status = RECIPE_RELEASED
        self.released_by = identity
        self.released_at_utc = datetime.now(timezone.utc).isoformat(
            timespec="milliseconds"
        ).replace("+00:00", "Z")
        self.release_hash = self.compute_release_hash()
        return self.release_hash

    def verify_release(self) -> tuple[bool, str]:
        if self.recipe_status != RECIPE_RELEASED:
            return False, "receita não está liberada"
        if not self.release_hash:
            return False, "receita liberada sem hash de integridade"
        if self.compute_release_hash() != self.release_hash:
            return False, "conteúdo da receita foi alterado após a liberação"
        if not self.released_by or not self.released_at_utc:
            return False, "metadados de liberação incompletos"
        return True, ""

    def start_new_revision(self) -> None:
        """Return a released/obsolete recipe to an editable next revision."""
        self.revision = max(1, int(self.revision)) + 1
        self.recipe_status = RECIPE_DRAFT
        self.released_by = ""
        self.released_at_utc = ""
        self.release_hash = ""

    # ---- edit helpers ----
    def add(self, step: Step, index: int | None = None) -> None:
        if index is None:
            self.steps.append(step)
        else:
            self.steps.insert(index, step)

    def remove(self, index: int) -> Step | None:
        if 0 <= index < len(self.steps):
            return self.steps.pop(index)
        return None

    def move(self, index: int, delta: int) -> int:
        """Move a step by ``delta`` positions; returns the new index."""
        new_index = index + delta
        if not (0 <= new_index < len(self.steps)):
            return index
        step = self.steps.pop(index)
        self.steps.insert(new_index, step)
        return new_index
