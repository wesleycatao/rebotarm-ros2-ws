"""Robust sampling of vision-guided recipe position envelopes.

The nominal Cartesian poses stored in a recipe describe one observation of a
part.  A production recipe may, however, be reused while the part appears at
different positions.  This module turns the declared work-area geometry and
localization uncertainty into a bounded, deterministic set of translated
program variants used by preflight, collision and joint-dynamics checks.

The sampling is deliberately conservative and bounded.  It is evidence over
the declared envelope, not a mathematical proof between samples and not a
replacement for MoveIt's online collision-aware planning.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from dataclasses import dataclass
from typing import Iterable

from .program import (
    ArcStep,
    IfStep,
    LineStep,
    LoopStep,
    MoveStep,
    PalletStep,
    Program,
    Step,
)


VISION_ENVELOPE_KEY = "_vision_envelope"
VISION_ENVELOPE_ANALYSIS_KEY = "_vision_envelope_analysis"
VISION_ENVELOPE_VERSION = 1
MAX_ENVELOPE_SAMPLES = 32


def _finite(value: object, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} inválido") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} deve ser finito")
    return result


def _xy_pair(value: object, label: str) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        raise ValueError(f"{label} deve conter X e Y")
    return _finite(value[0], f"{label} X"), _finite(value[1], f"{label} Y")


@dataclass(frozen=True)
class VisionEnvelopeSpec:
    """Validated recipe contract for a vision-variable Cartesian target."""

    anchor_m: tuple[float, float, float]
    affected_step_labels: tuple[str, ...]
    shape: str
    polygon_m: tuple[tuple[float, float], ...] = ()
    circle_m: tuple[float, float, float] | None = None
    point_m: tuple[float, float] | None = None
    z_bounds_m: tuple[float, float] = (0.0, 0.0)
    uncertainty_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    calibration_error_m: float = 0.0
    calibration_integrity_hash: str = ""
    production_ready: bool = False
    source: str = ""

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> "VisionEnvelopeSpec":
        if int(data.get("version", 0)) != VISION_ENVELOPE_VERSION:
            raise ValueError("versão do envelope de visão não suportada")
        anchor_raw = data.get("anchor_m", ())
        if not isinstance(anchor_raw, (list, tuple)) or len(anchor_raw) < 3:
            raise ValueError("âncora do envelope deve conter X, Y e Z")
        anchor = tuple(
            _finite(anchor_raw[index], f"âncora {axis}")
            for index, axis in enumerate(("X", "Y", "Z"))
        )
        labels_raw = data.get("affected_step_labels", ())
        if not isinstance(labels_raw, (list, tuple)):
            raise ValueError("etapas relativas à visão devem ser uma lista")
        labels = tuple(
            dict.fromkeys(
                str(value).strip()
                for value in labels_raw
                if str(value).strip()
            )
        )
        if not labels:
            raise ValueError("selecione ao menos uma etapa relativa à visão")

        shape = str(data.get("shape", "point")).strip().lower()
        polygon: tuple[tuple[float, float], ...] = ()
        circle: tuple[float, float, float] | None = None
        point: tuple[float, float] | None = None
        if shape == "polygon":
            raw_polygon = data.get("polygon_m", ())
            if not isinstance(raw_polygon, (list, tuple)):
                raise ValueError("polígono do envelope inválido")
            polygon = tuple(
                _xy_pair(item, f"vértice {index + 1}")
                for index, item in enumerate(raw_polygon)
            )
            if len(polygon) < 3:
                raise ValueError("polígono do envelope requer ao menos três vértices")
        elif shape == "circle":
            raw_circle = data.get("circle_m", ())
            if not isinstance(raw_circle, (list, tuple)) or len(raw_circle) < 3:
                raise ValueError("círculo do envelope deve conter centro e raio")
            cx, cy = _xy_pair(raw_circle, "centro do círculo")
            radius = _finite(raw_circle[2], "raio do círculo")
            if radius <= 0.0:
                raise ValueError("raio do envelope deve ser positivo")
            circle = (cx, cy, radius)
        elif shape == "point":
            point = _xy_pair(data.get("point_m", anchor[:2]), "posição observada")
        else:
            raise ValueError(f"forma de envelope desconhecida: {shape}")

        z_raw = data.get("z_bounds_m", (anchor[2], anchor[2]))
        if not isinstance(z_raw, (list, tuple)) or len(z_raw) < 2:
            raise ValueError("limites Z do envelope devem conter mínimo e máximo")
        z_bounds = (
            _finite(z_raw[0], "Z mínimo"),
            _finite(z_raw[1], "Z máximo"),
        )
        if z_bounds[0] > z_bounds[1]:
            raise ValueError("Z mínimo é maior que Z máximo")

        uncertainty_raw = data.get("uncertainty_m", (0.0, 0.0, 0.0))
        if not isinstance(uncertainty_raw, (list, tuple)) or len(uncertainty_raw) < 3:
            raise ValueError("incerteza deve conter X, Y e Z")
        uncertainty = tuple(
            _finite(uncertainty_raw[index], f"incerteza {axis}")
            for index, axis in enumerate(("X", "Y", "Z"))
        )
        if any(value < 0.0 for value in uncertainty):
            raise ValueError("incertezas não podem ser negativas")
        calibration_error = _finite(
            data.get("calibration_error_m", 0.0),
            "erro máximo da calibração",
        )
        if calibration_error < 0.0:
            raise ValueError("erro da calibração não pode ser negativo")

        return cls(
            anchor_m=(anchor[0], anchor[1], anchor[2]),
            affected_step_labels=labels,
            shape=shape,
            polygon_m=polygon,
            circle_m=circle,
            point_m=point,
            z_bounds_m=z_bounds,
            uncertainty_m=(uncertainty[0], uncertainty[1], uncertainty[2]),
            calibration_error_m=calibration_error,
            calibration_integrity_hash=str(
                data.get("calibration_integrity_hash", "")
            ).strip(),
            production_ready=bool(data.get("production_ready", shape != "point")),
            source=str(data.get("source", "")).strip(),
        )

    @property
    def effective_uncertainty_m(self) -> tuple[float, float, float]:
        # Calibration is planar, so its measured maximum residual expands X/Y.
        return (
            self.uncertainty_m[0] + self.calibration_error_m,
            self.uncertainty_m[1] + self.calibration_error_m,
            self.uncertainty_m[2],
        )

    def validate_for_production(self) -> None:
        if not self.production_ready or self.shape == "point":
            raise ValueError(
                "envelope contém apenas a posição observada; configure a área "
                "operacional calibrada na página Visão"
            )
        if not self.calibration_integrity_hash:
            raise ValueError(
                "área operacional não está vinculada a uma calibração aprovada"
            )

    def sample_positions(self) -> tuple[tuple[float, float, float], ...]:
        """Return deterministic critical samples including uncertainty margins."""

        ux, uy, uz = self.effective_uncertainty_m
        xy: list[tuple[float, float]] = []
        if self.shape == "circle" and self.circle_m is not None:
            cx, cy, radius = self.circle_m
            expanded = radius + math.hypot(ux, uy)
            xy.append((cx, cy))
            xy.extend(
                (
                    cx + expanded * math.cos(index * math.pi / 4.0),
                    cy + expanded * math.sin(index * math.pi / 4.0),
                )
                for index in range(8)
            )
        elif self.shape == "polygon":
            polygon = self.polygon_m
            x_min = min(point[0] for point in polygon) - ux
            x_max = max(point[0] for point in polygon) + ux
            y_min = min(point[1] for point in polygon) - uy
            y_max = max(point[1] for point in polygon) + uy
            # The expanded bounding grid intentionally includes positions
            # outside a non-rectangular polygon.  This is conservative and
            # covers component-wise ±X/±Y localization error without an
            # unbounded vertex × uncertainty product.
            xs = (x_min, (x_min + x_max) * 0.5, x_max)
            ys = (y_min, (y_min + y_max) * 0.5, y_max)
            xy.extend((x, y) for x in xs for y in ys)
        elif self.point_m is not None:
            x, y = self.point_m
            xy = [
                (x - ux, y - uy),
                (x - ux, y + uy),
                (x, y),
                (x + ux, y - uy),
                (x + ux, y + uy),
            ]

        z_values = [
            self.z_bounds_m[0] - uz,
            (self.z_bounds_m[0] + self.z_bounds_m[1]) * 0.5,
            self.z_bounds_m[1] + uz,
        ]
        if abs(z_values[0] - z_values[-1]) <= 1e-12:
            z_values = [z_values[0]]
        samples: list[tuple[float, float, float]] = []
        seen: set[tuple[float, float, float]] = set()
        for x, y in xy:
            for z in z_values:
                key = (round(x, 9), round(y, 9), round(z, 9))
                if key not in seen:
                    seen.add(key)
                    samples.append(key)
        if len(samples) > MAX_ENVELOPE_SAMPLES:
            stride = max(1, math.ceil(len(samples) / MAX_ENVELOPE_SAMPLES))
            samples = samples[::stride][:MAX_ENVELOPE_SAMPLES]
        return tuple(samples)

    def signature(self) -> str:
        payload = {
            "anchor_m": self.anchor_m,
            "affected_step_labels": self.affected_step_labels,
            "shape": self.shape,
            "polygon_m": self.polygon_m,
            "circle_m": self.circle_m,
            "point_m": self.point_m,
            "z_bounds_m": self.z_bounds_m,
            "uncertainty_m": self.uncertainty_m,
            "calibration_error_m": self.calibration_error_m,
            "calibration_integrity_hash": self.calibration_integrity_hash,
            "production_ready": self.production_ready,
        }
        canonical = json.dumps(
            payload, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
        return hashlib.sha256(canonical).hexdigest()


def envelope_spec(program: Program) -> VisionEnvelopeSpec | None:
    raw = program.parameters.get(VISION_ENVELOPE_KEY)
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ValueError("envelope de visão da receita deve ser um objeto")
    return VisionEnvelopeSpec.from_dict(raw)


def bind_calibration_error(program: Program, calibration_reference: dict[str, object]) -> None:
    """Bind the approved calibration's measured max residual to the recipe."""

    raw = program.parameters.get(VISION_ENVELOPE_KEY)
    if not isinstance(raw, dict):
        return
    updated = dict(raw)
    active_hash = str(calibration_reference.get("integrity_hash", "")).strip()
    bound_hash = str(updated.get("calibration_integrity_hash", "")).strip()
    if bool(updated.get("production_ready", False)):
        if not active_hash:
            raise ValueError("perfil de calibração ativo sem hash de integridade")
        if not bound_hash:
            raise ValueError(
                "área operacional não está vinculada à calibração que a projetou"
            )
        if bound_hash != active_hash:
            raise ValueError(
                "a área operacional foi projetada com outra calibração; "
                "reaplique o limite na página Visão"
            )
    updated["calibration_error_m"] = max(
        0.0,
        _finite(
            calibration_reference.get("max_error_m", 0.0),
            "erro da calibração",
        ),
    )
    program.parameters[VISION_ENVELOPE_KEY] = updated


def _translated_pose(
    pose: dict[str, float],
    offset: tuple[float, float, float],
) -> dict[str, float]:
    result = dict(pose)
    for axis, delta in zip(("x", "y", "z"), offset):
        result[axis] = float(result.get(axis, 0.0)) + delta
    return result


def _translate_steps(
    steps: Iterable[Step],
    labels: set[str],
    offset: tuple[float, float, float],
    matched: dict[str, int],
) -> None:
    for step in steps:
        if step.label in labels:
            matched[step.label] = matched.get(step.label, 0) + 1
            if isinstance(step, MoveStep):
                step.pose = _translated_pose(step.pose, offset)
            elif isinstance(step, LineStep):
                step.start = _translated_pose(step.start, offset)
                step.end = _translated_pose(step.end, offset)
            elif isinstance(step, ArcStep):
                step.p1 = _translated_pose(step.p1, offset)
                step.p2 = _translated_pose(step.p2, offset)
                step.p3 = _translated_pose(step.p3, offset)
            elif isinstance(step, PalletStep):
                step.corner = _translated_pose(step.corner, offset)
            else:
                raise ValueError(
                    f"etapa relativa à visão '{step.label}' não é cartesiana"
                )
        if isinstance(step, LoopStep):
            _translate_steps(step.steps, labels, offset, matched)
        elif isinstance(step, IfStep):
            _translate_steps(step.then_steps, labels, offset, matched)
            _translate_steps(step.else_steps, labels, offset, matched)


def _step_labels(steps: Iterable[Step]) -> set[str]:
    labels: set[str] = set()
    for step in steps:
        if step.label:
            labels.add(step.label)
        if isinstance(step, LoopStep):
            labels.update(_step_labels(step.steps))
        elif isinstance(step, IfStep):
            labels.update(_step_labels(step.then_steps))
            labels.update(_step_labels(step.else_steps))
    return labels


def translated_variant(
    program: Program,
    spec: VisionEnvelopeSpec,
    sample: tuple[float, float, float],
) -> Program:
    variant = copy.deepcopy(program)
    offset = tuple(sample[index] - spec.anchor_m[index] for index in range(3))
    matched: dict[str, int] = {}
    _translate_steps(
        variant.steps,
        set(spec.affected_step_labels),
        (offset[0], offset[1], offset[2]),
        matched,
    )
    missing = set(spec.affected_step_labels) - set(matched)
    execution_subset = bool(
        program.parameters.get("_execution_subset_start_index", 0)
    )
    if missing and not execution_subset:
        raise ValueError(
            "etapas relativas à visão não encontradas: " + ", ".join(sorted(missing))
        )
    duplicated = sorted(
        label for label, count in matched.items() if count != 1
    )
    if duplicated:
        raise ValueError(
            "rótulos de etapas relativas à visão devem ser únicos: "
            + ", ".join(duplicated)
        )
    return variant


def envelope_variants(
    program: Program,
) -> tuple[tuple[str, Program], ...]:
    """Return sampled variants, or the nominal program when no contract exists."""

    spec = envelope_spec(program)
    if spec is None:
        return (("nominal", program),)
    available_labels = _step_labels(program.steps)
    if (
        bool(program.parameters.get("_execution_subset_start_index", 0))
        and not available_labels.intersection(spec.affected_step_labels)
    ):
        return (("subconjunto sem etapas relativas à visão", program),)
    variants: list[tuple[str, Program]] = []
    for index, sample in enumerate(spec.sample_positions(), start=1):
        label = (
            f"amostra {index}/{len(spec.sample_positions())} "
            f"({sample[0]:+.3f}, {sample[1]:+.3f}, {sample[2]:+.3f})"
        )
        variants.append((label, translated_variant(program, spec, sample)))
    return tuple(variants)
