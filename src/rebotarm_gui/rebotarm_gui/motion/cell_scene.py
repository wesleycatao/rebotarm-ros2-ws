"""Persistent cell geometry and conservative TCP-clearance analysis.

The first scene-editor increment deliberately supports axis-aligned boxes.
They cover the most common controlled-cell fixtures (table, fences, chutes and
keep-out zones) while keeping persistence and clearance checks deterministic.
MoveIt remains the authority for full-link collision checking.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Sequence

from PySide6.QtCore import QObject, Signal

SCENE_SCHEMA_VERSION = 1
OBSTACLE = "obstaculo"
KEEP_OUT = "zona_proibida"
VALID_KINDS = (OBSTACLE, KEEP_OUT)
GUI_OBJECT_PREFIX = "rebotarm_gui__"
MONITORED_SHAPES = ("box", "sphere", "cylinder", "cone", "mesh", "plane")


def _finite(value: object, label: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{label} deve ser finito")
    return parsed


@dataclass(frozen=True)
class CellObstacle:
    """One axis-aligned box expressed in the scene reference frame."""

    object_id: str
    name: str
    kind: str = OBSTACLE
    center: tuple[float, float, float] = (0.4, 0.0, 0.2)
    size: tuple[float, float, float] = (0.2, 0.2, 0.4)
    margin_m: float = 0.0
    enabled: bool = True

    def validated(self) -> "CellObstacle":
        object_id = str(self.object_id).strip()
        if not object_id:
            raise ValueError("ID do objeto não pode ser vazio")
        if any(char not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for char in object_id):
            raise ValueError(
                f"ID '{object_id}' contém caracteres inválidos; use letras, números, _ ou -"
            )
        name = str(self.name).strip() or object_id
        kind = str(self.kind).strip().lower()
        if kind not in VALID_KINDS:
            raise ValueError(f"tipo inválido para '{name}': {kind}")
        raw_center = tuple(self.center)
        raw_size = tuple(self.size)
        if len(raw_center) != 3 or len(raw_size) != 3:
            raise ValueError(f"centro e dimensões de '{name}' devem conter X, Y e Z")
        center = tuple(
            _finite(value, f"centro {axis} de '{name}'")
            for axis, value in zip("XYZ", raw_center)
        )
        size = tuple(
            _finite(value, f"dimensão {axis} de '{name}'")
            for axis, value in zip("XYZ", raw_size)
        )
        if any(value <= 0.0 for value in size):
            raise ValueError(f"dimensões de '{name}' devem ser maiores que zero")
        margin = _finite(self.margin_m, f"margem de '{name}'")
        if margin < 0.0:
            raise ValueError(f"margem de '{name}' não pode ser negativa")
        return CellObstacle(
            object_id=object_id,
            name=name,
            kind=kind,
            center=center,
            size=size,
            margin_m=margin,
            enabled=bool(self.enabled),
        )

    def bounds(self, inflation_m: float = 0.0) -> tuple[tuple[float, ...], tuple[float, ...]]:
        inflation = max(0.0, float(inflation_m))
        half = tuple(value / 2.0 + inflation for value in self.size)
        lower = tuple(value - radius for value, radius in zip(self.center, half))
        upper = tuple(value + radius for value, radius in zip(self.center, half))
        return lower, upper


@dataclass(frozen=True)
class CellScene:
    """Versioned geometry used by the GUI and mirrored into MoveIt."""

    frame_id: str = "base_link"
    minimum_clearance_m: float = 0.05
    tcp_radius_m: float = 0.035
    obstacles: tuple[CellObstacle, ...] = ()
    revision: int = 1
    updated_at: str = ""
    schema_version: int = SCENE_SCHEMA_VERSION

    def validated(self) -> "CellScene":
        frame = str(self.frame_id).strip().lstrip("/")
        if not frame:
            raise ValueError("frame da cena não pode ser vazio")
        clearance = _finite(self.minimum_clearance_m, "folga mínima")
        tcp_radius = _finite(self.tcp_radius_m, "raio do envelope TCP")
        if clearance < 0.0 or tcp_radius < 0.0:
            raise ValueError("folga mínima e raio TCP não podem ser negativos")
        obstacles = tuple(obstacle.validated() for obstacle in self.obstacles)
        ids = [obstacle.object_id for obstacle in obstacles]
        if len(ids) != len(set(ids)):
            raise ValueError("IDs de objetos da célula devem ser únicos")
        updated = str(self.updated_at).strip() or datetime.now(timezone.utc).isoformat()
        return CellScene(
            frame_id=frame,
            minimum_clearance_m=clearance,
            tcp_radius_m=tcp_radius,
            obstacles=obstacles,
            revision=max(1, int(self.revision)),
            updated_at=updated,
            schema_version=SCENE_SCHEMA_VERSION,
        )

    @property
    def enabled_obstacles(self) -> tuple[CellObstacle, ...]:
        return tuple(obstacle for obstacle in self.obstacles if obstacle.enabled)

    @property
    def configured(self) -> bool:
        return bool(self.enabled_obstacles)

    def payload(self, *, include_metadata: bool = True) -> dict[str, object]:
        data: dict[str, object] = {
            "schema_version": SCENE_SCHEMA_VERSION,
            "frame_id": self.frame_id,
            "minimum_clearance_m": self.minimum_clearance_m,
            "tcp_radius_m": self.tcp_radius_m,
            "obstacles": [asdict(obstacle) for obstacle in self.obstacles],
        }
        if include_metadata:
            data["revision"] = self.revision
            data["updated_at"] = self.updated_at
        return data

    def signature(self) -> str:
        canonical = json.dumps(
            self.payload(include_metadata=False),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class MonitoredGeometry:
    """One read-only shape imported from MoveIt's monitored planning scene.

    The ROS message is converted to this bounded, immutable representation
    before crossing into Qt.  A collision object may therefore produce more
    than one geometry row.  Poses use ``x, y, z, qx, qy, qz, qw``.
    """

    object_id: str
    frame_id: str
    shape: str
    pose: tuple[float, float, float, float, float, float, float]
    dimensions: tuple[float, ...] = ()
    vertices: tuple[tuple[float, float, float], ...] = ()
    triangles: tuple[tuple[int, int, int], ...] = ()
    plane_coefficients: tuple[float, ...] = ()
    attached_to: str = ""
    gui_managed: bool = False
    source_frame_id: str = ""
    source_pose: tuple[float, ...] = ()
    transform_status: str = "native"

    def validated(self) -> "MonitoredGeometry":
        object_id = str(self.object_id).strip()
        frame_id = str(self.frame_id).strip().lstrip("/")
        shape = str(self.shape).strip().lower()
        if not object_id:
            raise ValueError("objeto monitorado sem ID")
        if not frame_id:
            raise ValueError(f"objeto monitorado '{object_id}' sem frame")
        if shape not in MONITORED_SHAPES:
            raise ValueError(f"geometria monitorada não suportada: {shape}")
        pose = tuple(_finite(value, f"pose de '{object_id}'") for value in self.pose)
        if len(pose) != 7:
            raise ValueError(f"pose de '{object_id}' deve conter 7 valores")
        quaternion_norm = math.sqrt(sum(value * value for value in pose[3:]))
        if quaternion_norm < 1e-9:
            raise ValueError(f"quaternion inválido em '{object_id}'")
        pose = pose[:3] + tuple(value / quaternion_norm for value in pose[3:])
        dimensions = tuple(
            _finite(value, f"dimensão de '{object_id}'")
            for value in self.dimensions
        )
        if any(value < 0.0 for value in dimensions):
            raise ValueError(f"dimensões negativas em '{object_id}'")
        plane_coefficients = tuple(
            _finite(value, f"coeficiente de plano de '{object_id}'")
            for value in self.plane_coefficients
        )
        if shape == "plane":
            if len(plane_coefficients) != 4:
                raise ValueError(
                    f"plano monitorado '{object_id}' deve conter 4 coeficientes"
                )
            normal_norm = math.sqrt(
                sum(value * value for value in plane_coefficients[:3])
            )
            if normal_norm < 1e-9:
                raise ValueError(
                    f"normal do plano monitorado '{object_id}' é inválida"
                )
            plane_coefficients = tuple(
                value / normal_norm for value in plane_coefficients
            )
            if not dimensions or dimensions[0] <= 0.0:
                raise ValueError(
                    f"extensão visual do plano '{object_id}' é inválida"
                )
            dimensions = (dimensions[0],)
        elif plane_coefficients:
            raise ValueError(
                f"coeficientes de plano inesperados em '{object_id}'"
            )
        vertices = tuple(
            tuple(_finite(value, f"vértice de '{object_id}'") for value in vertex)
            for vertex in self.vertices
        )
        if any(len(vertex) != 3 for vertex in vertices):
            raise ValueError(f"vértice inválido em '{object_id}'")
        triangles = tuple(tuple(int(value) for value in face) for face in self.triangles)
        if any(
            len(face) != 3
            or any(index < 0 or index >= len(vertices) for index in face)
            for face in triangles
        ):
            raise ValueError(f"triângulo inválido em '{object_id}'")
        source_frame_id = (
            str(self.source_frame_id).strip().lstrip("/") or frame_id
        )
        source_pose = tuple(
            _finite(value, f"pose original de '{object_id}'")
            for value in (self.source_pose or pose)
        )
        if len(source_pose) != 7:
            raise ValueError(
                f"pose original de '{object_id}' deve conter 7 valores"
            )
        source_quaternion_norm = math.sqrt(
            sum(value * value for value in source_pose[3:])
        )
        if source_quaternion_norm < 1e-9:
            raise ValueError(f"quaternion original inválido em '{object_id}'")
        source_pose = source_pose[:3] + tuple(
            value / source_quaternion_norm for value in source_pose[3:]
        )
        transform_status = str(self.transform_status).strip().lower()
        if transform_status not in {"native", "resolved", "unresolved"}:
            raise ValueError(
                f"estado TF inválido em '{object_id}': {transform_status}"
            )
        return MonitoredGeometry(
            object_id=object_id,
            frame_id=frame_id,
            shape=shape,
            pose=pose,
            dimensions=dimensions,
            vertices=vertices,
            triangles=triangles,
            plane_coefficients=plane_coefficients,
            attached_to=str(self.attached_to).strip(),
            gui_managed=bool(
                self.gui_managed or object_id.startswith(GUI_OBJECT_PREFIX)
            ),
            source_frame_id=source_frame_id,
            source_pose=source_pose,
            transform_status=transform_status,
        )

    @classmethod
    def from_payload(cls, data: dict[str, object]) -> "MonitoredGeometry":
        return cls(
            object_id=str(data.get("object_id", "")),
            frame_id=str(data.get("frame_id", "")),
            shape=str(data.get("shape", "")),
            pose=tuple(data.get("pose", ())),
            dimensions=tuple(data.get("dimensions", ())),
            vertices=tuple(tuple(vertex) for vertex in data.get("vertices", ())),
            triangles=tuple(
                tuple(triangle) for triangle in data.get("triangles", ())
            ),
            plane_coefficients=tuple(
                data.get("plane_coefficients", ())
            ),
            attached_to=str(data.get("attached_to", "")),
            gui_managed=bool(data.get("gui_managed", False)),
            source_frame_id=str(data.get("source_frame_id", "")),
            source_pose=tuple(data.get("source_pose", ())),
            transform_status=str(data.get("transform_status", "native")),
        ).validated()

    def payload(self) -> dict[str, object]:
        return asdict(self)

    def with_display_transform(
        self,
        *,
        frame_id: object,
        pose: object,
        transform_status: object,
    ) -> "MonitoredGeometry":
        """Return a pose-only TF update without revalidating large mesh data."""

        resolved_frame = str(frame_id).strip().lstrip("/")
        if not resolved_frame:
            raise ValueError(f"objeto monitorado '{self.object_id}' sem frame")
        try:
            resolved_pose = tuple(
                _finite(value, f"pose TF de '{self.object_id}'")
                for value in pose
            )
        except TypeError as exc:
            raise ValueError(
                f"pose TF de '{self.object_id}' inválida"
            ) from exc
        if len(resolved_pose) != 7:
            raise ValueError(
                f"pose TF de '{self.object_id}' deve conter 7 valores"
            )
        quaternion_norm = math.sqrt(
            sum(value * value for value in resolved_pose[3:])
        )
        if quaternion_norm < 1e-9:
            raise ValueError(f"quaternion TF inválido em '{self.object_id}'")
        resolved_pose = resolved_pose[:3] + tuple(
            value / quaternion_norm for value in resolved_pose[3:]
        )
        status = str(transform_status).strip().lower()
        if status not in {"native", "resolved", "unresolved"}:
            raise ValueError(
                f"estado TF inválido em '{self.object_id}': {status}"
            )
        return replace(
            self,
            frame_id=resolved_frame,
            pose=resolved_pose,
            transform_status=status,
        )


def monitored_scene_signature(
    geometries: Iterable[MonitoredGeometry],
    octomap: dict[str, object] | None = None,
) -> str:
    """Stable hash of imported collision geometry, independent of ROS stamps."""

    payload = []
    for geometry in sorted(
        geometries,
        key=lambda item: (
            item.object_id,
            item.attached_to,
            item.shape,
            item.source_pose or item.pose,
        ),
    ):
        item = geometry.validated()
        data = item.payload()
        # TF is a display concern. Hash the source geometry so attached
        # objects do not invalidate traceability merely because the arm moved.
        data["frame_id"] = item.source_frame_id
        data["pose"] = item.source_pose
        data.pop("source_frame_id", None)
        data.pop("source_pose", None)
        data.pop("transform_status", None)
        payload.append(data)
    canonical = json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    geometry_digest = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
    if not octomap or not bool(octomap.get("present", False)):
        return geometry_digest
    octomap_payload = {
        "frame_id": str(octomap.get("frame_id", "")),
        "pose": tuple(octomap.get("pose", ())),
        "tree_id": str(octomap.get("tree_id", "")),
        "resolution_m": float(octomap.get("resolution_m", 0.0)),
        "binary": bool(octomap.get("binary", False)),
        "data_bytes": int(octomap.get("data_bytes", 0)),
        "content_sha256": str(octomap.get("content_sha256", "")),
    }
    combined = json.dumps(
        {
            "geometry_sha256": geometry_digest,
            "octomap": octomap_payload,
        },
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(combined.encode("utf-8")).hexdigest()


def validated_octomap_metadata(
    data: object,
) -> dict[str, object]:
    """Validate the bounded, metadata-only OctoMap representation."""

    if data is None:
        return {"present": False}
    if not isinstance(data, dict):
        raise ValueError("metadados OctoMap inválidos")
    if not bool(data.get("present", False)):
        return {"present": False}
    frame_id = str(data.get("frame_id", "")).strip().lstrip("/")
    if not frame_id:
        raise ValueError("OctoMap sem frame de referência")
    pose = tuple(
        _finite(value, "pose do OctoMap")
        for value in data.get("pose", ())
    )
    if len(pose) != 7:
        raise ValueError("pose do OctoMap deve conter 7 valores")
    quaternion_norm = math.sqrt(sum(value * value for value in pose[3:]))
    if quaternion_norm < 1e-9:
        raise ValueError("quaternion do OctoMap inválido")
    pose = pose[:3] + tuple(
        value / quaternion_norm for value in pose[3:]
    )
    resolution = _finite(
        data.get("resolution_m", 0.0),
        "resolução do OctoMap",
    )
    if resolution <= 0.0:
        raise ValueError("resolução do OctoMap deve ser positiva")
    data_bytes = int(data.get("data_bytes", -1))
    if data_bytes <= 0:
        raise ValueError("OctoMap sem dados ocupados serializados")
    digest = str(data.get("content_sha256", "")).strip().lower()
    if len(digest) != 64 or any(
        character not in "0123456789abcdef" for character in digest
    ):
        raise ValueError("hash do OctoMap inválido")
    return {
        "present": True,
        "frame_id": frame_id,
        "pose": pose,
        "tree_id": str(data.get("tree_id", "")),
        "resolution_m": resolution,
        "binary": bool(data.get("binary", False)),
        "data_bytes": data_bytes,
        "content_sha256": digest,
        "render_status": "metadata_only",
    }


def _rotate_vector(
    vector: Sequence[float],
    quaternion: Sequence[float],
) -> tuple[float, float, float]:
    """Rotate one vector by a normalized ``x, y, z, w`` quaternion."""

    x, y, z, w = (float(value) for value in quaternion)
    vx, vy, vz = (float(value) for value in vector)
    # Quaternion-vector rotation expanded to avoid a numpy dependency here.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def monitored_world_vertices(
    geometry: MonitoredGeometry,
) -> tuple[tuple[float, float, float], ...]:
    """Return bounded display vertices transformed by the imported pose."""

    item = geometry.validated()
    local: tuple[tuple[float, float, float], ...]
    if item.shape == "box" and len(item.dimensions) >= 3:
        hx, hy, hz = (value / 2.0 for value in item.dimensions[:3])
        local = tuple(
            (x, y, z)
            for z in (-hz, hz)
            for y in (-hy, hy)
            for x in (-hx, hx)
        )
    elif item.shape == "sphere" and item.dimensions:
        radius = item.dimensions[0]
        local = tuple(
            (x, y, z)
            for z in (-radius, radius)
            for y in (-radius, radius)
            for x in (-radius, radius)
        )
    elif item.shape in {"cylinder", "cone"} and len(item.dimensions) >= 2:
        height, radius = item.dimensions[:2]
        local = tuple(
            (x, y, z)
            for z in (-height / 2.0, height / 2.0)
            for y in (-radius, radius)
            for x in (-radius, radius)
        )
    elif item.shape == "mesh":
        local = item.vertices
    elif item.shape == "plane":
        a, b, c, d = item.plane_coefficients
        normal = (a, b, c)
        origin = (-d * a, -d * b, -d * c)
        reference = (0.0, 0.0, 1.0) if abs(c) < 0.9 else (1.0, 0.0, 0.0)
        ux = normal[1] * reference[2] - normal[2] * reference[1]
        uy = normal[2] * reference[0] - normal[0] * reference[2]
        uz = normal[0] * reference[1] - normal[1] * reference[0]
        u_norm = math.sqrt(ux * ux + uy * uy + uz * uz)
        unit_u = (ux / u_norm, uy / u_norm, uz / u_norm)
        unit_v = (
            normal[1] * unit_u[2] - normal[2] * unit_u[1],
            normal[2] * unit_u[0] - normal[0] * unit_u[2],
            normal[0] * unit_u[1] - normal[1] * unit_u[0],
        )
        half = item.dimensions[0] / 2.0
        local = tuple(
            tuple(
                origin[axis] + first * half * unit_u[axis]
                + second * half * unit_v[axis]
                for axis in range(3)
            )
            for first, second in (
                (-1.0, -1.0),
                (1.0, -1.0),
                (1.0, 1.0),
                (-1.0, 1.0),
            )
        )
    else:
        local = ()
    translation = item.pose[:3]
    quaternion = item.pose[3:]
    return tuple(
        tuple(
            translated + offset
            for translated, offset in zip(
                _rotate_vector(vertex, quaternion),
                translation,
            )
        )
        for vertex in local
    )


def default_cell_scene_path() -> Path:
    config_home = os.environ.get("XDG_CONFIG_HOME")
    root = Path(config_home).expanduser() if config_home else Path.home() / ".config"
    return root / "rebotarm_gui" / "cell_scene.json"


def scene_from_dict(data: dict[str, object]) -> CellScene:
    if int(data.get("schema_version", 0)) != SCENE_SCHEMA_VERSION:
        raise ValueError("versão do arquivo de cena não suportada")
    raw_obstacles = data.get("obstacles", [])
    if not isinstance(raw_obstacles, list):
        raise ValueError("lista de obstáculos inválida")
    obstacles: list[CellObstacle] = []
    for raw in raw_obstacles:
        if not isinstance(raw, dict):
            raise ValueError("objeto da célula inválido")
        obstacles.append(
            CellObstacle(
                object_id=str(raw.get("object_id", "")),
                name=str(raw.get("name", "")),
                kind=str(raw.get("kind", OBSTACLE)),
                center=tuple(raw.get("center", ())),
                size=tuple(raw.get("size", ())),
                margin_m=float(raw.get("margin_m", 0.0)),
                enabled=bool(raw.get("enabled", True)),
            )
        )
    return CellScene(
        frame_id=str(data.get("frame_id", "base_link")),
        minimum_clearance_m=float(data.get("minimum_clearance_m", 0.05)),
        tcp_radius_m=float(data.get("tcp_radius_m", 0.035)),
        obstacles=tuple(obstacles),
        revision=int(data.get("revision", 1)),
        updated_at=str(data.get("updated_at", "")),
        schema_version=int(data.get("schema_version", 0)),
    ).validated()


def _load_cell_scene_strict(target: Path) -> CellScene:
    data = json.loads(target.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("raiz JSON inválida")
    return scene_from_dict(data)


def load_cell_scene(path: Path | None = None) -> CellScene:
    target = path or default_cell_scene_path()
    if not target.is_file():
        return CellScene().validated()
    try:
        return _load_cell_scene_strict(target)
    except Exception:
        # Convenience callers receive an empty, explicitly unconfigured scene.
        # CellSceneStore separately preserves the error and gates execution.
        return CellScene().validated()


def save_cell_scene(scene: CellScene, path: Path | None = None) -> Path:
    target = path or default_cell_scene_path()
    target.parent.mkdir(parents=True, exist_ok=True)
    validated = scene.validated()
    temporary = target.with_suffix(f"{target.suffix}.tmp")
    temporary.write_text(
        json.dumps(validated.payload(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.chmod(temporary, 0o600)
    temporary.replace(target)
    return target


@dataclass(frozen=True)
class ClearanceSample:
    path_index: int
    segment_index: int
    obstacle_id: str
    obstacle_name: str
    clearance_m: float


@dataclass(frozen=True)
class ClearanceReport:
    verified: bool
    minimum_clearance_m: float = math.inf
    required_clearance_m: float = 0.0
    limiting_obstacle: str = ""
    limiting_segment: int = -1
    violations: tuple[ClearanceSample, ...] = ()
    point_count: int = 0
    messages: tuple[str, ...] = ()

    @property
    def safe(self) -> bool:
        return self.verified and not self.violations

    def summary(self) -> str:
        if not self.verified:
            return self.messages[0] if self.messages else "análise de folga não verificada"
        if self.point_count == 0:
            return "sem movimentos cartesianos"
        if self.violations:
            return (
                f"folga insuficiente: {self.minimum_clearance_m * 1000:.1f} mm "
                f"junto a {self.limiting_obstacle}"
            )
        return (
            f"folga TCP mínima {self.minimum_clearance_m * 1000:.1f} mm "
            f"(requerida {self.required_clearance_m * 1000:.1f} mm)"
        )


def _point_box_distance(
    point: Sequence[float],
    lower: Sequence[float],
    upper: Sequence[float],
) -> float:
    """Signed distance from point to AABB; negative means inside."""

    outside = [
        max(float(lo) - float(value), 0.0, float(value) - float(hi))
        for value, lo, hi in zip(point, lower, upper)
    ]
    if any(value > 0.0 for value in outside):
        return math.sqrt(sum(value * value for value in outside))
    return -min(
        min(float(value) - float(lo), float(hi) - float(value))
        for value, lo, hi in zip(point, lower, upper)
    )


def _segment_intersects_box(
    start: Sequence[float],
    end: Sequence[float],
    lower: Sequence[float],
    upper: Sequence[float],
) -> bool:
    """Exact slab intersection for a line segment and an AABB."""

    low_t, high_t = 0.0, 1.0
    for origin, target, lo, hi in zip(start, end, lower, upper):
        delta = float(target) - float(origin)
        if abs(delta) < 1e-12:
            if float(origin) < float(lo) or float(origin) > float(hi):
                return False
            continue
        first = (float(lo) - float(origin)) / delta
        second = (float(hi) - float(origin)) / delta
        if first > second:
            first, second = second, first
        low_t = max(low_t, first)
        high_t = min(high_t, second)
        if low_t > high_t:
            return False
    return True


def _segment_box_clearance(
    start: Sequence[float],
    end: Sequence[float],
    obstacle: CellObstacle,
    tcp_radius_m: float,
) -> float:
    """Conservative signed clearance of a TCP segment to one physical box."""

    physical_lower, physical_upper = obstacle.bounds(obstacle.margin_m)
    inflated_lower, inflated_upper = obstacle.bounds(
        obstacle.margin_m + tcp_radius_m
    )
    if _segment_intersects_box(start, end, inflated_lower, inflated_upper):
        # Sampling only quantifies penetration; the exact slab test above is
        # what makes the unsafe decision independent from sampling density.
        segment_length = math.dist(tuple(start), tuple(end))
        samples = max(2, min(401, int(math.ceil(segment_length / 0.002)) + 1))
    else:
        samples = 81
    minimum = math.inf
    for index in range(samples):
        ratio = index / max(1, samples - 1)
        point = tuple(
            float(a) + (float(b) - float(a)) * ratio
            for a, b in zip(start, end)
        )
        clearance = (
            _point_box_distance(point, physical_lower, physical_upper)
            - float(tcp_radius_m)
        )
        minimum = min(minimum, clearance)
    return minimum


def analyze_tcp_clearance(
    points: Iterable[Sequence[float]],
    scene: CellScene,
) -> ClearanceReport:
    """Analyze straight TCP segments against configured cell volumes."""

    try:
        validated = scene.validated()
        route = [
            tuple(_finite(value, "coordenada da trajetória") for value in point)
            for point in points
        ]
    except (TypeError, ValueError) as exc:
        return ClearanceReport(verified=False, messages=(str(exc),))
    if any(len(point) != 3 for point in route):
        return ClearanceReport(
            verified=False,
            messages=("ponto da trajetória deve conter X, Y e Z",),
        )
    if not route:
        return ClearanceReport(
            verified=True,
            required_clearance_m=validated.minimum_clearance_m,
            point_count=0,
        )
    obstacles = validated.enabled_obstacles
    if not obstacles:
        return ClearanceReport(
            verified=True,
            minimum_clearance_m=math.inf,
            required_clearance_m=validated.minimum_clearance_m,
            point_count=len(route),
            messages=("cena local sem volumes habilitados",),
        )

    segments = list(zip(route, route[1:]))
    if not segments:
        segments = [(route[0], route[0])]
    samples: list[ClearanceSample] = []
    limiting: ClearanceSample | None = None
    for segment_index, (start, end) in enumerate(segments):
        for obstacle in obstacles:
            clearance = _segment_box_clearance(
                start,
                end,
                obstacle,
                validated.tcp_radius_m,
            )
            sample = ClearanceSample(
                path_index=0,
                segment_index=segment_index,
                obstacle_id=obstacle.object_id,
                obstacle_name=obstacle.name,
                clearance_m=clearance,
            )
            if limiting is None or clearance < limiting.clearance_m:
                limiting = sample
            if clearance < validated.minimum_clearance_m - 1e-9:
                samples.append(sample)
    assert limiting is not None
    return ClearanceReport(
        verified=True,
        minimum_clearance_m=limiting.clearance_m,
        required_clearance_m=validated.minimum_clearance_m,
        limiting_obstacle=limiting.obstacle_name,
        limiting_segment=limiting.segment_index,
        violations=tuple(samples),
        point_count=len(route),
        messages=(
            "pré-análise do envelope TCP; MoveIt valida todos os elos",
        ),
    )


def tcp_segment_clearances(
    points: Iterable[Sequence[float]],
    scene: CellScene,
) -> tuple[float, ...]:
    """Return the limiting TCP clearance for each displayed route segment."""

    validated = scene.validated()
    route = [tuple(float(value) for value in point) for point in points]
    if len(route) < 2:
        return ()
    obstacles = validated.enabled_obstacles
    if not obstacles:
        return tuple(math.inf for _ in range(len(route) - 1))
    return tuple(
        min(
            _segment_box_clearance(
                start,
                end,
                obstacle,
                validated.tcp_radius_m,
            )
            for obstacle in obstacles
        )
        for start, end in zip(route, route[1:])
    )


def program_tcp_points(program) -> tuple[tuple[float, float, float], ...]:
    """Extract compiled Cartesian targets without importing GUI widgets."""

    from .program import OP_MOVE

    points: list[tuple[float, float, float]] = []
    for opcode in program.flatten():
        if not opcode or opcode[0] != OP_MOVE:
            continue
        pose = opcode[1]
        points.append(
            (float(pose["x"]), float(pose["y"]), float(pose["z"]))
        )
    return tuple(points)


def analyze_program_tcp_clearance(
    program,
    scene: CellScene,
    *,
    start_index: int = 0,
) -> ClearanceReport:
    """Analyze every bounded LOOP/IF path using the dynamics path expander."""

    from .dynamics import MotionPathExpansionError, build_motion_paths

    try:
        paths, notes = build_motion_paths(
            program,
            start_index=start_index,
            speed_override=1.0,
        )
    except (MotionPathExpansionError, TypeError, ValueError) as exc:
        return ClearanceReport(verified=False, messages=(str(exc),))
    reports: list[ClearanceReport] = []
    violations: list[ClearanceSample] = []
    for path_index, path in enumerate(paths):
        report = analyze_tcp_clearance(
            (
                (target.pose["x"], target.pose["y"], target.pose["z"])
                for target in path
            ),
            scene,
        )
        if not report.verified:
            return report
        reports.append(report)
        violations.extend(
            ClearanceSample(
                path_index=path_index,
                segment_index=sample.segment_index,
                obstacle_id=sample.obstacle_id,
                obstacle_name=sample.obstacle_name,
                clearance_m=sample.clearance_m,
            )
            for sample in report.violations
        )
    if not reports:
        return ClearanceReport(
            verified=True,
            required_clearance_m=scene.minimum_clearance_m,
            messages=("sem caminhos executáveis",),
        )
    limiting_path, limiting = min(
        enumerate(reports),
        key=lambda item: item[1].minimum_clearance_m,
    )
    messages = tuple(notes) + (
        "pré-análise do envelope TCP; MoveIt valida todos os elos",
    )
    return ClearanceReport(
        verified=True,
        minimum_clearance_m=limiting.minimum_clearance_m,
        required_clearance_m=limiting.required_clearance_m,
        limiting_obstacle=limiting.limiting_obstacle,
        limiting_segment=limiting.limiting_segment,
        violations=tuple(violations),
        point_count=sum(report.point_count for report in reports),
        messages=messages + (f"caminho limitante {limiting_path + 1}",),
    )


class CellSceneStore(QObject):
    """Shared scene state with a volatile MoveIt-acknowledgement marker."""

    changed = Signal(object)
    applied_changed = Signal(bool)
    monitored_changed = Signal(object)
    monitored_status_changed = Signal(bool)

    def __init__(self, path: Path | None = None, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.path = path or default_cell_scene_path()
        self.load_error = ""
        if self.path.is_file():
            try:
                self.scene = _load_cell_scene_strict(self.path)
            except Exception as exc:
                self.scene = CellScene().validated()
                self.load_error = str(exc)
        else:
            self.scene = CellScene().validated()
        self.applied_signature = ""
        self.last_apply_error = ""
        self.last_apply_message = ""
        self.monitored_geometries: tuple[MonitoredGeometry, ...] = ()
        self.monitored_signature = ""
        self.monitored_scene_name = ""
        self.monitored_error = ""
        self.monitored_updated_at = ""
        self.monitored_tf_dynamic_enabled = False
        self.monitored_tf_refresh_rate_hz = 0.0
        self.monitored_tf_refresh_sequence = 0
        self.monitored_scene_generation = 0
        self.monitored_octomap: dict[str, object] = {"present": False}

    @property
    def external_geometries(self) -> tuple[MonitoredGeometry, ...]:
        return tuple(
            geometry
            for geometry in self.monitored_geometries
            if not geometry.gui_managed
        )

    @property
    def monitored_ready(self) -> bool:
        return bool(self.monitored_updated_at and not self.monitored_error)

    @property
    def is_applied(self) -> bool:
        return bool(
            self.scene.configured
            and self.applied_signature == self.scene.signature()
        )

    def replace(self, scene: CellScene, *, persist: bool = True) -> None:
        previous_signature = self.scene.signature()
        validated = scene.validated()
        if persist:
            save_cell_scene(validated, self.path)
        self.scene = validated
        self.load_error = ""
        if validated.signature() != previous_signature:
            self.applied_signature = ""
            self.last_apply_error = ""
            self.last_apply_message = ""
            self.applied_changed.emit(False)
        self.changed.emit(validated)

    def mark_applied(self, signature: str, success: bool, message: str = "") -> None:
        current = self.scene.signature()
        self.applied_signature = current if success and signature == current else ""
        self.last_apply_error = "" if success else str(message)
        self.last_apply_message = str(message) if success else ""
        self.applied_changed.emit(self.is_applied)

    def accept_monitored_result(
        self,
        success: bool,
        payload: object,
        message: str = "",
    ) -> None:
        """Accept a sanitized planning-scene snapshot from :class:`RosBridge`."""

        if not success or not isinstance(payload, dict):
            self.monitored_error = str(message or "cena monitorada indisponível")
            self.monitored_status_changed.emit(False)
            return
        if bool(payload.get("tf_dynamic_update", False)):
            self._accept_monitored_transform_update(payload)
            return
        try:
            raw = payload.get("geometries", ())
            if not isinstance(raw, (list, tuple)):
                raise ValueError("lista de geometrias monitoradas inválida")
            geometries = tuple(
                MonitoredGeometry.from_payload(item)
                for item in raw
                if isinstance(item, dict)
            )
            octomap = validated_octomap_metadata(payload.get("octomap"))
            signature = monitored_scene_signature(geometries, octomap)
        except (TypeError, ValueError) as exc:
            self.monitored_error = str(exc)
            self.monitored_status_changed.emit(False)
            return
        self.monitored_geometries = geometries
        self.monitored_signature = signature
        self.monitored_scene_name = str(payload.get("name", ""))
        self.monitored_octomap = octomap
        self.monitored_updated_at = datetime.now(timezone.utc).isoformat()
        self.monitored_tf_dynamic_enabled = bool(
            payload.get("tf_dynamic_refresh_enabled", False)
        )
        self.monitored_tf_refresh_rate_hz = max(
            0.0,
            float(payload.get("tf_refresh_rate_hz", 0.0)),
        )
        self.monitored_tf_refresh_sequence = max(
            0,
            int(payload.get("tf_refresh_sequence", 0)),
        )
        self.monitored_scene_generation = max(
            0,
            int(payload.get("tf_scene_generation", 0)),
        )
        self.monitored_error = ""
        self.monitored_changed.emit(geometries)
        self.monitored_status_changed.emit(True)

    def _accept_monitored_transform_update(
        self,
        payload: dict[str, object],
    ) -> None:
        """Apply a compact pose-only update to the current immutable geometry."""

        try:
            generation = max(
                0,
                int(payload.get("tf_scene_generation", 0)),
            )
            if generation != self.monitored_scene_generation:
                return
            raw_updates = payload.get("transform_updates", ())
            if not isinstance(raw_updates, (list, tuple)):
                raise ValueError("atualização TF da cena inválida")
            if len(raw_updates) != len(self.monitored_geometries):
                raise ValueError(
                    "atualização TF não corresponde à cena monitorada"
                )
            updated: list[MonitoredGeometry] = []
            for index, (current, raw) in enumerate(
                zip(self.monitored_geometries, raw_updates)
            ):
                if (
                    not isinstance(raw, dict)
                    or int(raw.get("geometry_index", -1)) != index
                    or str(raw.get("object_id", "")) != current.object_id
                    or str(raw.get("attached_to", "")) != current.attached_to
                    or (
                        str(raw.get("source_frame_id", ""))
                        .strip()
                        .lstrip("/")
                        != current.source_frame_id
                    )
                ):
                    raise ValueError(
                        "identidade da geometria mudou durante atualização TF"
                    )
                updated.append(
                    current.with_display_transform(
                        frame_id=raw.get("frame_id", ""),
                        pose=raw.get("pose", ()),
                        transform_status=raw.get(
                            "transform_status",
                            "unresolved",
                        ),
                    )
                )
            refresh_rate = max(
                0.0,
                float(payload.get("tf_refresh_rate_hz", 0.0)),
            )
            sequence = max(
                0,
                int(payload.get("tf_refresh_sequence", 0)),
            )
        except (TypeError, ValueError) as exc:
            self.monitored_error = str(exc)
            self.monitored_status_changed.emit(False)
            return
        self.monitored_geometries = tuple(updated)
        self.monitored_updated_at = datetime.now(timezone.utc).isoformat()
        self.monitored_tf_dynamic_enabled = bool(
            payload.get("tf_dynamic_refresh_enabled", True)
        )
        self.monitored_tf_refresh_rate_hz = refresh_rate
        self.monitored_tf_refresh_sequence = sequence
        self.monitored_error = ""
        self.monitored_changed.emit(self.monitored_geometries)
        self.monitored_status_changed.emit(True)
