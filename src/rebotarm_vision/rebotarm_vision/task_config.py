"""JSON configuration for YOLO detection and object-pick planning.

The classes in this module intentionally do not depend on ROS.  They provide
one validated contract that can be consumed by the GUI, segmentation node and
pick scheduler.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import Any


BACKENDS = {"auto", "opencv", "yolo"}
ORIENTATION_FILTERS = {"any", "horizontal", "vertical"}
ROUTE_STRATEGIES = {
    "score",
    "nearest_neighbor",
    "left_to_right",
    "right_to_left",
    "front_to_back",
    "back_to_front",
}
WORKSPACE_SHAPES = {"polygon", "circle"}


def _number(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _clamp(value: Any, minimum: float, maximum: float, default: float) -> float:
    return min(max(_number(value, default), minimum), maximum)


@dataclass
class DetectorConfig:
    backend: str = "yolo"
    model_path: str = ""
    confidence_percent: float = 35.0
    image_size: int = 416
    target_classes: list[str] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "DetectorConfig":
        backend = str(data.get("backend", "yolo")).lower()
        if backend not in BACKENDS:
            backend = "yolo"
        confidence = data.get("confidence_percent", 35.0)
        image_size = int(round(_clamp(data.get("image_size"), 64.0, 4096.0, 416.0)))
        classes = data.get("target_classes", [])
        if isinstance(classes, str):
            classes = [item.strip() for item in classes.split(",")]
        if not isinstance(classes, list):
            classes = []
        return cls(
            backend=backend,
            model_path=str(data.get("model_path", "")).strip(),
            confidence_percent=_clamp(confidence, 0.0, 100.0, 35.0),
            image_size=image_size,
            target_classes=[str(item).strip() for item in classes if str(item).strip()],
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "backend": self.backend,
            "model_path": self.model_path,
            "confidence_percent": float(self.confidence_percent),
            "image_size": int(self.image_size),
            "target_classes": list(self.target_classes),
        }


@dataclass
class ObjectConfig:
    orientation: str = "any"
    auxiliary_attribute: str = ""

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ObjectConfig":
        orientation = str(data.get("orientation", "any")).lower()
        if orientation not in ORIENTATION_FILTERS:
            orientation = "any"
        return cls(
            orientation=orientation,
            auxiliary_attribute=str(data.get("auxiliary_attribute", "")).strip(),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "orientation": self.orientation,
            "auxiliary_attribute": self.auxiliary_attribute,
        }


@dataclass
class WorkspaceConfig:
    enabled: bool = False
    shape: str = "polygon"
    vertices: list[list[float]] = field(default_factory=list)
    circle: list[float] = field(default_factory=lambda: [320.0, 240.0, 180.0])
    clearance_px: float = 0.0

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "WorkspaceConfig":
        shape = str(data.get("shape", "polygon")).lower()
        if shape not in WORKSPACE_SHAPES:
            shape = "polygon"
        raw_vertices = data.get("vertices", [])
        vertices: list[list[float]] = []
        if isinstance(raw_vertices, list):
            for point in raw_vertices:
                if isinstance(point, (list, tuple)) and len(point) >= 2:
                    vertices.append([float(point[0]), float(point[1])])
        raw_circle = data.get("circle", [320.0, 240.0, 180.0])
        circle = [320.0, 240.0, 180.0]
        if isinstance(raw_circle, (list, tuple)) and len(raw_circle) >= 3:
            circle = [float(raw_circle[0]), float(raw_circle[1]), max(0.0, float(raw_circle[2]))]
        valid_geometry = (shape == "circle" and circle[2] > 0.0) or (
            shape == "polygon" and len(vertices) >= 3
        )
        return cls(
            enabled=bool(data.get("enabled", valid_geometry)),
            shape=shape,
            vertices=vertices,
            circle=circle,
            clearance_px=max(0.0, _number(data.get("clearance_px"), 0.0)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": bool(self.enabled),
            "shape": self.shape,
            "vertices": [[float(x), float(y)] for x, y in self.vertices],
            "circle": [float(value) for value in self.circle],
            "clearance_px": float(self.clearance_px),
        }


@dataclass
class GraspConfig:
    clearance_px: float = 0.0
    min_clearance_horizontal_px: float = 4.0
    min_clearance_vertical_px: float = 20.0

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "GraspConfig":
        return cls(
            clearance_px=max(0.0, _number(data.get("clearance_px"), 0.0)),
            min_clearance_horizontal_px=max(
                0.0,
                _number(data.get("min_clearance_horizontal_px"), 4.0),
            ),
            min_clearance_vertical_px=max(
                0.0,
                _number(data.get("min_clearance_vertical_px"), 20.0),
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "clearance_px": float(self.clearance_px),
            "min_clearance_horizontal_px": float(self.min_clearance_horizontal_px),
            "min_clearance_vertical_px": float(self.min_clearance_vertical_px),
        }


@dataclass
class PickRouteConfig:
    strategy: str = "score"
    max_objects: int = 8
    stable_frames: int = 1
    origin_xy_m: list[float] = field(default_factory=lambda: [0.0, 0.0])

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "PickRouteConfig":
        strategy = str(data.get("strategy", "score")).lower()
        if strategy not in ROUTE_STRATEGIES:
            strategy = "score"
        origin = data.get("origin_xy_m", [0.0, 0.0])
        if not isinstance(origin, (list, tuple)) or len(origin) < 2:
            origin = [0.0, 0.0]
        return cls(
            strategy=strategy,
            max_objects=max(0, int(_number(data.get("max_objects"), 8.0))),
            stable_frames=max(1, int(_number(data.get("stable_frames"), 1.0))),
            origin_xy_m=[float(origin[0]), float(origin[1])],
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "strategy": self.strategy,
            "max_objects": int(self.max_objects),
            "stable_frames": int(self.stable_frames),
            "origin_xy_m": [float(value) for value in self.origin_xy_m[:2]],
        }


@dataclass
class VisionTaskConfig:
    schema_version: int = 3
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    object: ObjectConfig = field(default_factory=ObjectConfig)
    workspace: WorkspaceConfig = field(default_factory=WorkspaceConfig)
    grasp: GraspConfig = field(default_factory=GraspConfig)
    pick_route: PickRouteConfig = field(default_factory=PickRouteConfig)
    orientation_profiles: dict[str, dict[str, Any]] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "VisionTaskConfig":
        if not isinstance(data, dict):
            raise ValueError("a configuração JSON deve ser um objeto")
        profiles = data.get("orientation_profiles", {})
        if not isinstance(profiles, dict):
            profiles = {}
        parsed_profiles = {
            name: dict(profiles[name])
            for name in ("horizontal", "vertical")
            if isinstance(profiles.get(name), dict)
        }
        workspace_data = data.get("workspace", {})
        return cls(
            schema_version=int(_number(data.get("schema_version"), 3.0)),
            detector=DetectorConfig.from_dict(data.get("detector", data)),
            object=ObjectConfig.from_dict(data.get("object", {})),
            workspace=WorkspaceConfig.from_dict(workspace_data if isinstance(workspace_data, dict) else {}),
            grasp=GraspConfig.from_dict(data.get("grasp", {})),
            pick_route=PickRouteConfig.from_dict(data.get("pick_route", {})),
            orientation_profiles=parsed_profiles,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": int(self.schema_version),
            "detector": self.detector.to_dict(),
            "object": self.object.to_dict(),
            "workspace": self.workspace.to_dict(),
            "grasp": self.grasp.to_dict(),
            "pick_route": self.pick_route.to_dict(),
            "orientation_profiles": self.orientation_profiles,
        }

    @classmethod
    def from_json(cls, text: str) -> "VisionTaskConfig":
        return cls.from_dict(json.loads(text))

    @classmethod
    def load(cls, path: str | Path) -> "VisionTaskConfig":
        return cls.from_json(Path(path).expanduser().read_text(encoding="utf-8"))

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False, sort_keys=True)

    def save(self, path: str | Path) -> Path:
        target = Path(path).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(self.to_json(), encoding="utf-8")
        return target
