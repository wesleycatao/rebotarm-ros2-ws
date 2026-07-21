"""Vision calibration profile data model, validation and API schema.

This module is the single source of truth for the JSON profile exchanged
between the AI auto-calibration service and the local OpenCV pipeline. It
does not depend on ROS and can be imported by the GUI for validation.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


# Allowed token values for the OpenCV segmentation pipeline.
OPENCY_MODE_TOKENS = {"bright", "edge", "hsv", "rgb"}
CENTER_METHODS = {"moments", "min_area_rect", "bounding_box", "contour_mean", "rect"}
DIRECTION_METHODS = {"min_area_rect", "pca", "none"}
DIRECTION_SENSE_METHODS = {
    "none",
    "bright_end",
    "dark_end",
    "positive_x",
    "negative_x",
    "positive_y",
    "negative_y",
    "right",
    "left",
    "down",
    "up",
}
SHAPE_FILTERS = {"any", "circle", "square", "rectangle", "triangle", "object"}
WORKSPACE_SHAPES = {"polygon", "circle"}
ORIENTATIONS = {"horizontal", "vertical", "ambiguous"}

# Limits used by the API schema and local sanitisation.
UINT8_MAX = 255
HUE_MAX = 179
MORPH_KERNEL_MAX = 31
MORPH_OPERATION_MAX = 8
EDGE_APERTURE_SIZES = {3, 5, 7}
ASPECT_RATIO_MAX = 20.0


@dataclass
class Scene:
    """High-level scene description returned by the AI."""

    expected_objects: int = 0
    workspace_polygon: list[list[float]] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "expected_objects": int(self.expected_objects),
            "workspace_polygon": [
                [float(p[0]), float(p[1])] for p in self.workspace_polygon if len(p) >= 2
            ],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Scene:
        polygon: list[list[float]] = []
        for point in data.get("workspace_polygon", []):
            if isinstance(point, (list, tuple)) and len(point) >= 2:
                polygon.append([float(point[0]), float(point[1])])
        return cls(
            expected_objects=int(data.get("expected_objects", 0)),
            workspace_polygon=polygon,
        )


@dataclass
class RotatedBBox:
    """Normalised rotated bounding box (cx, cy, w, h, angle_deg)."""

    cx: float = 0.5
    cy: float = 0.5
    w: float = 0.1
    h: float = 0.1
    angle_deg: float = 0.0

    def to_dict(self) -> dict[str, float]:
        return {
            "cx": _clamp01(float(self.cx)),
            "cy": _clamp01(float(self.cy)),
            "w": max(0.0, float(self.w)),
            "h": max(0.0, float(self.h)),
            "angle_deg": float(self.angle_deg),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> RotatedBBox:
        return cls(
            cx=float(data.get("cx", 0.5)),
            cy=float(data.get("cy", 0.5)),
            w=float(data.get("w", 0.1)),
            h=float(data.get("h", 0.1)),
            angle_deg=float(data.get("angle_deg", 0.0)),
        )


@dataclass
class ReferenceObject:
    """A single object annotation used as ground truth during calibration."""

    class_name: str = "object"
    bbox_rotated: RotatedBBox = field(default_factory=RotatedBBox)
    orientation: str = "ambiguous"
    grasp_zone: list[list[float]] = field(default_factory=list)
    grasp_inset_ratio: float = 0.10
    confidence: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "class": str(self.class_name),
            "bbox_rotated": self.bbox_rotated.to_dict(),
            "orientation": str(self.orientation) if str(self.orientation) in ORIENTATIONS else "ambiguous",
            "grasp_zone": [[float(p[0]), float(p[1])] for p in self.grasp_zone if len(p) >= 2],
            "grasp_inset_ratio": _clamp(float(self.grasp_inset_ratio), 0.0, 0.5),
            "confidence": _clamp(float(self.confidence), 0.0, 1.0),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ReferenceObject:
        bbox = data.get("bbox_rotated", {})
        if isinstance(bbox, dict):
            bbox = RotatedBBox.from_dict(bbox)
        else:
            bbox = RotatedBBox()
        zone: list[list[float]] = []
        for point in data.get("grasp_zone", []):
            if isinstance(point, (list, tuple)) and len(point) >= 2:
                zone.append([float(point[0]), float(point[1])])
        orientation = str(data.get("orientation", "ambiguous"))
        if orientation not in ORIENTATIONS:
            orientation = "ambiguous"
        return cls(
            class_name=str(data.get("class", "object")),
            bbox_rotated=bbox,
            orientation=orientation,
            grasp_zone=zone,
            grasp_inset_ratio=float(data.get("grasp_inset_ratio", 0.10)),
            confidence=float(data.get("confidence", 0.0)),
        )


@dataclass
class CalibratedProfile:
    """OpenCV parameter set for one orientation (vertical or horizontal)."""

    opencv_mode: str = "bright"
    active_filters: list[str] = field(default_factory=list)
    center_method: str = "moments"
    direction_method: str = "min_area_rect"
    direction_sense_method: str = "none"
    shape_filter: str = "any"
    min_area_px: float = 500.0
    max_area_px: float = 0.0
    min_aspect_ratio: float = 0.0
    max_aspect_ratio: float = 0.0
    morph_kernel: int = 7
    morph_operation: int = 1
    roi_margin_px: int = 0
    brightness_threshold: int = 105
    brightness_invert: bool = False
    blur_kernel: int = 5
    canny_low: int = 45
    canny_high: int = 130
    edge_aperture_size: int = 3
    edge_operation: int = 0
    hsv_lower: list[int] = field(default_factory=lambda: [0, 40, 40])
    hsv_upper: list[int] = field(default_factory=lambda: [179, 255, 255])
    rgb_lower: list[int] = field(default_factory=lambda: [0, 0, 0])
    rgb_upper: list[int] = field(default_factory=lambda: [255, 255, 255])
    shape_epsilon_ratio: float = 0.035
    circle_min_circularity: float = 0.72
    square_aspect_tolerance: float = 0.22
    lying_aspect_ratio_threshold: float = 1.8
    min_solidity: float = 0.0
    min_extent: float = 0.0
    max_extent: float = 1.0
    use_workspace_mask: bool = False
    draw_workspace_limit: bool = True
    workspace_limit_shape: str = "polygon"
    workspace_limit_pixels: list[float] = field(default_factory=list)
    workspace_circle_pixel: list[float] = field(
        default_factory=lambda: [320.0, 240.0, 180.0]
    )
    workspace_mask_margin_px: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "opencv_mode": str(self.opencv_mode),
            "active_filters": [str(f) for f in self.active_filters],
            "center_method": str(self.center_method),
            "direction_method": str(self.direction_method),
            "direction_sense_method": str(self.direction_sense_method),
            "shape_filter": str(self.shape_filter),
            "min_area_px": float(self.min_area_px),
            "max_area_px": float(self.max_area_px),
            "min_aspect_ratio": float(self.min_aspect_ratio),
            "max_aspect_ratio": float(self.max_aspect_ratio),
            "morph_kernel": int(self.morph_kernel),
            "morph_operation": int(self.morph_operation),
            "roi_margin_px": int(self.roi_margin_px),
            "brightness_threshold": int(self.brightness_threshold),
            "brightness_invert": bool(self.brightness_invert),
            "blur_kernel": int(self.blur_kernel),
            "canny_low": int(self.canny_low),
            "canny_high": int(self.canny_high),
            "edge_aperture_size": int(self.edge_aperture_size),
            "edge_operation": int(self.edge_operation),
            "hsv_lower": [int(v) for v in self.hsv_lower],
            "hsv_upper": [int(v) for v in self.hsv_upper],
            "rgb_lower": [int(v) for v in self.rgb_lower],
            "rgb_upper": [int(v) for v in self.rgb_upper],
            "shape_epsilon_ratio": float(self.shape_epsilon_ratio),
            "circle_min_circularity": float(self.circle_min_circularity),
            "square_aspect_tolerance": float(self.square_aspect_tolerance),
            "lying_aspect_ratio_threshold": float(self.lying_aspect_ratio_threshold),
            "min_solidity": float(self.min_solidity),
            "min_extent": float(self.min_extent),
            "max_extent": float(self.max_extent),
            "use_workspace_mask": bool(self.use_workspace_mask),
            "draw_workspace_limit": bool(self.draw_workspace_limit),
            "workspace_limit_shape": str(self.workspace_limit_shape),
            "workspace_limit_pixels": [float(v) for v in self.workspace_limit_pixels],
            "workspace_circle_pixel": [float(v) for v in self.workspace_circle_pixel],
            "workspace_mask_margin_px": int(self.workspace_mask_margin_px),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> CalibratedProfile:
        return cls(
            opencv_mode=str(data.get("opencv_mode", "bright")),
            active_filters=[str(f) for f in data.get("active_filters", [])],
            center_method=str(data.get("center_method", "moments")),
            direction_method=str(data.get("direction_method", "min_area_rect")),
            direction_sense_method=str(data.get("direction_sense_method", "none")),
            shape_filter=str(data.get("shape_filter", "any")),
            min_area_px=float(data.get("min_area_px", 500.0)),
            max_area_px=float(data.get("max_area_px", 0.0)),
            min_aspect_ratio=float(data.get("min_aspect_ratio", 0.0)),
            max_aspect_ratio=float(data.get("max_aspect_ratio", 0.0)),
            morph_kernel=int(data.get("morph_kernel", 7)),
            morph_operation=int(data.get("morph_operation", 1)),
            roi_margin_px=int(data.get("roi_margin_px", 0)),
            brightness_threshold=int(data.get("brightness_threshold", 105)),
            brightness_invert=bool(data.get("brightness_invert", False)),
            blur_kernel=int(data.get("blur_kernel", 5)),
            canny_low=int(data.get("canny_low", 45)),
            canny_high=int(data.get("canny_high", 130)),
            edge_aperture_size=int(data.get("edge_aperture_size", 3)),
            edge_operation=int(data.get("edge_operation", 0)),
            hsv_lower=_int_triplet(data.get("hsv_lower"), [0, 40, 40]),
            hsv_upper=_int_triplet(data.get("hsv_upper"), [179, 255, 255]),
            rgb_lower=_int_triplet(data.get("rgb_lower"), [0, 0, 0]),
            rgb_upper=_int_triplet(data.get("rgb_upper"), [255, 255, 255]),
            shape_epsilon_ratio=float(data.get("shape_epsilon_ratio", 0.035)),
            circle_min_circularity=float(data.get("circle_min_circularity", 0.72)),
            square_aspect_tolerance=float(data.get("square_aspect_tolerance", 0.22)),
            lying_aspect_ratio_threshold=float(data.get("lying_aspect_ratio_threshold", 1.8)),
            min_solidity=float(data.get("min_solidity", 0.0)),
            min_extent=float(data.get("min_extent", 0.0)),
            max_extent=float(data.get("max_extent", 1.0)),
            use_workspace_mask=bool(data.get("use_workspace_mask", False)),
            draw_workspace_limit=bool(data.get("draw_workspace_limit", True)),
            workspace_limit_shape=str(data.get("workspace_limit_shape", "polygon")),
            workspace_limit_pixels=[float(v) for v in data.get("workspace_limit_pixels", [])],
            workspace_circle_pixel=[
                float(v) for v in data.get("workspace_circle_pixel", [320.0, 240.0, 180.0])
            ],
            workspace_mask_margin_px=int(data.get("workspace_mask_margin_px", 0)),
        )


@dataclass
class MergeSettings:
    """Parameters used to merge vertical and horizontal detection results."""

    rotated_iou_threshold: float = 0.45
    center_distance_threshold: float = 0.05

    def to_dict(self) -> dict[str, float]:
        return {
            "rotated_iou_threshold": _clamp(float(self.rotated_iou_threshold), 0.0, 1.0),
            "center_distance_threshold": max(0.0, float(self.center_distance_threshold)),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> MergeSettings:
        return cls(
            rotated_iou_threshold=float(data.get("rotated_iou_threshold", 0.45)),
            center_distance_threshold=float(data.get("center_distance_threshold", 0.05)),
        )


@dataclass
class IterationInfo:
    """Iteration metadata returned by the AI."""

    number: int = 0
    converged: bool = False
    notes: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "number": max(0, int(self.number)),
            "converged": bool(self.converged),
            "notes": str(self.notes),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> IterationInfo:
        return cls(
            number=int(data.get("number", 0)),
            converged=bool(data.get("converged", False)),
            notes=str(data.get("notes", "")),
        )


@dataclass
class ProfileMetrics:
    """Objective metrics produced by local validation."""

    count_error: int = 0
    mean_iou: float = 0.0
    duplicate_count: int = 0
    orientation_accuracy: float = 0.0
    coverage: float = 0.0
    grasp_zone_violation_count: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            "count_error": int(self.count_error),
            "mean_iou": _clamp(float(self.mean_iou), 0.0, 1.0),
            "duplicate_count": int(self.duplicate_count),
            "orientation_accuracy": _clamp(float(self.orientation_accuracy), 0.0, 1.0),
            "coverage": _clamp(float(self.coverage), 0.0, 1.0),
            "grasp_zone_violation_count": int(self.grasp_zone_violation_count),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ProfileMetrics:
        return cls(
            count_error=int(data.get("count_error", 0)),
            mean_iou=float(data.get("mean_iou", 0.0)),
            duplicate_count=int(data.get("duplicate_count", 0)),
            orientation_accuracy=float(data.get("orientation_accuracy", 0.0)),
            coverage=float(data.get("coverage", 0.0)),
            grasp_zone_violation_count=int(data.get("grasp_zone_violation_count", 0)),
        )


@dataclass
class ProfileStatus:
    """Approval / convergence status of a profile."""

    approved: bool = False
    converged: bool = False
    reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "approved": bool(self.approved),
            "converged": bool(self.converged),
            "reason": str(self.reason),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ProfileStatus:
        return cls(
            approved=bool(data.get("approved", False)),
            converged=bool(data.get("converged", False)),
            reason=str(data.get("reason", "")),
        )


@dataclass
class VisionProfile:
    """Full calibration profile persisted locally and used at runtime."""

    schema_version: str = "1.0.0"
    created_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    image_hash: str = ""
    image_width: int = 0
    image_height: int = 0
    prompt: str = ""
    model: str = ""
    base_url: str = ""
    scene: Scene = field(default_factory=Scene)
    reference_objects: list[ReferenceObject] = field(default_factory=list)
    profiles: dict[str, CalibratedProfile] = field(
        default_factory=lambda: {
            "vertical": CalibratedProfile(),
            "horizontal": CalibratedProfile(),
        }
    )
    merge: MergeSettings = field(default_factory=MergeSettings)
    iteration: IterationInfo = field(default_factory=IterationInfo)
    metrics: ProfileMetrics = field(default_factory=ProfileMetrics)
    status: ProfileStatus = field(default_factory=ProfileStatus)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": str(self.schema_version),
            "created_at": str(self.created_at),
            "image_hash": str(self.image_hash),
            "image_width": int(self.image_width),
            "image_height": int(self.image_height),
            "prompt": str(self.prompt),
            "model": str(self.model),
            "base_url": str(self.base_url),
            "scene": self.scene.to_dict(),
            "reference_objects": [obj.to_dict() for obj in self.reference_objects],
            "profiles": {
                name: profile.to_dict() for name, profile in self.profiles.items()
            },
            "merge": self.merge.to_dict(),
            "iteration": self.iteration.to_dict(),
            "metrics": self.metrics.to_dict(),
            "status": self.status.to_dict(),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> VisionProfile:
        profiles: dict[str, CalibratedProfile] = {}
        raw_profiles = data.get("profiles", {})
        if isinstance(raw_profiles, dict):
            for name in ("vertical", "horizontal"):
                profile_data = raw_profiles.get(name, {})
                if isinstance(profile_data, dict):
                    profiles[name] = CalibratedProfile.from_dict(profile_data)
        if not profiles:
            profiles = {
                "vertical": CalibratedProfile(),
                "horizontal": CalibratedProfile(),
            }
        reference_objects: list[ReferenceObject] = []
        for obj in data.get("reference_objects", []):
            if isinstance(obj, dict):
                reference_objects.append(ReferenceObject.from_dict(obj))
        return cls(
            schema_version=str(data.get("schema_version", "1.0.0")),
            created_at=str(data.get("created_at", datetime.now(timezone.utc).isoformat())),
            image_hash=str(data.get("image_hash", "")),
            image_width=int(data.get("image_width", 0)),
            image_height=int(data.get("image_height", 0)),
            prompt=str(data.get("prompt", "")),
            model=str(data.get("model", "")),
            base_url=str(data.get("base_url", "")),
            scene=Scene.from_dict(data.get("scene", {})),
            reference_objects=reference_objects,
            profiles=profiles,
            merge=MergeSettings.from_dict(data.get("merge", {})),
            iteration=IterationInfo.from_dict(data.get("iteration", {})),
            metrics=ProfileMetrics.from_dict(data.get("metrics", {})),
            status=ProfileStatus.from_dict(data.get("status", {})),
        )

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False, sort_keys=True)

    @classmethod
    def from_json(cls, text: str) -> VisionProfile:
        return cls.from_dict(json.loads(text))


def apply_scene_workspace_polygon(profile: VisionProfile) -> VisionProfile:
    """Use scene.workspace_polygon as the canonical ROI for every detector profile."""
    polygon = [
        [float(point[0]), float(point[1])]
        for point in profile.scene.workspace_polygon
        if isinstance(point, (list, tuple)) and len(point) >= 2
    ]
    if len(polygon) < 3:
        return profile

    flat_pixels = [coord for point in polygon for coord in point[:2]]
    for calibrated in profile.profiles.values():
        calibrated.use_workspace_mask = True
        calibrated.draw_workspace_limit = True
        calibrated.workspace_limit_shape = "polygon"
        calibrated.workspace_limit_pixels = list(flat_pixels)
    return profile


def image_hash(image_bytes: bytes) -> str:
    """Return a stable SHA-256 hex digest for an image buffer."""
    return hashlib.sha256(image_bytes).hexdigest()


def profile_storage_dir() -> Path:
    """Default directory for persisted profiles."""
    path = Path.home() / ".config" / "rebotarm_vision" / "profiles"
    path.mkdir(parents=True, exist_ok=True)
    return path


def save_profile(profile: VisionProfile, directory: Path | None = None) -> Path:
    """Persist a profile to disk. The API key is never stored."""
    directory = directory or profile_storage_dir()
    directory.mkdir(parents=True, exist_ok=True)
    filename = f"{profile.image_hash or 'unknown'}_{profile.created_at.replace(':', '-').replace('+', '_')}.json"
    path = directory / filename
    path.write_text(profile.to_json(), encoding="utf-8")
    return path


def load_profile(path: Path) -> VisionProfile:
    """Load a profile from disk."""
    return VisionProfile.from_json(path.read_text(encoding="utf-8"))


def _detection_keys_from_mode(mode: str) -> list[str]:
    tokens = {token for token in str(mode).lower().split("_or_") if token}
    keys: list[str] = []
    if "hsv" in tokens:
        keys.append("detect_hsv")
    if "rgb" in tokens:
        keys.append("detect_rgb")
    if "bright" in tokens:
        keys.append("detect_bright")
    if "edge" in tokens:
        keys.append("detect_edge")
    return keys


VALID_FILTER_KEYS = {
    "detect_bright",
    "detect_edge",
    "detect_hsv",
    "detect_rgb",
    "min_area",
    "max_area",
    "min_aspect",
    "max_aspect",
    "shape_filter",
    "min_solidity",
    "min_extent",
    "max_extent",
}


def sanitize_profile(
    profile: CalibratedProfile,
    image_width: int,
    image_height: int,
) -> CalibratedProfile:
    """Clamp every field to safe limits and normalise derived values."""
    max_area = max(1, int(image_width) * int(image_height))

    # Normalise opencv_mode tokens.
    mode = str(profile.opencv_mode).lower()
    tokens = {token for token in mode.split("_or_") if token}
    valid_tokens = tokens & OPENCY_MODE_TOKENS
    if not valid_tokens:
        valid_tokens = {"bright"}
    profile.opencv_mode = "_or_".join(sorted(valid_tokens))

    # Active filters must include detection + basic geometric filters.
    filters = [str(f) for f in profile.active_filters if str(f) in VALID_FILTER_KEYS]
    filters = list(dict.fromkeys(filters))
    detection_keys = _detection_keys_from_mode(profile.opencv_mode)
    if not any(key in filters for key in ("detect_bright", "detect_edge", "detect_hsv", "detect_rgb")):
        filters.extend(detection_keys)
    for key in ("min_area", "max_area", "min_aspect", "max_aspect"):
        if key not in filters:
            filters.append(key)
    profile.active_filters = list(dict.fromkeys(filters))

    # Enumerated methods.
    profile.center_method = _choose(str(profile.center_method), CENTER_METHODS, "moments")
    profile.direction_method = _choose(str(profile.direction_method), DIRECTION_METHODS, "min_area_rect")
    profile.direction_sense_method = _choose(
        str(profile.direction_sense_method), DIRECTION_SENSE_METHODS, "none"
    )
    profile.shape_filter = _choose(str(profile.shape_filter), SHAPE_FILTERS, "any")
    profile.workspace_limit_shape = _choose(
        str(profile.workspace_limit_shape), WORKSPACE_SHAPES, "polygon"
    )

    # Integer clamping.
    profile.brightness_threshold = _clamp_int(profile.brightness_threshold, 0, UINT8_MAX, 105)
    profile.blur_kernel = _clamp_int(profile.blur_kernel, 1, MORPH_KERNEL_MAX, 3, odd=True)
    profile.morph_kernel = _clamp_int(profile.morph_kernel, 1, MORPH_KERNEL_MAX, 7, odd=True)
    profile.morph_operation = _clamp_int(profile.morph_operation, -MORPH_OPERATION_MAX, MORPH_OPERATION_MAX, 1)
    profile.roi_margin_px = _clamp_int(profile.roi_margin_px, 0, max(image_width, image_height), 0)
    profile.canny_low = _clamp_int(profile.canny_low, 0, UINT8_MAX, 45)
    profile.canny_high = _clamp_int(profile.canny_high, 0, UINT8_MAX, 130)
    profile.edge_aperture_size = _clamp_int(
        profile.edge_aperture_size, 3, 7, 3, allowed=EDGE_APERTURE_SIZES
    )
    profile.edge_operation = _clamp_int(profile.edge_operation, -MORPH_OPERATION_MAX, MORPH_OPERATION_MAX, 0)
    profile.workspace_mask_margin_px = _clamp_int(
        profile.workspace_mask_margin_px, 0, max(image_width, image_height), 0
    )

    # Colour triplets.
    profile.hsv_lower = _hsv_triplet(profile.hsv_lower, [0, 40, 40])
    profile.hsv_upper = _hsv_triplet(profile.hsv_upper, [179, 255, 255])
    profile.rgb_lower = _uint8_triplet(profile.rgb_lower, [0, 0, 0])
    profile.rgb_upper = _uint8_triplet(profile.rgb_upper, [255, 255, 255])

    # Float clamping.
    profile.min_area_px = _clamp_default(profile.min_area_px, 0.0, float(max_area), 500.0)
    profile.max_area_px = _clamp_default(profile.max_area_px, 0.0, float(max_area), 0.0)
    profile.min_aspect_ratio = _clamp_default(profile.min_aspect_ratio, 0.0, ASPECT_RATIO_MAX, 0.0)
    profile.max_aspect_ratio = _clamp_default(profile.max_aspect_ratio, 0.0, ASPECT_RATIO_MAX, 0.0)
    profile.min_solidity = _clamp_default(profile.min_solidity, 0.0, 1.0, 0.0)
    profile.min_extent = _clamp_default(profile.min_extent, 0.0, 1.0, 0.0)
    profile.max_extent = _clamp_default(profile.max_extent, 0.0, 1.0, 1.0)
    profile.shape_epsilon_ratio = _clamp_default(profile.shape_epsilon_ratio, 0.005, 0.20, 0.035)
    profile.circle_min_circularity = _clamp_default(profile.circle_min_circularity, 0.10, 1.0, 0.72)
    profile.square_aspect_tolerance = _clamp_default(profile.square_aspect_tolerance, 0.01, 1.0, 0.22)
    profile.lying_aspect_ratio_threshold = _clamp_default(
        profile.lying_aspect_ratio_threshold, 1.0, 10.0, 1.8
    )

    # Consistency between min/max.
    if profile.max_area_px > 0.0:
        profile.max_area_px = max(profile.max_area_px, profile.min_area_px)
    if profile.max_aspect_ratio > 0.0:
        profile.max_aspect_ratio = max(profile.max_aspect_ratio, profile.min_aspect_ratio)
    if profile.max_extent < profile.min_extent:
        profile.max_extent = profile.min_extent

    # Workspace lists.
    max_px = float(max(image_width, image_height))
    profile.workspace_limit_pixels = [
        _clamp_default(float(v), 0.0, max_px, 0.0) for v in profile.workspace_limit_pixels
    ]
    profile.workspace_circle_pixel = [
        _clamp_default(float(v), 0.0, max_px, 0.0) for v in profile.workspace_circle_pixel
    ]
    if len(profile.workspace_circle_pixel) < 3:
        profile.workspace_circle_pixel = [
            image_width * 0.5,
            image_height * 0.5,
            min(image_width, image_height) * 0.4,
        ]

    return profile


def profile_to_segmentation_params(
    profile: CalibratedProfile,
    image_width: int | None = None,
    image_height: int | None = None,
) -> dict[str, Any]:
    """Convert a CalibratedProfile into the ROS parameter dict used by SegmentationNode.

    The SegmentationNode already understands orientation profiles via
    ``use_orientation_profiles``, ``vertical_profile_json`` and
    ``horizontal_profile_json``; this helper builds one of those JSON blobs.
    When ``image_width``/``image_height`` are supplied, workspace pixels and
    area limits are clamped relative to the actual frame size.
    """
    if image_width is None or image_height is None:
        sanitized = sanitize_profile(profile, 640, 480)
    else:
        sanitized = sanitize_profile(profile, int(image_width), int(image_height))
    morph_operation = int(sanitized.morph_operation)
    edge_operation = int(sanitized.edge_operation)
    return {
        "opencv_mode": sanitized.opencv_mode,
        "center_method": sanitized.center_method,
        "direction_method": sanitized.direction_method,
        "direction_sense_method": sanitized.direction_sense_method,
        "shape_filter": sanitized.shape_filter,
        "min_area_px": float(sanitized.min_area_px),
        "max_area_px": float(sanitized.max_area_px),
        "min_aspect_ratio": float(sanitized.min_aspect_ratio),
        "max_aspect_ratio": float(sanitized.max_aspect_ratio),
        "morph_kernel": int(sanitized.morph_kernel),
        "morph_open_iterations": abs(morph_operation) if morph_operation < 0 else 0,
        "morph_close_iterations": morph_operation if morph_operation > 0 else 0,
        "roi_margin_px": int(sanitized.roi_margin_px),
        "brightness_threshold": int(sanitized.brightness_threshold),
        "brightness_invert": bool(sanitized.brightness_invert),
        "blur_kernel": int(sanitized.blur_kernel),
        "canny_low": int(sanitized.canny_low),
        "canny_high": int(sanitized.canny_high),
        "edge_aperture_size": int(sanitized.edge_aperture_size),
        "edge_dilate_iterations": edge_operation if edge_operation > 0 else 0,
        "edge_erode_iterations": abs(edge_operation) if edge_operation < 0 else 0,
        "hsv_lower": [int(v) for v in sanitized.hsv_lower],
        "hsv_upper": [int(v) for v in sanitized.hsv_upper],
        "rgb_lower": [int(v) for v in sanitized.rgb_lower],
        "rgb_upper": [int(v) for v in sanitized.rgb_upper],
        "shape_epsilon_ratio": float(sanitized.shape_epsilon_ratio),
        "circle_min_circularity": float(sanitized.circle_min_circularity),
        "square_aspect_tolerance": float(sanitized.square_aspect_tolerance),
        "lying_aspect_ratio_threshold": float(sanitized.lying_aspect_ratio_threshold),
        "min_solidity": float(sanitized.min_solidity),
        "min_extent": float(sanitized.min_extent),
        "max_extent": float(sanitized.max_extent),
        "use_workspace_mask": bool(sanitized.use_workspace_mask),
        "draw_workspace_limit": bool(sanitized.draw_workspace_limit),
        "workspace_limit_shape": sanitized.workspace_limit_shape,
        "workspace_limit_pixels": [float(v) for v in sanitized.workspace_limit_pixels],
        "workspace_circle_pixel": [float(v) for v in sanitized.workspace_circle_pixel],
        "workspace_mask_margin_px": int(sanitized.workspace_mask_margin_px),
    }


def api_response_schema() -> dict[str, Any]:
    """Return the strict JSON Schema sent to the AI Responses API."""
    number = {"type": "number"}
    integer = {"type": "integer"}
    boolean = {"type": "boolean"}
    string = {"type": "string"}
    uint8_triplet = {
        "type": "array",
        "minItems": 3,
        "maxItems": 3,
        "items": {"type": "integer", "minimum": 0, "maximum": UINT8_MAX},
    }
    hsv_triplet = {
        "type": "array",
        "minItems": 3,
        "maxItems": 3,
        # OpenAI's strict schema does not support tuple validation (array of
        # schemas per position). Hue is clamped to [0, HUE_MAX] during
        # sanitization after parsing.
        "items": {"type": "integer", "minimum": 0, "maximum": UINT8_MAX},
    }
    point_list = {
        "type": "array",
        "items": {
            "type": "array",
            "minItems": 2,
            "maxItems": 2,
            "items": {"type": "number"},
        },
    }

    profile_properties: dict[str, Any] = {
        "opencv_mode": {
            **string,
            "description": (
                "Detection mode. Combine tokens 'bright', 'edge', 'hsv', 'rgb' "
                "with '_or_' (e.g. 'bright_or_edge')."
            ),
        },
        "active_filters": {
            "type": "array",
            "items": {"type": "string", "enum": sorted(VALID_FILTER_KEYS)},
        },
        "center_method": {**string, "enum": sorted(CENTER_METHODS)},
        "direction_method": {**string, "enum": sorted(DIRECTION_METHODS)},
        "direction_sense_method": {**string, "enum": sorted(DIRECTION_SENSE_METHODS)},
        "shape_filter": {**string, "enum": sorted(SHAPE_FILTERS)},
        "min_area_px": {**number, "minimum": 0},
        "max_area_px": {**number, "minimum": 0},
        "min_aspect_ratio": {**number, "minimum": 0, "maximum": ASPECT_RATIO_MAX},
        "max_aspect_ratio": {**number, "minimum": 0, "maximum": ASPECT_RATIO_MAX},
        "morph_kernel": {
            **integer,
            "minimum": 1,
            "maximum": MORPH_KERNEL_MAX,
            "description": "Odd kernel size for morphology operations.",
        },
        "morph_operation": {
            **integer,
            "minimum": -MORPH_OPERATION_MAX,
            "maximum": MORPH_OPERATION_MAX,
            "description": "Negative = open iterations, positive = close iterations, 0 = disabled.",
        },
        "roi_margin_px": {**integer, "minimum": 0},
        "brightness_threshold": {**integer, "minimum": 0, "maximum": UINT8_MAX},
        "brightness_invert": boolean,
        "blur_kernel": {
            **integer,
            "minimum": 1,
            "maximum": MORPH_KERNEL_MAX,
            "description": "Odd Gaussian blur kernel. Even values are rounded up.",
        },
        "canny_low": {**integer, "minimum": 0, "maximum": UINT8_MAX},
        "canny_high": {**integer, "minimum": 0, "maximum": UINT8_MAX},
        "edge_aperture_size": {**integer, "enum": sorted(EDGE_APERTURE_SIZES)},
        "edge_operation": {
            **integer,
            "minimum": -MORPH_OPERATION_MAX,
            "maximum": MORPH_OPERATION_MAX,
            "description": "Negative = erode iterations, positive = dilate iterations, 0 = disabled.",
        },
        "hsv_lower": hsv_triplet,
        "hsv_upper": hsv_triplet,
        "rgb_lower": uint8_triplet,
        "rgb_upper": uint8_triplet,
        "shape_epsilon_ratio": {**number, "minimum": 0.005, "maximum": 0.20},
        "circle_min_circularity": {**number, "minimum": 0.10, "maximum": 1.0},
        "square_aspect_tolerance": {**number, "minimum": 0.01, "maximum": 1.0},
        "lying_aspect_ratio_threshold": {**number, "minimum": 1.0, "maximum": 10.0},
        "min_solidity": {**number, "minimum": 0.0, "maximum": 1.0},
        "min_extent": {**number, "minimum": 0.0, "maximum": 1.0},
        "max_extent": {**number, "minimum": 0.0, "maximum": 1.0},
        "use_workspace_mask": boolean,
        "draw_workspace_limit": boolean,
        "workspace_limit_shape": {**string, "enum": sorted(WORKSPACE_SHAPES)},
        "workspace_limit_pixels": {
            "type": "array",
            "items": {"type": "number", "minimum": 0},
        },
        "workspace_circle_pixel": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": {"type": "number", "minimum": 0},
        },
        "workspace_mask_margin_px": {**integer, "minimum": 0},
    }
    profile_required = sorted(profile_properties.keys())

    reference_object = {
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "class": string,
            "bbox_rotated": {
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    "cx": {**number, "minimum": 0.0, "maximum": 1.0},
                    "cy": {**number, "minimum": 0.0, "maximum": 1.0},
                    "w": {**number, "minimum": 0.0, "maximum": 1.0},
                    "h": {**number, "minimum": 0.0, "maximum": 1.0},
                    "angle_deg": number,
                },
                "required": ["cx", "cy", "w", "h", "angle_deg"],
            },
            "orientation": {**string, "enum": sorted(ORIENTATIONS)},
            "grasp_inset_ratio": {**number, "minimum": 0.0, "maximum": 0.5},
            "confidence": {**number, "minimum": 0.0, "maximum": 1.0},
        },
        "required": ["class", "bbox_rotated", "orientation", "grasp_inset_ratio", "confidence"],
    }

    return {
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "scene": {
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    "expected_objects": {**integer, "minimum": 0},
                    "workspace_polygon": {
                        **point_list,
                        "description": (
                            "Pixel vertices of the polygon that delimits the area of interest. "
                            "The model should infer this ROI from the image and exclude rails, "
                            "robot parts, floor, frame borders and other non-workspace regions."
                        ),
                    },
                },
                "required": ["expected_objects", "workspace_polygon"],
            },
            "profiles": {
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    "vertical": {
                        "type": "object",
                        "additionalProperties": False,
                        "properties": profile_properties,
                        "required": profile_required,
                    },
                    "horizontal": {
                        "type": "object",
                        "additionalProperties": False,
                        "properties": profile_properties,
                        "required": profile_required,
                    },
                },
                "required": ["vertical", "horizontal"],
            },
            "merge": {
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    "rotated_iou_threshold": {**number, "minimum": 0.0, "maximum": 1.0},
                    "center_distance_threshold": {**number, "minimum": 0.0},
                },
                "required": ["rotated_iou_threshold", "center_distance_threshold"],
            },
            "reference_objects": {
                "type": "array",
                "minItems": 0,
                "items": reference_object,
                "description": (
                    "Optional list of annotated reference objects. Only include "
                    "objects you are highly confident about."
                ),
            },
            "iteration": {
                "type": "object",
                "additionalProperties": False,
                "properties": {
                    "number": {**integer, "minimum": 0},
                    "converged": boolean,
                    "notes": string,
                },
                "required": ["number", "converged", "notes"],
            },
        },
        "required": [
            "scene",
            "profiles",
            "merge",
            "iteration",
            "reference_objects",
        ],
    }


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def _clamp(value: float, lower: float, upper: float) -> float:
    return float(max(lower, min(upper, value)))


def _clamp_default(value: float, lower: float, upper: float, default: float) -> float:
    try:
        parsed = float(value)
    except Exception:
        parsed = float(default)
    return _clamp(parsed, lower, upper)


def _clamp01(value: float) -> float:
    return _clamp(value, 0.0, 1.0)


def _clamp_int(
    value: Any,
    minimum: int,
    maximum: int,
    default: int,
    *,
    odd: bool = False,
    allowed: set[int] | None = None,
) -> int:
    try:
        parsed = int(round(float(value)))
    except Exception:
        parsed = int(default)
    if allowed:
        parsed = min(allowed, key=lambda item: abs(item - parsed))
    parsed = max(minimum, min(maximum, parsed))
    if odd and parsed % 2 == 0:
        parsed = min(maximum, parsed + 1) if parsed < maximum else max(minimum, parsed - 1)
    return parsed


def _int_triplet(value: Any, default: list[int]) -> list[int]:
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return list(default)
    return [int(round(float(v))) for v in value[:3]]


def _uint8_triplet(value: Any, default: list[int]) -> list[int]:
    triplet = _int_triplet(value, default)
    return [max(0, min(UINT8_MAX, v)) for v in triplet]


def _hsv_triplet(value: Any, default: list[int]) -> list[int]:
    triplet = _int_triplet(value, default)
    return [
        max(0, min(HUE_MAX, triplet[0])),
        max(0, min(UINT8_MAX, triplet[1])),
        max(0, min(UINT8_MAX, triplet[2])),
    ]


def _choose(value: str, allowed: set[str], default: str) -> str:
    return value if value in allowed else default
