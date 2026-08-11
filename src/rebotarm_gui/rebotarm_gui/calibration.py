"""Versioned work-plane calibration profiles for vision-guided production.

The calibration maps camera pixels ``(u, v)`` to the robot base work plane
``(x, y)``.  Approved profiles are immutable, integrity checked and tied to
the camera resolution/frame and active TCP/tool context.

This module deliberately has no Qt or ROS dependency so fitting, persistence
and production gates can be tested without a running cell.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
import os
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Iterable

import numpy as np


CALIBRATION_SCHEMA_VERSION = 1
STATUS_DRAFT = "draft"
STATUS_APPROVED = "approved"
STATUS_OBSOLETE = "obsolete"
VALID_STATUSES = {STATUS_DRAFT, STATUS_APPROVED, STATUS_OBSOLETE}

MIN_FIT_POINTS = 4
MIN_APPROVAL_POINTS = 5
DEFAULT_RANSAC_THRESHOLD_M = 0.006
DEFAULT_MAX_RMS_ERROR_M = 0.004
DEFAULT_MAX_ERROR_M = 0.010
DEFAULT_MIN_COVERAGE = 0.05
MAX_RANSAC_CANDIDATES = 256


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace(
        "+00:00", "Z"
    )


def _parse_utc(value: str) -> datetime | None:
    text = str(value).strip()
    if not text:
        return None
    try:
        parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def default_calibration_dir() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME", "").strip()
    root = Path(base).expanduser() if base else Path.home() / ".config"
    return root / "rebotarm_gui" / "calibrations"


@dataclass(frozen=True)
class CalibrationPoint:
    u: float
    v: float
    x: float
    y: float
    enabled: bool = True

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> "CalibrationPoint":
        return cls(
            u=float(data["u"]),
            v=float(data["v"]),
            x=float(data["x"]),
            y=float(data["y"]),
            enabled=bool(data.get("enabled", True)),
        )


@dataclass
class CalibrationMetrics:
    sample_count: int = 0
    inlier_count: int = 0
    rms_error_m: float = math.inf
    max_error_m: float = math.inf
    p95_error_m: float = math.inf
    pixel_coverage: float = 0.0
    condition_number: float = math.inf
    residuals_m: list[float] = field(default_factory=list)
    inliers: list[bool] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> "CalibrationMetrics":
        return cls(
            sample_count=int(data.get("sample_count", 0)),
            inlier_count=int(data.get("inlier_count", 0)),
            rms_error_m=float(data.get("rms_error_m", math.inf)),
            max_error_m=float(data.get("max_error_m", math.inf)),
            p95_error_m=float(data.get("p95_error_m", math.inf)),
            pixel_coverage=float(data.get("pixel_coverage", 0.0)),
            condition_number=float(data.get("condition_number", math.inf)),
            residuals_m=[
                float(value) for value in data.get("residuals_m", [])
            ],
            inliers=[bool(value) for value in data.get("inliers", [])],
        )

    def summary(self) -> str:
        if self.sample_count <= 0 or not math.isfinite(self.rms_error_m):
            return "não validada"
        return (
            f"RMS {self.rms_error_m * 1000.0:.2f} mm · "
            f"máx. {self.max_error_m * 1000.0:.2f} mm · "
            f"{self.inlier_count}/{self.sample_count} amostras · "
            f"cobertura {self.pixel_coverage * 100.0:.1f}%"
        )


@dataclass
class CalibrationProfile:
    name: str = "calibração da célula"
    profile_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    revision: int = 1
    schema_version: int = CALIBRATION_SCHEMA_VERSION
    status: str = STATUS_DRAFT
    camera_frame: str = "camera_link"
    target_frame: str = "base_link"
    image_width: int = 0
    image_height: int = 0
    table_z_m: float = 0.0
    tcp_frame: str = "gripper_tcp"
    tool_id: str = "gripper"
    payload_kg: float = 0.0
    validity_days: int = 180
    points: list[CalibrationPoint] = field(default_factory=list)
    homography: list[float] = field(default_factory=list)
    metrics: CalibrationMetrics = field(default_factory=CalibrationMetrics)
    created_at_utc: str = field(default_factory=utc_now)
    updated_at_utc: str = field(default_factory=utc_now)
    approved_at_utc: str = ""
    approved_by: str = ""
    expires_at_utc: str = ""
    integrity_hash: str = ""

    @classmethod
    def from_dict(cls, data: dict[str, object]) -> "CalibrationProfile":
        metrics = data.get("metrics", {})
        points = data.get("points", [])
        profile = cls(
            name=str(data.get("name", "calibração da célula")),
            profile_id=str(data.get("profile_id", "")).strip() or uuid.uuid4().hex,
            revision=max(1, int(data.get("revision", 1))),
            schema_version=int(
                data.get("schema_version", CALIBRATION_SCHEMA_VERSION)
            ),
            status=str(data.get("status", STATUS_DRAFT)),
            camera_frame=str(data.get("camera_frame", "camera_link")).strip(),
            target_frame=str(data.get("target_frame", "base_link")).strip(),
            image_width=max(0, int(data.get("image_width", 0))),
            image_height=max(0, int(data.get("image_height", 0))),
            table_z_m=float(data.get("table_z_m", 0.0)),
            tcp_frame=str(data.get("tcp_frame", "gripper_tcp")).strip(),
            tool_id=str(data.get("tool_id", "gripper")).strip(),
            payload_kg=max(0.0, float(data.get("payload_kg", 0.0))),
            validity_days=max(1, int(data.get("validity_days", 180))),
            points=[
                CalibrationPoint.from_dict(point)
                for point in points
                if isinstance(point, dict)
            ],
            homography=[float(value) for value in data.get("homography", [])],
            metrics=CalibrationMetrics.from_dict(
                metrics if isinstance(metrics, dict) else {}
            ),
            created_at_utc=str(data.get("created_at_utc", "")) or utc_now(),
            updated_at_utc=str(data.get("updated_at_utc", "")) or utc_now(),
            approved_at_utc=str(data.get("approved_at_utc", "")),
            approved_by=str(data.get("approved_by", "")),
            expires_at_utc=str(data.get("expires_at_utc", "")),
            integrity_hash=str(data.get("integrity_hash", "")),
        )
        if profile.status not in VALID_STATUSES:
            profile.status = STATUS_DRAFT
        return profile

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

    def integrity_payload(self) -> dict[str, object]:
        payload = self.to_dict()
        payload.pop("integrity_hash", None)
        return payload

    def compute_integrity_hash(self) -> str:
        canonical = json.dumps(
            self.integrity_payload(),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        return hashlib.sha256(canonical).hexdigest()

    def verify_integrity(self) -> tuple[bool, str]:
        if self.status != STATUS_APPROVED:
            return False, "perfil não está aprovado"
        if not self.integrity_hash:
            return False, "perfil aprovado sem hash de integridade"
        try:
            expected = self.compute_integrity_hash()
        except (TypeError, ValueError):
            return False, "perfil contém valores não serializáveis"
        if expected != self.integrity_hash:
            return False, "conteúdo do perfil mudou após a aprovação"
        return True, ""

    def start_new_revision(self) -> None:
        self.revision = max(1, int(self.revision)) + 1
        self.status = STATUS_DRAFT
        self.approved_at_utc = ""
        self.approved_by = ""
        self.expires_at_utc = ""
        self.integrity_hash = ""
        self.updated_at_utc = utc_now()

    def fit(self, *, ransac_threshold_m: float = DEFAULT_RANSAC_THRESHOLD_M) -> None:
        enabled = [point for point in self.points if point.enabled]
        homography, metrics = fit_workplane_homography(
            enabled,
            image_width=self.image_width,
            image_height=self.image_height,
            ransac_threshold_m=ransac_threshold_m,
        )
        self.homography = [float(value) for value in homography.reshape(-1)]
        self.metrics = metrics
        self.status = STATUS_DRAFT
        self.approved_at_utc = ""
        self.approved_by = ""
        self.expires_at_utc = ""
        self.integrity_hash = ""
        self.updated_at_utc = utc_now()

    def approval_readiness(
        self,
        *,
        max_rms_error_m: float = DEFAULT_MAX_RMS_ERROR_M,
        max_error_m: float = DEFAULT_MAX_ERROR_M,
        min_coverage: float = DEFAULT_MIN_COVERAGE,
    ) -> tuple[bool, str]:
        if len([point for point in self.points if point.enabled]) < MIN_APPROVAL_POINTS:
            return (
                False,
                f"colete ao menos {MIN_APPROVAL_POINTS} pontos para medir o erro",
            )
        if len(self.homography) != 9:
            return False, "valide o ajuste antes de aprovar"
        metrics = self.metrics
        if metrics.inlier_count < MIN_APPROVAL_POINTS:
            return False, "menos de cinco amostras consistentes"
        if not math.isfinite(metrics.condition_number) or metrics.condition_number > 1e12:
            return False, "geometria dos pontos está degenerada ou mal condicionada"
        if metrics.pixel_coverage < float(min_coverage):
            return (
                False,
                "pontos cobrem uma área muito pequena da imagem; distribua-os pelo envelope",
            )
        if metrics.rms_error_m > float(max_rms_error_m):
            return (
                False,
                f"erro RMS {metrics.rms_error_m * 1000.0:.2f} mm excede "
                f"{float(max_rms_error_m) * 1000.0:.2f} mm",
            )
        if metrics.max_error_m > float(max_error_m):
            return (
                False,
                f"erro máximo {metrics.max_error_m * 1000.0:.2f} mm excede "
                f"{float(max_error_m) * 1000.0:.2f} mm",
            )
        if self.image_width <= 0 or self.image_height <= 0:
            return False, "resolução da câmera é obrigatória"
        if not self.camera_frame or not self.target_frame or not self.tcp_frame:
            return False, "frames da câmera, base e TCP são obrigatórios"
        if not self.tool_id:
            return False, "identificação da ferramenta é obrigatória"
        return True, ""

    def approve(
        self,
        operator: str,
        *,
        max_rms_error_m: float = DEFAULT_MAX_RMS_ERROR_M,
        max_error_m: float = DEFAULT_MAX_ERROR_M,
        min_coverage: float = DEFAULT_MIN_COVERAGE,
    ) -> str:
        ready, reason = self.approval_readiness(
            max_rms_error_m=max_rms_error_m,
            max_error_m=max_error_m,
            min_coverage=min_coverage,
        )
        if not ready:
            raise ValueError(reason)
        identity = str(operator).strip()
        if not identity:
            raise ValueError("operador aprovador não identificado")
        approved = datetime.now(timezone.utc)
        self.status = STATUS_APPROVED
        self.approved_by = identity
        self.approved_at_utc = approved.isoformat(timespec="seconds").replace(
            "+00:00", "Z"
        )
        self.expires_at_utc = (
            approved + timedelta(days=max(1, int(self.validity_days)))
        ).isoformat(timespec="seconds").replace("+00:00", "Z")
        self.updated_at_utc = self.approved_at_utc
        self.integrity_hash = self.compute_integrity_hash()
        return self.integrity_hash

    def production_status(
        self,
        *,
        camera_frame: str = "",
        image_width: int = 0,
        image_height: int = 0,
        target_frame: str = "base_link",
        tcp_frame: str = "gripper_tcp",
        tool_id: str = "",
        now: datetime | None = None,
    ) -> tuple[bool, str]:
        valid, reason = self.verify_integrity()
        if not valid:
            return False, reason
        expires = _parse_utc(self.expires_at_utc)
        current = now or datetime.now(timezone.utc)
        if expires is None or current.astimezone(timezone.utc) >= expires:
            return False, "perfil de calibração vencido"
        if camera_frame and self.camera_frame != str(camera_frame).strip():
            return (
                False,
                f"frame da câmera mudou ({self.camera_frame} → {camera_frame})",
            )
        if image_width > 0 and self.image_width != int(image_width):
            return (
                False,
                f"largura da imagem mudou ({self.image_width} → {int(image_width)})",
            )
        if image_height > 0 and self.image_height != int(image_height):
            return (
                False,
                f"altura da imagem mudou ({self.image_height} → {int(image_height)})",
            )
        if target_frame and self.target_frame != str(target_frame).strip():
            return False, "frame base incompatível com a célula"
        if tcp_frame and self.tcp_frame != str(tcp_frame).strip():
            return False, "TCP ativo incompatível com o perfil"
        if tool_id and self.tool_id != str(tool_id).strip():
            return False, "ferramenta ativa incompatível com o perfil"
        return True, ""

    def production_reference(self) -> dict[str, object]:
        return {
            "version": 1,
            "profile_id": self.profile_id,
            "revision": self.revision,
            "integrity_hash": self.integrity_hash,
            "camera_frame": self.camera_frame,
            "target_frame": self.target_frame,
            "image_size": [self.image_width, self.image_height],
            "tcp_frame": self.tcp_frame,
            "tool_id": self.tool_id,
            "expires_at_utc": self.expires_at_utc,
            "rms_error_m": self.metrics.rms_error_m,
            "max_error_m": self.metrics.max_error_m,
        }


def _convex_hull_area(points: np.ndarray) -> float:
    unique = sorted({(float(point[0]), float(point[1])) for point in points})
    if len(unique) < 3:
        return 0.0

    def cross(origin, first, second) -> float:
        return (
            (first[0] - origin[0]) * (second[1] - origin[1])
            - (first[1] - origin[1]) * (second[0] - origin[0])
        )

    lower: list[tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    hull = lower[:-1] + upper[:-1]
    return abs(
        sum(
            hull[index][0] * hull[(index + 1) % len(hull)][1]
            - hull[(index + 1) % len(hull)][0] * hull[index][1]
            for index in range(len(hull))
        )
    ) * 0.5


def _normalization(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    center = np.mean(points, axis=0)
    distances = np.linalg.norm(points - center, axis=1)
    mean_distance = float(np.mean(distances))
    if mean_distance <= 1e-12:
        raise ValueError("pontos coincidentes")
    scale = math.sqrt(2.0) / mean_distance
    transform = np.array(
        [
            [scale, 0.0, -scale * center[0]],
            [0.0, scale, -scale * center[1]],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    homogeneous = np.column_stack([points, np.ones(len(points))])
    normalized = (transform @ homogeneous.T).T
    return normalized[:, :2], transform


def _fit_homography(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, float]:
    if len(src) < MIN_FIT_POINTS:
        raise ValueError(f"são necessários ao menos {MIN_FIT_POINTS} pontos")
    if _convex_hull_area(src) <= 1e-6 or _convex_hull_area(dst) <= 1e-12:
        raise ValueError("pontos colineares ou sem cobertura")
    src_n, src_transform = _normalization(src)
    dst_n, dst_transform = _normalization(dst)
    rows: list[list[float]] = []
    for (u, v), (x, y) in zip(src_n, dst_n):
        rows.append([-u, -v, -1.0, 0.0, 0.0, 0.0, x * u, x * v, x])
        rows.append([0.0, 0.0, 0.0, -u, -v, -1.0, y * u, y * v, y])
    matrix = np.asarray(rows, dtype=float)
    _u, singular, vh = np.linalg.svd(matrix)
    if len(singular) < 2 or singular[-2] <= 1e-14:
        raise ValueError("ajuste singular")
    normalized_h = vh[-1].reshape((3, 3))
    homography = np.linalg.inv(dst_transform) @ normalized_h @ src_transform
    if abs(float(homography[2, 2])) <= 1e-14:
        raise ValueError("homografia degenerada")
    homography /= homography[2, 2]
    condition = float(singular[0] / singular[-2])
    if not np.all(np.isfinite(homography)):
        raise ValueError("homografia não finita")
    return homography, condition


def project_points(homography: np.ndarray, src: np.ndarray) -> np.ndarray:
    homogeneous = np.column_stack([src, np.ones(len(src))])
    projected = (homography @ homogeneous.T).T
    denominator = projected[:, 2]
    if np.any(np.abs(denominator) <= 1e-14):
        raise ValueError("projeção no infinito")
    return projected[:, :2] / denominator[:, None]


def fit_workplane_homography(
    points: Iterable[CalibrationPoint],
    *,
    image_width: int,
    image_height: int,
    ransac_threshold_m: float = DEFAULT_RANSAC_THRESHOLD_M,
) -> tuple[np.ndarray, CalibrationMetrics]:
    samples = [point for point in points if point.enabled]
    if len(samples) < MIN_FIT_POINTS:
        raise ValueError(f"são necessários ao menos {MIN_FIT_POINTS} pontos")
    src = np.asarray([(point.u, point.v) for point in samples], dtype=float)
    dst = np.asarray([(point.x, point.y) for point in samples], dtype=float)
    if not np.all(np.isfinite(src)) or not np.all(np.isfinite(dst)):
        raise ValueError("amostras contêm valores não finitos")
    if len({(point.u, point.v) for point in samples}) != len(samples):
        raise ValueError("há pixels duplicados na calibração")

    threshold = max(0.0005, float(ransac_threshold_m))
    best_mask = np.ones(len(samples), dtype=bool)
    best_score = (-1, math.inf)
    candidates = itertools.islice(
        itertools.combinations(range(len(samples)), MIN_FIT_POINTS),
        MAX_RANSAC_CANDIDATES,
    )
    for indexes in candidates:
        try:
            candidate, _condition = _fit_homography(
                src[list(indexes)], dst[list(indexes)]
            )
            residuals = np.linalg.norm(project_points(candidate, src) - dst, axis=1)
        except (ValueError, np.linalg.LinAlgError):
            continue
        mask = residuals <= threshold
        inlier_count = int(np.count_nonzero(mask))
        if inlier_count < MIN_FIT_POINTS:
            continue
        rms = float(math.sqrt(np.mean(np.square(residuals[mask]))))
        score = (inlier_count, -rms)
        if score > (best_score[0], -best_score[1]):
            best_score = (inlier_count, rms)
            best_mask = mask

    homography, condition = _fit_homography(src[best_mask], dst[best_mask])
    residuals = np.linalg.norm(project_points(homography, src) - dst, axis=1)
    final_mask = residuals <= threshold
    if int(np.count_nonzero(final_mask)) >= MIN_FIT_POINTS and not np.array_equal(
        final_mask, best_mask
    ):
        homography, condition = _fit_homography(src[final_mask], dst[final_mask])
        residuals = np.linalg.norm(project_points(homography, src) - dst, axis=1)
        best_mask = residuals <= threshold
    else:
        best_mask = final_mask
    inlier_residuals = residuals[best_mask]
    if len(inlier_residuals) < MIN_FIT_POINTS:
        raise ValueError("não foi possível encontrar quatro amostras consistentes")

    image_area = max(1.0, float(image_width) * float(image_height))
    coverage = min(1.0, _convex_hull_area(src[best_mask]) / image_area)
    metrics = CalibrationMetrics(
        sample_count=len(samples),
        inlier_count=int(np.count_nonzero(best_mask)),
        rms_error_m=float(math.sqrt(np.mean(np.square(inlier_residuals)))),
        max_error_m=float(np.max(inlier_residuals)),
        p95_error_m=float(np.percentile(inlier_residuals, 95)),
        pixel_coverage=float(coverage),
        condition_number=condition,
        residuals_m=[float(value) for value in residuals],
        inliers=[bool(value) for value in best_mask],
    )
    return homography, metrics


class CalibrationStore:
    """Atomic owner-only persistence for calibration revisions and active profile."""

    def __init__(self, directory: str | Path | None = None) -> None:
        self.directory = (
            Path(directory).expanduser() if directory else default_calibration_dir()
        )
        self.active_path = self.directory / "active.json"
        self.last_error = ""

    @staticmethod
    def _safe_id(profile_id: str) -> str:
        safe = "".join(char for char in str(profile_id) if char.isalnum() or char in "-_")
        if not safe:
            raise ValueError("identificador de perfil inválido")
        return safe[:96]

    def profile_path(self, profile: CalibrationProfile) -> Path:
        return self.directory / (
            f"{self._safe_id(profile.profile_id)}_r{max(1, int(profile.revision)):04d}.json"
        )

    def _write_json(self, target: Path, payload: dict[str, object]) -> None:
        self.directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        try:
            os.chmod(self.directory, 0o700)
        except OSError:
            pass
        temporary = target.with_suffix(target.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(
                payload,
                stream,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o600)
        os.replace(temporary, target)

    def save(self, profile: CalibrationProfile, *, activate: bool = False) -> Path:
        profile.updated_at_utc = utc_now()
        if profile.status == STATUS_APPROVED:
            profile.integrity_hash = profile.compute_integrity_hash()
        target = self.profile_path(profile)
        self._write_json(target, profile.to_dict())
        if activate:
            if profile.status != STATUS_APPROVED:
                raise ValueError("somente um perfil aprovado pode ser ativado")
            self._write_json(
                self.active_path,
                {
                    "schema_version": 1,
                    "profile_file": target.name,
                    "profile_id": profile.profile_id,
                    "revision": profile.revision,
                    "integrity_hash": profile.integrity_hash,
                    "activated_at_utc": utc_now(),
                },
            )
        return target

    def load_path(self, path: str | Path) -> CalibrationProfile:
        source = Path(path).expanduser()
        data = json.loads(source.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("arquivo de calibração não contém um objeto")
        profile = CalibrationProfile.from_dict(data)
        if profile.schema_version != CALIBRATION_SCHEMA_VERSION:
            raise ValueError(
                f"versão de calibração {profile.schema_version} não suportada"
            )
        return profile

    def active(self) -> CalibrationProfile | None:
        self.last_error = ""
        if not self.active_path.is_file():
            self.last_error = "nenhum perfil de calibração ativo"
            return None
        try:
            pointer = json.loads(self.active_path.read_text(encoding="utf-8"))
            if not isinstance(pointer, dict):
                raise ValueError("índice ativo inválido")
            filename = Path(str(pointer["profile_file"])).name
            profile = self.load_path(self.directory / filename)
            if profile.profile_id != str(pointer.get("profile_id", "")):
                raise ValueError("identificador do perfil ativo não confere")
            if profile.revision != int(pointer.get("revision", 0)):
                raise ValueError("revisão do perfil ativo não confere")
            if profile.integrity_hash != str(pointer.get("integrity_hash", "")):
                raise ValueError("hash do perfil ativo não confere")
            valid, reason = profile.verify_integrity()
            if not valid:
                raise ValueError(reason)
            return profile
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.last_error = f"perfil ativo inválido: {exc}"
            return None

    def revisions(self) -> list[Path]:
        if not self.directory.is_dir():
            return []
        return sorted(
            self.directory.glob("*_r[0-9][0-9][0-9][0-9].json"),
            key=lambda path: path.stat().st_mtime_ns,
            reverse=True,
        )

    def production_status(self, **runtime: object) -> tuple[bool, str, CalibrationProfile | None]:
        profile = self.active()
        if profile is None:
            return False, self.last_error or "calibração indisponível", None
        valid, reason = profile.production_status(**runtime)
        return valid, reason, profile
