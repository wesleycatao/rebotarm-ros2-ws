"""Persistent GUI settings."""

from __future__ import annotations

import getpass
import json
import os
from dataclasses import dataclass, field
from pathlib import Path


FONT_SCALE_MIN = 90
FONT_SCALE_MAX = 130
FONT_SCALE_DEFAULT = 100

COMMISSIONING_CHECKLIST_KEYS = (
    "calibration_approved",
    "cell_scene_applied",
    "vision_validated",
    "program_generated",
)


def _default_commissioning_checklist() -> dict[str, bool]:
    return {key: False for key in COMMISSIONING_CHECKLIST_KEYS}


@dataclass
class AppSettings:
    # User profile gating advanced pages: "operador" | "supervisor" | "engenharia"
    profile: str = "supervisor"
    # Human-readable workstation/operator identifier stored with audit events.
    operator_id: str = field(default_factory=getpass.getuser)
    # GUI-wide motion override. 1.0 = 100%; values are deliberately capped at
    # 100% so this control can slow commissioning/production moves but cannot
    # make a validated program faster than its authored speed.
    speed_override: float = 1.0
    # Production is fail-closed: only released recipes may execute and command
    # authorization is enforced by the RosBridge capability gate.
    production_mode: bool = False
    # Require MoveIt collision-aware execution for real recipe movements.
    collision_planning_required: bool = True
    # Color theme: "dark" (default, low ambient light) or "light" (high ambient
    # light on a factory floor where the dark theme is hard to read).
    theme: str = "dark"
    # Interface language code. "pt" (default) uses the in-code Portuguese
    # strings; any other code loads ``locale/<code>.json`` at startup via i18n.
    language: str = "pt"
    # Application-wide typography scale. QSS point sizes are scaled together
    # so hierarchy and spacing remain predictable on small and high-DPI panels.
    font_scale_percent: int = FONT_SCALE_DEFAULT
    # Persistent progress for the commissioning guide shown on Início.
    commissioning_checklist: dict[str, bool] = field(
        default_factory=_default_commissioning_checklist
    )


VALID_PROFILES = ("operador", "supervisor", "engenharia")
VALID_THEMES = ("dark", "light")
VALID_LANGUAGES = ("pt", "en")
SPEED_OVERRIDE_MIN = 0.05
SPEED_OVERRIDE_MAX = 1.0


def clamp_font_scale(value: object) -> int:
    """Return a supported integer font scale (90–130 percent)."""
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return FONT_SCALE_DEFAULT
    return max(FONT_SCALE_MIN, min(FONT_SCALE_MAX, parsed))


def normalize_commissioning_checklist(value: object) -> dict[str, bool]:
    """Keep only known checklist steps and coerce their persisted state."""
    data = value if isinstance(value, dict) else {}
    return {
        key: bool(data.get(key, False))
        for key in COMMISSIONING_CHECKLIST_KEYS
    }


def clamp_speed_override(value: object) -> float:
    """Return a finite GUI speed override inside the supported safe range."""
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return SPEED_OVERRIDE_MAX
    if not (parsed == parsed and abs(parsed) != float("inf")):
        return SPEED_OVERRIDE_MAX
    return max(SPEED_OVERRIDE_MIN, min(SPEED_OVERRIDE_MAX, parsed))


def default_settings_path() -> Path:
    return Path.home() / ".config" / "rebotarm_gui" / "settings.json"


def load_app_settings(path: Path | None = None) -> AppSettings:
    settings_path = path or default_settings_path()
    if not settings_path.is_file():
        return AppSettings()
    # Mantém o arquivo de preferências restrito ao usuário.
    try:
        if settings_path.stat().st_mode & 0o077:
            os.chmod(settings_path, 0o600)
    except OSError:
        pass
    try:
        data = json.loads(settings_path.read_text(encoding="utf-8"))
    except Exception:
        return AppSettings()
    profile = str(data.get("profile", "supervisor"))
    if profile not in VALID_PROFILES:
        profile = "supervisor"
    theme = str(data.get("theme", "dark"))
    if theme not in VALID_THEMES:
        theme = "dark"
    language = str(data.get("language", "pt")).strip().lower() or "pt"
    if language not in VALID_LANGUAGES:
        language = "pt"
    operator_id = str(data.get("operator_id", "")).strip() or getpass.getuser()
    speed_override = clamp_speed_override(data.get("speed_override", 1.0))
    production_mode = bool(data.get("production_mode", False))
    collision_planning_required = bool(data.get("collision_planning_required", True))
    font_scale_percent = clamp_font_scale(
        data.get("font_scale_percent", FONT_SCALE_DEFAULT)
    )
    commissioning_checklist = normalize_commissioning_checklist(
        data.get("commissioning_checklist", {})
    )
    return AppSettings(
        profile=profile,
        operator_id=operator_id,
        speed_override=speed_override,
        production_mode=production_mode,
        collision_planning_required=collision_planning_required,
        theme=theme,
        language=language,
        font_scale_percent=font_scale_percent,
        commissioning_checklist=commissioning_checklist,
    )


def save_app_settings(settings: AppSettings, path: Path | None = None) -> Path:
    settings_path = path or default_settings_path()
    settings_path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "profile": settings.profile if settings.profile in VALID_PROFILES else "supervisor",
        "operator_id": settings.operator_id.strip(),
        "speed_override": clamp_speed_override(settings.speed_override),
        "production_mode": bool(settings.production_mode),
        "collision_planning_required": bool(settings.collision_planning_required),
        "theme": settings.theme if settings.theme in VALID_THEMES else "dark",
        "language": settings.language if settings.language in VALID_LANGUAGES else "pt",
        "font_scale_percent": clamp_font_scale(settings.font_scale_percent),
        "commissioning_checklist": normalize_commissioning_checklist(
            settings.commissioning_checklist
        ),
    }
    settings_path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")
    # Preferências operacionais e de segurança permanecem restritas ao dono.
    os.chmod(settings_path, 0o600)
    return settings_path
