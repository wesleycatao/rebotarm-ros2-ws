"""Persistent GUI settings."""

from __future__ import annotations

import getpass
import json
import os
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class OpenAISettings:
    api_key: str = ""
    save_api_key: bool = False
    model: str = "gpt-5-mini"
    base_url: str = "https://api.openai.com/v1"
    timeout_s: int = 45

    def effective_api_key(self) -> str:
        return self.api_key.strip() or os.environ.get("OPENAI_API_KEY", "").strip()


@dataclass
class AppSettings:
    openai: OpenAISettings = field(default_factory=OpenAISettings)
    # User profile gating advanced pages: "operador" | "supervisor" | "engenharia"
    profile: str = "supervisor"
    # Human-readable workstation/operator identifier stored with audit events.
    operator_id: str = field(default_factory=getpass.getuser)


VALID_PROFILES = ("operador", "supervisor", "engenharia")


def default_settings_path() -> Path:
    return Path.home() / ".config" / "rebotarm_gui" / "settings.json"


def load_app_settings(path: Path | None = None) -> AppSettings:
    settings_path = path or default_settings_path()
    if not settings_path.is_file():
        return AppSettings()
    # Pode conter API key: corrige silenciosamente permissões abertas demais.
    try:
        if settings_path.stat().st_mode & 0o077:
            os.chmod(settings_path, 0o600)
    except OSError:
        pass
    try:
        data = json.loads(settings_path.read_text(encoding="utf-8"))
    except Exception:
        return AppSettings()
    openai_data = data.get("openai", {})
    openai = OpenAISettings()
    if isinstance(openai_data, dict):
        openai.save_api_key = bool(openai_data.get("save_api_key", False))
        if openai.save_api_key:
            openai.api_key = str(openai_data.get("api_key", ""))
        openai.model = str(openai_data.get("model", openai.model))
        openai.base_url = str(openai_data.get("base_url", openai.base_url))
        try:
            openai.timeout_s = int(openai_data.get("timeout_s", openai.timeout_s))
        except Exception:
            openai.timeout_s = 45
    profile = str(data.get("profile", "supervisor"))
    if profile not in VALID_PROFILES:
        profile = "supervisor"
    operator_id = str(data.get("operator_id", "")).strip() or getpass.getuser()
    return AppSettings(openai=openai, profile=profile, operator_id=operator_id)


def save_app_settings(settings: AppSettings, path: Path | None = None) -> Path:
    settings_path = path or default_settings_path()
    settings_path.parent.mkdir(parents=True, exist_ok=True)
    openai = settings.openai
    defaults = OpenAISettings()
    data = {
        "openai": {
            "save_api_key": bool(openai.save_api_key),
            "api_key": openai.api_key.strip() if openai.save_api_key else "",
            "model": openai.model.strip() or defaults.model,
            "base_url": openai.base_url.strip() or defaults.base_url,
            "timeout_s": int(openai.timeout_s),
        },
        "profile": settings.profile if settings.profile in VALID_PROFILES else "supervisor",
        "operator_id": settings.operator_id.strip(),
    }
    settings_path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")
    # O arquivo pode conter a API key em texto plano: acesso restrito ao dono.
    os.chmod(settings_path, 0o600)
    return settings_path
