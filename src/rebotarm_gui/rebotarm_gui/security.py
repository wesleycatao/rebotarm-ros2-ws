"""Local authentication and capability-based command authorization.

The GUI policy is intentionally independent from page visibility.  A hidden
button is a usability feature; :class:`AuthorizationContext` is the command
gate used immediately before ROS requests are dispatched.
"""

from __future__ import annotations

import base64
import getpass
import hashlib
import hmac
import json
import os
import secrets
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


ROLE_OPERATOR = "operador"
ROLE_SUPERVISOR = "supervisor"
ROLE_ENGINEERING = "engenharia"
ROLES = (ROLE_OPERATOR, ROLE_SUPERVISOR, ROLE_ENGINEERING)

CAP_EXECUTE_RELEASED = "execute_released"
CAP_MANUAL_MOTION = "manual_motion"
CAP_EDIT_RECIPE = "edit_recipe"
CAP_RELEASE_RECIPE = "release_recipe"
CAP_PRODUCTION_MODE = "production_mode"
CAP_SAFETY_RESET = "safety_reset"
CAP_SAFETY_CONFIG = "safety_config"
CAP_CALIBRATION_APPROVE = "calibration_approve"
CAP_IO_WRITE = "io_write"
CAP_RAW_CONTROL = "raw_control"
CAP_SPEED_OVERRIDE = "speed_override"
CAP_STOP = "stop"

ROLE_CAPABILITIES = {
    ROLE_OPERATOR: {
        CAP_EXECUTE_RELEASED,
        CAP_MANUAL_MOTION,
        CAP_SPEED_OVERRIDE,
        CAP_STOP,
    },
    ROLE_SUPERVISOR: {
        CAP_EXECUTE_RELEASED,
        CAP_MANUAL_MOTION,
        CAP_EDIT_RECIPE,
        CAP_RELEASE_RECIPE,
        CAP_PRODUCTION_MODE,
        CAP_SAFETY_RESET,
        CAP_CALIBRATION_APPROVE,
        CAP_IO_WRITE,
        CAP_SPEED_OVERRIDE,
        CAP_STOP,
    },
    ROLE_ENGINEERING: {
        CAP_EXECUTE_RELEASED,
        CAP_MANUAL_MOTION,
        CAP_EDIT_RECIPE,
        CAP_RELEASE_RECIPE,
        CAP_PRODUCTION_MODE,
        CAP_SAFETY_RESET,
        CAP_SAFETY_CONFIG,
        CAP_CALIBRATION_APPROVE,
        CAP_IO_WRITE,
        CAP_RAW_CONTROL,
        CAP_SPEED_OVERRIDE,
        CAP_STOP,
    },
}

PBKDF2_ITERATIONS = 310_000


def default_auth_path() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME")
    root = Path(base).expanduser() if base else Path.home() / ".config"
    return root / "rebotarm_gui" / "auth.json"


def default_authorization_secret_path() -> Path:
    configured = os.environ.get("REBOTARM_AUTH_SECRET_PATH", "").strip()
    if configured:
        return Path(configured).expanduser()
    return default_auth_path().with_name("authorization.key")


def ensure_authorization_secret(path: str | Path | None = None) -> Path:
    target = Path(path).expanduser() if path else default_authorization_secret_path()
    if target.is_file():
        return target
    target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    target.write_bytes(secrets.token_bytes(32))
    os.chmod(target, 0o600)
    return target


def signed_lease(
    identity: Identity, ttl_s: float = 90.0, *, production_mode: bool = False
) -> str:
    """Return a canonical HMAC-signed controller authorization lease."""
    secret_path = ensure_authorization_secret()
    secret = secret_path.read_bytes()
    now = time.time()
    capabilities = set(ROLE_CAPABILITIES.get(identity.role, {CAP_STOP}))
    if production_mode:
        capabilities.intersection_update(
            {
                CAP_EXECUTE_RELEASED,
                CAP_SAFETY_RESET,
                CAP_SPEED_OVERRIDE,
                CAP_STOP,
            }
        )
    payload = {
        "version": 1,
        "user": identity.user,
        "role": identity.role,
        "capabilities": sorted(capabilities),
        "issued_at": now,
        "expires_at": now + max(10.0, min(float(ttl_s), 300.0)),
        "nonce": secrets.token_hex(16),
    }
    canonical = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    payload["signature"] = hmac.new(secret, canonical, hashlib.sha256).hexdigest()
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


@dataclass(frozen=True)
class Identity:
    user: str
    role: str
    authenticated: bool


class CredentialStore:
    """Small PBKDF2 credential database with owner-only file permissions."""

    def __init__(self, path: str | Path | None = None) -> None:
        self.path = Path(path).expanduser() if path else default_auth_path()

    def _load(self) -> dict:
        if not self.path.is_file():
            return {"version": 1, "users": {}}
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {"version": 1, "users": {}}
        return data if isinstance(data, dict) else {"version": 1, "users": {}}

    def has_users(self) -> bool:
        users = self._load().get("users", {})
        return isinstance(users, dict) and bool(users)

    def set_user(self, user: str, role: str, pin: str) -> None:
        name = str(user).strip()
        if not name:
            raise ValueError("usuário obrigatório")
        if role not in ROLES:
            raise ValueError(f"perfil inválido: {role}")
        secret = str(pin)
        if len(secret) < 6:
            raise ValueError("o PIN deve ter ao menos 6 caracteres")
        salt = secrets.token_bytes(16)
        digest = hashlib.pbkdf2_hmac(
            "sha256", secret.encode("utf-8"), salt, PBKDF2_ITERATIONS
        )
        data = self._load()
        users = data.setdefault("users", {})
        users[name] = {
            "role": role,
            "salt": base64.b64encode(salt).decode("ascii"),
            "digest": base64.b64encode(digest).decode("ascii"),
            "iterations": PBKDF2_ITERATIONS,
        }
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            stream.write(json.dumps(data, indent=2, sort_keys=True))
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o600)
        os.replace(temporary, self.path)
        ensure_authorization_secret()

    def authenticate(self, user: str, pin: str) -> Identity:
        name = str(user).strip()
        record = self._load().get("users", {}).get(name, {})
        if not isinstance(record, dict):
            return Identity(name, ROLE_OPERATOR, False)
        try:
            salt = base64.b64decode(record["salt"], validate=True)
            expected = base64.b64decode(record["digest"], validate=True)
            iterations = int(record.get("iterations", PBKDF2_ITERATIONS))
            actual = hashlib.pbkdf2_hmac(
                "sha256", str(pin).encode("utf-8"), salt, iterations
            )
        except (KeyError, TypeError, ValueError):
            return Identity(name, ROLE_OPERATOR, False)
        role = str(record.get("role", ROLE_OPERATOR))
        valid = hmac.compare_digest(expected, actual) and role in ROLES
        return Identity(name, role if valid else ROLE_OPERATOR, valid)


class AuthorizationContext:
    """Current authenticated identity and fail-closed production policy."""

    def __init__(self, *, enforce_rbac: bool = False) -> None:
        # The OS login identifies the workstation user for commissioning.
        # Privileged production transitions still require CredentialStore auth.
        self.identity = Identity(getpass.getuser(), ROLE_OPERATOR, True)
        self.production_mode = False
        self.enforce_rbac = bool(enforce_rbac)

    def set_identity(self, identity: Identity) -> None:
        self.identity = identity

    def set_production_mode(self, enabled: bool) -> None:
        self.production_mode = bool(enabled)

    def set_enforcement(self, enabled: bool) -> None:
        self.enforce_rbac = bool(enabled)

    def capabilities(self) -> set[str]:
        if not self.identity.authenticated:
            return {CAP_STOP}
        return set(ROLE_CAPABILITIES.get(self.identity.role, {CAP_STOP}))

    def permits(self, capability: str) -> bool:
        # A station with no credential database is in explicit commissioning
        # bootstrap. Once the first account exists, command permissions remain
        # enforced both inside and outside Production mode.
        if not self.production_mode and not self.enforce_rbac:
            return True
        if self.production_mode and capability not in {
            CAP_EXECUTE_RELEASED,
            CAP_SAFETY_RESET,
            CAP_SPEED_OVERRIDE,
            CAP_STOP,
        }:
            return False
        return capability in self.capabilities()

    def require(self, capability: str) -> tuple[bool, str]:
        if self.permits(capability):
            return True, ""
        who = self.identity.user or "não autenticado"
        return (
            False,
            f"comando bloqueado: {who}/{self.identity.role} não possui "
            f"a permissão {capability} no Modo Produção",
        )

    def can_any(self, capabilities: Iterable[str]) -> bool:
        return any(self.permits(capability) for capability in capabilities)
