"""HMAC-verified, short-lived capability leases for controller commands."""

from __future__ import annotations

import hashlib
import hmac
import json
import os
from pathlib import Path
import threading
import time


CAP_EXECUTE_RELEASED = "execute_released"
CAP_MANUAL_MOTION = "manual_motion"
CAP_SAFETY_RESET = "safety_reset"
CAP_SAFETY_CONFIG = "safety_config"
CAP_IO_WRITE = "io_write"
CAP_RAW_CONTROL = "raw_control"
CAP_SPEED_OVERRIDE = "speed_override"
CAP_STOP = "stop"


def default_secret_path() -> Path:
    configured = os.environ.get("REBOTARM_AUTH_SECRET_PATH", "").strip()
    if configured:
        return Path(configured).expanduser()
    return Path.home() / ".config" / "rebotarm_gui" / "authorization.key"


class AuthorizationGuard:
    """Validate signed leases and reject expired/replayed permissions."""

    def __init__(
        self,
        *,
        required: bool,
        secret_path: str | Path | None = None,
        max_clock_skew_s: float = 5.0,
    ) -> None:
        self.required = bool(required)
        self.secret_path = (
            Path(secret_path).expanduser() if secret_path else default_secret_path()
        )
        self.max_clock_skew_s = max(0.0, float(max_clock_skew_s))
        self._lock = threading.RLock()
        self._capabilities: set[str] = set()
        self._expires_at = 0.0
        self._identity = ""
        self._nonces: dict[str, float] = {}

    def update(self, raw: str) -> tuple[bool, str]:
        try:
            payload = json.loads(str(raw))
            signature = str(payload.pop("signature"))
            canonical = json.dumps(
                payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True
            ).encode("utf-8")
            secret = self.secret_path.read_bytes()
            expected = hmac.new(secret, canonical, hashlib.sha256).hexdigest()
            now = time.time()
            issued = float(payload["issued_at"])
            expires = float(payload["expires_at"])
            nonce = str(payload["nonce"])
            capabilities = {str(item) for item in payload.get("capabilities", [])}
            if not hmac.compare_digest(signature, expected):
                raise ValueError("invalid signature")
            if issued > now + self.max_clock_skew_s:
                raise ValueError("lease issued in the future")
            if expires <= now or expires - issued > 300.0 + self.max_clock_skew_s:
                raise ValueError("expired or overlong lease")
            with self._lock:
                self._nonces = {
                    key: value for key, value in self._nonces.items() if value > now
                }
                if nonce in self._nonces:
                    raise ValueError("replayed lease")
                self._nonces[nonce] = expires
                self._capabilities = capabilities
                self._expires_at = expires
                self._identity = f"{payload.get('user', '')}/{payload.get('role', '')}"
                # Receiving the first valid lease is the atomic transition into
                # fail-closed authorization mode. It cannot silently fall back
                # to permissive operation when that lease later expires.
                self.required = True
            return True, self._identity
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            return False, str(exc)

    def permits(self, capability: str) -> bool:
        if capability == CAP_STOP:
            return True
        if not self.required:
            return True
        with self._lock:
            return time.time() < self._expires_at and capability in self._capabilities

    def require(self, capability: str) -> None:
        if not self.permits(capability):
            raise PermissionError(
                f"authorization denied for {capability}: missing or expired signed lease"
            )
