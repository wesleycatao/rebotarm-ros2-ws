"""Lightweight internationalisation foundation.

This module lays the groundwork for translating the GUI without forcing a
risky rewrite today. Every user-visible string still flows through
:func:`tr`, which currently echoes the provided default Portuguese text, so
existing tests that assert on exact strings keep passing.

Future work can load overrides from a JSON file (``locale/<lang>.json``) via
:func:`load_translations`; the ``tr`` calls will then return the translated
value with no further code changes at call sites.

Usage::

    from .i18n import tr
    label.setText(tr("nav.home", "Início"))
"""

from __future__ import annotations

import json
from pathlib import Path
from threading import RLock
from typing import Mapping

_LOCK = RLock()
_TABLE: dict[str, str] = {}
_FALLBACKS: dict[str, str] = {}


def load_translations(path: str | Path) -> int:
    """Load a ``{key: text}`` JSON file. Returns the number of keys loaded."""
    p = Path(path)
    if not p.is_file():
        return 0
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except Exception:
        return 0
    if not isinstance(data, Mapping):
        return 0
    with _LOCK:
        _TABLE.update({str(k): str(v) for k, v in data.items()})
    return len(data)


def reset() -> None:
    """Clear any loaded translations (useful for tests)."""
    with _LOCK:
        _TABLE.clear()


def available() -> int:
    with _LOCK:
        return len(_TABLE)


def tr(key: str, default: str) -> str:
    """Return the translated text for ``key`` or ``default`` when absent.

    The default is recorded so translators can discover strings that have not
    been catalogued yet.
    """
    with _LOCK:
        if key in _TABLE:
            return _TABLE[key]
        _FALLBACKS.setdefault(key, default)
        return default


def missing_keys() -> dict[str, str]:
    """Return keys seen via :func:`tr` but not present in the loaded table."""
    with _LOCK:
        return dict(_FALLBACKS)
