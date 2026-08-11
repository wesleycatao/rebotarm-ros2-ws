"""Single source of truth for semantic theme colours.

Historically every status/severity colour lived as an inline
``widget.setStyleSheet("color: #3fb950")`` call. Those literals were tuned for
the dark theme only, so switching to the light theme (factory floor) silently
broke contrast and the accessibility audit. This module defines each semantic
colour exactly once per theme; both the generated stylesheet fragment
(:mod:`rebotarm_gui.styles.theme`) and the runtime ``apply_status`` helper
resolve through here, so a colour can never drift between the QSS and the code.
"""

from __future__ import annotations

VALID_THEMES = ("dark", "light")

# Semantic status levels used by ``apply_status`` / the ``[status="..."]`` QSS.
STATUS_OK = "ok"
STATUS_WARN = "warn"
STATUS_FAIL = "fail"
STATUS_INFO = "info"
STATUS_LEVELS = (STATUS_OK, STATUS_WARN, STATUS_FAIL, STATUS_INFO)

# Alarm-severity levels (kept distinct from status levels so the alarm centre
# keeps self-describing names; they map 1:1 onto the header indicator QSS).
SEVERITY_ERROR = "error"
SEVERITY_WARNING = "warning"
SEVERITY_INFO = "info"
SEVERITY_LEVELS = (SEVERITY_ERROR, SEVERITY_WARNING, SEVERITY_INFO)

# Application background per theme. Used by the contrast regression test to
# guarantee every status colour stays WCAG-AA on the theme it belongs to.
THEME_BACKGROUND = {
    "dark": "#121417",
    "light": "#eef1f5",
}

# Status foregrounds. Dark values reproduce the prior GitHub-dark inline palette
# so the default theme does not change visually; light values are darkened from
# GitHub's palette so each stays WCAG-AA (>= 4.5:1) on the #eef1f5 background
# (GitHub tuned theirs for pure white, which is a hair brighter than our panel).
_STATUS_COLORS = {
    "dark": {
        STATUS_OK: "#3fb950",
        STATUS_WARN: "#f0883e",
        STATUS_FAIL: "#f85149",
        STATUS_INFO: "#58a6ff",
    },
    "light": {
        STATUS_OK: "#157033",
        STATUS_WARN: "#7a5200",
        STATUS_FAIL: "#c1121f",
        STATUS_INFO: "#0a58c4",
    },
}

# Header alarm-indicator icon foregrounds. Dark values match the previous
# ``_SEVERITY_META`` colours exactly; light values are AA-compliant.
_SEVERITY_COLORS = {
    "dark": {
        SEVERITY_ERROR: "#ff6b6b",
        SEVERITY_WARNING: "#f0a500",
        SEVERITY_INFO: "#7aa2ff",
    },
    "light": {
        SEVERITY_ERROR: "#c1121f",
        SEVERITY_WARNING: "#7a5200",
        SEVERITY_INFO: "#0a58c4",
    },
}


def _normalise_theme(theme: str) -> str:
    return theme if theme in VALID_THEMES else "dark"


def status_color(theme: str, level: str) -> str:
    """Return the status foreground colour for ``level`` under ``theme``."""
    table = _STATUS_COLORS[_normalise_theme(theme)]
    return table.get(level, table[STATUS_INFO])


def severity_color(theme: str, severity: str) -> str:
    """Return the alarm-severity foreground colour for ``severity``."""
    table = _SEVERITY_COLORS[_normalise_theme(theme)]
    return table.get(severity, table[SEVERITY_INFO])


def status_palette(theme: str) -> dict[str, str]:
    """Return a copy of the full ``level -> colour`` map for ``theme``."""
    return dict(_STATUS_COLORS[_normalise_theme(theme)])
