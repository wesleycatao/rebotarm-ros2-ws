"""Theme composition: tuned base QSS + a generated semantic status fragment.

The base widget styles (``dark.qss`` / ``light.qss``) remain the carefully
hand-tuned, per-theme foundation. The semantic *status* and *severity* colours
that used to be inline ``setStyleSheet`` literals are generated here from
:mod:`theme_tokens`, so they are defined once and automatically follow the
active theme. Adding a new status level therefore touches a single module
instead of every call site and both QSS files.
"""

from __future__ import annotations

from pathlib import Path
import re

from .theme_tokens import (
    SEVERITY_LEVELS,
    STATUS_LEVELS,
    STATUS_WARN,
    VALID_THEMES,
    severity_color,
    status_color,
)

_BASE_DIR = Path(__file__).resolve().parent
_FONT_SIZE_RE = re.compile(
    r"(?P<prefix>font-size\s*:\s*)(?P<size>\d+(?:\.\d+)?)pt(?P<suffix>\s*;)",
    re.IGNORECASE,
)


def _candidate_base_paths(theme: str) -> list[Path]:
    name = "light" if theme == "light" else "dark"
    candidates = [_BASE_DIR / f"{name}.qss"]
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("rebotarm_gui")) / "styles" / f"{name}.qss"
        )
    except ImportError:
        pass
    return candidates


def load_base_stylesheet(theme: str = "dark") -> str:
    """Load the tuned base QSS for ``theme`` (empty string if unavailable)."""
    for qss in _candidate_base_paths(theme):
        if qss.is_file():
            return qss.read_text(encoding="utf-8")
    return ""


def semantic_fragment(theme: str = "dark") -> str:
    """Generate the theme-aware ``[status]`` / ``[severity]`` QSS block.

    These rules did not exist in the hand-written QSS; emitting them from the
    token module keeps the semantic palette in one place and guarantees both
    themes define the same selectors with contrast-safe colours.
    """
    theme = theme if theme in VALID_THEMES else "dark"
    lines: list[str] = [
        "/* === generated semantic status tokens (single source: theme_tokens) === */"
    ]
    for level in STATUS_LEVELS:
        colour = status_color(theme, level)
        lines.append(
            f'QLabel[status="{level}"], QFrame[status="{level}"] '
            f"{{ color: {colour}; }}"
        )
    # The header alarm-indicator icon foreground follows the active severity.
    for severity in SEVERITY_LEVELS:
        colour = severity_color(theme, severity)
        lines.append(
            f'QLabel#HeaderAlarmIndicatorIcon[severity="{severity}"] '
            f"{{ color: {colour}; }}"
        )
    # objectName-based warning label (program-editor IF-empty hint) kept for
    # any widget that still sets the objectName instead of the property.
    lines.append(
        f"QLabel#WarningLabel {{ color: {status_color(theme, STATUS_WARN)}; }}"
    )
    return "\n".join(lines) + "\n"


def scale_font_sizes(stylesheet: str, percent: int = 100) -> str:
    """Scale every QSS point size while preserving the theme's hierarchy."""
    scale = max(90, min(130, int(percent))) / 100.0
    if scale == 1.0:
        return stylesheet

    def replace(match: re.Match[str]) -> str:
        scaled = float(match.group("size")) * scale
        value = f"{scaled:.2f}".rstrip("0").rstrip(".")
        return f"{match.group('prefix')}{value}pt{match.group('suffix')}"

    return _FONT_SIZE_RE.sub(replace, stylesheet)


def build_stylesheet(theme: str = "dark", font_scale_percent: int = 100) -> str:
    """Return the composed theme with all point sizes scaled together."""
    theme = theme if theme in VALID_THEMES else "dark"
    base = load_base_stylesheet(theme)
    fragment = semantic_fragment(theme)
    if not base:
        return scale_font_sizes(fragment, font_scale_percent)
    composed = base.rstrip() + "\n\n" + fragment
    return scale_font_sizes(composed, font_scale_percent)
