"""Lightweight SVG icon loader for the reBotArm GUI.

Icons are optional assets: every call site degrades gracefully to a text-only
control when a file or the SVG rendering plugin is unavailable, so the GUI
keeps working on minimal/offscreen installs (including the test suite).
"""

from __future__ import annotations

from pathlib import Path

from PySide6.QtGui import QIcon


def _candidates(name: str) -> list[Path]:
    here = Path(__file__).resolve().parent
    found = [here / "icons" / f"{name}.svg"]
    try:
        from ament_index_python.packages import get_package_share_directory

        found.append(
            Path(get_package_share_directory("rebotarm_gui")) / "icons" / f"{name}.svg"
        )
    except Exception:
        pass
    return found


def load_icon(name: str) -> QIcon:
    """Return the SVG icon ``name`` or an empty :class:`QIcon` when unavailable.

    An empty icon is intentional: callers should check ``icon.isNull()`` (or
    simply call ``setIcon`` regardless) so the absence of an icon never breaks
    layout or tests.
    """
    if not name:
        return QIcon()
    for candidate in _candidates(name):
        if candidate.is_file():
            icon = QIcon(str(candidate))
            if not icon.isNull():
                return icon
    return QIcon()


def has_icon(name: str) -> bool:
    """True when an SVG asset for ``name`` exists on disk."""
    return any(candidate.is_file() for candidate in _candidates(name))
