"""Runtime helper for semantic status colours.

Call sites used to write ``widget.setStyleSheet("color: #3fb950")`` with a
dark-theme literal. That bypassed the stylesheet and broke contrast under the
light theme. ``apply_status`` instead sets a ``status`` dynamic property and
re-polishes the widget, so the colour is resolved by the theme-aware
``[status="..."]`` rules generated in :mod:`theme_tokens` / :mod:`theme`.

Usage::

    apply_status(label, STATUS_OK)      # green/ok variant for the active theme
    apply_status(label, STATUS_FAIL)    # red/fail variant
    clear_status(label)                 # back to default text colour
"""

from __future__ import annotations

from PySide6.QtWidgets import QWidget

from .theme_tokens import STATUS_FAIL, STATUS_INFO, STATUS_OK, STATUS_WARN

__all__ = [
    "STATUS_OK",
    "STATUS_WARN",
    "STATUS_FAIL",
    "STATUS_INFO",
    "apply_status",
    "clear_status",
]


def apply_status(widget: QWidget, level: str | None) -> None:
    """Apply a semantic ``status`` property and re-polish ``widget``.

    ``level`` must be one of the ``STATUS_*`` constants or ``None`` / ``""`` to
    clear. Re-polishing is required because Qt only re-evaluates property
    selectors when the style is invalidated.
    """
    widget.setProperty("status", level if level else "")
    widget.style().unpolish(widget)
    widget.style().polish(widget)


def clear_status(widget: QWidget) -> None:
    """Reset ``widget`` to its default (property-less) text colour."""
    apply_status(widget, None)
