"""Button factory and shared widget construction helpers.

Centralises the repetitive pattern of building a ``QPushButton`` with an
object name, minimum height, tooltip and content-fitted width. Using the
factory keeps the visual style consistent across tabs and removes dozens of
boilerplate lines from the tab modules.
"""

from __future__ import annotations

from typing import Iterable

from PySide6.QtWidgets import QPushButton

from .layout_utils import fit_button_text


def make_button(
    text: str,
    *,
    primary: bool = False,
    warning: bool = False,
    danger: bool = False,
    estop: bool = False,
    draw: bool = False,
    checkable: bool = False,
    checked: bool = False,
    checked_draw: bool = False,
    min_height: int | None = None,
    min_width: int | None = None,
    tip: str | None = None,
    fit: bool = True,
    enabled: bool = True,
) -> QPushButton:
    """Build a styled :class:`QPushButton` ready to drop into a layout.

    Exactly one of the style flags (``primary``/``warning``/``danger``/
    ``estop``/``draw``) should be set. When none is given the default neutral
    style is used.
    """
    button = QPushButton(text)
    flags_set = sum(bool(f) for f in (primary, warning, danger, estop, draw))
    if flags_set > 1:
        raise ValueError("make_button: informe no máximo um estilo (primary/warning/danger/estop/draw).")
    if primary:
        button.setObjectName("PrimaryButton")
    elif warning:
        button.setObjectName("WarningButton")
    elif danger:
        button.setObjectName("DangerButton")
    elif estop:
        button.setObjectName("EStopButton")
    elif draw:
        button.setObjectName("DrawModeButton")
    if checkable:
        button.setCheckable(True)
        if checked:
            button.setChecked(True)
    if checked_draw:
        button.setObjectName("DrawModeButton")
    if min_height is not None:
        button.setMinimumHeight(int(min_height))
    if min_width is not None:
        button.setMinimumWidth(int(min_width))
    if tip is not None:
        button.setToolTip(tip)
        button.setAccessibleDescription(tip)
    if text:
        button.setAccessibleName(text.replace("&", ""))
    button.setEnabled(bool(enabled))
    if fit and text:
        fit_button_text(button)
    return button


def fit_buttons(buttons: Iterable[QPushButton]) -> None:
    """Apply :func:`fit_button_text` to every button in ``buttons``."""
    for button in buttons:
        fit_button_text(button)
