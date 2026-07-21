"""Layout helpers to keep buttons and input widgets sized to their content."""

from __future__ import annotations

from PySide6.QtWidgets import (
    QAbstractSpinBox,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QWidget,
)


def fit_button_text(button: QPushButton, extra: int = 24) -> None:
    """Set the button minimum width so its text is never clipped.

    ``extra`` adds a small safety margin for borders/padding applied by the
    application stylesheet.
    """
    if not button.text():
        return
    hint = button.sizeHint()
    min_w = max(button.minimumWidth(), hint.width() + extra)
    button.setMinimumWidth(min_w)


def shrink_widget(
    widget: QWidget,
    max_width: int | None = None,
    horizontal: QSizePolicy.Policy = QSizePolicy.Policy.Preferred,
) -> None:
    """Make a widget keep a compact width when its content is short.

    Useful for QLineEdit/QSpinBox fields that otherwise stretch and leave a
    lot of empty space inside the box.
    """
    widget.setSizePolicy(horizontal, QSizePolicy.Policy.Fixed)
    if max_width is not None:
        widget.setMaximumWidth(max_width)


def shrink_line_edit(
    edit: QLineEdit,
    max_width: int | None = None,
    horizontal: QSizePolicy.Policy = QSizePolicy.Policy.Preferred,
) -> None:
    """Compact a QLineEdit so it does not waste horizontal space."""
    shrink_widget(edit, max_width=max_width, horizontal=horizontal)


def shrink_spinbox(
    spin: QAbstractSpinBox,
    max_width: int | None = None,
    horizontal: QSizePolicy.Policy = QSizePolicy.Policy.Preferred,
) -> None:
    """Compact a spin box so it does not waste horizontal space."""
    shrink_widget(spin, max_width=max_width, horizontal=horizontal)
