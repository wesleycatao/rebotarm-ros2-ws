"""Shared keyboard and assistive-technology helpers for the GUI.

Qt exposes most native controls to accessibility APIs automatically, but
custom layouts still need explicit label associations and meaningful names.
This module keeps those rules in one place so every tab gets the same baseline.
"""

from __future__ import annotations

import re
from collections.abc import Iterator

from PySide6.QtCore import Qt
from PySide6.QtGui import QAccessible, QAccessibleAnnouncementEvent
from PySide6.QtWidgets import (
    QAbstractButton,
    QAbstractItemView,
    QAbstractSlider,
    QAbstractSpinBox,
    QComboBox,
    QFormLayout,
    QGridLayout,
    QLabel,
    QLayout,
    QLineEdit,
    QTabBar,
    QTextEdit,
    QWidget,
)


_INTERACTIVE_TYPES = (
    QAbstractButton,
    QAbstractItemView,
    QAbstractSlider,
    QAbstractSpinBox,
    QComboBox,
    QLineEdit,
    QTabBar,
    QTextEdit,
)
_SYMBOL_ONLY = re.compile(r"^[\s×+−▲▼◀▶…._/-]+$")


def _plain(text: str) -> str:
    """Return UI text without mnemonic markers or trailing punctuation."""
    return " ".join(str(text).replace("&&", "&").replace("&", "").split()).rstrip(":")


def _layout_tree(layout: QLayout | None) -> Iterator[QLayout]:
    if layout is None:
        return
    yield layout
    for index in range(layout.count()):
        item = layout.itemAt(index)
        child_layout = item.layout()
        if child_layout is not None:
            yield from _layout_tree(child_layout)
        widget = item.widget()
        if widget is not None and widget.layout() is not None:
            yield from _layout_tree(widget.layout())


def _first_interactive_in_item(item) -> QWidget | None:
    widget = item.widget() if item is not None else None
    if isinstance(widget, _INTERACTIVE_TYPES):
        return widget
    if widget is not None:
        children = widget.findChildren(QWidget)
        return next((child for child in children if isinstance(child, _INTERACTIVE_TYPES)), None)
    layout = item.layout() if item is not None else None
    if layout is not None:
        for index in range(layout.count()):
            found = _first_interactive_in_item(layout.itemAt(index))
            if found is not None:
                return found
    return None


def _associate_form_labels(root: QWidget) -> None:
    """Set QLabel buddies and field names for every QFormLayout under ``root``."""
    layouts = list(root.findChildren(QFormLayout))
    if isinstance(root.layout(), QFormLayout):
        layouts.insert(0, root.layout())
    for layout in layouts:
        for row in range(layout.rowCount()):
            label_item = layout.itemAt(row, QFormLayout.LabelRole)
            field_item = layout.itemAt(row, QFormLayout.FieldRole)
            label = label_item.widget() if label_item is not None else None
            field = _first_interactive_in_item(field_item)
            if not isinstance(label, QLabel) or field is None:
                continue
            name = _plain(label.text())
            if not name:
                continue
            label.setBuddy(field)
            if not field.accessibleName():
                field.setAccessibleName(name)


def _associate_grid_labels(root: QWidget) -> None:
    """Associate explicit ``FieldLabel`` cells with the next control in a row."""
    layouts = list(root.findChildren(QGridLayout))
    if isinstance(root.layout(), QGridLayout):
        layouts.insert(0, root.layout())
    for layout in layouts:
        for index in range(layout.count()):
            item = layout.itemAt(index)
            label = item.widget()
            if not isinstance(label, QLabel) or label.objectName() != "FieldLabel":
                continue
            row, column, _row_span, _column_span = layout.getItemPosition(index)
            field = None
            for candidate_column in range(column + 1, layout.columnCount()):
                field = _first_interactive_in_item(
                    layout.itemAtPosition(row, candidate_column)
                )
                if field is not None:
                    break
            name = _plain(label.text())
            if field is not None and name:
                label.setBuddy(field)
                if not field.accessibleName():
                    field.setAccessibleName(name)


def effective_accessible_name(widget: QWidget) -> str:
    """Return the explicit or native text name exposed for a widget."""
    explicit = _plain(widget.accessibleName())
    if explicit:
        return explicit
    if isinstance(widget, QAbstractButton):
        text = _plain(widget.text())
        if text and not _SYMBOL_ONLY.match(text):
            return text
    if isinstance(widget, QLineEdit):
        placeholder = _plain(widget.placeholderText())
        if placeholder:
            return placeholder
    return ""


def configure_accessibility(root: QWidget) -> None:
    """Apply the common accessible-name, description and buddy baseline."""
    _associate_form_labels(root)
    _associate_grid_labels(root)
    widgets = [root, *root.findChildren(QWidget)]
    for widget in widgets:
        if not isinstance(widget, _INTERACTIVE_TYPES):
            continue
        tip = _plain(widget.toolTip())
        name = effective_accessible_name(widget)
        if not name and isinstance(widget, QAbstractButton) and tip:
            name = tip.split("(", 1)[0].strip().rstrip(".")
        if not name and tip:
            name = tip.split("\n", 1)[0].rstrip(".")
        if not name and widget.objectName():
            name = _plain(widget.objectName().replace("_", " "))
        if name and not widget.accessibleName():
            widget.setAccessibleName(name)
        if tip and not widget.accessibleDescription():
            widget.setAccessibleDescription(tip)


def first_focusable(root: QWidget) -> QWidget | None:
    """Find the first enabled, visible control in Qt's normal tab focus set."""
    candidates = [root, *root.findChildren(QWidget)]
    for widget in candidates:
        if not isinstance(widget, _INTERACTIVE_TYPES):
            continue
        if not widget.isEnabled() or not widget.isVisibleTo(root):
            continue
        if widget.focusPolicy() & Qt.TabFocus:
            return widget
    return None


def focus_first(root: QWidget) -> QWidget | None:
    """Move keyboard focus to the first operable control under ``root``."""
    target = first_focusable(root)
    if target is not None:
        target.setFocus(Qt.ShortcutFocusReason)
    return target


def announce(widget: QWidget, message: str, *, assertive: bool = False) -> None:
    """Send an explicit live announcement to the platform accessibility API."""
    text = _plain(message)
    if not text:
        return
    widget.setAccessibleDescription(text)
    event = QAccessibleAnnouncementEvent(widget, text)
    event.setPoliteness(
        QAccessible.AnnouncementPoliteness.Assertive
        if assertive
        else QAccessible.AnnouncementPoliteness.Polite
    )
    QAccessible.updateAccessibility(event)
