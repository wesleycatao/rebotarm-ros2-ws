"""Reusable combo-box widget with popup-always-closes workaround.

Some PySide6/Qt configurations leave the QComboBox dropdown open after the user
selects an item.  :class:`RetractingComboBox` forces ``hidePopup`` on the
activation signals so the widget behaves like a normal combo box.
"""

from __future__ import annotations

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QComboBox, QWidget


class RetractingComboBox(QComboBox):
    """QComboBox that hides its popup immediately after user selection.

    This works around a PySide6/Qt behaviour observed in this project where the
    dropdown list stays open after an item is clicked, only closing when the
    user clicks somewhere else.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.activated.connect(self._retract)
        # Some styles/views do not reliably emit ``activated`` on a mouse press,
        # so also hook the view's pressed signal as a fallback.
        self.view().pressed.connect(self._retract)

    def _retract(self, *_args) -> None:
        """Close the popup and clear focus after the current event finishes."""
        QTimer.singleShot(0, self.hidePopup)
        QTimer.singleShot(0, self.clearFocus)
