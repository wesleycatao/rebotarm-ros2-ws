"""Transient, non-blocking confirmation toasts.

Confirmations such as "Programa salvo" or "Perfil aplicado" used to land only
in the hidden event-log dock, so mouse/touchpad operators got no visible
feedback. A toast is a small overlay anchored to the bottom-right of the main
window that auto-dismisses after a few seconds, closing the feedback loop
without a modal dialog.
"""

from __future__ import annotations

from PySide6.QtCore import QObject, QPoint, QRect, QTimer, Qt
from PySide6.QtWidgets import QFrame, QLabel, QVBoxLayout


TOAST_DURATION_MS = 2600   # visible long enough to read, short enough to be polite
TOAST_SPACING = 8          # vertical gap between stacked toasts
TOAST_MARGIN = 18          # inset from the window's bottom-right corner
TOAST_MAX_VISIBLE = 4      # avoid an endless column during a burst of events


class Toast(QFrame):
    """A single transient message overlay."""

    def __init__(self, text: str, kind: str, parent) -> None:
        super().__init__(parent)
        self.setObjectName("Toast")
        self.setProperty("kind", kind)
        self.setAttribute(Qt.WA_DeleteOnClose, True)
        self.setFocusPolicy(Qt.NoFocus)
        self._kind = kind

        self._label = QLabel(text, self)
        self._label.setObjectName("ToastText")
        self._label.setWordWrap(True)
        self._label.setTextInteractionFlags(Qt.NoTextInteraction)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 9, 14, 9)
        lay.addWidget(self._label)

        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(TOAST_DURATION_MS)
        self._timer.timeout.connect(self.close)

    def start(self) -> None:
        self._timer.start()

    def kind(self) -> str:
        return self._kind

    def text(self) -> str:
        return self._label.text()

    def size_hint(self) -> QRect:
        return QRect(0, 0, self.sizeHint().width(), self.sizeHint().height())


class ToastManager(QObject):
    """Owns the stack of active toasts anchored to the bottom-right corner.

    ``parent`` is the window over which toasts float; it is repolished only
    when its stylesheet already defines ``#Toast`` so absence never raises.
    """

    def __init__(self, parent) -> None:
        super().__init__(parent)
        self._parent = parent
        self._active: list[Toast] = []

    def show(self, text: str, *, kind: str = "info") -> None:
        """Display ``text`` as a transient toast, stacking above prior ones."""
        # Coalesce an identical toast already on screen to avoid flicker spam
        # (e.g. rapidly repeated "Perfil aplicado").
        for toast in self._active:
            if toast.kind() == kind and toast.text() == text:
                toast.start()
                return
        # Drop the oldest beyond the cap so the column never grows unbounded.
        while len(self._active) >= TOAST_MAX_VISIBLE:
            self._active[0].close()
        toast = Toast(text, kind, self._parent)
        toast.show()
        toast.start()
        toast.destroyed.connect(self._on_toast_destroyed)
        self._active.append(toast)
        self._relayout()

    def _on_toast_destroyed(self, *_args) -> None:
        self._active = [t for t in self._active if sip_isvalid(t)]
        self._relayout()

    def _relayout(self) -> None:
        """Reposition visible toasts bottom-up against the window corner."""
        if self._parent is None or not sip_isvalid(self._parent):
            return
        try:
            rect = self._parent.rect()
        except RuntimeError:
            return
        anchor_x = rect.right() - TOAST_MARGIN
        anchor_y = rect.bottom() - TOAST_MARGIN
        for toast in reversed(self._active):
            if not sip_isvalid(toast):
                continue
            hint = toast.sizeHint()
            width = hint.width()
            height = hint.height()
            top_left = QPoint(anchor_x - width, anchor_y - height)
            toast.move(top_left)
            toast.resize(width, height)
            toast.raise_()
            anchor_y -= height + TOAST_SPACING


def sip_isvalid(obj) -> bool:
    """Return False once the underlying C++ object has been destroyed."""
    try:
        obj.isVisible()  # any method touch; raises RuntimeError if deleted
        return True
    except RuntimeError:
        return False
