"""Keyboard jog (opt-in, tab-scoped) routing tests."""

from __future__ import annotations

import os
import sys
import types
import unittest
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QEvent, QObject, Qt, Signal  # noqa: E402
from PySide6.QtGui import QKeyEvent  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from rebotarm_gui.tabs.joints_tab import JointsTab  # noqa: E402
from rebotarm_gui.tabs.pose_tab import PoseTab  # noqa: E402


class _FakeBridge(QObject):
    action_result = Signal(str, bool, str)
    move_feedback = Signal(float, object)

    def __init__(self) -> None:
        super().__init__()
        self.node = None
        self.jog_calls: list = []

    def joint_limits(self):
        return {}

    def tcp_pose_fk(self):
        return None

    def current_tcp_pose(self):
        return None

    def follow_joint_trajectory(self, names, positions, duration):
        self.jog_calls.append(("fjt", list(names), list(positions), duration))
        return False


def _press(widget, key, *, release: bool = False) -> None:
    ev = QKeyEvent(
        QEvent.KeyRelease if release else QEvent.KeyPress,
        key,
        Qt.NoModifier,
    )
    widget.eventFilter(widget, ev)


class TestKeyboardJog(unittest.TestCase):
    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _make_joints_tab(self) -> JointsTab:
        bridge = _FakeBridge()
        tab = JointsTab(bridge)
        tab.show()
        self._app.processEvents()
        tab.update_joint_state(
            types.SimpleNamespace(name=["joint1", "joint2"], position=[0.0, 0.0])
        )
        self._app.processEvents()
        tab.set_robot_ready(True)
        tab._chk_kbd_jog.setChecked(True)
        # Give focus to a non-editable control inside the tab.
        tab._btn_apply.setFocus()
        self._app.processEvents()
        return tab

    def test_joints_kbd_jog_is_off_until_enabled(self) -> None:
        tab = self._make_joints_tab()
        tab._chk_kbd_jog.setChecked(False)
        pressed: list = []
        tab._on_jog_pressed = lambda j, d: pressed.append((j, d))  # type: ignore
        _press(tab, Qt.Key_Up)
        self.assertEqual(pressed, [])

    def test_joints_up_down_route_to_jog(self) -> None:
        tab = self._make_joints_tab()
        pressed: list = []
        released: list = []
        tab._on_jog_pressed = lambda j, d: pressed.append((j, d))  # type: ignore
        tab._on_jog_released = lambda j, d: released.append((j, d))  # type: ignore
        active = tab._kbd_active_joint()
        self.assertIsNotNone(active)

        _press(tab, Qt.Key_Up)
        self.assertEqual(pressed, [(active, 1)])
        _press(tab, Qt.Key_Up, release=True)
        self.assertEqual(tab._kbd_held, None)

        _press(tab, Qt.Key_Down)
        self.assertEqual(pressed, [(active, 1), (active, -1)])

    def test_joints_left_right_cycle_active_joint(self) -> None:
        tab = self._make_joints_tab()
        first = tab._kbd_active_joint()
        _press(tab, Qt.Key_Right)
        second = tab._kbd_active_joint()
        self.assertNotEqual(first, second)
        _press(tab, Qt.Key_Left)
        self.assertEqual(tab._kbd_active_joint(), first)

    def test_joints_does_not_hijack_editable_fields(self) -> None:
        tab = self._make_joints_tab()
        pressed: list = []
        tab._on_jog_pressed = lambda j, d: pressed.append((j, d))  # type: ignore
        # A spin box is an editable field: keys must pass through.
        tab._duration.setFocus()
        self._app.processEvents()
        _press(tab, Qt.Key_Up)
        self.assertEqual(pressed, [])

    def test_pose_kbd_jog_routes_axes(self) -> None:
        bridge = _FakeBridge()
        tab = PoseTab(bridge)
        tab.show()
        self._app.processEvents()
        tab.set_robot_ready(True)
        tab._chk_kbd_jog.setChecked(True)
        called: list = []
        tab._jog = lambda axis, sign: called.append((axis, sign))  # type: ignore
        tab._btn_move.setFocus()  # non-editable control inside the tab
        self._app.processEvents()

        _press(tab, Qt.Key_Left)
        _press(tab, Qt.Key_Right)
        _press(tab, Qt.Key_Up)
        _press(tab, Qt.Key_Down)
        _press(tab, Qt.Key_PageUp)
        _press(tab, Qt.Key_PageDown)
        self.assertEqual(
            called,
            [
                ("x", -1.0),
                ("x", 1.0),
                ("y", 1.0),
                ("y", -1.0),
                ("z", 1.0),
                ("z", -1.0),
            ],
        )


if __name__ == "__main__":
    unittest.main()
