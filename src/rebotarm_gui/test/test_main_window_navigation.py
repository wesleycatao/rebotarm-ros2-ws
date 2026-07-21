"""UI-shell tests for the desktop-oriented reBotArm navigation."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/rebotarm_gui_test_mpl")

from PySide6.QtCore import QPoint  # noqa: E402
from PySide6.QtWidgets import QApplication, QMessageBox, QScrollArea  # noqa: E402

from rebotarm_gui.app import MainWindow  # noqa: E402


class TestMainWindowNavigation(unittest.TestCase):
    """The daily workflow stays visible while engineering pages stay secondary."""

    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self) -> None:
        self.window = MainWindow(start_bridge=False)
        # Ignore any profile persisted on the developer machine.
        self.window._apply_profile("supervisor", persist=False)
        self.window.show()
        self._app.processEvents()

    def tearDown(self) -> None:
        self.window.close()
        self.window.deleteLater()
        self._app.processEvents()

    def test_daily_pages_are_primary_and_engineering_pages_are_secondary(self) -> None:
        self.assertEqual(
            list(self.window._nav_buttons),
            [
                "home",
                "vision",
                "grasp",
                "motion",
                "pose",
                "joints",
                "gripper",
                "robot3d",
                "safety",
            ],
        )
        self.assertEqual(
            set(self.window._advanced_actions),
            {"system", "io", "telemetry", "diagnostics", "mit", "settings"},
        )
        self.assertTrue(
            set(self.window._nav_buttons).isdisjoint(self.window._advanced_actions)
        )

    def test_home_is_default_and_navigation_updates_context(self) -> None:
        self.assertIs(self.window._stack.currentWidget(), self.window._pages["home"])
        self.assertEqual(self.window._page_title.text(), "Início")

        self.window.navigate("motion")
        self.assertIs(self.window._stack.currentWidget(), self.window._tab_motion)
        self.assertEqual(self.window._page_title.text(), "Programa")
        self.assertTrue(self.window._nav_buttons["motion"].isChecked())

        self.window.navigate("settings")
        self.assertIs(self.window._stack.currentWidget(), self.window._tab_settings)
        self.assertEqual(self.window._page_title.text(), "Configurações")
        self.assertFalse(any(button.isChecked() for button in self.window._nav_buttons.values()))

    def test_event_log_is_hidden_outside_vision_until_requested(self) -> None:
        self.window.navigate("vision")
        self.assertFalse(self.window._event_dock.isVisible())
        self.assertIsNot(self.window._log.parentWidget(), self.window._tab_vision)

        self.window._toggle_event_dock()
        self._app.processEvents()
        self.assertTrue(self.window._event_dock.isVisible())

    def test_unknown_page_is_rejected(self) -> None:
        with self.assertRaises(KeyError):
            self.window.navigate("not-a-page")

    def test_robot3d_page_is_registered(self) -> None:
        self.window.navigate("robot3d")
        self.assertIs(self.window._stack.currentWidget(), self.window._tab_robot3d)
        self.assertEqual(self.window._page_title.text(), "Robô 3D")

    def test_ros_diagnostics_page_is_advanced_and_profile_gated(self) -> None:
        self.window.navigate("diagnostics")
        self.assertIs(self.window._stack.currentWidget(), self.window._tab_diagnostics)
        self.assertEqual(self.window._page_title.text(), "Diagnóstico ROS")
        self.assertEqual(self.window._advanced_button.text(), "Ferramentas")

        self.window._apply_profile("operador", persist=False)
        self.window.navigate("diagnostics")
        self.assertIsNot(self.window._stack.currentWidget(), self.window._tab_diagnostics)

    def test_estop_button_triggers_bridge_emergency_stop(self) -> None:
        calls = []
        original = self.window._bridge.emergency_stop
        self.window._bridge.emergency_stop = lambda: calls.append(True)
        try:
            self.window._btn_estop.click()
        finally:
            self.window._bridge.emergency_stop = original
        self.assertTrue(calls, "botão de parada de software não acionou emergency_stop da bridge")

    def test_software_stop_is_not_presented_as_physical_estop(self) -> None:
        self.assertEqual(self.window._btn_estop.text(), "PARADA SOFTWARE")
        shortcut_names = [shortcut.key().toString() for shortcut in self.window._shortcuts]
        self.assertNotIn("Esc", shortcut_names)

    def test_disable_requires_confirmation(self) -> None:
        calls = []
        original_call = self.window._bridge.disable
        original_dialog = QMessageBox.question
        self.window._bridge.disable = lambda: calls.append(True)
        try:
            QMessageBox.question = lambda *args, **kwargs: QMessageBox.Cancel
            self.window._confirm_disable()
            self.assertFalse(calls, "desabilitar executou mesmo com Cancel")

            QMessageBox.question = lambda *args, **kwargs: QMessageBox.Yes
            self.window._confirm_disable()
            self.assertTrue(calls, "desabilitar não executou após confirmação")
        finally:
            self.window._bridge.disable = original_call
            QMessageBox.question = original_dialog

    def test_status_led_follows_arm_heartbeat(self) -> None:
        self.window._on_arm_alive(True)
        self.assertEqual(self.window._led._color_key, "green")
        self.window._on_arm_alive(False)
        self.assertEqual(self.window._led._color_key, "amber")

    def test_heartbeat_timeout_invalidates_cached_state_and_commands(self) -> None:
        self.window._header_arm.setText("BRAÇO ATIVO")
        self.window._dash_arm.setText("ATIVO")
        self.window._btn_home.setEnabled(True)
        self.window._btn_disable.setEnabled(True)
        self.window._tab_system.set_robot_state(True, True)

        self.window._on_arm_alive(False)

        self.assertEqual(self.window._header_arm.text(), "BRAÇO SEM SINAL")
        self.assertEqual(self.window._dash_arm.text(), "SEM SINAL")
        self.assertFalse(self.window._btn_home.isEnabled())
        self.assertFalse(self.window._btn_disable.isEnabled())
        self.assertFalse(self.window._tab_system._btn_home.isEnabled())
        self.assertFalse(self.window._tab_joints._btn_apply.isEnabled())
        self.assertFalse(self.window._tab_pose._btn_move.isEnabled())
        self.assertFalse(self.window._tab_gripper._btn_cmd.isEnabled())

    def test_profile_gates_engineering_pages(self) -> None:
        self.window._apply_profile("operador", persist=False)
        self.window.navigate("mit")
        self.assertIsNot(
            self.window._stack.currentWidget(),
            self.window._tab_mit,
            "MIT/Raw deveria ser bloqueado para Operador",
        )
        self.assertFalse(self.window._advanced_button.isVisible())

        self.window._apply_profile("engenharia", persist=False)
        self.window.navigate("mit")
        self.assertIs(self.window._stack.currentWidget(), self.window._tab_mit)

    def test_interface_mode_gates_safety_editing_inside_page(self) -> None:
        self.window._apply_profile("operador", persist=False)
        self.assertFalse(self.window._tab_safety._btn_apply.isEnabled())
        self.assertFalse(self.window._tab_safety._sections.isTabEnabled(1))
        self.assertTrue(self.window._tab_safety._btn_reset_fault.isEnabled())

        self.window._apply_profile("supervisor", persist=False)
        self.assertTrue(self.window._tab_safety._btn_apply.isEnabled())
        self.assertFalse(self.window._tab_safety._sections.isTabEnabled(1))

        self.window._apply_profile("engenharia", persist=False)
        self.assertTrue(self.window._tab_safety._sections.isTabEnabled(1))
        self.assertTrue(self.window._tab_safety._sections.isTabEnabled(2))

    def test_vision_keeps_operator_actions_visible_and_details_collapsed(self) -> None:
        self.window.navigate("vision")
        self.assertTrue(self.window._tab_vision._btn_process.isVisible())
        self.assertTrue(self.window._tab_vision._btn_apply.isVisible())
        self.assertFalse(self.window._tab_vision._task_box.isVisible())
        self.assertTrue(self.window._tab_vision._objects.isColumnHidden(6))
        self.assertFalse(self.window._tab_vision._objects.isColumnHidden(19))

    def test_failed_action_uses_persistent_alert(self) -> None:
        self.window._on_action_result("move_to_pose", False, "controlador indisponível")
        self.assertTrue(self.window._alert_banner.isVisible())
        self.assertIn("controlador indisponível", self.window._alert_label.text())

    def test_window_close_can_be_blocked_by_unsaved_program(self) -> None:
        original = self.window._tab_motion.confirm_close
        self.window._tab_motion.confirm_close = lambda: False
        try:
            self.window.close()
            self._app.processEvents()
            self.assertTrue(self.window.isVisible())
        finally:
            self.window._tab_motion.confirm_close = original

    def test_program_toolbar_fits_supported_minimum_width(self) -> None:
        self.window.resize(1360, 720)
        self.window.navigate("motion")
        self._app.processEvents()
        tab = self.window._tab_motion
        for widget in (
            tab._btn_save,
            tab._btn_load,
            tab._btn_new,
            tab._btn_undo,
            tab._btn_redo,
            tab._btn_freedrive,
            tab._btn_teach,
            tab._btn_view_list,
            tab._btn_view_flow,
            tab._btn_more_actions,
        ):
            left = widget.mapTo(tab, QPoint(0, 0)).x()
            self.assertLessEqual(left + widget.width(), tab.width())

    def test_compact_header_and_home_scroll_fit_supported_minimum(self) -> None:
        self.window.resize(1360, 720)
        self.window.navigate("home")
        self._app.processEvents()

        self.assertTrue(self.window._top_compact)
        self.assertIsInstance(self.window._pages["home"], QScrollArea)
        top = self.window._top_safety.parentWidget()
        for widget in (
            self.window._top_title_host,
            self.window._top_chips,
            self.window._top_ops,
            self.window._top_safety,
        ):
            left = widget.mapTo(top, QPoint(0, 0)).x()
            self.assertGreaterEqual(left, 0)
            self.assertLessEqual(left + widget.width(), top.width())
        for button in (
            self.window._btn_enable,
            self.window._btn_home,
            self.window._btn_disable,
            self.window._btn_estop,
        ):
            self.assertGreaterEqual(button.width(), button.sizeHint().width())

    def test_system_page_uses_same_home_and_disable_confirmations(self) -> None:
        calls = []
        original_home = self.window._bridge.safe_home
        original_disable = self.window._bridge.disable
        original_dialog = QMessageBox.question
        self.window._bridge.safe_home = lambda: calls.append("home")
        self.window._bridge.disable = lambda: calls.append("disable")
        self.window._tab_system.set_robot_state(True, True)
        try:
            QMessageBox.question = lambda *args, **kwargs: QMessageBox.Cancel
            self.window._tab_system._btn_home.click()
            self.window._tab_system._btn_disable.click()
            self.assertEqual(calls, [])

            QMessageBox.question = lambda *args, **kwargs: QMessageBox.Yes
            self.window._tab_system._btn_home.click()
            self.window._tab_system._btn_disable.click()
            self.assertEqual(calls, ["home", "disable"])
        finally:
            self.window._bridge.safe_home = original_home
            self.window._bridge.disable = original_disable
            QMessageBox.question = original_dialog


if __name__ == "__main__":
    unittest.main()
