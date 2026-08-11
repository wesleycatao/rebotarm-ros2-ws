"""Tests for the UX-polish items: undo/redo toolbar buttons, toast,
command palette, breadcrumb, collapsible sidebar and light theme."""

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

from PySide6.QtWidgets import QApplication  # noqa: E402

from rebotarm_gui.app import MainWindow  # noqa: E402
from rebotarm_gui.widgets.command_palette import CommandPalette  # noqa: E402


class TestUxPolish(unittest.TestCase):
    _app: QApplication | None = None
    _window: MainWindow | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)
        # MainWindow builds a large widget tree (+ bridge), so reconstructing
        # it per test piled up deferred deletions (deleteLater) and made the
        # suite progressively slow until it timed out. Build it once and reset
        # only the touched state between cases.
        cls._window = MainWindow(start_bridge=False)
        cls._window._apply_profile("supervisor", persist=False)
        cls._window.show()

    @classmethod
    def tearDownClass(cls) -> None:
        win = cls._window
        if win is not None:
            try:
                win._tab_motion._set_modified(False)
            except Exception:
                pass
            # Leave the persisted sidebar preference expanded so this suite
            # never pollutes other suites sharing the same QSettings.
            try:
                win._session_settings().setValue("session/sidebarCollapsed", False)
                win._session_settings().sync()
            except Exception:
                pass
            win.close()
            win.deleteLater()
        cls._app.processEvents()

    def setUp(self) -> None:
        assert self._window is not None
        self.window = self._window
        self._reset_window()

    def tearDown(self) -> None:
        # Some tests leave the motion program marked modified, which would
        # prompt an unsaved-changes modal on the shared window's closeEvent.
        try:
            self.window._tab_motion._set_modified(False)
        except Exception:
            pass
        self._app.processEvents()

    def _reset_window(self) -> None:
        """Return the shared window to the deterministic state each test expects."""
        win = self.window
        # Event log + async-operation tracking.
        win._log.clear()
        win._async_operations.clear()
        # Dismiss any leftover toasts so list-length checks start clean.
        manager = win._toast
        for toast in list(manager._active):
            try:
                toast.close()
                toast.deleteLater()
            except Exception:
                pass
        manager._active.clear()
        # Motion editor undo/redo stacks + modified flag.
        motion = win._tab_motion
        try:
            motion._set_modified(False)
            motion._undo_stack.clear()
            motion._redo_stack.clear()
            motion._update_undo_buttons()
        except Exception:
            pass
        # Sidebar normalised to expanded (matches the QSettings contract above).
        win._set_sidebar_collapsed(False)
        # Theme back to the default dark variant.
        win.apply_theme("dark")
        # Neutral starting page.
        win.navigate("home")
        self._app.processEvents()

    # ------------------------------------------------------------------
    # Item 5: visible Undo/Redo toolbar buttons
    # ------------------------------------------------------------------
    def test_undo_redo_buttons_start_disabled(self) -> None:
        motion = self.window._tab_motion
        self.assertFalse(motion._btn_undo.isEnabled())
        self.assertFalse(motion._btn_redo.isEnabled())

    def test_undo_redo_toolbar_buttons_trigger_history(self) -> None:
        motion = self.window._tab_motion
        # Seed an undo snapshot, then the buttons must enable.
        motion._push_undo_snapshot()
        self.assertTrue(motion._btn_undo.isEnabled())
        motion._btn_undo.click()
        self._app.processEvents()
        # After undo, redo becomes available.
        self.assertTrue(motion._btn_redo.isEnabled())

    # ------------------------------------------------------------------
    # Item 6: transient confirmation toast
    # ------------------------------------------------------------------
    def test_show_toast_creates_a_transient_overlay(self) -> None:
        self.window.show_toast("Programa salvo", kind="success")
        self._app.processEvents()
        self.assertEqual(len(self.window._toast._active), 1)
        toast = self.window._toast._active[0]
        self.assertEqual(toast.text(), "Programa salvo")
        self.assertEqual(toast.kind(), "success")
        self.assertTrue(toast.isVisible())

    def test_toast_coalesces_identical_messages(self) -> None:
        for _ in range(3):
            self.window.show_toast("Perfil aplicado", kind="success")
        self._app.processEvents()
        self.assertEqual(len(self.window._toast._active), 1)

    def test_ai_calibration_reports_start_and_completion(self) -> None:
        self.window._tab_vision.async_operation_started.emit(
            "vision_ai_calibration",
            "Calibração de visão por IA iniciada.",
        )
        self.window._bridge.service_result.emit(
            "vision_calibrate",
            True,
            "perfil recebido",
        )
        self._app.processEvents()

        log = self.window._log.toPlainText()
        self.assertIn("[op:vision_ai_calibration] INÍCIO", log)
        self.assertIn("[op:vision_ai_calibration] OK", log)
        self.assertNotIn("vision_ai_calibration", self.window._async_operations)
        self.assertTrue(
            any(
                toast.kind() == "success"
                and "Calibração de visão por IA concluída" in toast.text()
                for toast in self.window._toast._active
            )
        )

    def test_moveit_scene_reports_start_and_failure_once(self) -> None:
        self.window._tab_robot3d.async_operation_started.emit(
            "apply_cell_scene",
            "Aplicação da cena da célula no MoveIt iniciada.",
        )
        self.window._bridge.service_result.emit(
            "apply_cell_scene",
            False,
            "serviço indisponível",
        )
        # A synchronous bridge failure may also be relayed by the tab. It must
        # not create a duplicate terminal event.
        self.window._tab_robot3d.async_operation_finished.emit(
            "apply_cell_scene",
            False,
            "serviço indisponível",
        )
        self._app.processEvents()

        log = self.window._log.toPlainText()
        self.assertEqual(log.count("[op:apply_cell_scene] ERRO"), 1)
        self.assertNotIn("apply_cell_scene", self.window._async_operations)
        self.assertTrue(
            any(
                toast.kind() == "error"
                and "Aplicação da cena no MoveIt falhou" in toast.text()
                for toast in self.window._toast._active
            )
        )

    def test_diagnostic_export_feedback_reaches_toast_and_events(self) -> None:
        self.window._tab_diagnostics.snapshot_exported.emit(
            True,
            "Snapshot de diagnóstico exportado: /tmp/snapshot.json",
        )
        self._app.processEvents()

        self.assertIn(
            "[diagnóstico:exportar] OK",
            self.window._log.toPlainText(),
        )
        self.assertTrue(
            any(
                toast.kind() == "success"
                and "Snapshot de diagnóstico exportado" in toast.text()
                for toast in self.window._toast._active
            )
        )

    # ------------------------------------------------------------------
    # Item 7: command palette (Ctrl+K)
    # ------------------------------------------------------------------
    def test_command_palette_lists_pages_and_actions(self) -> None:
        self.window._open_command_palette()
        self._app.processEvents()
        palette = self.window._command_palette
        self.assertIsInstance(palette, CommandPalette)
        ids = {cmd.id for cmd in palette._commands}
        self.assertIn("nav:home", ids)
        self.assertIn("nav:motion", ids)
        self.assertIn("act:sidebar", ids)
        self.assertIn("act:theme", ids)

    def test_command_palette_filter_narrows_results(self) -> None:
        self.window._open_command_palette()
        palette = self.window._command_palette
        palette._search.setText("prog")
        self._app.processEvents()
        titles = [
            palette._list.item(i).data(0x0100)  # Qt.UserRole
            for i in range(palette._list.count())
        ]
        self.assertIn("nav:motion", titles)
        self.assertNotIn("nav:vision", titles)

    def test_ctrl_k_shortcut_is_registered(self) -> None:
        keys = [s.key().toString() for s in self.window._shortcuts]
        self.assertIn("Ctrl+K", keys)

    # ------------------------------------------------------------------
    # Item 8: breadcrumb flow indicator
    # ------------------------------------------------------------------
    def test_breadcrumb_shown_on_flow_pages(self) -> None:
        self.window.navigate("motion")
        self._app.processEvents()
        self.assertTrue(self.window._breadcrumb.isVisible())

    def test_breadcrumb_hidden_on_offflow_pages(self) -> None:
        self.window.navigate("settings")
        self._app.processEvents()
        self.assertFalse(self.window._breadcrumb.isVisible())

    def test_breadcrumb_marks_current_step(self) -> None:
        self.window.navigate("grasp")
        self._app.processEvents()
        labels = [
            self.window._breadcrumb_lay.itemAt(i).widget()
            for i in range(self.window._breadcrumb_lay.count())
            if self.window._breadcrumb_lay.itemAt(i).widget() is not None
        ]
        current = [w for w in labels if w.property("current") == "true"]
        self.assertEqual(len(current), 1)
        self.assertEqual(current[0].text(), "Pegas")

    # ------------------------------------------------------------------
    # Item 9: collapsible sidebar (icon-only mode)
    # ------------------------------------------------------------------
    def test_sidebar_collapse_toggles_width_and_text(self) -> None:
        expanded = self.window._sidebar.width()
        self.window._toggle_sidebar()
        self._app.processEvents()
        self.assertTrue(self.window._sidebar_collapsed)
        self.assertLess(self.window._sidebar.width(), expanded)
        # Nav buttons drop their text in collapsed mode.
        self.assertEqual(self.window._nav_buttons["home"].text(), "")
        # Expanding restores the labels.
        self.window._toggle_sidebar()
        self._app.processEvents()
        self.assertEqual(self.window._nav_buttons["home"].text(), "Início")

    def test_sidebar_collapse_persists_preference(self) -> None:
        self.window._set_sidebar_collapsed(True)
        self._app.processEvents()
        stored = self.window._session_settings().value("session/sidebarCollapsed")
        self.assertTrue(bool(stored))

    # ------------------------------------------------------------------
    # Item 10: light theme + selector
    # ------------------------------------------------------------------
    def test_apply_theme_sets_app_property_and_settings(self) -> None:
        self.window.apply_theme("light")
        self.assertEqual(self.window._settings.theme, "light")
        self.assertEqual(
            self._app.property("rebotarm_theme"), "light"
        )

    def test_apply_theme_is_idempotent_for_repeated_calls(self) -> None:
        # Re-applying the same theme must not raise or thrash.
        self.window.apply_theme("dark")
        self.window.apply_theme("dark")
        self.assertEqual(self.window._settings.theme, "dark")

    def test_settings_tab_has_theme_selector_synced(self) -> None:
        self.window.apply_theme("light")
        combo = self.window._tab_settings._theme_combo
        self.assertEqual(combo.currentData(), "light")


if __name__ == "__main__":
    unittest.main()
