"""Accessibility, keyboard and operator-test regression coverage."""

from __future__ import annotations

import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/rebotarm_gui_test_mpl")

from PySide6.QtCore import Qt  # noqa: E402
from PySide6.QtWidgets import (  # noqa: E402
    QAbstractButton,
    QAbstractItemView,
    QAbstractSlider,
    QAbstractSpinBox,
    QApplication,
    QComboBox,
    QFrame,
    QLineEdit,
    QTabBar,
    QTextEdit,
    QWidget,
)

from rebotarm_gui.accessibility import effective_accessible_name  # noqa: E402
from rebotarm_gui.app import MainWindow, load_stylesheet  # noqa: E402
from rebotarm_gui.widgets.operator_test_panel import OperatorTestPanel  # noqa: E402
from rebotarm_gui.widgets.status_led import StatusLed  # noqa: E402


INTERACTIVE_TYPES = (
    QAbstractButton,
    QAbstractItemView,
    QAbstractSlider,
    QAbstractSpinBox,
    QComboBox,
    QLineEdit,
    QTabBar,
    QTextEdit,
)


def _relative_luminance(hex_color: str) -> float:
    channels = [int(hex_color[i : i + 2], 16) / 255.0 for i in (1, 3, 5)]
    linear = [
        value / 12.92
        if value <= 0.04045
        else ((value + 0.055) / 1.055) ** 2.4
        for value in channels
    ]
    return 0.2126 * linear[0] + 0.7152 * linear[1] + 0.0722 * linear[2]


def _contrast(foreground: str, background: str) -> float:
    lighter, darker = sorted(
        (_relative_luminance(foreground), _relative_luminance(background)),
        reverse=True,
    )
    return (lighter + 0.05) / (darker + 0.05)


class TestAccessibility(unittest.TestCase):
    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self) -> None:
        self.window = MainWindow(start_bridge=False)
        self.window._apply_profile("engenharia", persist=False)
        self.window.show()
        self._app.processEvents()

    def tearDown(self) -> None:
        self.window.close()
        self.window.deleteLater()
        self._app.processEvents()

    def test_every_visible_keyboard_control_has_a_text_name(self) -> None:
        missing: list[str] = []
        for key, page in self.window._pages.items():
            self.window.navigate(key)
            self._app.processEvents()
            for widget in page.findChildren(QWidget):
                if not isinstance(widget, INTERACTIVE_TYPES):
                    continue
                if not widget.isEnabled() or not widget.isVisibleTo(page):
                    continue
                if not widget.focusPolicy() & Qt.TabFocus:
                    continue
                if not effective_accessible_name(widget):
                    missing.append(f"{key}:{type(widget).__name__}:{widget.objectName()}")
        self.assertEqual(missing, [])

    def test_form_labels_are_exposed_as_field_names_and_buddies(self) -> None:
        position = self.window._tab_gripper._position
        self.assertEqual(position.accessibleName(), "Posição alvo")
        labels = self.window._tab_gripper.findChildren(QWidget)
        position_label = next(
            label for label in labels
            if hasattr(label, "text") and label.text() == "Posição alvo"
        )
        self.assertIs(position_label.buddy(), position)

    def test_keyboard_navigation_enters_page_and_cycles_regions(self) -> None:
        self.window.navigate("motion", focus_content=True)
        self._app.processEvents()
        focused = QApplication.focusWidget()
        self.assertIsNotNone(focused)
        self.assertTrue(self.window._tab_motion.isAncestorOf(focused))

        self.window._nav_buttons["motion"].setFocus()
        self.window._cycle_focus_region(1)
        self._app.processEvents()
        focused = QApplication.focusWidget()
        self.assertTrue(self.window._tab_motion.isAncestorOf(focused))

    def test_keyboard_help_and_region_shortcuts_are_registered(self) -> None:
        shortcuts = {shortcut.key().toString() for shortcut in self.window._shortcuts}
        self.assertTrue({"F1", "F6", "Shift+F6", "Ctrl+Shift+A"} <= shortcuts)
        self.assertEqual(
            self.window._btn_keyboard_help.accessibleName(),
            "Ajuda (F1)",
        )

    def test_jog_controls_are_keyboard_focusable_and_named(self) -> None:
        from rebotarm_gui.widgets.joint_slider import JointSlider

        slider = JointSlider("joint_1", 0)
        self.assertTrue(slider._btn_jog_minus.focusPolicy() & Qt.TabFocus)
        self.assertIn("joint_1", slider._btn_jog_minus.accessibleName())
        self.assertIn("negativo", slider._btn_jog_minus.accessibleName())
        slider.deleteLater()

    def test_status_is_not_encoded_by_color_alone(self) -> None:
        led = StatusLed("amber")
        self.assertEqual(led._shape_key, "triangle")
        self.assertIn("aguardando", led.accessibleDescription())
        self.window._on_arm_alive(False)
        self.assertIn("sem sinal", self.window._lbl_ros_status.text().lower())
        self.assertIn("sem sinal", self.window._header_arm.text().lower())
        led.deleteLater()

    def test_alarm_announces_textual_severity_origin_and_action(self) -> None:
        self.window.emit_alarm(
            "Falha simulada",
            severity="error",
            origin="Teste",
            recommended_action="Revisar conexão",
        )
        banner = self.window._alert_banner
        self.assertEqual(banner.severity_label.text(), "ERRO")
        self.assertEqual(banner.origin_label.text(), "Origem: Teste")
        self.assertIn("Revisar conexão", banner.action_label.text())
        self.assertIn("Falha simulada", banner.accessibleDescription())

    def test_informational_dashboard_cards_do_not_claim_button_semantics(self) -> None:
        self.assertIsInstance(self.window._card_error, QFrame)
        self.assertNotIsInstance(self.window._card_error, QAbstractButton)
        self.assertIsInstance(self.window._card_program, QAbstractButton)

    def test_theme_contrast_and_focus_tokens_meet_baseline(self) -> None:
        pairs = [
            ("#edf2f7", "#121417"),
            ("#9aa8b5", "#121417"),
            ("#8391a0", "#181d23"),
            ("#aab4c0", "#1b2027"),
            ("#79e6ce", "#17352f"),
            ("#ffd37a", "#3a3020"),
            ("#ffe9ec", "#472429"),
            ("#7d8995", "#171b20"),
        ]
        for foreground, background in pairs:
            self.assertGreaterEqual(_contrast(foreground, background), 4.5)
        self.assertGreaterEqual(_contrast("#8ff3dc", "#121417"), 3.0)
        qss = load_stylesheet()
        for selector in ("QPushButton:focus", "QLineEdit:focus", "QSlider::groove"):
            self.assertIn(selector, qss)


class TestOperatorTestPanel(unittest.TestCase):
    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_records_all_timed_tasks_and_exports_anonymous_json(self) -> None:
        panel = OperatorTestPanel()
        panel.participant.setText("OP-01")
        for index in range(len(panel._tasks)):
            panel.start_current_task()
            time.sleep(0.002)
            panel.errors.setValue(index % 2)
            panel.finish_current_task(success=index != 5)

        self.assertTrue(panel.btn_save.isEnabled())
        payload = panel.session_payload()
        self.assertTrue(payload["completed"])
        self.assertEqual(len(payload["results"]), len(panel._tasks))
        self.assertNotIn("name", payload)
        self.assertGreaterEqual(payload["results"][0]["elapsed_s"], 0.0)

        with tempfile.TemporaryDirectory() as directory:
            target = panel.save_results(Path(directory) / "session.json")
            saved = json.loads(target.read_text(encoding="utf-8"))
        self.assertEqual(saved["participant_code"], "OP-01")
        self.assertFalse(saved["results"][-1]["success"])
        panel.deleteLater()


if __name__ == "__main__":
    unittest.main()
