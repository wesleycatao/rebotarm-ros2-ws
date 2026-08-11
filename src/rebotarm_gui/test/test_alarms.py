"""Tests for the centralized operational alarm system."""

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

from PySide6.QtWidgets import QApplication  # noqa: E402

from rebotarm_gui.app import MainWindow  # noqa: E402
from rebotarm_gui.widgets.alarm import (  # noqa: E402
    Alarm,
    AlarmBanner,
    AlarmCenterPanel,
    HeaderAlarmIndicator,
    SEV_ERROR,
    SEV_INFO,
    SEV_WARNING,
    STATE_ACKNOWLEDGED,
    STATE_CLEARED,
)


def _make_alarm(**kwargs) -> Alarm:
    base = dict(message="Falha de teste", severity=SEV_ERROR, origin="Testes")
    base.update(kwargs)
    return Alarm(**base)


class TestAlarmModel(unittest.TestCase):
    def test_default_action_is_derived_from_origin(self) -> None:
        a = Alarm(message="x", severity=SEV_ERROR, origin="Action move_to_pose")
        self.assertIn("célula", a.recommended_action.lower())

    def test_explicit_action_is_preserved(self) -> None:
        a = Alarm(message="x", severity=SEV_WARNING, origin="Foo",
                  recommended_action="Faça X")
        self.assertEqual(a.recommended_action, "Faça X")

    def test_unknown_severity_falls_back_to_warning(self) -> None:
        a = Alarm(message="x", severity="bogus")
        self.assertEqual(a.severity, SEV_WARNING)

    def test_timestamp_is_set(self) -> None:
        self.assertIsNotNone(Alarm(message="x").timestamp)


class TestAlarmBanner(unittest.TestCase):
    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_render_and_hide(self) -> None:
        banner = AlarmBanner()
        self.assertFalse(banner.isVisible())
        banner.render_alarm(_make_alarm(message="motor sobreaquecido"), pending=2)
        self.assertTrue(banner.isVisible())
        self.assertIn("motor sobreaquecido", banner.message_label.text())
        self.assertTrue(banner.counter_button.isVisible())
        self.assertEqual(banner.counter_button.text(), "+2")
        banner.render_alarm(None)
        self.assertFalse(banner.isVisible())

    def test_banner_shows_origin_time_and_action(self) -> None:
        banner = AlarmBanner()
        alarm = _make_alarm(
            message="colisão", origin="Segurança",
            recommended_action="Inspecione a célula",
        )
        banner.render_alarm(alarm)
        self.assertIn("Segurança", banner.origin_label.text())
        self.assertIn("Horário", banner.time_label.text())
        self.assertIn("Inspecione a célula", banner.action_label.text())


class TestAlarmCenterPanel(unittest.TestCase):
    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_add_and_count(self) -> None:
        panel = AlarmCenterPanel()
        panel.add_alarm(_make_alarm(message="a", severity=SEV_ERROR))
        panel.add_alarm(_make_alarm(message="b", severity=SEV_WARNING))
        panel.add_alarm(_make_alarm(message="c", severity=SEV_INFO))
        self.assertEqual(panel.alarm_count(), 3)
        # Newest first -> 3 rows.
        self.assertEqual(panel.table.rowCount(), 3)

    def test_severity_filter(self) -> None:
        panel = AlarmCenterPanel()
        for i, sev in enumerate([SEV_ERROR, SEV_WARNING, SEV_INFO, SEV_ERROR]):
            panel.add_alarm(_make_alarm(message=f"m{i}", severity=sev))
        panel.combo.setCurrentIndex(1)  # Erros
        self.assertEqual(panel.table.rowCount(), 2)
        panel.combo.setCurrentIndex(0)  # Todos
        self.assertEqual(panel.table.rowCount(), 4)

    def test_search_filter(self) -> None:
        panel = AlarmCenterPanel()
        panel.add_alarm(_make_alarm(message="colisão detectada"))
        panel.add_alarm(_make_alarm(message="temperatura ok"))
        panel.search.setText("colisão")
        self.assertEqual(panel.table.rowCount(), 1)

    def test_clear_empties_history(self) -> None:
        panel = AlarmCenterPanel()
        panel.add_alarm(_make_alarm())
        panel.clear()
        self.assertEqual(panel.alarm_count(), 0)


class TestMainWindowAlarms(unittest.TestCase):
    """End-to-end: producers route through emit_alarm into banner + dock."""

    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self) -> None:
        self.window = MainWindow(start_bridge=False)
        self.window._apply_profile("supervisor", persist=False)
        self.window.show()
        self._app.processEvents()

    def tearDown(self) -> None:
        self.window.close()
        self.window.deleteLater()
        self._app.processEvents()

    def test_action_failure_lands_in_banner_and_history(self) -> None:
        self.window._on_action_result("move_to_pose", False, "controlador indisponível")
        self.assertTrue(self.window._alert_banner.isVisible())
        self.assertIn("controlador indisponível", self.window._alert_label.text())
        self.assertEqual(self.window._alarm_center.alarm_count(), 1)

    def test_alarm_carries_origin_and_action(self) -> None:
        self.window._on_service_result("enable", False, "timeout")
        alarm = self.window._alarm_center._alarms[-1]
        self.assertIn("enable", alarm.origin)
        self.assertTrue(alarm.recommended_action)

    def test_safety_alarm_uses_error_severity_for_fail(self) -> None:
        self.window._on_safety_alarm("FAIL", "torque acima do limite")
        self.assertEqual(self.window._alert_queue[0].severity, SEV_ERROR)

    def test_heartbeat_loss_emits_warning_alarm(self) -> None:
        self.window._on_arm_alive(False)
        self.assertTrue(any(a.origin == "Heartbeat arm_status"
                            for a in self.window._alarm_center._alarms))

    def test_dismiss_dequeues_and_marks_acknowledged(self) -> None:
        self.window._on_action_result("gripper_command", False, "falha 1")
        self.window._on_action_result("move_to_pose", False, "falha 2")
        self.assertEqual(len(self.window._alert_queue), 2)
        self.window._dismiss_alert()
        self.assertEqual(len(self.window._alert_queue), 1)
        # History keeps both even after dismissal.
        self.assertEqual(self.window._alarm_center.alarm_count(), 2)

    def test_acknowledged_alarm_can_later_be_normalized(self) -> None:
        alarm = self.window.emit_alarm("condição ativa", code="COND")
        self.window._dismiss_alert()
        self.assertEqual(alarm.state, "acknowledged")

        self.assertEqual(self.window.clear_alarm(code="COND"), 1)
        self.assertEqual(alarm.state, "cleared")

    def test_next_cycles_queue(self) -> None:
        self.window._on_action_result("a", False, "msg 1")
        self.window._on_action_result("b", False, "msg 2")
        first = self.window._alert_queue[0].message
        self.window._show_next_alert()
        # Head rotated to the back; queue[1] now holds the previous head.
        self.assertEqual(self.window._alert_queue[1].message, first)
        self.assertEqual(self.window._alert_queue[0].message, "Falha na ação b: msg 2")

    def test_alarm_center_dock_is_initially_hidden(self) -> None:
        self.assertFalse(self.window._alarm_dock.isVisible())
        self.window._show_alarm_center()
        self.assertTrue(self.window._alarm_dock.isVisible())

    def test_status_bar_has_alarms_button(self) -> None:
        self.assertEqual(self.window._btn_alarms.text(), "Alarmes")


class TestHeaderAlarmIndicator(unittest.TestCase):
    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _alarm(self, **kwargs) -> Alarm:
        return Alarm(**{**dict(message="m", severity=SEV_WARNING, origin="T"), **kwargs})

    def test_empty_state_is_hidden_and_announces_zero(self) -> None:
        ind = HeaderAlarmIndicator()
        ind.update_alarms([])
        self.assertFalse(ind.isVisible())
        self.assertIn("nenhum alarme ativo", ind.accessibleName().lower())

    def test_single_warning_shows_count_severity_and_summary(self) -> None:
        ind = HeaderAlarmIndicator()
        a = self._alarm(message="falha de comunicação", severity=SEV_WARNING, origin="Bridge")
        ind.update_alarms([a])
        self.assertTrue(ind.isVisible())
        self.assertEqual(ind._count.text(), "1")
        self.assertEqual(ind._severity.text(), "AVISO")
        self.assertIn("falha de comunicação", ind._summary.text())
        self.assertIn("Bridge", ind.toolTip())
        self.assertIn("Ação recomendada", ind.toolTip())

    def test_multiple_alarms_show_highest_severity_and_most_critical_first(self) -> None:
        ind = HeaderAlarmIndicator()
        info = self._alarm(message="info", severity=SEV_INFO)
        warning = self._alarm(message="aviso", severity=SEV_WARNING)
        error = self._alarm(message="erro crítico", severity=SEV_ERROR)
        ind.update_alarms([info, warning, error])
        self.assertEqual(ind._count.text(), "3")
        self.assertEqual(ind._severity.text(), "ERRO")
        self.assertIn("erro crítico", ind._summary.text())

    def test_acknowledged_but_active_alarm_remains_in_count(self) -> None:
        ind = HeaderAlarmIndicator()
        a = self._alarm(message="condição", severity=SEV_WARNING)
        a.state = STATE_ACKNOWLEDGED
        a.acknowledged = True
        ind.update_alarms([a])
        self.assertEqual(ind._count.text(), "1")
        self.assertIn("condição", ind._summary.text())

    def test_cleared_alarm_is_removed_from_count(self) -> None:
        ind = HeaderAlarmIndicator()
        a = self._alarm(message="normalizado", severity=SEV_ERROR)
        a.state = STATE_CLEARED
        ind.update_alarms([a])
        self.assertFalse(ind.isVisible())
        self.assertEqual(ind._count.text(), "0")

    def test_clicking_opens_alarm_center(self) -> None:
        ind = HeaderAlarmIndicator()
        fired = []
        ind.clicked.connect(lambda: fired.append(True))
        ind.clicked.emit()
        self.assertEqual(len(fired), 1)

    def test_compact_mode_hides_summary_and_keeps_count(self) -> None:
        ind = HeaderAlarmIndicator()
        a = self._alarm(message="mensagem longa que seria cortada", severity=SEV_WARNING)
        ind.update_alarms([a])
        ind.set_compact(True)
        self.assertFalse(ind._summary.isVisible())
        self.assertFalse(ind._severity.isVisible())
        self.assertEqual(ind._count.text(), "1")
        self.assertTrue(ind.isVisible())

    def test_accessible_name_includes_count_severity_and_action(self) -> None:
        ind = HeaderAlarmIndicator()
        a = self._alarm(message="falha", severity=SEV_ERROR, recommended_action="Reiniciar")
        ind.update_alarms([a])
        name = ind.accessibleName().lower()
        self.assertIn("1 ativo", name)
        self.assertIn("erro", name)
        self.assertIn("falha", name)
        self.assertIn("reiniciar", name)
        self.assertIn("centro de alarmes", name)


if __name__ == "__main__":
    unittest.main()
