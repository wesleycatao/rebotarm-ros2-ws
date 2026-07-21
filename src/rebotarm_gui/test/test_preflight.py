"""Tests for the motion-program pre-flight validation."""

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

from PySide6.QtCore import QObject, Signal  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from rebotarm_gui.motion import (  # noqa: E402
    ConditionExpr,
    GripperStep,
    IfStep,
    LineStep,
    LoopStep,
    MoveStep,
    OutputStep,
    Program,
    WaitForSignalStep,
    WaitForStateMachineStep,
    Condition,
    SRC_SIGNAL,
    SRC_STATE_MACHINE,
)
from rebotarm_gui.motion.preflight import (  # noqa: E402
    CAT_CONDITIONS,
    CAT_GRIPPER,
    CAT_IO,
    CAT_REACHABILITY,
    CAT_VELOCITIES,
    CAT_WAIT_STATE,
    PreflightReport,
    ReachabilityProbe,
    run_preflight,
    run_sync_checks,
)


class FakeBridge(QObject):
    """Minimal stand-in for RosBridge exercising the preflight surface."""

    ik_pose_checked = Signal(str, bool, str, object)

    def __init__(self, *, connected=True, ready=True, ik_ready=True) -> None:
        super().__init__()
        self._connected = connected
        self._ready = ready
        self._ik_ready = ik_ready
        self.ik_service_up = True
        self.joint_limits_dict: dict = {}
        self.joint_names_list: list = []
        self._q_solution_override: list | None = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def node(self):
        return None

    def motion_readiness(self, action_name: str = "move_to_pose"):
        if not self._connected:
            return False, "Não conectado ao ROS 2."
        if not self._ready:
            return False, "Braço não está habilitado."
        return True, ""

    def action_server_ready(self, name: str) -> bool:
        return self._ready

    def ik_service_ready(self, timeout_sec: float = 0.05) -> bool:
        return bool(self.ik_service_up)

    def dio_input_ids(self) -> list:
        return ["IN1", "IN2"]

    def dio_output_ids(self) -> list:
        return ["OUT1", "OUT2"]

    def joint_limits(self) -> dict:
        return dict(self.joint_limits_dict)

    def joint_names(self) -> list:
        return list(self.joint_names_list)

    def check_pose_reachable(self, key: str, pose_dict: dict) -> bool:
        # Emit synchronously so tests are deterministic.
        ok = self._ik_ready
        q = self._q_solution_override if self._q_solution_override is not None else (
            [0.0] * 6 if ok else []
        )
        self.ik_pose_checked.emit(key, ok, "" if ok else "sem solução", q)
        return True


def _move_program(**kwargs) -> Program:
    p = Program(name="t")
    p.steps.append(MoveStep(pose={"x": 0.3, "y": 0.0, "z": 0.3,
                                  "roll": 0, "pitch": 0, "yaw": 0}, **kwargs))
    return p


class TestSyncChecks(unittest.TestCase):
    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_empty_program_is_blocked(self) -> None:
        report = run_sync_checks(Program(), FakeBridge(), for_real_run=True)
        self.assertFalse(report.ok)
        self.assertTrue(any("vazio" in i.message.lower() for i in report.errors))

    def test_program_without_actions_is_blocked(self) -> None:
        p = Program(name="t")
        p.steps.append(IfStep(condition=ConditionExpr()))  # no real actions
        report = run_sync_checks(p, FakeBridge(), for_real_run=True)
        self.assertFalse(report.ok)

    def test_valid_program_passes_for_real_run(self) -> None:
        report = run_sync_checks(_move_program(duration=2.0), FakeBridge(), for_real_run=True)
        self.assertTrue(report.ok, [i.message for i in report.errors])

    def test_dry_run_requires_connection_only(self) -> None:
        report = run_sync_checks(_move_program(duration=2.0), FakeBridge(connected=False),
                                 for_real_run=False)
        self.assertFalse(report.ok)
        self.assertTrue(any("ros 2" in i.message.lower() for i in report.errors))

    def test_malformed_if_condition_is_blocked(self) -> None:
        p = _move_program(duration=2.0)
        p.steps.append(IfStep(condition=ConditionExpr(
            items=[Condition(source=SRC_SIGNAL, params={}, op="==", value=1)]
        )))
        report = run_sync_checks(p, FakeBridge(), for_real_run=True)
        cond_errors = [i for i in report.errors if i.category == CAT_CONDITIONS]
        self.assertTrue(cond_errors, [i.message for i in report.items])
        self.assertIn("io_id", cond_errors[0].message)

    def test_bad_operator_for_string_value(self) -> None:
        p = _move_program(duration=2.0)
        p.steps.append(IfStep(condition=ConditionExpr(items=[
            Condition(source=SRC_STATE_MACHINE, params={}, op=">", value="IDLE",
                      value_type="string")
        ])))
        report = run_sync_checks(p, FakeBridge(), for_real_run=True)
        self.assertTrue(any("operador" in i.message.lower() for i in report.errors))

    def test_gripper_out_of_range_is_warning_not_error(self) -> None:
        p = _move_program(duration=2.0)
        p.steps.append(GripperStep(position_mm=200.0))
        report = run_sync_checks(p, FakeBridge(), for_real_run=True)
        self.assertTrue(report.ok)  # warning only, does not block
        self.assertTrue(any(i.category == CAT_GRIPPER for i in report.warnings))

    def test_bad_signal_edge_is_error(self) -> None:
        p = _move_program(duration=2.0)
        p.steps.append(WaitForSignalStep(io_id="IN1", value=1, edge="bogus"))
        report = run_sync_checks(p, FakeBridge(), for_real_run=True)
        self.assertFalse(report.ok)
        self.assertTrue(any(i.category == CAT_IO for i in report.errors))

    def test_unknown_io_is_warning(self) -> None:
        p = _move_program(duration=2.0)
        p.steps.append(OutputStep(io_id="OUT99", action="on"))
        report = run_sync_checks(p, FakeBridge(), for_real_run=True)
        self.assertTrue(report.ok)
        self.assertTrue(any("OUT99" in i.message for i in report.warnings))

    def test_speed_scale_out_of_range_warns(self) -> None:
        p = _move_program(duration=2.0)
        p.speed_scale = 10.0
        report = run_sync_checks(p, FakeBridge(), for_real_run=True)
        self.assertTrue(any(i.category == CAT_VELOCITIES for i in report.warnings))

    def test_zero_speed_is_error(self) -> None:
        p = _move_program(duration=2.0)
        p.steps.append(LineStep(start={"x": 0.3, "y": -0.1, "z": 0.3,
                                       "roll": 0, "pitch": 0, "yaw": 0},
                                end={"x": 0.3, "y": 0.1, "z": 0.3,
                                     "roll": 0, "pitch": 0, "yaw": 0}, speed=0.0))
        report = run_sync_checks(p, FakeBridge(), for_real_run=True)
        self.assertFalse(report.ok)

    def test_invalid_state_machine_warns(self) -> None:
        p = _move_program(duration=2.0)
        p.steps.append(WaitForStateMachineStep(state="BOGUS"))
        report = run_sync_checks(p, FakeBridge(), for_real_run=True)
        self.assertTrue(any(i.category == CAT_WAIT_STATE for i in report.warnings))

    def test_nested_loop_steps_are_walked(self) -> None:
        p = _move_program(duration=2.0)
        loop = LoopStep(count=2)
        loop.steps.append(GripperStep(position_mm=-5.0))  # warning inside loop
        p.steps.append(loop)
        report = run_sync_checks(p, FakeBridge(), for_real_run=True)
        self.assertTrue(any(i.category == CAT_GRIPPER for i in report.warnings))


class TestReachabilityProbe(unittest.TestCase):
    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_reachable_pose_passes(self) -> None:
        bridge = FakeBridge(ik_ready=True)
        report = run_sync_checks(_move_program(duration=2.0), bridge, for_real_run=True)
        received: list[PreflightReport] = []
        probe = ReachabilityProbe(_move_program(duration=2.0), bridge, report)
        probe.finished.connect(lambda r: received.append(r))
        probe.start()
        self._app.processEvents()
        self.assertEqual(len(received), 1)
        final = received[0]
        self.assertTrue(final.reachability_done)
        reach_items = [i for i in final.items if i.category == CAT_REACHABILITY]
        self.assertTrue(any("alcançável" in i.message for i in reach_items), [i.message for i in reach_items])
        self.assertTrue(final.ok)

    def test_unreachable_pose_blocks(self) -> None:
        bridge = FakeBridge(ik_ready=False)
        report = run_sync_checks(_move_program(duration=2.0), bridge, for_real_run=True)
        received: list[PreflightReport] = []
        probe = ReachabilityProbe(_move_program(duration=2.0), bridge, report)
        probe.finished.connect(lambda r: received.append(r))
        probe.start()
        self._app.processEvents()
        self.assertEqual(len(received), 1)
        final = received[0]
        self.assertFalse(final.ok)
        self.assertTrue(any(i.category == CAT_REACHABILITY for i in final.errors))

    def test_joint_limit_violation_becomes_warning(self) -> None:
        bridge = FakeBridge(ik_ready=True)
        bridge.joint_names_list = ["joint1", "joint2"]
        bridge.joint_limits_dict = {"joint1": (-1.0, 1.0), "joint2": (-1.0, 1.0)}
        # Solution within joint1 range but outside joint2's range.
        bridge._q_solution_override = [0.0, 5.0]

        report = run_sync_checks(_move_program(duration=2.0), bridge, for_real_run=True)
        received: list[PreflightReport] = []
        probe = ReachabilityProbe(_move_program(duration=2.0), bridge, report)
        probe.finished.connect(lambda r: received.append(r))
        probe.start()
        self._app.processEvents()
        self.assertEqual(len(received), 1)
        final = received[0]
        # Joint limit violation is a warning, not an error: run still allowed.
        self.assertTrue(final.ok)
        self.assertTrue(any("fora de" in i.message for i in final.warnings))


class TestRunPreflightBlocking(unittest.TestCase):
    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_sync_errors_skip_reachability(self) -> None:
        bridge = FakeBridge(connected=False)
        report = run_preflight(
            _move_program(duration=2.0), bridge,
            for_real_run=False, with_reachability=True,
        )
        self.assertFalse(report.ok)
        self.assertTrue(report.reachability_skipped_reason)

    def test_blocking_call_returns_after_reachability(self) -> None:
        bridge = FakeBridge(ik_ready=True)
        report = run_preflight(
            _move_program(duration=2.0), bridge,
            for_real_run=True, with_reachability=True,
            reachability_timeout_ms=1500,
        )
        self.assertTrue(report.reachability_done)
        self.assertTrue(report.ok)


class TestMotionTabGate(unittest.TestCase):
    """The Motion tab wires a 'Validar' button and blocks runs on errors."""

    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self) -> None:
        from rebotarm_gui.ros_bridge import RosBridge
        from rebotarm_gui.tabs.motion_tab import MotionTab
        # Patch QMessageBox to never block during the test. ``_show_preflight_block``
        # drives an instance ``box.exec()``, so patch the instance method too.
        from PySide6.QtWidgets import QMessageBox
        self._QMB = QMessageBox
        QMessageBox.critical = staticmethod(lambda *a, **k: None)
        QMessageBox.information = staticmethod(lambda *a, **k: None)
        QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.Yes)
        self._orig_exec = QMessageBox.exec
        QMessageBox.exec = lambda self_box, *a, **k: 0
        self.bridge = RosBridge()  # not started -> not connected
        self.tab = MotionTab(self.bridge)

    def tearDown(self) -> None:
        from PySide6.QtWidgets import QMessageBox
        QMessageBox.critical = self._QMB.critical
        QMessageBox.information = self._QMB.information
        QMessageBox.question = self._QMB.question
        QMessageBox.exec = self._orig_exec

    def test_validate_button_exists(self) -> None:
        self.assertEqual(self.tab._btn_validate.text(), "Validar")

    def test_gate_blocks_when_no_program(self) -> None:
        self.tab._chk_dry_run.setChecked(True)
        self.assertFalse(self.tab._preflight_gate())

    def test_gate_blocks_on_malformed_condition(self) -> None:
        self.tab._program.steps.append(MoveStep(
            pose={"x": 0.3, "y": 0.0, "z": 0.3, "roll": 0, "pitch": 0, "yaw": 0},
            duration=2.0,
        ))
        self.tab._program.steps.append(IfStep(condition=ConditionExpr(items=[
            Condition(source=SRC_SIGNAL, params={}, op="==", value=1)
        ])))
        self.tab._chk_dry_run.setChecked(True)
        self.assertFalse(self.tab._preflight_gate())
        self.assertFalse(self.tab._last_preflight.ok)


if __name__ == "__main__":
    unittest.main()
