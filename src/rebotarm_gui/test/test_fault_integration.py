"""Real ROS failure scenarios exercised against the configurable mock arm.

Each test starts an isolated ROS process and uses the real ``RosBridge`` action
clients/subscriptions.  The suite intentionally covers failure transitions,
not only successful endpoint discovery.
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
import unittest
from pathlib import Path
from typing import Callable

WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

from PySide6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from rebotarm_gui.motion.program import MoveStep, Program, pose  # noqa: E402
from rebotarm_gui.ros_bridge import RosBridge, make_pose  # noqa: E402
from rebotarm_gui.tabs.motion_tab import MotionTab  # noqa: E402


class FaultIntegrationTest(unittest.TestCase):
    """Validate fail-closed behavior using DDS traffic and real action servers."""

    _app: QApplication | None = None
    _sequence = 0

    @classmethod
    def setUpClass(cls) -> None:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self) -> None:
        type(self)._sequence += 1
        self.namespace = f"rebotarm_fault_{type(self)._sequence}"
        self.bridge: RosBridge | None = None
        self.mock_proc: subprocess.Popen[str] | None = None

    def tearDown(self) -> None:
        if self.bridge is not None:
            self.bridge.shutdown()
            self.bridge = None
        self._stop_mock()

    def _start_mock(self, *fault_args: str) -> None:
        cmd = [
            sys.executable,
            "-m",
            "rebotarm_gui.mock_arm",
            "--namespace",
            self.namespace,
            *fault_args,
        ]
        self.mock_proc = subprocess.Popen(
            cmd,
            cwd=str(WORKSPACE),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        time.sleep(0.5)
        if self.mock_proc.poll() is not None:
            self.fail("mock_arm encerrou durante a inicialização")

    def _stop_mock(self) -> str:
        proc = self.mock_proc
        if proc is None:
            return ""
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=4.0)
        self.mock_proc = None
        return ""

    def _start_bridge(self) -> RosBridge:
        self.bridge = RosBridge(namespace=self.namespace)
        self.bridge.start()
        self.assertTrue(
            self._wait_until(
                lambda: bool(self.bridge and self.bridge._arm_alive_state),
                timeout=8.0,
            ),
            "bridge não recebeu o heartbeat inicial do mock",
        )
        return self.bridge

    def _wait_until(self, predicate: Callable[[], bool], timeout: float = 6.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert self._app is not None
            self._app.processEvents()
            if predicate():
                return True
            time.sleep(0.03)
        self._app.processEvents()
        return bool(predicate())

    def test_heartbeat_loss_expires_readiness(self) -> None:
        self._start_mock("--heartbeat-stop-after", "2.0")
        bridge = self._start_bridge()
        alive_events: list[bool] = []
        bridge.arm_alive.connect(alive_events.append)

        self.assertTrue(
            self._wait_until(lambda: False in alive_events, timeout=5.0),
            "heartbeat interrompido não expirou na bridge",
        )
        snapshot = bridge.diagnostic_snapshot()
        ready, reason = bridge.motion_readiness()
        self.assertFalse(snapshot["heartbeat"]["alive"])
        self.assertGreater(float(snapshot["heartbeat"]["age_ms"]), 1000.0)
        self.assertFalse(ready)
        self.assertIn("heartbeat", reason)

    def test_hung_action_times_out_and_is_cancelled(self) -> None:
        self._start_mock("--hang-action", "move_to_pose")
        bridge = self._start_bridge()
        bridge.action_result_grace_s = 0.35
        results: list[tuple[str, bool, str]] = []
        logs: list[tuple[str, str]] = []
        bridge.action_result.connect(lambda name, ok, msg: results.append((name, ok, msg)))
        bridge.log_message.connect(lambda level, msg: logs.append((level, msg)))

        self.assertTrue(bridge.move_to_pose(make_pose(0.2, 0.0, 0.3), 0.1))
        self.assertTrue(self._wait_until(lambda: bool(results), timeout=4.0))
        self.assertEqual(results[0][0:2], ("move_to_pose", False))
        self.assertIn("timeout aguardando resultado", results[0][2])
        self.assertEqual(bridge._action_requests, {})
        self.assertTrue(any("timeout aguardando resultado" in msg for _, msg in logs))

    def test_rejected_goal_is_reported(self) -> None:
        self._start_mock("--reject-action", "move_to_pose")
        bridge = self._start_bridge()
        results: list[tuple[str, bool, str]] = []
        bridge.action_result.connect(lambda name, ok, msg: results.append((name, ok, msg)))

        self.assertTrue(bridge.move_to_pose(make_pose(0.2, 0.0, 0.3), 0.2))
        self.assertTrue(self._wait_until(lambda: bool(results), timeout=3.0))
        self.assertEqual(results[0][0:2], ("move_to_pose", False))
        self.assertIn("goal rejeitado", results[0][2])
        self.assertEqual(bridge._action_requests, {})

    def test_disconnect_during_motion_enters_guided_recovery(self) -> None:
        self._start_mock()
        bridge = self._start_bridge()
        tab = MotionTab(bridge)
        bridge.arm_alive.connect(
            lambda alive: None
            if alive
            else tab.handle_robot_unavailable(
                "Execução interrompida: heartbeat de arm_status perdido."
            )
        )
        tab.set_robot_ready(True)
        tab.load_program(
            Program(
                name="disconnect_during_motion",
                steps=[
                    MoveStep(
                        label="movimento longo",
                        pose=pose(0.3, 0.0, 0.4),
                        duration=5.0,
                    )
                ],
            )
        )
        tab._chk_dry_run.setChecked(False)
        tab._dry_run = False

        original_question = QMessageBox.question
        QMessageBox.question = lambda *args, **kwargs: QMessageBox.Yes
        try:
            tab._run()
            self.assertTrue(self._wait_until(lambda: tab._running, timeout=2.0))
            self._stop_mock()
            self.assertTrue(
                self._wait_until(
                    lambda: tab._running and tab._paused and tab._recovery_required,
                    timeout=5.0,
                ),
                "programa não entrou em recuperação guiada após a perda do nó remoto",
            )
        finally:
            QMessageBox.question = original_question
            tab.close()

        status = tab._lbl_status.text().lower()
        # A disconnect during motion can surface in the status via either the
        # heartbeat-loss handler ("heartbeat") or the running step failing
        # because the action server vanished ("indisponível"). Both are valid
        # symptoms of the remote node being lost; the guided-recovery
        # assertions above already confirm the recovery state was entered.
        self.assertTrue(
            "heartbeat" in status or "indispon" in status,
            f"status não reflete a perda do nó remoto: {status!r}",
        )
        self.assertEqual(bridge._action_requests, {})
        self.assertFalse(bridge.motion_readiness()[0])
        self.assertTrue(tab._recovery_store.load() is not None)

    def test_invalid_telemetry_is_rejected_and_logged(self) -> None:
        self._start_mock("--invalid-telemetry-after", "2.0")
        bridge = self._start_bridge()
        logs: list[tuple[str, str]] = []
        bridge.log_message.connect(lambda level, msg: logs.append((level, msg)))

        self.assertTrue(
            self._wait_until(
                lambda: bridge.node is not None and bridge.node.joint_state is not None,
                timeout=3.0,
            )
        )
        self.assertTrue(
            self._wait_until(
                lambda: any("telemetria inválida" in msg for _, msg in logs),
                timeout=5.0,
            ),
            "telemetria malformada não foi sinalizada",
        )
        cached = bridge.node.joint_state if bridge.node is not None else None
        self.assertIsNotNone(cached)
        self.assertEqual(len(cached.name), len(cached.position))
        self.assertTrue(all(value == value for value in cached.position))
        self.assertTrue(any(level == "error" for level, _ in logs))

    def test_stop_while_goal_acceptance_is_late_cancels_late_goal(self) -> None:
        self._start_mock(
            "--late-accept-action",
            "move_to_pose",
            "--accept-delay",
            "1.2",
        )
        bridge = self._start_bridge()
        bridge.action_accept_timeout_s = 3.0
        results: list[tuple[str, bool, str]] = []
        logs: list[tuple[str, str]] = []
        bridge.action_result.connect(lambda name, ok, msg: results.append((name, ok, msg)))
        bridge.log_message.connect(lambda level, msg: logs.append((level, msg)))

        self.assertTrue(bridge.move_to_pose(make_pose(0.2, 0.0, 0.3), 2.0))
        self.assertTrue(self._wait_until(lambda: bool(bridge._action_requests), timeout=1.0))
        self.assertEqual(bridge.cancel_active_actions({"move_to_pose"}), 1)
        self.assertTrue(
            self._wait_until(
                lambda: any("goal aceito após parada" in msg for _, msg in logs),
                timeout=4.0,
            ),
            "goal aceito tardiamente não recebeu cancelamento imediato",
        )
        self.assertEqual(bridge._action_requests, {})
        self.assertEqual(results, [])


if __name__ == "__main__":
    unittest.main()
