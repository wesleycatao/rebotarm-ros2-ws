"""Teste de integração GUI + mock do braço.

Sobe o nó mock_rebotarm em subprocesso, instancia a aba Planejamento
(MotionTab) com um RosBridge real e executa um pequeno programa para
verificar se a interface responde corretamente:

- execução em modo simulação (dry-run)
- execução real contra o mock (move + gripper + I/O + wait)
- destaque de etapa, barra de progresso e status final

Uso manual:
    cd ~/ros2_ws
    source install/setup.bash
    QT_QPA_PLATFORM=offscreen python3 -m pytest src/rebotarm_gui/test/test_mock_integration.py -v

Ou:
    python3 -m unittest src.rebotarm_gui.test.test_mock_integration
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
import unittest
from pathlib import Path

# Permite importar rebotarm_gui a partir do source tree quando o teste é
# executado diretamente (o install/setup.bash já deve estar no PYTHONPATH).
WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication, QMessageBox

from rebotarm_gui.motion.program import (
    GripperStep,
    MoveStep,
    OutputStep,
    Program,
    WaitStep,
    pose,
)
from rebotarm_gui.ros_bridge import RosBridge
from rebotarm_gui.tabs.motion_tab import MotionTab


class MockIntegrationTest(unittest.TestCase):
    """Simula sequências de comandos contra o mock do braço."""

    _mock_proc: subprocess.Popen | None = None
    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        # Força renderização offscreen para rodar sem display.
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

        # Garante uma única QApplication no processo de teste.
        cls._app = QApplication.instance() or QApplication(sys.argv)

        # Sobe o mock em subprocesso isolado.
        cmd = [sys.executable, "-m", "rebotarm_gui.mock_arm"]
        cls._mock_proc = subprocess.Popen(
            cmd,
            cwd=str(WORKSPACE),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        # Aguarda o mock publicar estado (tipicamente < 1 s).
        time.sleep(2.5)
        if cls._mock_proc.poll() is not None:
            out = cls._mock_proc.stdout.read() if cls._mock_proc.stdout else ""
            raise RuntimeError(f"mock_arm morreu durante inicialização:\n{out}")

    @classmethod
    def tearDownClass(cls) -> None:
        if cls._mock_proc is not None:
            cls._mock_proc.terminate()
            try:
                cls._mock_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                cls._mock_proc.kill()
                cls._mock_proc.wait(timeout=5)
        if cls._app is not None:
            cls._app.quit()

    def _build_program(self) -> Program:
        """Retorna programa curto: move -> gripper -> wait -> output."""
        return Program(
            name="mock_test_sequence",
            speed_scale=2.0,
            steps=[
                MoveStep(label="aproxima", pose=pose(0.30, -0.10, 0.40), duration=0.3),
                MoveStep(label="desce", pose=pose(0.30, -0.10, 0.25), duration=0.3),
                GripperStep(label="fecha_gripper", position_mm=20.0, max_effort=5.0, timeout=1.0),
                WaitStep(label="aguarda", time=0.2),
                OutputStep(label="liga_saida", io_id="OUT1", action="on"),
                MoveStep(label="sobe", pose=pose(0.30, -0.10, 0.40), duration=0.3),
                OutputStep(label="desliga_saida", io_id="OUT1", action="off"),
                GripperStep(label="abre_gripper", position_mm=80.0, max_effort=5.0, timeout=1.0),
            ],
        )

    def _wait_run_finished(
        self, tab: MotionTab, progress_samples: list[int], timeout: float = 30.0
    ) -> tuple[bool, str]:
        """Aguarda _running == False e retorna (ok, status_text)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and tab._running:
            progress_samples.append(tab._progress.value())
            self._app.processEvents()
            time.sleep(0.05)
        progress_samples.append(tab._progress.value())
        self._app.processEvents()
        text = tab._lbl_status.text()
        ok = not tab._running and "interrompida" not in text.lower() and "erro" not in text.lower()
        return ok, text

    def _run_program(self, dry_run: bool) -> tuple[bool, str, int]:
        bridge = RosBridge(namespace="rebotarm")
        bridge.start()
        try:
            tab = MotionTab(bridge)
            tab.show()

            # Aguarda a bridge receber arm_status habilitado.
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                self._app.processEvents()
                if bridge.is_arm_enabled or dry_run:
                    if bridge.state_machine:
                        break
                time.sleep(0.05)

            # Carrega programa.
            tab.load_program(self._build_program(), "Programa de teste carregado.")
            self._app.processEvents()

            # Configura modo.
            tab._chk_dry_run.setChecked(dry_run)
            tab._dry_run = dry_run
            tab._chk_step.setChecked(False)
            tab._step_by_step = False

            # Monkeypatch no diálogo de confirmação para não bloquear o teste.
            original_question = QMessageBox.question
            QMessageBox.question = lambda *args, **kwargs: QMessageBox.Yes

            progress_samples: list[int] = []
            try:
                tab._run()
                self._app.processEvents()
                ok, status = self._wait_run_finished(tab, progress_samples)
            finally:
                QMessageBox.question = original_question

            max_progress = max(progress_samples) if progress_samples else 0
            return ok and max_progress >= 95, status, max_progress
        finally:
            bridge.shutdown()

    def test_dry_run_sequence(self) -> None:
        """Executa o programa em modo simulação (dry-run)."""
        ok, status, progress = self._run_program(dry_run=True)
        self.assertTrue(ok, f"Dry-run falhou ou não completou: {status} (progresso={progress}%)")
        print(f"[DRY-RUN OK] status='{status}' progresso={progress}%")

    def test_ros_diagnostics_snapshot(self) -> None:
        """Métricas e descoberta refletem as interfaces publicadas pelo mock."""
        bridge = RosBridge(namespace="rebotarm")
        bridge.start()
        try:
            deadline = time.monotonic() + 10.0
            snapshot = bridge.diagnostic_snapshot()
            while time.monotonic() < deadline:
                self._app.processEvents()
                snapshot = bridge.diagnostic_snapshot()
                heartbeat = snapshot["heartbeat"]
                if heartbeat["alive"] and heartbeat["hz"] is not None:
                    break
                time.sleep(0.05)

            self.assertTrue(snapshot["connected"])
            self.assertTrue(snapshot["heartbeat"]["alive"])
            self.assertGreater(float(snapshot["heartbeat"]["hz"]), 0.0)
            topics = {row["name"]: row for row in snapshot["topics"]}
            self.assertTrue(topics["/rebotarm/joint_states"]["received"])
            services = {row["name"]: row["ready"] for row in snapshot["services"]}
            actions = {row["name"]: row["ready"] for row in snapshot["actions"]}
            self.assertTrue(services["/rebotarm/enable"])
            self.assertTrue(actions["/rebotarm/move_to_pose"])
            self.assertTrue(actions["/rebotarm/follow_joint_trajectory"])
            self.assertTrue(actions["/rebotarm/gripper/command"])
        finally:
            bridge.shutdown()

    def test_real_run_sequence(self) -> None:
        """Executa o programa contra o mock real."""
        ok, status, progress = self._run_program(dry_run=False)
        self.assertTrue(ok, f"Execução real falhou: {status} (progresso={progress}%)")
        print(f"[REAL-RUN OK] status='{status}' progresso={progress}%")

    def test_joint_recording(self) -> None:
        """Executa com gravação de juntas habilitada e verifica o histórico."""
        bridge = RosBridge(namespace="rebotarm")
        bridge.start()
        try:
            tab = MotionTab(bridge)
            tab.show()

            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                self._app.processEvents()
                if bridge.is_arm_enabled and bridge.state_machine:
                    break
                time.sleep(0.05)

            tab.load_program(self._build_program(), "Programa de teste carregado.")
            self._app.processEvents()
            tab._chk_dry_run.setChecked(False)
            tab._dry_run = False
            tab._chk_record.setChecked(True)

            original_question = QMessageBox.question
            QMessageBox.question = lambda *args, **kwargs: QMessageBox.Yes
            try:
                tab._run()
                self._app.processEvents()
                ok, _ = self._wait_run_finished(tab, [])
            finally:
                QMessageBox.question = original_question

            self.assertTrue(ok, "Execução com gravação falhou")
            self.assertTrue(len(tab._joint_history) > 5, "Histórico de juntas muito curto")
            self.assertTrue(tab._btn_plot_joints.isEnabled(), "Botão de plot não foi habilitado")
            print(f"[RECORDING OK] amostras={len(tab._joint_history)}")
        finally:
            bridge.shutdown()

    def test_joint_recording_dry_run(self) -> None:
        """Executa em dry-run com gravação e verifica que o plot é liberado."""
        bridge = RosBridge(namespace="rebotarm")
        bridge.start()
        try:
            tab = MotionTab(bridge)
            tab.show()

            # Dry-run não exige braço habilitado.
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                self._app.processEvents()
                if bridge.state_machine or bridge.is_connected:
                    break
                time.sleep(0.05)

            tab.load_program(self._build_program(), "Programa de teste carregado.")
            self._app.processEvents()
            tab._chk_dry_run.setChecked(True)
            tab._dry_run = True
            tab._chk_record.setChecked(True)

            original_question = QMessageBox.question
            QMessageBox.question = lambda *args, **kwargs: QMessageBox.Yes
            try:
                tab._run()
                self._app.processEvents()
                ok, _ = self._wait_run_finished(tab, [])
            finally:
                QMessageBox.question = original_question

            self.assertTrue(ok, "Dry-run com gravação falhou")
            self.assertTrue(len(tab._joint_history) > 3, "Histórico de juntas do dry-run está vazio")
            self.assertTrue(tab._btn_plot_joints.isEnabled(), "Botão de plot não foi habilitado no dry-run")
            print(f"[DRY-RUN RECORDING OK] amostras={len(tab._joint_history)}")
        finally:
            bridge.shutdown()


if __name__ == "__main__":
    unittest.main()
