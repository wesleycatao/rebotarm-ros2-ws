"""Testes da aba Robô 3D (Robot3dTab).

Verifica que a aba instancia sem bridge iniciada e que o slot
``update_joint_state`` não explode quando ``/robot_description`` está
indisponível (caso do mock_arm), mantendo a mensagem de espera visível.

Uso:
    cd ~/ros2_ws
    QT_QPA_PLATFORM=offscreen python3 -m pytest src/rebotarm_gui/test/test_robot_3d_tab.py -v
"""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

# Permite importar rebotarm_gui a partir do source tree (mesmo padrão dos
# demais testes do pacote).
WORKSPACE = Path(__file__).resolve().parents[3]
SRC_PKG = WORKSPACE / "src" / "rebotarm_gui"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from rebotarm_gui.ros_bridge import RosBridge  # noqa: E402
from rebotarm_gui.tabs.robot_3d_tab import Robot3dTab  # noqa: E402


class TestRobot3dTab(unittest.TestCase):
    """A aba abre sem URDF e processa joint states sem exceções."""

    _app: QApplication | None = None

    @classmethod
    def setUpClass(cls) -> None:
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self) -> None:
        self.bridge = RosBridge()  # não iniciado: node é None, sem URDF
        self.tab = Robot3dTab(self.bridge)
        self.tab.show()
        self._app.processEvents()

    def tearDown(self) -> None:
        self.tab.close()
        self.tab.deleteLater()
        self._app.processEvents()

    def test_tcp_label_exists(self) -> None:
        self.assertIsNotNone(self.tab._lbl_tcp)
        self.assertIn("TCP", self.tab._lbl_tcp.text())

    def test_update_joint_state_without_urdf_shows_waiting_message(self) -> None:
        msg = SimpleNamespace(
            name=["joint_1", "joint_2"],
            position=[0.1, -0.2],
            velocity=[0.0, 0.0],
            effort=[0.0, 0.0],
        )
        # Não pode lançar exceção mesmo sem /robot_description.
        self.tab.update_joint_state(msg)
        self._app.processEvents()
        self.assertEqual(self.tab._view._message, "Aguardando /robot_description...")
        self.assertIn("---", self.tab._lbl_tcp.text())

    def test_reset_view_button_restores_camera(self) -> None:
        self.tab._view._yaw = 2.0
        self.tab._view._zoom = 3.0
        self.tab._btn_reset_view.click()
        self.assertAlmostEqual(self.tab._view._yaw, self.tab._view._DEFAULT_YAW)
        self.assertAlmostEqual(self.tab._view._zoom, self.tab._view._DEFAULT_ZOOM)


if __name__ == "__main__":
    unittest.main()
