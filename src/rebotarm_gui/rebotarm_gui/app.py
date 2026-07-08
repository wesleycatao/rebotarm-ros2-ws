"""Main application window for the reBotArm GUI."""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtGui import QIcon
from PySide6.QtWidgets import (
    QApplication,
    QDockWidget,
    QLabel,
    QMainWindow,
    QMessageBox,
    QStatusBar,
    QTabWidget,
    QTextEdit,
    QWidget,
)

from .ros_bridge import RosBridge
from .tabs import (
    GripperTab,
    JointsTab,
    MitTab,
    PoseTab,
    SystemTab,
    TelemetryTab,
)
from .widgets.status_led import StatusLed


class MainWindow(QMainWindow):
    """Top-level window hosting the tabs and a ROS log dock."""

    def __init__(self, namespace: str = "rebotarm") -> None:
        super().__init__()
        self.setWindowTitle("reBotArm Controller - ROS 2 GUI")
        self.setMinimumSize(1280, 720)
        self.resize(1600, 900)

        # ---- ROS bridge ----
        self._bridge = RosBridge(namespace=namespace)

        # ---- Central tabs ----
        self._tabs = QTabWidget()
        self._tabs.setDocumentMode(True)
        self._tabs.tabBar().setUsesScrollButtons(False)
        self._tab_system = SystemTab(self._bridge)
        self._tab_joints = JointsTab(self._bridge)
        self._tab_pose = PoseTab(self._bridge)
        self._tab_gripper = GripperTab(self._bridge)
        self._tab_telemetry = TelemetryTab(self._bridge)
        self._tab_mit = MitTab(self._bridge)

        self._tabs.addTab(self._tab_system, "Sistema")
        self._tabs.addTab(self._tab_joints, "Juntas")
        self._tabs.addTab(self._tab_pose, "Pose")
        self._tabs.addTab(self._tab_gripper, "Gripper")
        self._tabs.addTab(self._tab_telemetry, "Telemetria")
        self._tabs.addTab(self._tab_mit, "MIT / Raw")

        self.setCentralWidget(self._tabs)

        # ---- Status bar ----
        self._led = StatusLed("amber", 18)
        self._lbl_ns = QLabel(f"/{namespace}")
        self._lbl_ns.setObjectName("MutedLabel")
        bar = QStatusBar()
        bar.addWidget(self._led)
        bar.addWidget(QLabel("ROS:"))
        bar.addWidget(self._lbl_ns)
        self.setStatusBar(bar)

        # ---- Log dock ----
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumHeight(220)
        dock = QDockWidget("Eventos ROS", self)
        dock.setWidget(self._log)
        dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)

        # ---- Signals ----
        b = self._bridge
        b.connected.connect(self._on_connected)
        b.joint_state_changed.connect(self._on_joint_state)
        b.arm_status_changed.connect(self._on_arm_status)
        b.gripper_state_changed.connect(self._on_gripper_state)
        b.joint_detail_changed.connect(self._on_joint_detail)
        b.service_result.connect(self._on_service_result)
        b.action_result.connect(self._on_action_result)
        b.log_message.connect(self._on_log_message)

        self._bridge.start()
        self._append_log(f"GUI iniciada — namespace /{namespace}")

    # ------------------------------------------------------------------
    # Bridge signal handlers
    # ------------------------------------------------------------------
    def _on_connected(self, ok: bool) -> None:
        self._led.set_color("green" if ok else "amber")
        self._append_log("Conectado ao ROS 2." if ok else "Aguardando ROS 2...")

    def _on_joint_state(self, msg) -> None:
        self._tab_joints.update_joint_state(msg)
        self._tab_telemetry.update_joint_state(msg)
        self._tab_mit.update_joint_state(msg)
        self._tab_pose.update_joint_state(msg)

    def _on_arm_status(self, msg) -> None:
        self._tab_system.update_arm_status(msg)

    def _on_gripper_state(self, msg) -> None:
        self._tab_gripper.update_gripper_state(msg)

    def _on_joint_detail(self, joint_name: str, msg) -> None:
        self._tab_telemetry.update_joint_detail(joint_name, msg)

    def _on_service_result(self, name: str, ok: bool, message: str) -> None:
        tag = "OK" if ok else "ERRO"
        self._append_log(f"[svc:{name}] {tag}: {message}")

    def _on_action_result(self, name: str, ok: bool, message: str) -> None:
        tag = "OK" if ok else "ERRO"
        self._append_log(f"[action:{name}] {tag}: {message}")

    def _on_log_message(self, level: str, text: str) -> None:
        self._append_log(f"[{level}] {text}")

    def _append_log(self, text: str) -> None:
        self._log.append(text)

    # ------------------------------------------------------------------
    def closeEvent(self, event) -> None:  # noqa: N802 (Qt naming)
        try:
            self._bridge.shutdown()
        finally:
            super().closeEvent(event)


def load_stylesheet() -> str:
    """Load the dark QSS installed alongside the package."""
    here = Path(__file__).resolve().parent
    qss = here / "styles" / "dark.qss"
    if qss.is_file():
        return qss.read_text(encoding="utf-8")
    return ""


def run(namespace: str = "rebotarm") -> int:
    """Application entry point."""
    app = QApplication.instance() or QApplication([])
    app.setApplicationName("rebotarm_gui")
    style = load_stylesheet()
    if style:
        app.setStyleSheet(style)
    win = MainWindow(namespace=namespace)
    win.show()
    return app.exec()
