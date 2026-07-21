"""Tab pages for the reBotArm GUI."""

from .diagnostics_tab import DiagnosticsTab
from .gripper_tab import GripperTab
from .grasp_tab import GraspTab
from .io_tab import IoTab
from .joints_tab import JointsTab
from .mit_tab import MitTab
from .motion_tab import MotionTab
from .pose_tab import PoseTab
from .robot_3d_tab import Robot3dTab
from .safety_tab import SafetyTab
from .settings_tab import SettingsTab
from .system_tab import SystemTab
from .telemetry_tab import TelemetryTab
from .vision_tab import VisionTab

__all__ = [
    "DiagnosticsTab",
    "GripperTab",
    "GraspTab",
    "IoTab",
    "JointsTab",
    "MitTab",
    "MotionTab",
    "PoseTab",
    "Robot3dTab",
    "SafetyTab",
    "SettingsTab",
    "SystemTab",
    "TelemetryTab",
    "VisionTab",
]
