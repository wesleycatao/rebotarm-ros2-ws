"""Tab pages for the reBotArm GUI."""

from .gripper_tab import GripperTab
from .joints_tab import JointsTab
from .mit_tab import MitTab
from .pose_tab import PoseTab
from .system_tab import SystemTab
from .telemetry_tab import TelemetryTab

__all__ = [
    "GripperTab",
    "JointsTab",
    "MitTab",
    "PoseTab",
    "SystemTab",
    "TelemetryTab",
]