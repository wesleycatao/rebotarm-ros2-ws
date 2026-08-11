# ruff: noqa: F401
"""Shared imports, constants and helpers for MotionTab mixins.

All mixin modules import from here via ``from ._common import *`` so that
method bodies can reference the same names that were available in the
original monolithic ``motion_tab.py``.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import time
from pathlib import Path
from typing import Any, Callable, Optional

import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt  # noqa: E402

from PySide6.QtCore import QSize, Qt, QTimer, Signal  # noqa: E402
from PySide6.QtGui import QKeySequence, QShortcut  # noqa: E402
from PySide6.QtWidgets import (  # noqa: E402
    QButtonGroup,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMenu,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from ...audit import ExecutionAuditLog, new_run_id, utc_now_iso  # noqa: E402
from ...motion import (  # noqa: E402
    ACTION_OPS,
    EDGE_FALLING,
    EDGE_LEVEL,
    EDGE_RISING,
    KIND_IF,
    OP_BRANCH,
    OP_GOTO,
    OP_GRIPPER,
    OP_LABEL,
    OP_LOOP_BACK,
    OP_LOOP_INIT,
    OP_MOVE,
    OP_OUTPUT,
    OP_WAIT,
    OP_WAIT_SIG,
    OP_WAIT_SM,
    OUTPUT_OFF,
    OUTPUT_TOGGLE,
    MoveStep,
    Program,
    RECIPE_RELEASED,
    build_label_index,
    evaluate,
    quat_to_rpy,
    rpy_to_quat,
    scale_duration,
)
from ...motion.recovery import RecoveryCheckpoint, RecoveryStore  # noqa: E402
from ...motion.dynamics import (  # noqa: E402
    DynamicsAnalysisReport,
    limits_signature,
    run_dynamic_analysis,
)
from ...motion.cell_scene import (  # noqa: E402
    CellSceneStore,
    ClearanceReport,
    analyze_program_tcp_clearance,
)
from ...motion.preflight import (  # noqa: E402
    REACHABILITY_DEFAULT_TIMEOUT_MS,
    PreflightReport,
    run_preflight,
)
from ...motion.program_validator import (  # noqa: E402
    ProgramValidationResult,
    validate_program,
)
from ...motion.vision_envelope import (  # noqa: E402
    VISION_ENVELOPE_ANALYSIS_KEY,
    bind_calibration_error,
    envelope_spec,
    envelope_variants,
)
from ...ros_bridge import make_pose  # noqa: E402
from ...security import CAP_EDIT_RECIPE, CAP_RELEASE_RECIPE  # noqa: E402
from ...icons import load_icon  # noqa: E402
from ...widgets.layout_utils import fit_button_text  # noqa: E402
from ...widgets.program_flow_widget import ProgramFlowWidget  # noqa: E402
from ...widgets.step_list_editor import StepListEditor  # noqa: E402
from ...widgets.preflight_dialog import PreflightDialog  # noqa: E402
from ...widgets.trajectory_preview_dialog import TrajectoryPreviewDialog  # noqa: E402

VM_ITER_CAP = 200_000
POLL_INTERVAL_MS = 100
RECENT_PROGRAMS_MAX = 8

_INVALID_GENERIC_STEP_IDS = {"", "step", "etapa", "nova_etapa"}


def _config_dir() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME")
    root = Path(base) if base else Path.home() / ".config"
    return root / "rebotarm_gui"


def _preload_path() -> Path:
    return _config_dir() / "motion_program.json"


def _dict_to_pose(d: dict):
    qx, qy, qz, qw = rpy_to_quat(float(d["roll"]), float(d["pitch"]), float(d["yaw"]))
    return make_pose(d["x"], d["y"], d["z"], qx, qy, qz, qw)


def _pose_msg_to_dict(msg) -> Optional[dict]:
    """Convert a geometry_msgs/Pose to a PoseDict (x, y, z, roll, pitch, yaw)."""
    if msg is None:
        return None
    try:
        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        qw = float(msg.orientation.w)
        roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)
        return {
            "x": float(msg.position.x),
            "y": float(msg.position.y),
            "z": float(msg.position.z),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }
    except Exception:
        return None
