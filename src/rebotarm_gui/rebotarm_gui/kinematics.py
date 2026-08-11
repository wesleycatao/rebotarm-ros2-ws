"""URDF parsing and forward kinematics for the reBotArm GUI.

Self-contained (stdlib + numpy) helpers used by the manual-motion tabs and
the 3D visualization page:

- :func:`joint_limits` extracts per-joint position limits from a URDF string;
- :func:`load_chain` builds the ordered joint chain between two links;
- :func:`fk` computes the 4x4 transform of every frame along the chain for a
  given joint configuration, which is enough to draw the arm skeleton and to
  report the TCP pose without depending on tf2 or MoveIt.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

Vec3 = Tuple[float, float, float]


@dataclass
class ChainJoint:
    """One URDF joint in a kinematic chain."""

    name: str
    kind: str  # "revolute" | "prismatic" | "fixed" | others (treated as fixed)
    parent_link: str
    child_link: str
    origin_xyz: Vec3
    origin_rpy: Vec3
    axis: Vec3


def _floats(text: Optional[str], default: Vec3) -> Vec3:
    if not text:
        return default
    parts = [float(v) for v in text.split()]
    if len(parts) != 3:
        return default
    return (parts[0], parts[1], parts[2])


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis RPY -> rotation matrix (R = Rz(yaw) @ Ry(pitch) @ Rx(roll))."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _axis_matrix(axis: Vec3, angle: float) -> np.ndarray:
    """Rodrigues rotation of ``angle`` rad around unit ``axis``."""
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / norm, y / norm, z / norm
    c, s = math.cos(angle), math.sin(angle)
    c1 = 1.0 - c
    return np.array(
        [
            [c + x * x * c1, x * y * c1 - z * s, x * z * c1 + y * s],
            [y * x * c1 + z * s, c + y * y * c1, y * z * c1 - x * s],
            [z * x * c1 - y * s, z * y * c1 + x * s, c + z * z * c1],
        ]
    )


def _origin_transform(joint: ChainJoint) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = _rpy_matrix(*joint.origin_rpy)
    t[:3, 3] = joint.origin_xyz
    return t


def _motion_transform(joint: ChainJoint, q: float) -> np.ndarray:
    t = np.eye(4)
    if joint.kind == "revolute" or joint.kind == "continuous":
        t[:3, :3] = _axis_matrix(joint.axis, q)
    elif joint.kind == "prismatic":
        ax = np.array(joint.axis, dtype=float)
        norm = float(np.linalg.norm(ax)) or 1.0
        t[:3, 3] = ax / norm * q
    return t


def _parse_joints(urdf_text: str) -> List[ChainJoint]:
    root = ET.fromstring(urdf_text)
    joints: List[ChainJoint] = []
    for j in root.iter("joint"):
        parent = j.find("parent")
        child = j.find("child")
        if parent is None or child is None:
            continue
        origin = j.find("origin")
        axis = j.find("axis")
        joints.append(
            ChainJoint(
                name=j.get("name", ""),
                kind=j.get("type", "fixed"),
                parent_link=parent.get("link", ""),
                child_link=child.get("link", ""),
                origin_xyz=_floats(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0)),
                origin_rpy=_floats(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0)),
                axis=_floats(axis.get("xyz") if axis is not None else None, (0.0, 0.0, 1.0)),
            )
        )
    return joints


def joint_limits(urdf_text: str) -> Dict[str, Tuple[float, float]]:
    """Return ``{joint_name: (lower, upper)}`` for revolute/prismatic joints."""
    limits: Dict[str, Tuple[float, float]] = {}
    root = ET.fromstring(urdf_text)
    for j in root.iter("joint"):
        if j.get("type") not in ("revolute", "prismatic"):
            continue
        lim = j.find("limit")
        if lim is None:
            continue
        try:
            lower = float(lim.get("lower", "0"))
            upper = float(lim.get("upper", "0"))
        except ValueError:
            continue
        limits[j.get("name", "")] = (lower, upper)
    return limits


def load_chain(
    urdf_text: str,
    root_link: str = "base_link",
    tip_link: str = "gripper_link",
) -> List[ChainJoint]:
    """Ordered joints from ``root_link`` to ``tip_link`` (empty if not connected)."""
    joints = _parse_joints(urdf_text)
    by_child = {j.child_link: j for j in joints}
    chain: List[ChainJoint] = []
    link = tip_link
    seen = set()
    while link != root_link and link in by_child and link not in seen:
        seen.add(link)
        joint = by_child[link]
        chain.append(joint)
        link = joint.parent_link
    if link != root_link:
        return []
    chain.reverse()
    return chain


def fk(chain: List[ChainJoint], q: Dict[str, float]) -> List[np.ndarray]:
    """4x4 transform of every frame along ``chain`` for configuration ``q``.

    The returned list has ``len(chain) + 1`` entries: index 0 is the root link
    frame (identity) and index ``i + 1`` is the child-link frame of
    ``chain[i]`` after applying the joint motion.
    """
    transforms = [np.eye(4)]
    current = np.eye(4)
    for joint in chain:
        current = current @ _origin_transform(joint) @ _motion_transform(
            joint, float(q.get(joint.name, 0.0))
        )
        transforms.append(current)
    return transforms


def tcp_transform(chain: List[ChainJoint], q: Dict[str, float]) -> np.ndarray:
    """4x4 transform of the tip frame for configuration ``q``."""
    return fk(chain, q)[-1]


def rotation_to_quaternion(rot: np.ndarray) -> Tuple[float, float, float, float]:
    """3x3 rotation matrix -> ``(x, y, z, w)`` quaternion."""
    m = rot
    trace = float(m[0, 0] + m[1, 1] + m[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
            0.25 * s,
        )
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return (
            0.25 * s,
            (m[0, 1] + m[1, 0]) / s,
            (m[0, 2] + m[2, 0]) / s,
            (m[2, 1] - m[1, 2]) / s,
        )
    if i == 1:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return (
            (m[0, 1] + m[1, 0]) / s,
            0.25 * s,
            (m[1, 2] + m[2, 1]) / s,
            (m[0, 2] - m[2, 0]) / s,
        )
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return (
        (m[0, 2] + m[2, 0]) / s,
        (m[1, 2] + m[2, 1]) / s,
        0.25 * s,
        (m[1, 0] - m[0, 1]) / s,
    )
