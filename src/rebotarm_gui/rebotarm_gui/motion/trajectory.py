"""Trajectory geometry helpers for the motion-planning tab.

All functions are pure-Python (no numpy) and operate on plain tuples /
``Pose`` dicts. A ``Pose`` dict has the shape::

    {"x", "y", "z", "roll", "pitch", "yaw"}   # meters and radians

They return ordered lists of ``(pose_dict, duration)`` pairs ready to be
sent to the ``MoveToPose`` action one segment at a time.
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

# Public type aliases -------------------------------------------------
Vec3 = Tuple[float, float, float]
PoseDict = Dict[str, float]
Waypoint = Tuple[PoseDict, float]  # (pose, duration_seconds)

_MIN_DT = 0.25  # seconds, floor for a single MoveToPose segment
_MAX_DT = 60.0  # seconds, ceiling


def scale_duration(raw_dt: float, scale: float) -> float:
    """Apply a global speed scale to a single-segment duration.

    A scale > 1 speeds the motion up (shorter ``dt``); a scale < 1 slows it
    down (longer ``dt``). The result is clamped to the per-segment limits
    ``[_MIN_DT, _MAX_DT]`` so individual waypoints never violate the planner's
    own bounds, regardless of the global scale.
    """
    scale = max(float(scale), 1e-3)
    return min(max(float(raw_dt) / scale, _MIN_DT), _MAX_DT)


# ----------------------------------------------------------------------
# small vector helpers (tuples)
# ----------------------------------------------------------------------
def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a: Vec3) -> float:
    return math.sqrt(_dot(a, a))


def _normalize(a: Vec3) -> Vec3:
    n = _norm(a)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


# ----------------------------------------------------------------------
# quaternion helpers (qx, qy, qz, qw)
# ----------------------------------------------------------------------
def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert RPY (radians) into a quaternion (qx, qy, qz, qw)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def quat_from_matrix(c0: Vec3, c1: Vec3, c2: Vec3) -> Tuple[float, float, float, float]:
    """Quaternion from a rotation matrix given by its columns (axes)."""
    m00, m10, m20 = c0
    m01, m11, m21 = c1
    m02, m12, m22 = c2
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    return _normalize_quat((qx, qy, qz, qw))


def _normalize_quat(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


# ----------------------------------------------------------------------
# pose / waypoint helpers
# ----------------------------------------------------------------------
def _pose_xyz(p: PoseDict) -> Vec3:
    return (float(p["x"]), float(p["y"]), float(p["z"]))


def _pose_rpy(p: PoseDict) -> Tuple[float, float, float]:
    return float(p["roll"]), float(p["pitch"]), float(p["yaw"])


def _make_pose(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> PoseDict:
    return {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw}


def _clamp_dt(dt: float) -> float:
    return max(_MIN_DT, min(_MAX_DT, dt))


def _dt_for_distance(distance: float, speed: float) -> float:
    """Duration for a segment given length (m) and speed (m/s)."""
    speed = max(speed, 1e-6)
    return _clamp_dt(distance / speed)


# ----------------------------------------------------------------------
# 1) LINE  -- straight segment between two points, fixed orientation, speed
# ----------------------------------------------------------------------
def line_waypoints(
    start: PoseDict,
    end: PoseDict,
    speed: float,
    *,
    max_step: float = 0.02,
    orientation_from: str = "start",
) -> List[Waypoint]:
    """Cartesian waypoints along a straight line.

    ``orientation_from`` controls which pose supplies the RPY used along the
    whole segment: ``"start"`` or ``"end"``. The orientation is held constant.
    """
    a = _pose_xyz(start)
    b = _pose_xyz(end)
    delta = _sub(b, a)
    length = _norm(delta)
    if length < 1e-9:
        return [(_make_pose(*a, *_pose_rpy(start)), _clamp_dt(0.5))]

    n = max(1, int(math.ceil(length / max(max_step, 1e-6))))
    roll, pitch, yaw = _pose_rpy(end if orientation_from == "end" else start)

    out: List[Waypoint] = []
    for i in range(1, n + 1):
        t = i / n
        p = _add(a, _scale(delta, t))
        seg_len = length / n
        out.append((_make_pose(p[0], p[1], p[2], roll, pitch, yaw), _dt_for_distance(seg_len, speed)))
    return out


# ----------------------------------------------------------------------
# 2) ARC  -- circular arc through 3 points
# ----------------------------------------------------------------------
def _circumcenter(p1: Vec3, p2: Vec3, p3: Vec3) -> Tuple[Vec3, float, Vec3]:
    """Return (center, radius, normal) of the circle through 3 points."""
    u = _sub(p2, p1)
    v = _sub(p3, p1)
    n = _cross(u, v)
    if _norm(n) < 1e-9:
        raise ValueError("os 3 pontos do arco são colineares (não definem um círculo)")
    normal = _normalize(n)

    uu = _dot(u, u)
    vv = _dot(v, v)
    uv = _dot(u, v)
    det = 4.0 * (uu * vv - uv * uv)
    if abs(det) < 1e-12:
        raise ValueError("circunferência degenerada")

    # Solve the 2x2 system via Cramer's rule:
    #   2*uu*a + 2*uv*b = uu
    #   2*uv*a + 2*vv*b = vv
    a = (uu * (2.0 * vv) - vv * (2.0 * uv)) / det
    b = ((2.0 * uu) * vv - (2.0 * uv) * uu) / det

    center = _add(p1, _add(_scale(u, a), _scale(v, b)))
    radius = _norm(_sub(center, p1))
    return center, radius, normal


def _angle_of(point: Vec3, center: Vec3, e1: Vec3, e2: Vec3) -> float:
    d = _sub(point, center)
    return math.atan2(_dot(d, e2), _dot(d, e1))


def arc_waypoints(
    p1: PoseDict,
    p2: PoseDict,
    p3: PoseDict,
    speed: float,
    *,
    samples: int = 24,
    point_to_center: bool = False,
    fixed_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> List[Waypoint]:
    """Circular arc waypoints through p1 -> p2 -> p3.

    When ``point_to_center`` is True the tool Z-axis is aimed at the circle
    center along the whole arc (useful for welding torches). Otherwise a
    constant RPY (``fixed_rpy``) is applied.
    """
    a1 = _pose_xyz(p1)
    a2 = _pose_xyz(p2)
    a3 = _pose_xyz(p3)

    center, radius, normal = _circumcenter(a1, a2, a3)
    e1 = _normalize(_sub(a2, a1)) if _norm(_sub(a2, a1)) > 1e-9 else (1.0, 0.0, 0.0)
    e2 = _normalize(_cross(normal, e1))

    ang1 = _angle_of(a1, center, e1, e2)
    ang2 = _angle_of(a2, center, e1, e2)
    ang3 = _angle_of(a3, center, e1, e2)

    two_pi = 2.0 * math.pi
    pos3 = (ang3 - ang1) % two_pi
    pos2 = (ang2 - ang1) % two_pi
    eps = 1e-6
    if pos2 <= pos3 + eps:
        direction, sweep = 1.0, pos3
    else:
        direction, sweep = -1.0, two_pi - pos3

    samples = max(2, int(samples))
    arc_len = radius * sweep
    seg_len = arc_len / samples

    roll, pitch, yaw = fixed_rpy
    out: List[Waypoint] = []
    for i in range(1, samples + 1):
        t = i / samples
        ang = ang1 + direction * sweep * t
        radial = _add(_scale(e1, math.cos(ang)), _scale(e2, math.sin(ang)))
        pt = _add(center, _scale(radial, radius))
        if point_to_center:
            rpy = _rpy_pointing_to(pt, center, normal)
        else:
            rpy = (roll, pitch, yaw)
        out.append((_make_pose(pt[0], pt[1], pt[2], *rpy), _dt_for_distance(seg_len, speed)))
    return out


def _rpy_pointing_to(point: Vec3, center: Vec3, normal: Vec3) -> Tuple[float, float, float]:
    """RPY such that the tool +Z axis points from ``point`` to ``center``."""
    z_axis = _normalize(_sub(center, point))
    if _norm(z_axis) < 1e-9:
        z_axis = (0.0, 0.0, -1.0)
    x_axis = _normalize(_cross(normal, z_axis))
    if _norm(x_axis) < 1e-9:
        # z_axis parallel to plane normal; pick any perpendicular vector
        x_axis = _normalize(_cross(z_axis, (0.0, 0.0, 1.0)))
        if _norm(x_axis) < 1e-9:
            x_axis = (1.0, 0.0, 0.0)
    y_axis = _cross(z_axis, x_axis)
    qx, qy, qz, qw = quat_from_matrix(x_axis, y_axis, z_axis)
    return quat_to_rpy(qx, qy, qz, qw)


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Convert a quaternion to roll/pitch/yaw (radians)."""
    qx, qy, qz, qw = _normalize_quat((qx, qy, qz, qw))
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# ----------------------------------------------------------------------
# 3) PALLET  -- 3D palletizing grid with approach/retract
# ----------------------------------------------------------------------
def pallet_waypoints(
    corner: PoseDict,
    nx: int,
    ny: int,
    nz: int,
    dx: float,
    dy: float,
    dz: float,
    speed: float,
    *,
    approach_height: float = 0.05,
    snake: bool = True,
) -> List[Waypoint]:
    """Generate a 3D palletizing motion.

    For every cell ``(i, j, k)`` (col, row, layer) the sequence is:
    move to the cell *approach* pose (``approach_height`` above it) then
    descend to the cell *place* pose. Layers run outer-most so a layer is
    finished before climbing to the next one. ``snake`` zig-zags rows.
    """
    nx = max(1, int(nx))
    ny = max(1, int(ny))
    nz = max(1, int(nz))
    x0, y0, z0 = _pose_xyz(corner)
    roll, pitch, yaw = _pose_rpy(corner)

    out: List[Waypoint] = []
    for k in range(nz):
        for j in range(ny):
            row = j if (not snake or (k % 2 == 0)) else (ny - 1 - j)
            rev = snake and (j % 2 == 1)
            cols = range(nx - 1, -1, -1) if rev else range(nx)
            for i in cols:
                cx = x0 + i * dx
                cy = y0 + row * dy
                cz = z0 + k * dz
                approach = (cx, cy, cz + abs(approach_height))
                place = (cx, cy, cz)
                # travel to approach (horizontal move)
                out.append(
                    (_make_pose(approach[0], approach[1], approach[2], roll, pitch, yaw),
                     _dt_for_distance(abs(approach_height) + 0.02, speed))
                )
                # descend to place
                out.append(
                    (_make_pose(place[0], place[1], place[2], roll, pitch, yaw),
                     _dt_for_distance(abs(approach_height), speed))
                )
    return out
