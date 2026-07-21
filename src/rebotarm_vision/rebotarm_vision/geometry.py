from __future__ import annotations

import math
from typing import Iterable

from geometry_msgs.msg import Quaternion


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_to_quaternion(yaw: float, roll: float = 0.0, pitch: float = 0.0) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    quat = Quaternion()
    quat.w = cr * cp * cy + sr * sp * sy
    quat.x = sr * cp * cy - cr * sp * sy
    quat.y = cr * sp * cy + sr * cp * sy
    quat.z = cr * cp * sy - sr * sp * cy
    return quat


def matrix3(values: Iterable[float]) -> tuple[float, ...] | None:
    data = tuple(float(value) for value in values)
    return data if len(data) == 9 else None


def apply_homography(h: tuple[float, ...], u: float, v: float) -> tuple[float, float] | None:
    denom = h[6] * u + h[7] * v + h[8]
    if abs(denom) < 1e-9:
        return None
    x = (h[0] * u + h[1] * v + h[2]) / denom
    y = (h[3] * u + h[4] * v + h[5]) / denom
    return x, y


def stable_id(prefix: str, index: int, class_name: str, u: float, v: float) -> str:
    return f"{prefix}_{index:02d}_{class_name}_{int(round(u))}_{int(round(v))}"


def normalize_angle(angle: float) -> float:
    """Normalize an angle to the range (-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return float(angle)


def rotated_bbox_axes(
    angle_rad: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return (long_axis, short_axis) unit vectors for a rotated rectangle.

    long_axis points along the rectangle's first side (angle_rad);
    short_axis is perpendicular.
    """
    long_axis = (math.cos(angle_rad), math.sin(angle_rad))
    short_axis = (-math.sin(angle_rad), math.cos(angle_rad))
    return long_axis, short_axis


def local_to_image_point(
    center: tuple[float, float],
    long_axis: tuple[float, float],
    short_axis: tuple[float, float],
    half_long: float,
    half_short: float,
    local_point: tuple[float, float],
) -> tuple[float, float]:
    """Map a point from the object's local [-1,1]x[-1,1] frame to image pixels."""
    x_local, y_local = local_point
    return (
        center[0]
        + x_local * long_axis[0] * half_long
        + y_local * short_axis[0] * half_short,
        center[1]
        + x_local * long_axis[1] * half_long
        + y_local * short_axis[1] * half_short,
    )


def polygon_area(points: list[tuple[float, float]]) -> float:
    """Shoelace formula for a closed polygon."""
    if len(points) < 3:
        return 0.0
    area = 0.0
    for i in range(len(points)):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % len(points)]
        area += x1 * y2 - x2 * y1
    return abs(area) * 0.5


def shrink_polygon(
    points: list[tuple[float, float]],
    inset_x: float,
    inset_y: float,
    center: tuple[float, float],
    long_axis: tuple[float, float],
    short_axis: tuple[float, float],
) -> list[tuple[float, float]]:
    """Shrink a polygon along local axes by given offsets."""
    if len(points) < 3:
        return []
    cx, cy = center
    result: list[tuple[float, float]] = []
    for px, py in points:
        dx = px - cx
        dy = py - cy
        long_proj = dx * long_axis[0] + dy * long_axis[1]
        short_proj = dx * short_axis[0] + dy * short_axis[1]
        new_long = math.copysign(max(0.0, abs(long_proj) - inset_x), long_proj)
        new_short = math.copysign(max(0.0, abs(short_proj) - inset_y), short_proj)
        result.append(
            (
                cx + new_long * long_axis[0] + new_short * short_axis[0],
                cy + new_long * long_axis[1] + new_short * short_axis[1],
            )
        )
    return result


def rotated_iou(
    box_a: tuple[tuple[float, float], tuple[float, float], float, float, float],
    box_b: tuple[tuple[float, float], tuple[float, float], float, float, float],
) -> float:
    """Compute IoU between two rotated bounding boxes.

    Each box is defined by (center, axes, half_long, half_short, angle_rad).
    This is a Monte-Carlo approximation sufficient for merge decisions.
    """
    try:
        import cv2
        import numpy as np
    except Exception:
        return 0.0

    (ca, ax_a, hl_a, hs_a, _angle_a) = box_a
    (cb, ax_b, hl_b, hs_b, _angle_b) = box_b
    short_a = (-ax_a[1], ax_a[0])
    short_b = (-ax_b[1], ax_b[0])

    def corners(center, long_axis, short_axis, half_long, half_short):
        pts = []
        for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
            pts.append(
                (
                    center[0] + sx * long_axis[0] * half_long + sy * short_axis[0] * half_short,
                    center[1] + sx * long_axis[1] * half_long + sy * short_axis[1] * half_short,
                )
            )
        return pts

    poly_a = np.array(corners(ca, ax_a, short_a, hl_a, hs_a), dtype=np.int32)
    poly_b = np.array(corners(cb, ax_b, short_b, hl_b, hs_b), dtype=np.int32)

    # Bounding box of both polygons for the raster grid.
    all_pts = np.vstack([poly_a, poly_b])
    x0, y0 = all_pts.min(axis=0)
    x1, y1 = all_pts.max(axis=0)
    w = max(1, int(x1 - x0) + 1)
    h = max(1, int(y1 - y0) + 1)

    mask_a = np.zeros((h, w), dtype=np.uint8)
    mask_b = np.zeros((h, w), dtype=np.uint8)
    shifted_a = poly_a - np.array([x0, y0])
    shifted_b = poly_b - np.array([x0, y0])
    cv2.fillPoly(mask_a, [shifted_a], 1)
    cv2.fillPoly(mask_b, [shifted_b], 1)

    intersection = np.logical_and(mask_a, mask_b).sum()
    union = np.logical_or(mask_a, mask_b).sum()
    if union == 0:
        return 0.0
    return float(intersection) / float(union)
