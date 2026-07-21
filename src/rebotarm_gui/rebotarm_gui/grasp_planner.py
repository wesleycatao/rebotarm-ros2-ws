"""Pixel-plane grasp candidate planning for detected vision objects."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


Point2 = tuple[float, float]


@dataclass(frozen=True)
class GraspPlannerConfig:
    gripper_open_width_px: float = 140.0
    gripper_closed_width_px: float = 12.0
    gripper_body_width_px: float = 95.0
    gripper_body_length_px: float = 150.0
    finger_thickness_px: float = 12.0
    min_side_clearance_px: float = 4.0
    min_front_clearance_px: float = 20.0
    approach_distance_px: float = 55.0
    vertical_pick_margin_px: float = 8.0
    mm_per_px: float = 1.0
    lying_aspect_min: float = 1.60
    standing_aspect_max: float = 1.35
    enable_top_pick: bool = True
    enable_lateral_pick: bool = True
    enable_end_pick: bool = False
    prefer_vertical_for_lying: bool = True
    lateral_search_step_deg: float = 10.0
    lateral_candidate_count: int = 3
    lateral_search_max_distance_px: float = 2500.0
    workspace_shape: str = "none"
    workspace_polygon: tuple[Point2, ...] = ()
    workspace_circle: tuple[float, float, float] | None = None
    workspace_margin_px: float = 0.0


@dataclass(frozen=True)
class OrientedObject2D:
    source: object
    number: int
    label: str
    center: Point2
    long_axis: Point2
    short_axis: Point2
    half_long: float
    half_short: float
    yaw: float
    clearance_left_px: float
    clearance_right_px: float
    clearance_up_px: float
    clearance_down_px: float

    @property
    def long_side(self) -> float:
        return self.half_long * 2.0

    @property
    def short_side(self) -> float:
        return self.half_short * 2.0

    @property
    def aspect_ratio(self) -> float:
        return self.long_side / max(self.short_side, 1e-6)


@dataclass(frozen=True)
class GraspCandidate:
    object_number: int
    object_label: str
    object_type: str
    strategy: str
    approach: str
    valid: bool
    score: float
    reason: str
    center: Point2
    yaw: float
    available_clearance_px: float
    required_clearance_px: float
    gripper_opening_px: float
    gripper_opening_mm: float
    gripper_rect: tuple[Point2, Point2, Point2, Point2]
    approach_rect: tuple[Point2, Point2, Point2, Point2] | None = None


def plan_grasps(
    objects: Iterable[object],
    config: GraspPlannerConfig,
) -> list[GraspCandidate]:
    oriented = [
        box
        for index, obj in enumerate(objects, start=1)
        if (box := object_to_oriented_box(obj, index)) is not None
    ]
    candidates: list[GraspCandidate] = []
    for item in oriented:
        object_type = classify_object(item, config)
        if config.enable_top_pick and object_type in ("deitado", "indefinido"):
            candidates.append(_with_workspace_check(_top_pick_candidate(item, object_type, config), config))
        if config.enable_lateral_pick and object_type == "em pe":
            candidates.extend(_free_space_lateral_candidates(item, object_type, config, oriented))
        if config.enable_end_pick and object_type == "indefinido":
            candidates.extend(
                [
                    _with_workspace_check(
                        _side_pick_candidate(item, object_type, config, item.long_axis, -1.0, "ponta L-"),
                        config,
                    ),
                    _with_workspace_check(
                        _side_pick_candidate(item, object_type, config, item.long_axis, 1.0, "ponta L+"),
                        config,
                    ),
                ]
            )
    return sorted(
        candidates,
        key=lambda cand: (not cand.valid, -cand.score, cand.required_clearance_px, cand.object_number),
    )


def best_candidates(candidates: Iterable[GraspCandidate]) -> dict[int, GraspCandidate]:
    best: dict[int, GraspCandidate] = {}
    for cand in candidates:
        current = best.get(cand.object_number)
        if not cand.valid:
            continue
        if current is None or cand.score > current.score:
            best[cand.object_number] = cand
    return best


def classify_object(item: OrientedObject2D, config: GraspPlannerConfig) -> str:
    label = item.label.lower()
    if "lying" in label or "deitado" in label:
        return "deitado"
    if item.aspect_ratio >= config.lying_aspect_min:
        return "deitado"
    if item.aspect_ratio <= config.standing_aspect_max:
        return "em pe"
    return "indefinido"


def object_to_oriented_box(obj: object, number: int) -> OrientedObject2D | None:
    points = list(getattr(obj, "bounding_box_pixels", []))
    if len(points) < 4:
        return None
    xy = [(float(point.x), float(point.y)) for point in points[:4]]
    center_x = sum(point[0] for point in xy) / 4.0
    center_y = sum(point[1] for point in xy) / 4.0
    edges: list[tuple[float, float, float]] = []
    for index in range(4):
        x0, y0 = xy[index]
        x1, y1 = xy[(index + 1) % 4]
        dx = x1 - x0
        dy = y1 - y0
        edges.append((math.hypot(dx, dy), dx, dy))
    long_index, long_edge = max(enumerate(edges), key=lambda item: item[1][0])
    short_edge = edges[(long_index + 1) % 4]
    if long_edge[0] <= 1e-6:
        yaw = float(getattr(obj, "axis_yaw", getattr(obj, "yaw", 0.0)))
        long_axis = (math.cos(yaw), math.sin(yaw))
        short_axis = (-math.sin(yaw), math.cos(yaw))
        half_long = 0.0
        half_short = 0.0
    else:
        long_axis = _unit((long_edge[1], long_edge[2]))
        short_axis = _unit((short_edge[1], short_edge[2])) if short_edge[0] > 1e-6 else (-long_axis[1], long_axis[0])
        projections_long = [_dot(point, long_axis) for point in xy]
        projections_short = [_dot(point, short_axis) for point in xy]
        half_long = (max(projections_long) - min(projections_long)) * 0.5
        half_short = (max(projections_short) - min(projections_short)) * 0.5
        yaw = math.atan2(long_axis[1], long_axis[0])
    return OrientedObject2D(
        source=obj,
        number=number,
        label=str(getattr(obj, "class_name", "")),
        center=(center_x, center_y),
        long_axis=long_axis,
        short_axis=short_axis,
        half_long=max(half_long, 0.0),
        half_short=max(half_short, 0.0),
        yaw=yaw,
        clearance_left_px=float(getattr(obj, "clearance_left_px", 0.0)),
        clearance_right_px=float(getattr(obj, "clearance_right_px", 0.0)),
        clearance_up_px=float(getattr(obj, "clearance_up_px", 0.0)),
        clearance_down_px=float(getattr(obj, "clearance_down_px", 0.0)),
    )


def _top_pick_candidate(
    item: OrientedObject2D,
    object_type: str,
    config: GraspPlannerConfig,
) -> GraspCandidate:
    needed_side = config.finger_thickness_px + config.min_side_clearance_px
    opening_margin = min(config.min_side_clearance_px, max(2.0, config.finger_thickness_px * 0.5))
    required_opening = item.short_side + opening_margin
    opening_px = min(config.gripper_open_width_px, max(config.gripper_closed_width_px, required_opening))
    available = min(item.clearance_left_px, item.clearance_right_px)
    width_ok = config.gripper_open_width_px >= required_opening
    close_ok = config.gripper_closed_width_px <= item.short_side
    clearance_ok = available >= needed_side
    valid = width_ok and close_ok and clearance_ok
    reason = "ok"
    if not width_ok:
        reason = "abertura insuficiente"
    elif not close_ok:
        reason = "objeto menor que abertura minima"
    elif not clearance_ok:
        reason = "clearance lateral insuficiente"
    half_long = item.half_long + config.vertical_pick_margin_px
    half_short = item.half_short + needed_side
    rect = _rect(item.center, item.long_axis, item.short_axis, half_long, half_short)
    score = _score(available, needed_side, preferred=1.0 if config.prefer_vertical_for_lying else 0.75)
    return GraspCandidate(
        object_number=item.number,
        object_label=item.label,
        object_type=object_type,
        strategy="superior vertical",
        approach="Z-",
        valid=valid,
        score=score if valid else 0.0,
        reason=reason,
        center=item.center,
        yaw=item.yaw,
        available_clearance_px=available,
        required_clearance_px=needed_side,
        gripper_opening_px=opening_px,
        gripper_opening_mm=opening_px * config.mm_per_px,
        gripper_rect=rect,
    )


def _side_pick_candidate(
    item: OrientedObject2D,
    object_type: str,
    config: GraspPlannerConfig,
    axis: Point2,
    sign: float,
    label: str,
) -> GraspCandidate:
    axis = _unit(axis)
    approach_axis = (axis[0] * sign, axis[1] * sign)
    along_long = abs(_dot(axis, item.long_axis)) > abs(_dot(axis, item.short_axis))
    if along_long:
        object_half_along = item.half_long
        object_half_across = item.half_short
        available = item.clearance_up_px if sign < 0.0 else item.clearance_down_px
        perp_minus = item.clearance_left_px
        perp_plus = item.clearance_right_px
        yaw = item.yaw
    else:
        object_half_along = item.half_short
        object_half_across = item.half_long
        available = item.clearance_left_px if sign < 0.0 else item.clearance_right_px
        perp_minus = item.clearance_up_px
        perp_plus = item.clearance_down_px
        yaw = item.yaw + (math.pi * 0.5)

    required_along = config.gripper_body_length_px + config.approach_distance_px + config.min_front_clearance_px
    required_opening = (object_half_across * 2.0) + (config.min_side_clearance_px * 2.0)
    opening_px = min(config.gripper_open_width_px, max(config.gripper_closed_width_px, required_opening))
    extra_across = max(0.0, (config.gripper_body_width_px * 0.5) - object_half_across) + config.min_side_clearance_px
    width_ok = config.gripper_open_width_px >= required_opening
    close_ok = config.gripper_closed_width_px <= (object_half_across * 2.0)
    along_ok = available >= required_along
    across_ok = min(perp_minus, perp_plus) >= extra_across
    valid = width_ok and close_ok and along_ok and across_ok

    if not width_ok:
        reason = "abertura insuficiente"
    elif not close_ok:
        reason = "objeto menor que abertura minima"
    elif not along_ok:
        reason = "clearance de aproximação insuficiente"
    elif not across_ok:
        reason = "largura do gripper sem espaço lateral"
    else:
        reason = "ok"

    edge_center = _add(item.center, _scale(approach_axis, object_half_along))
    gripper_center = _add(edge_center, _scale(approach_axis, config.gripper_body_length_px * 0.5))
    side_axis = (-approach_axis[1], approach_axis[0])
    gripper_rect = _rect(
        gripper_center,
        approach_axis,
        side_axis,
        config.gripper_body_length_px * 0.5,
        config.gripper_body_width_px * 0.5,
    )
    corridor_center = _add(edge_center, _scale(approach_axis, required_along * 0.5))
    approach_rect = _rect(
        corridor_center,
        approach_axis,
        side_axis,
        required_along * 0.5,
        config.gripper_body_width_px * 0.5,
    )
    return GraspCandidate(
        object_number=item.number,
        object_label=item.label,
        object_type=object_type,
        strategy="lateral horizontal" if not along_long else "pela ponta",
        approach=label,
        valid=valid,
        score=_score(min(available, min(perp_minus, perp_plus)), max(required_along, extra_across)) if valid else 0.0,
        reason=reason,
        center=gripper_center,
        yaw=_normalize_angle(yaw),
        available_clearance_px=available,
        required_clearance_px=required_along,
        gripper_opening_px=opening_px,
        gripper_opening_mm=opening_px * config.mm_per_px,
        gripper_rect=gripper_rect,
        approach_rect=approach_rect,
    )


def _free_space_lateral_candidates(
    item: OrientedObject2D,
    object_type: str,
    config: GraspPlannerConfig,
    objects: list[OrientedObject2D],
) -> list[GraspCandidate]:
    step_deg = max(float(config.lateral_search_step_deg), 1.0)
    count = max(int(config.lateral_candidate_count), 1)
    tested: list[GraspCandidate] = []
    steps = max(1, int(math.ceil(360.0 / step_deg)))
    blockers = [other for other in objects if other.number != item.number]
    for index in range(steps):
        angle = math.radians(index * 360.0 / steps)
        tested.append(_free_space_lateral_candidate(item, object_type, config, blockers, angle))
    valid = [cand for cand in tested if cand.valid]
    if valid:
        return sorted(valid, key=lambda cand: (-cand.score, -cand.available_clearance_px, cand.required_clearance_px))[:count]
    if not tested:
        return []
    best_invalid = max(tested, key=lambda cand: (cand.available_clearance_px, cand.score))
    return [best_invalid]


def _free_space_lateral_candidate(
    item: OrientedObject2D,
    object_type: str,
    config: GraspPlannerConfig,
    blockers: list[OrientedObject2D],
    angle: float,
) -> GraspCandidate:
    approach_axis = (math.cos(angle), math.sin(angle))
    side_axis = (-approach_axis[1], approach_axis[0])
    object_half_along = _projected_half_extent(item, approach_axis)
    object_half_across = _projected_half_extent(item, side_axis)
    edge_center = _add(item.center, _scale(approach_axis, object_half_along))

    required_along = config.gripper_body_length_px + config.approach_distance_px + config.min_front_clearance_px
    required_opening = (object_half_across * 2.0) + (config.min_side_clearance_px * 2.0)
    opening_px = min(config.gripper_open_width_px, max(config.gripper_closed_width_px, required_opening))
    half_swept_width = max(config.gripper_body_width_px * 0.5, required_opening * 0.5) + config.min_side_clearance_px
    available = _available_approach_distance(edge_center, approach_axis, half_swept_width, blockers, config)

    width_ok = config.gripper_open_width_px >= required_opening
    close_ok = config.gripper_closed_width_px <= (object_half_across * 2.0)
    along_ok = available >= required_along
    valid = width_ok and close_ok and along_ok

    if not width_ok:
        reason = "abertura insuficiente"
    elif not close_ok:
        reason = "objeto menor que abertura minima"
    elif not along_ok:
        reason = "sem corredor livre suficiente"
    else:
        reason = "ok"

    gripper_center = _add(edge_center, _scale(approach_axis, config.gripper_body_length_px * 0.5))
    gripper_rect = _rect(
        gripper_center,
        approach_axis,
        side_axis,
        config.gripper_body_length_px * 0.5,
        config.gripper_body_width_px * 0.5,
    )
    corridor_center = _add(edge_center, _scale(approach_axis, required_along * 0.5))
    approach_rect = _rect(corridor_center, approach_axis, side_axis, required_along * 0.5, half_swept_width)
    degrees = int(round(math.degrees(_normalize_angle(angle)))) % 360
    score = _score(available, required_along, preferred=0.72) if valid else max(0.0, min(0.65, available / max(required_along, 1.0)))
    return GraspCandidate(
        object_number=item.number,
        object_label=item.label,
        object_type=object_type,
        strategy="lateral horizontal",
        approach=f"livre {degrees:03d}°",
        valid=valid,
        score=score if valid else 0.0,
        reason=reason,
        center=gripper_center,
        yaw=_normalize_angle(angle),
        available_clearance_px=available,
        required_clearance_px=required_along,
        gripper_opening_px=opening_px,
        gripper_opening_mm=opening_px * config.mm_per_px,
        gripper_rect=gripper_rect,
        approach_rect=approach_rect,
    )


def _projected_half_extent(item: OrientedObject2D, axis: Point2) -> float:
    axis = _unit(axis)
    return abs(_dot(item.long_axis, axis)) * item.half_long + abs(_dot(item.short_axis, axis)) * item.half_short


def _available_approach_distance(
    edge_center: Point2,
    approach_axis: Point2,
    half_width: float,
    blockers: list[OrientedObject2D],
    config: GraspPlannerConfig,
) -> float:
    max_distance = max(float(config.lateral_search_max_distance_px), 1.0)
    step = max(4.0, min(25.0, max_distance / 80.0))
    low = 0.0
    high = min(step, max_distance)
    while high < max_distance and _approach_corridor_clear(edge_center, approach_axis, half_width, high, blockers, config):
        low = high
        high = min(max_distance, high + step)
    if high >= max_distance and _approach_corridor_clear(edge_center, approach_axis, half_width, high, blockers, config):
        return max_distance
    for _attempt in range(8):
        mid = (low + high) * 0.5
        if _approach_corridor_clear(edge_center, approach_axis, half_width, mid, blockers, config):
            low = mid
        else:
            high = mid
    return low


def _approach_corridor_clear(
    edge_center: Point2,
    approach_axis: Point2,
    half_width: float,
    length: float,
    blockers: list[OrientedObject2D],
    config: GraspPlannerConfig,
) -> bool:
    if length <= 1e-6:
        return _point_inside_workspace(edge_center, config)
    approach_axis = _unit(approach_axis)
    side_axis = (-approach_axis[1], approach_axis[0])
    corridor = _rect(
        _add(edge_center, _scale(approach_axis, length * 0.5)),
        approach_axis,
        side_axis,
        length * 0.5,
        half_width,
    )
    if _workspace_enabled(config) and not _shape_inside_workspace(corridor, config):
        return False
    return all(not _convex_polygons_intersect(corridor, _object_polygon(blocker)) for blocker in blockers)


def _score(available: float, required: float, preferred: float = 0.75) -> float:
    margin = max(0.0, available - required)
    return min(1.0, preferred + margin / max(required * 4.0, 1.0))


def _with_workspace_check(cand: GraspCandidate, config: GraspPlannerConfig) -> GraspCandidate:
    if not cand.valid or not _workspace_enabled(config):
        return cand
    rects = [cand.gripper_rect]
    if cand.approach_rect is not None:
        rects.append(cand.approach_rect)
    for rect in rects:
        if not _shape_inside_workspace(rect, config):
            return GraspCandidate(
                object_number=cand.object_number,
                object_label=cand.object_label,
                object_type=cand.object_type,
                strategy=cand.strategy,
                approach=cand.approach,
                valid=False,
                score=0.0,
                reason="fora do limite operacional",
                center=cand.center,
                yaw=cand.yaw,
                available_clearance_px=cand.available_clearance_px,
                required_clearance_px=cand.required_clearance_px,
                gripper_opening_px=cand.gripper_opening_px,
                gripper_opening_mm=cand.gripper_opening_mm,
                gripper_rect=cand.gripper_rect,
                approach_rect=cand.approach_rect,
            )
    return cand


def _workspace_enabled(config: GraspPlannerConfig) -> bool:
    shape = config.workspace_shape.lower()
    return (shape == "polygon" and len(config.workspace_polygon) >= 3) or (
        shape == "circle" and config.workspace_circle is not None and config.workspace_circle[2] > 0.0
    )


def _shape_inside_workspace(points: tuple[Point2, ...], config: GraspPlannerConfig) -> bool:
    samples = list(points)
    samples.append(
        (
            sum(point[0] for point in points) / max(len(points), 1),
            sum(point[1] for point in points) / max(len(points), 1),
        )
    )
    for index, point in enumerate(points):
        next_point = points[(index + 1) % len(points)]
        samples.append(((point[0] + next_point[0]) * 0.5, (point[1] + next_point[1]) * 0.5))
    return all(_point_inside_workspace(point, config) for point in samples)


def _point_inside_workspace(point: Point2, config: GraspPlannerConfig) -> bool:
    margin = max(0.0, float(config.workspace_margin_px))
    shape = config.workspace_shape.lower()
    if shape == "circle" and config.workspace_circle is not None:
        cx, cy, radius = config.workspace_circle
        return math.hypot(point[0] - cx, point[1] - cy) <= max(float(radius) - margin, 0.0)
    if shape == "polygon" and len(config.workspace_polygon) >= 3:
        polygon = config.workspace_polygon
        if not _point_in_polygon(point, polygon):
            return False
        if margin <= 0.0:
            return True
        return min(_point_segment_distance(point, polygon[index], polygon[(index + 1) % len(polygon)]) for index in range(len(polygon))) >= margin
    return True


def _point_in_polygon(point: Point2, polygon: tuple[Point2, ...]) -> bool:
    x, y = point
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        denom = yj - yi
        if abs(denom) <= 1e-12:
            denom = 1e-12
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / denom + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def _point_segment_distance(point: Point2, start: Point2, end: Point2) -> float:
    px, py = point
    x0, y0 = start
    x1, y1 = end
    dx = x1 - x0
    dy = y1 - y0
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(px - x0, py - y0)
    t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / length_sq))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


def _object_polygon(item: OrientedObject2D) -> tuple[Point2, Point2, Point2, Point2]:
    return _rect(item.center, item.long_axis, item.short_axis, item.half_long, item.half_short)


def _convex_polygons_intersect(poly_a: tuple[Point2, ...], poly_b: tuple[Point2, ...]) -> bool:
    for polygon in (poly_a, poly_b):
        for index in range(len(polygon)):
            p0 = polygon[index]
            p1 = polygon[(index + 1) % len(polygon)]
            edge = (p1[0] - p0[0], p1[1] - p0[1])
            axis = _unit((-edge[1], edge[0]))
            min_a, max_a = _project_polygon(poly_a, axis)
            min_b, max_b = _project_polygon(poly_b, axis)
            if max_a <= min_b or max_b <= min_a:
                return False
    return True


def _project_polygon(polygon: tuple[Point2, ...], axis: Point2) -> tuple[float, float]:
    projections = [_dot(point, axis) for point in polygon]
    return (min(projections), max(projections))


def _rect(center: Point2, long_axis: Point2, short_axis: Point2, half_long: float, half_short: float) -> tuple[Point2, Point2, Point2, Point2]:
    lx = _scale(long_axis, half_long)
    sx = _scale(short_axis, half_short)
    return (
        _add(_add(center, _scale(lx, -1.0)), _scale(sx, -1.0)),
        _add(_add(center, lx), _scale(sx, -1.0)),
        _add(_add(center, lx), sx),
        _add(_add(center, _scale(lx, -1.0)), sx),
    )


def _unit(vector: Point2) -> Point2:
    length = math.hypot(vector[0], vector[1])
    if length <= 1e-9:
        return (1.0, 0.0)
    return (vector[0] / length, vector[1] / length)


def _dot(a: Point2, b: Point2) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _add(a: Point2, b: Point2) -> Point2:
    return (a[0] + b[0], a[1] + b[1])


def _scale(a: Point2, factor: float) -> Point2:
    return (a[0] * factor, a[1] * factor)


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= math.pi * 2.0
    while angle <= -math.pi:
        angle += math.pi * 2.0
    return angle
