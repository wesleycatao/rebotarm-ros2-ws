"""Read-only spatial preview for compiled motion-program waypoints."""

from __future__ import annotations

import math
from dataclasses import dataclass

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PySide6.QtWidgets import QDialog, QLabel, QPushButton, QVBoxLayout, QWidget

from ..motion import ACTION_OPS, OP_MOVE, Program, scale_duration


@dataclass(frozen=True)
class PreviewPoint:
    x: float
    y: float
    z: float
    duration_s: float
    root_index: int
    step_id: str


@dataclass(frozen=True)
class TrajectoryPreviewData:
    points: tuple[PreviewPoint, ...]
    distance_m: float
    motion_duration_s: float
    action_count: int
    non_motion_action_count: int
    has_dynamic_flow: bool
    start_index: int


def _contains_dynamic_flow(steps) -> bool:
    for step in steps:
        if getattr(step, "kind", "") in {"loop", "if"}:
            return True
        children = list(getattr(step, "steps", []))
        children += list(getattr(step, "then_steps", []))
        children += list(getattr(step, "else_steps", []))
        if _contains_dynamic_flow(children):
            return True
    return False


def build_trajectory_preview(
    program: Program,
    start_index: int = 0,
) -> TrajectoryPreviewData:
    """Compile Cartesian waypoints without sending any command to ROS."""
    start = max(0, min(int(start_index), len(program.steps)))
    subset = Program(
        name=program.name,
        version=program.version,
        speed_scale=program.speed_scale,
        steps=list(program.steps[start:]),
    )
    ops, meta = subset.flatten_with_meta()
    pc_to_root = meta.get("pc_to_root_index", {})
    points: list[PreviewPoint] = []
    for pc, op in enumerate(ops):
        if not op or op[0] != OP_MOVE:
            continue
        pose = op[1]
        points.append(
            PreviewPoint(
                x=float(pose["x"]),
                y=float(pose["y"]),
                z=float(pose["z"]),
                duration_s=scale_duration(float(op[2]), float(program.speed_scale)),
                root_index=int(pc_to_root.get(pc, 0)) + start,
                step_id=str(op[-1]),
            )
        )

    distance = 0.0
    for previous, current in zip(points, points[1:]):
        distance += math.dist(
            (previous.x, previous.y, previous.z),
            (current.x, current.y, current.z),
        )
    action_count = sum(1 for op in ops if op and op[0] in ACTION_OPS)
    return TrajectoryPreviewData(
        points=tuple(points),
        distance_m=distance,
        motion_duration_s=sum(point.duration_s for point in points),
        action_count=action_count,
        non_motion_action_count=action_count - len(points),
        has_dynamic_flow=_contains_dynamic_flow(subset.steps),
        start_index=start,
    )


class TrajectoryPreviewDialog(QDialog):
    """Show a 3-D XYZ route and an execution-oriented summary."""

    def __init__(
        self,
        program: Program,
        start_index: int = 0,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Pré-visualização da trajetória")
        self.resize(920, 650)
        self.data = build_trajectory_preview(program, start_index)

        self._summary = QLabel(self._summary_text())
        self._summary.setWordWrap(True)

        self._figure = Figure(figsize=(8.5, 5.2), tight_layout=True)
        self._canvas = FigureCanvas(self._figure)
        self._draw()

        close_button = QPushButton("Fechar")
        close_button.clicked.connect(self.reject)

        layout = QVBoxLayout(self)
        layout.addWidget(self._summary)
        layout.addWidget(self._canvas, 1)
        layout.addWidget(close_button, 0)

    def _summary_text(self) -> str:
        data = self.data
        start = data.start_index + 1
        text = (
            f"A partir da etapa {start} · {len(data.points)} waypoint(s) · "
            f"{data.distance_m:.3f} m entre waypoints · "
            f"{data.motion_duration_s:.2f} s de movimento · "
            f"{data.non_motion_action_count} ação(ões) não espaciais."
        )
        if data.has_dynamic_flow:
            text += (
                " LOOP/IF são exibidos em ordem estrutural; repetições e o ramo "
                "escolhido em tempo real podem alterar o percurso executado."
            )
        return text

    def _draw(self) -> None:
        axis = self._figure.add_subplot(111, projection="3d")
        axis.set_title("Trajetória cartesiana compilada")
        axis.set_xlabel("X (m)")
        axis.set_ylabel("Y (m)")
        axis.set_zlabel("Z (m)")
        points = self.data.points
        if not points:
            axis.text2D(
                0.5,
                0.5,
                "Nenhum waypoint cartesiano nesta faixa.",
                transform=axis.transAxes,
                ha="center",
                va="center",
            )
            self._canvas.draw_idle()
            return

        xs = [point.x for point in points]
        ys = [point.y for point in points]
        zs = [point.z for point in points]
        roots = [point.root_index for point in points]
        if len(points) > 1:
            axis.plot(xs, ys, zs, color="#8b949e", linewidth=1.5, alpha=0.85)
        axis.scatter(xs, ys, zs, c=roots, cmap="tab10", s=26, depthshade=False)
        axis.scatter([xs[0]], [ys[0]], [zs[0]], color="#2ea043", s=80, label="Início")
        axis.scatter([xs[-1]], [ys[-1]], [zs[-1]], color="#f85149", s=80, label="Fim")
        if len(points) <= 20:
            for index, point in enumerate(points, start=1):
                axis.text(point.x, point.y, point.z, str(index), fontsize=7)
        axis.legend(loc="upper right")
        self._set_equal_limits(axis, xs, ys, zs)
        self._canvas.draw_idle()

    @staticmethod
    def _set_equal_limits(axis, xs, ys, zs) -> None:
        centers = [(min(values) + max(values)) / 2.0 for values in (xs, ys, zs)]
        span = max(max(values) - min(values) for values in (xs, ys, zs))
        radius = max(span / 2.0, 0.05)
        axis.set_xlim(centers[0] - radius, centers[0] + radius)
        axis.set_ylim(centers[1] - radius, centers[1] + radius)
        axis.set_zlim(centers[2] - radius, centers[2] + radius)
