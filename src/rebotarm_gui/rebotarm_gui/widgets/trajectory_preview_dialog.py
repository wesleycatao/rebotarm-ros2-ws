"""Read-only spatial preview for compiled motion-program waypoints."""

from __future__ import annotations

import math
import uuid
from dataclasses import dataclass

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PySide6.QtWidgets import (
    QDialog,
    QLabel,
    QProgressBar,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from .. import kinematics
from ..motion import ACTION_OPS, OP_MOVE, Program, scale_duration
from ..motion.cell_scene import (
    KEEP_OUT,
    CellScene,
    CellSceneStore,
    analyze_program_tcp_clearance,
    monitored_world_vertices,
    tcp_segment_clearances,
)
from ..motion.dynamics import (
    DynamicsAnalysisReport,
    DynamicsProbe,
    MotionPathExpansionError,
    build_motion_paths,
)
from ..styles.styling import STATUS_FAIL, STATUS_OK, STATUS_WARN, apply_status


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
    speed_override: float


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
    speed_override: float = 1.0,
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
    global_override = max(0.05, min(1.0, float(speed_override)))
    for pc, op in enumerate(ops):
        if not op or op[0] != OP_MOVE:
            continue
        pose = op[1]
        points.append(
            PreviewPoint(
                x=float(pose["x"]),
                y=float(pose["y"]),
                z=float(pose["z"]),
                duration_s=(
                    scale_duration(float(op[2]), float(program.speed_scale))
                    / global_override
                ),
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
        speed_override=global_override,
    )


class TrajectoryPreviewDialog(QDialog):
    """Show a 3-D XYZ route and an execution-oriented summary."""

    def __init__(
        self,
        program: Program,
        start_index: int = 0,
        parent: QWidget | None = None,
        *,
        speed_override: float = 1.0,
        bridge=None,
        cell_scene_store: CellSceneStore | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Pré-visualização da trajetória")
        self.resize(1020, 820)
        self.data = build_trajectory_preview(program, start_index, speed_override)
        self._cell_scene = (
            cell_scene_store.scene if cell_scene_store is not None else CellScene()
        )
        self._cell_scene_error = (
            cell_scene_store.load_error if cell_scene_store is not None else ""
        )
        self._monitored_geometries = (
            cell_scene_store.external_geometries
            if cell_scene_store is not None
            else ()
        )
        self._monitored_error = (
            cell_scene_store.monitored_error
            if cell_scene_store is not None
            else ""
        )
        self._monitored_signature = (
            cell_scene_store.monitored_signature
            if cell_scene_store is not None
            else ""
        )
        self._monitored_octomap = (
            cell_scene_store.monitored_octomap
            if cell_scene_store is not None
            else {"present": False}
        )
        self._planned_tcp_paths: list[list[tuple[float, float, float]]] = []
        self._full_robot_clearance: dict[str, object] = {}
        self._moveit_tag = uuid.uuid4().hex
        self._moveit_bridge = bridge
        self.clearance_report = analyze_program_tcp_clearance(
            program,
            self._cell_scene,
            start_index=start_index,
        )
        self.segment_clearances = tcp_segment_clearances(
            ((point.x, point.y, point.z) for point in self.data.points),
            self._cell_scene,
        )
        self.dynamics_report: DynamicsAnalysisReport | None = None
        self._dynamics_probe: DynamicsProbe | None = None

        self._summary = QLabel(self._summary_text())
        self._summary.setWordWrap(True)

        self._figure = Figure(figsize=(8.5, 5.2), tight_layout=True)
        self._canvas = FigureCanvas(self._figure)
        self._draw()

        self._dynamics_status = QLabel(
            "Análise dinâmica: aguardando dados do controlador."
        )
        self._dynamics_status.setWordWrap(True)
        self._dynamics_progress = QProgressBar()
        self._dynamics_progress.setRange(0, 1)
        self._dynamics_progress.setValue(0)
        self._dynamics_table = QTableWidget(0, 9)
        self._dynamics_table.setHorizontalHeaderLabels(
            [
                "Junta",
                "Pico vel.",
                "Limite vel.",
                "Uso vel.",
                "Pico acel.",
                "Limite acel.",
                "Uso acel.",
                "Margem",
                "Trecho limitante",
            ]
        )
        self._dynamics_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._dynamics_table.setSelectionMode(QTableWidget.NoSelection)
        self._dynamics_table.verticalHeader().setVisible(False)
        self._dynamics_table.horizontalHeader().setStretchLastSection(True)
        self._dynamics_table.setMaximumHeight(230)

        self._moveit_status = QLabel(
            "MoveIt: prévia do caminho planejado ainda não solicitada."
        )
        self._moveit_status.setWordWrap(True)
        self._moveit_progress = QProgressBar()
        self._moveit_progress.setRange(0, 1)
        self._moveit_progress.setValue(0)

        close_button = QPushButton("Fechar")
        close_button.clicked.connect(self.reject)

        layout = QVBoxLayout(self)
        layout.addWidget(self._summary)
        layout.addWidget(self._canvas, 1)
        self._clearance_status = QLabel(self._clearance_text())
        self._clearance_status.setWordWrap(True)
        if self._cell_scene_error or not self.clearance_report.verified:
            apply_status(self._clearance_status, STATUS_WARN)
        elif self.clearance_report.safe:
            apply_status(self._clearance_status, STATUS_OK)
        else:
            apply_status(self._clearance_status, STATUS_FAIL)
        layout.addWidget(self._clearance_status)
        layout.addWidget(self._moveit_status)
        layout.addWidget(self._moveit_progress)
        layout.addWidget(self._dynamics_status)
        layout.addWidget(self._dynamics_progress)
        layout.addWidget(self._dynamics_table)
        layout.addWidget(close_button, 0)

        if bridge is not None:
            self._start_dynamics(program, start_index, speed_override, bridge)
            if (
                hasattr(bridge, "motion_plan_preview_result")
                and hasattr(bridge, "request_motion_plan_preview")
            ):
                self._start_moveit_preview(
                    program,
                    start_index,
                    speed_override,
                    bridge,
                )

    def _summary_text(self) -> str:
        data = self.data
        start = data.start_index + 1
        text = (
            f"A partir da etapa {start} · {len(data.points)} waypoint(s) · "
            f"{data.distance_m:.3f} m entre waypoints · "
            f"{data.motion_duration_s:.2f} s de movimento · "
            f"{data.non_motion_action_count} ação(ões) não espaciais · "
            f"controle global {data.speed_override * 100:.0f}%."
        )
        if data.has_dynamic_flow:
            text += (
                " LOOP/IF são exibidos em ordem estrutural; repetições e o ramo "
                "escolhido em tempo real podem alterar o percurso executado."
            )
        return text

    def _clearance_text(self) -> str:
        scene = self._cell_scene
        if self._cell_scene_error:
            return (
                "Cena da célula: ARQUIVO INVÁLIDO — "
                f"{self._cell_scene_error}. Execuções reais serão bloqueadas."
            )
        if not scene.configured:
            text = "Cena da célula: nenhum volume local habilitado."
            if self._monitored_geometries:
                text += (
                    f" {len(self._monitored_geometries)} geometria(s) externa(s) "
                    "monitorada(s) aparecem em azul."
                )
            if bool(self._monitored_octomap.get("present", False)):
                text += (
                    " OctoMap ativo no MoveIt; voxels não desenhados nesta prévia."
                )
            return text
        report = self.clearance_report
        scope = (
            "Pré-análise do envelope TCP; a colisão de todos os elos e a "
            "trajetória planejada são validadas pelo MoveIt."
        )
        monitored = ""
        if self._monitored_geometries:
            resolved_tf = sum(
                geometry.transform_status == "resolved"
                for geometry in self._monitored_geometries
            )
            unresolved_tf = sum(
                geometry.transform_status == "unresolved"
                or geometry.frame_id != scene.frame_id
                for geometry in self._monitored_geometries
            )
            monitored = (
                f" {len(self._monitored_geometries)} geometria(s) externa(s) "
                f"monitorada(s), hash {self._monitored_signature[:12]}…"
            )
            if resolved_tf:
                monitored += f" {resolved_tf} resolvida(s) por TF."
            if unresolved_tf:
                monitored += (
                    f" {unresolved_tf} sem TF para {scene.frame_id} "
                    "não desenhada(s)."
                )
        elif self._monitored_error:
            monitored = f" Cena monitorada indisponível: {self._monitored_error}."
        if bool(self._monitored_octomap.get("present", False)):
            monitored += (
                " OctoMap ativo no MoveIt; voxels não desenhados nesta prévia."
            )
        return f"Cena da célula: {report.summary()}. {scope}{monitored}"

    def _draw(self) -> None:
        self._figure.clear()
        axis = self._figure.add_subplot(111, projection="3d")
        axis.set_title(
            "Trajetória planejada pelo MoveIt"
            if self._planned_tcp_paths
            else "Trajetória cartesiana compilada"
        )
        axis.set_xlabel("X (m)")
        axis.set_ylabel("Y (m)")
        axis.set_zlabel("Z (m)")
        points = self.data.points
        bounds_x: list[float] = []
        bounds_y: list[float] = []
        bounds_z: list[float] = []
        for obstacle in self._cell_scene.enabled_obstacles:
            lower, upper = obstacle.bounds()
            color = "#f85149" if obstacle.kind == KEEP_OUT else "#f0b429"
            axis.bar3d(
                lower[0],
                lower[1],
                lower[2],
                obstacle.size[0],
                obstacle.size[1],
                obstacle.size[2],
                color=color,
                alpha=0.18,
                edgecolor=color,
                linewidth=0.8,
                shade=False,
            )
            axis.text(
                obstacle.center[0],
                obstacle.center[1],
                upper[2],
                obstacle.name,
                color=color,
                fontsize=7,
            )
            bounds_x.extend((lower[0], upper[0]))
            bounds_y.extend((lower[1], upper[1]))
            bounds_z.extend((lower[2], upper[2]))
        self._draw_monitored_scene(
            axis,
            bounds_x,
            bounds_y,
            bounds_z,
        )
        if not points:
            axis.text2D(
                0.5,
                0.5,
                "Nenhum waypoint cartesiano nesta faixa.",
                transform=axis.transAxes,
                ha="center",
                va="center",
            )
            if bounds_x:
                self._set_equal_limits(axis, bounds_x, bounds_y, bounds_z)
            self._canvas.draw_idle()
            return

        xs = [point.x for point in points]
        ys = [point.y for point in points]
        zs = [point.z for point in points]
        bounds_x.extend(xs)
        bounds_y.extend(ys)
        bounds_z.extend(zs)
        roots = [point.root_index for point in points]
        if len(points) > 1:
            required = self._cell_scene.minimum_clearance_m
            for index, (previous, current) in enumerate(zip(points, points[1:])):
                clearance = self.segment_clearances[index]
                if not self._cell_scene.configured:
                    color = "#8b949e"
                elif clearance < required:
                    color = "#f85149"
                elif clearance < max(required * 2.0, required + 0.02):
                    color = "#f0b429"
                else:
                    color = "#3fb950"
                axis.plot(
                    [previous.x, current.x],
                    [previous.y, current.y],
                    [previous.z, current.z],
                    color=color,
                    linewidth=2.4,
                    alpha=0.95,
                )
        axis.scatter(xs, ys, zs, c=roots, cmap="tab10", s=26, depthshade=False)
        axis.scatter([xs[0]], [ys[0]], [zs[0]], color="#2ea043", s=80, label="Início")
        axis.scatter([xs[-1]], [ys[-1]], [zs[-1]], color="#f85149", s=80, label="Fim")
        for index, planned in enumerate(self._planned_tcp_paths):
            if not planned:
                continue
            planned_x = [point[0] for point in planned]
            planned_y = [point[1] for point in planned]
            planned_z = [point[2] for point in planned]
            bounds_x.extend(planned_x)
            bounds_y.extend(planned_y)
            bounds_z.extend(planned_z)
            axis.plot(
                planned_x,
                planned_y,
                planned_z,
                color="#58a6ff",
                linewidth=2.0,
                alpha=0.95,
                label="Plano MoveIt" if index == 0 else None,
            )
        clearance = self._full_robot_clearance
        nearest_1 = clearance.get("nearest_point_1", ())
        nearest_2 = clearance.get("nearest_point_2", ())
        if (
            isinstance(nearest_1, (list, tuple))
            and isinstance(nearest_2, (list, tuple))
            and len(nearest_1) == 3
            and len(nearest_2) == 3
        ):
            marker_color = (
                "#3fb950" if bool(clearance.get("safe", False)) else "#f85149"
            )
            marker_x = [float(nearest_1[0]), float(nearest_2[0])]
            marker_y = [float(nearest_1[1]), float(nearest_2[1])]
            marker_z = [float(nearest_1[2]), float(nearest_2[2])]
            bounds_x.extend(marker_x)
            bounds_y.extend(marker_y)
            bounds_z.extend(marker_z)
            axis.plot(
                marker_x,
                marker_y,
                marker_z,
                color=marker_color,
                linewidth=3.0,
                linestyle=":",
                label="Folga mínima",
            )
            axis.scatter(
                marker_x,
                marker_y,
                marker_z,
                color=marker_color,
                s=34,
                depthshade=False,
            )
        if len(points) <= 20:
            for index, point in enumerate(points, start=1):
                axis.text(point.x, point.y, point.z, str(index), fontsize=7)
        axis.legend(loc="upper right")
        self._set_equal_limits(axis, bounds_x, bounds_y, bounds_z)
        self._canvas.draw_idle()

    def _draw_monitored_scene(
        self,
        axis,
        bounds_x: list[float],
        bounds_y: list[float],
        bounds_z: list[float],
    ) -> None:
        for geometry in self._monitored_geometries:
            if geometry.frame_id != self._cell_scene.frame_id:
                continue
            vertices = monitored_world_vertices(geometry)
            if not vertices:
                continue
            bounds_x.extend(point[0] for point in vertices)
            bounds_y.extend(point[1] for point in vertices)
            bounds_z.extend(point[2] for point in vertices)
            if geometry.shape == "mesh" and geometry.triangles:
                edges = {
                    tuple(sorted((first, second)))
                    for triangle in geometry.triangles[:5000]
                    for first, second in (
                        (triangle[0], triangle[1]),
                        (triangle[1], triangle[2]),
                        (triangle[2], triangle[0]),
                    )
                }
            elif len(vertices) == 8:
                edges = {
                    (0, 1), (1, 3), (3, 2), (2, 0),
                    (4, 5), (5, 7), (7, 6), (6, 4),
                    (0, 4), (1, 5), (2, 6), (3, 7),
                }
            elif geometry.shape == "plane" and len(vertices) == 4:
                edges = {(0, 1), (1, 2), (2, 3), (3, 0), (0, 2)}
            else:
                edges = set()
            color = "#bc8cff" if geometry.attached_to else "#58a6ff"
            for first, second in list(edges)[:10_000]:
                if first >= len(vertices) or second >= len(vertices):
                    continue
                a, b = vertices[first], vertices[second]
                axis.plot(
                    [a[0], b[0]],
                    [a[1], b[1]],
                    [a[2], b[2]],
                    color=color,
                    linewidth=0.7,
                    alpha=0.45,
                )
            highest = max(vertices, key=lambda point: point[2])
            label = geometry.object_id
            if geometry.attached_to:
                label += f" [{geometry.attached_to}]"
            if (
                geometry.transform_status == "resolved"
                and geometry.source_frame_id != geometry.frame_id
            ):
                label += f" ← {geometry.source_frame_id}"
            if geometry.shape == "plane":
                label += " [plano ∞; vista 2 m]"
            axis.text(*highest, label, color=color, fontsize=6)

    def _start_moveit_preview(
        self,
        program: Program,
        start_index: int,
        speed_override: float,
        bridge,
    ) -> None:
        try:
            paths, _notes = build_motion_paths(
                program,
                start_index=start_index,
                speed_override=speed_override,
            )
            pose_paths = [
                [dict(target.pose) for target in path]
                for path in paths
                if path
            ]
        except (MotionPathExpansionError, TypeError, ValueError) as exc:
            self._moveit_status.setText(
                f"MoveIt: prévia não iniciada — {exc}"
            )
            apply_status(self._moveit_status, STATUS_WARN)
            return
        bridge.motion_plan_preview_progress.connect(
            self._on_moveit_preview_progress
        )
        bridge.motion_plan_preview_result.connect(
            self._on_moveit_preview_result
        )
        self._moveit_status.setText(
            "MoveIt: solicitando trajetória plan-only; nenhum comando será executado…"
        )
        self._moveit_progress.setRange(0, 0)
        if not bridge.request_motion_plan_preview(
            self._moveit_tag,
            pose_paths,
            speed_scale=speed_override,
            required_clearance_m=(
                self._cell_scene.minimum_clearance_m
                if self._cell_scene.configured
                else 0.0
            ),
        ):
            self._moveit_progress.setRange(0, 1)
            self._moveit_progress.setValue(0)

    def _on_moveit_preview_progress(
        self,
        tag: str,
        done: int,
        total: int,
        message: str,
    ) -> None:
        if str(tag) != self._moveit_tag:
            return
        self._moveit_progress.setRange(0, max(1, int(total)))
        self._moveit_progress.setValue(min(int(done), max(1, int(total))))
        self._moveit_status.setText(f"MoveIt: {message}.")

    def _on_moveit_preview_result(
        self,
        tag: str,
        success: bool,
        payload: object,
        message: str,
    ) -> None:
        if str(tag) != self._moveit_tag:
            return
        self._moveit_progress.setRange(0, 1)
        self._moveit_progress.setValue(1 if success else 0)
        data = payload if isinstance(payload, dict) else {}
        self._planned_tcp_paths = self._planned_tcp_from_payload(data)
        validation = data.get("state_validation", {})
        clearance = data.get("full_robot_clearance", {})
        self._full_robot_clearance = (
            dict(clearance) if isinstance(clearance, dict) else {}
        )
        available = bool(
            isinstance(validation, dict)
            and validation.get("available", False)
        )
        verified = bool(
            isinstance(validation, dict)
            and validation.get("verified", False)
        )
        invalid = (
            int(validation.get("invalid_count", 0))
            if isinstance(validation, dict)
            else 0
        )
        clearance_available = bool(
            isinstance(clearance, dict)
            and clearance.get("available", False)
            and clearance.get("verified", False)
        )
        clearance_safe = bool(
            isinstance(clearance, dict)
            and clearance.get("safe", False)
        )
        if (
            success
            and available
            and verified
            and invalid == 0
            and clearance_available
            and clearance_safe
        ):
            sample_count = int(validation.get("sample_count", 0))
            distance_mm = (
                float(clearance.get("minimum_distance_m", 0.0)) * 1000.0
            )
            required_mm = (
                float(clearance.get("required_clearance_m", 0.0)) * 1000.0
            )
            pair = (
                f"{clearance.get('body_1', '?')} ↔ "
                f"{clearance.get('body_2', '?')}"
            )
            text = (
                "MoveIt: APROVADO — caminho plan-only obtido e "
                f"{sample_count} estado(s) do robô completo validado(s); "
                f"folga mínima {distance_mm:.1f} mm em {pair} "
                f"(requerida {required_mm:.1f} mm)."
            )
            apply_status(self._moveit_status, STATUS_OK)
        elif invalid or (clearance_available and not clearance_safe):
            reasons: list[str] = []
            if invalid:
                reasons.append(
                    f"{invalid} estado(s) amostrado(s) em colisão"
                )
            if clearance_available:
                reasons.append(
                    "folga mínima "
                    f"{float(clearance.get('minimum_distance_m', 0.0)) * 1000:.1f} "
                    "mm abaixo dos "
                    f"{float(clearance.get('required_clearance_m', 0.0)) * 1000:.1f} "
                    "mm requeridos"
                )
            text = (
                "MoveIt: BLOQUEADO — "
                f"{'; '.join(reasons)}. {message}"
            )
            apply_status(self._moveit_status, STATUS_FAIL)
        elif (
            success
            and available
            and verified
            and invalid == 0
            and not clearance_available
        ):
            text = (
                "MoveIt: caminho sem colisão nos estados amostrados, mas a "
                f"folga métrica do robô completo não foi verificada. {message}"
            )
            apply_status(self._moveit_status, STATUS_WARN)
        elif self._planned_tcp_paths:
            text = (
                f"MoveIt: plano obtido, mas validação independente dos estados "
                f"não foi certificada. {message}"
            )
            apply_status(self._moveit_status, STATUS_WARN)
        else:
            text = f"MoveIt: prévia indisponível — {message}"
            apply_status(self._moveit_status, STATUS_WARN)
        self._moveit_status.setText(text)
        self._draw()

    def _planned_tcp_from_payload(
        self,
        payload: dict[str, object],
    ) -> list[list[tuple[float, float, float]]]:
        node = getattr(self._moveit_bridge, "node", None)
        urdf = getattr(node, "robot_description", "") if node is not None else ""
        if not urdf:
            return []
        try:
            chain = kinematics.load_chain(urdf, tip_link="gripper_tcp")
            if not chain:
                chain = kinematics.load_chain(urdf, tip_link="gripper_link")
        except Exception:
            return []
        if not chain:
            return []
        tcp_paths: list[list[tuple[float, float, float]]] = []
        for path in payload.get("paths", ()):
            if not isinstance(path, dict):
                continue
            tcp_points: list[tuple[float, float, float]] = []
            for segment in path.get("segments", ()):
                if not isinstance(segment, dict):
                    continue
                names = [str(name) for name in segment.get("joint_names", ())]
                for point in segment.get("points", ()):
                    if not isinstance(point, dict):
                        continue
                    positions = point.get("positions", ())
                    q = {
                        name: float(value)
                        for name, value in zip(names, positions)
                    }
                    try:
                        transform = kinematics.tcp_transform(chain, q)
                    except Exception:
                        continue
                    tcp = tuple(float(value) for value in transform[:3, 3])
                    if not tcp_points or math.dist(tcp_points[-1], tcp) > 1e-9:
                        tcp_points.append(tcp)
            if tcp_points:
                tcp_paths.append(tcp_points)
        return tcp_paths

    def closeEvent(self, event) -> None:
        bridge = self._moveit_bridge
        if bridge is not None and hasattr(bridge, "cancel_motion_plan_preview"):
            bridge.cancel_motion_plan_preview(self._moveit_tag)
        super().closeEvent(event)

    def _start_dynamics(
        self,
        program: Program,
        start_index: int,
        speed_override: float,
        bridge,
    ) -> None:
        self._dynamics_status.setText(
            "Análise dinâmica: resolvendo os waypoints por IK…"
        )
        self._dynamics_progress.setRange(0, 0)
        self._dynamics_probe = DynamicsProbe(
            program,
            bridge,
            start_index=start_index,
            speed_override=speed_override,
            parent=self,
        )
        self._dynamics_probe.progress.connect(self._on_dynamics_progress)
        self._dynamics_probe.finished.connect(self._on_dynamics_finished)
        self._dynamics_probe.start()

    def _on_dynamics_progress(self, done: int, total: int) -> None:
        self._dynamics_progress.setRange(0, max(1, int(total)))
        self._dynamics_progress.setValue(min(int(done), max(1, int(total))))

    def _on_dynamics_finished(self, report: DynamicsAnalysisReport) -> None:
        self.dynamics_report = report
        self._dynamics_progress.setRange(0, 1)
        self._dynamics_progress.setValue(1)
        self._dynamics_status.setText(
            f"Análise dinâmica: {report.summary()}. "
            f"{report.path_count} caminho(s), {report.target_count} movimento(s), "
            f"fator de segurança {report.safety_factor:.2f}."
        )
        if not report.verified:
            apply_status(self._dynamics_status, STATUS_WARN)
        elif report.within_limits:
            apply_status(self._dynamics_status, STATUS_OK)
        else:
            apply_status(self._dynamics_status, STATUS_FAIL)
        self._dynamics_table.setRowCount(len(report.metrics))
        for row, metric in enumerate(report.metrics):
            values = (
                metric.joint_name,
                f"{metric.peak_velocity:.3f} rad/s",
                f"{metric.velocity_limit:.3f} rad/s",
                f"{metric.velocity_utilization * 100:.1f}%",
                f"{metric.peak_acceleration:.3f} rad/s²",
                f"{metric.acceleration_limit:.3f} rad/s²",
                f"{metric.acceleration_utilization * 100:.1f}%",
                f"{metric.margin_percent:+.1f}%",
                metric.limiting_step_id,
            )
            for column, value in enumerate(values):
                self._dynamics_table.setItem(
                    row, column, QTableWidgetItem(value)
                )
        self._dynamics_table.resizeColumnsToContents()

    @staticmethod
    def _set_equal_limits(axis, xs, ys, zs) -> None:
        centers = [(min(values) + max(values)) / 2.0 for values in (xs, ys, zs)]
        span = max(max(values) - min(values) for values in (xs, ys, zs))
        radius = max(span / 2.0, 0.05)
        axis.set_xlim(centers[0] - radius, centers[0] + radius)
        axis.set_ylim(centers[1] - radius, centers[1] + radius)
        axis.set_zlim(centers[2] - radius, centers[2] + radius)
