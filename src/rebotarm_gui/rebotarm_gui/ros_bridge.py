"""ROS 2 bridge for the reBotArm GUI.

This module runs a dedicated ``rclpy`` node inside a ``QThread``. All ROS
subscriptions, service clients and action clients live here, and results are
delivered to the Qt UI layer via signals (thread-safe).

The bridge is a pure client: it never touches hardware directly, only the
interfaces exposed by the ``reBotArmController`` node under ``/rebotarm``.
"""

from __future__ import annotations

import hashlib
import json
import math
import threading
import time
import uuid
from collections import deque
from typing import Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, Pose as GeometryPose, Quaternion
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as RclpyDuration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from rclpy.task import Future
from rclpy.time import Time
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from PySide6.QtCore import QObject, QThread, Signal
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.msg import (
    ArmStatus,
    DigitalIO,
    DigitalIOState,
    JointMitCmd,
    JointMotorState,
    JointPosVelCmd,
    SegmentedObjectArray,
    TimingStatus,
)
from rebotarm_msgs.srv import (
    CalibrateVision,
    CheckTorqueProfile,
    CheckTrajectoryClearance,
)
from rebotarm_msgs.srv import GripperCommand as GripperCommandSrv
from rebotarm_msgs.srv import MoveToPoseIK, SetDigitalIOConfig, SetDigitalOutput, SetGripper, SetZero
from rebotarm_msgs.srv import SetSafetyStep, SetTorqueProfile
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformException, TransformListener

from .settings import clamp_speed_override
from .security import (
    AuthorizationContext,
    CAP_EXECUTE_RELEASED,
    CAP_IO_WRITE,
    CAP_MANUAL_MOTION,
    CAP_RAW_CONTROL,
    CAP_SAFETY_CONFIG,
    CAP_SAFETY_RESET,
    CAP_SPEED_OVERRIDE,
    signed_lease,
)

from . import kinematics

_PLANNING_SCENE_TF_REFRESH_HZ = 5.0
_PLANNING_SCENE_TF_REFRESH_INTERVAL_S = (
    1.0 / _PLANNING_SCENE_TF_REFRESH_HZ
)
_MONITORED_PLANE_DISPLAY_SIZE_M = 2.0
_MAX_PLANES_PER_COLLISION_OBJECT = 64
_GUI_SCENE_OBJECT_PREFIX = "rebotarm_gui__"

try:  # MoveIt is optional in commissioning, mandatory when production requires it.
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (
        CollisionObject,
        Constraints,
        MoveItErrorCodes,
        OrientationConstraint,
        PlanningScene,
        PlanningSceneComponents,
        PositionConstraint,
        RobotState,
    )
    from moveit_msgs.srv import (
        ApplyPlanningScene,
        GetMotionPlan,
        GetPlanningScene,
        GetStateValidity,
    )
    from shape_msgs.msg import SolidPrimitive

    MOVEIT_AVAILABLE = True
except ImportError:  # pragma: no cover - depends on the ROS image
    MoveGroup = None
    CollisionObject = Constraints = MoveItErrorCodes = None
    OrientationConstraint = PlanningScene = PlanningSceneComponents = None
    PositionConstraint = RobotState = ApplyPlanningScene = None
    GetMotionPlan = GetPlanningScene = GetStateValidity = None
    SolidPrimitive = None
    MOVEIT_AVAILABLE = False


def duration_msg(seconds: float) -> Duration:
    """Convert seconds into a ``builtin_interfaces/Duration``."""
    sec = int(seconds)
    nanosec = int((float(seconds) - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def parse_trajectory_limits_payload(text: str) -> dict[str, object]:
    """Validate the controller's latched joint-limit JSON payload."""

    data = json.loads(str(text))
    names = [str(value) for value in data["joint_names"]]
    velocities = [float(value) for value in data["max_velocity"]]
    accelerations = [float(value) for value in data["max_acceleration"]]
    safety_factor = float(data["safety_factor"])
    if (
        not names
        or len(set(names)) != len(names)
        or len(velocities) != len(names)
        or len(accelerations) != len(names)
        or any(not math.isfinite(value) or value <= 0.0 for value in velocities)
        or any(not math.isfinite(value) or value <= 0.0 for value in accelerations)
        or not math.isfinite(safety_factor)
        or not 0.0 < safety_factor <= 1.0
    ):
        raise ValueError("valores de limites dinâmicos inválidos")
    return {
        "joint_names": names,
        "max_velocity": velocities,
        "max_acceleration": accelerations,
        "safety_factor": safety_factor,
    }


def make_pose(
    x: float,
    y: float,
    z: float,
    qx: float = 0.0,
    qy: float = 0.0,
    qz: float = 0.0,
    qw: float = 1.0,
) -> Pose:
    """Build a :class:`geometry_msgs/Pose` from position + quaternion."""
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation = Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))
    return pose


def _pose_payload(pose) -> list[float]:
    """Convert a ROS pose into the bounded representation consumed by Qt."""

    return [
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    ]


def _normalized_quaternion(pose) -> tuple[float, float, float, float]:
    if pose is None:
        return (0.0, 0.0, 0.0, 1.0)
    values = (
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        # Default-constructed ROS poses commonly leave qw at zero even when
        # the semantic intent is "no object-level transform".
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in values)


def _normalized_quaternion_values(
    values,
) -> tuple[float, float, float, float]:
    quaternion = tuple(float(value) for value in values)
    if len(quaternion) != 4:
        raise ValueError("quaternion deve conter quatro valores")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-12:
        raise ValueError("quaternion com norma zero")
    return tuple(value / norm for value in quaternion)


def _rotate_by_quaternion(
    vector: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _composed_pose_payload(object_pose, shape_pose) -> list[float]:
    """Compose ``header -> object`` and ``object -> shape`` ROS poses."""

    if object_pose is None:
        return [
            float(shape_pose.position.x),
            float(shape_pose.position.y),
            float(shape_pose.position.z),
            *_normalized_quaternion(shape_pose),
        ]
    return _compose_pose_values(
        (
            float(object_pose.position.x),
            float(object_pose.position.y),
            float(object_pose.position.z),
            *_normalized_quaternion(object_pose),
        ),
        (
            float(shape_pose.position.x),
            float(shape_pose.position.y),
            float(shape_pose.position.z),
            *_normalized_quaternion(shape_pose),
        ),
    )


def _compose_pose_values(base_pose, local_pose) -> list[float]:
    """Compose two ``x, y, z, qx, qy, qz, qw`` pose sequences."""

    base = tuple(float(value) for value in base_pose)
    local = tuple(float(value) for value in local_pose)
    if len(base) != 7 or len(local) != 7:
        raise ValueError("poses compostas devem conter sete valores")
    if not all(math.isfinite(value) for value in (*base, *local)):
        raise ValueError("pose não finita")
    base_q = _normalized_quaternion_values(base[3:])
    local_q = _normalized_quaternion_values(local[3:])
    rotated = _rotate_by_quaternion(local[:3], base_q)
    position = tuple(base[index] + rotated[index] for index in range(3))
    bx, by, bz, bw = base_q
    lx, ly, lz, lw = local_q
    quaternion = (
        bw * lx + bx * lw + by * lz - bz * ly,
        bw * ly - bx * lz + by * lw + bz * lx,
        bw * lz + bx * ly - by * lx + bz * lw,
        bw * lw - bx * lx - by * ly - bz * lz,
    )
    return [*position, *quaternion]


def transform_planning_scene_payload(
    payload: dict[str, object],
    target_frame: str,
    transforms: dict[str, tuple[float, ...]],
) -> dict[str, object]:
    """Resolve serialized planning-scene geometry into one display frame.

    ``transforms`` maps each source frame to the pose ``target <- source``.
    Missing transforms preserve the source pose and are explicitly marked
    unresolved so the GUI can omit them without losing scene traceability.
    """

    target = str(target_frame).strip().lstrip("/")
    if not target:
        raise ValueError("frame alvo da cena não pode ser vazio")
    output = dict(payload)
    transformed: list[dict[str, object]] = []
    resolved_frames: set[str] = set()
    unresolved_frames: set[str] = set()
    for raw in payload.get("geometries", ()):
        if not isinstance(raw, dict):
            continue
        geometry = dict(raw)
        source = str(geometry.get("frame_id", "")).strip().lstrip("/")
        pose = tuple(float(value) for value in geometry.get("pose", ()))
        geometry["source_frame_id"] = source
        geometry["source_pose"] = list(pose)
        if source == target:
            geometry["transform_status"] = "native"
        elif source in transforms:
            geometry["pose"] = _compose_pose_values(
                transforms[source], pose
            )
            geometry["frame_id"] = target
            geometry["transform_status"] = "resolved"
            resolved_frames.add(source)
        else:
            geometry["transform_status"] = "unresolved"
            unresolved_frames.add(source)
        transformed.append(geometry)
    output["geometries"] = transformed
    output["display_frame_id"] = target
    output["tf_resolved_frames"] = sorted(resolved_frames)
    output["tf_unresolved_frames"] = sorted(unresolved_frames)
    output["tf_resolved_geometry_count"] = sum(
        item.get("transform_status") == "resolved" for item in transformed
    )
    output["tf_unresolved_geometry_count"] = sum(
        item.get("transform_status") == "unresolved" for item in transformed
    )
    return output


def _normalized_plane_coefficients(values) -> list[float]:
    coefficients = tuple(float(value) for value in values)
    if len(coefficients) != 4:
        raise ValueError("plano deve conter quatro coeficientes")
    if not all(math.isfinite(value) for value in coefficients):
        raise ValueError("coeficiente de plano não finito")
    norm = math.sqrt(sum(value * value for value in coefficients[:3]))
    if norm < 1e-9:
        raise ValueError("normal de plano inválida")
    return [value / norm for value in coefficients]


def _octomap_data_digest(data) -> tuple[str, int]:
    """Hash signed octomap bytes without retaining a second full-size copy."""

    size = len(data)
    digest = hashlib.sha256()
    try:
        view = memoryview(data)
        digest.update(view.cast("B"))
        return digest.hexdigest(), size
    except (TypeError, ValueError):
        chunk = bytearray()
        for value in data:
            chunk.append(int(value) & 0xFF)
            if len(chunk) >= 65_536:
                digest.update(chunk)
                chunk.clear()
        if chunk:
            digest.update(chunk)
    return digest.hexdigest(), size


def _serialize_octomap(world) -> dict[str, object]:
    """Return bounded OctoMap audit metadata, never the raw voxel payload."""

    container = getattr(world, "octomap", None)
    octomap = getattr(container, "octomap", None)
    data = getattr(octomap, "data", ()) if octomap is not None else ()
    if data is None or not len(data):
        return {"present": False}
    digest, data_bytes = _octomap_data_digest(data)
    header = getattr(container, "header", None)
    origin = getattr(container, "origin", None)
    resolution = float(getattr(octomap, "resolution", 0.0))
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("resolução OctoMap inválida")
    return {
        "present": True,
        "frame_id": str(
            getattr(header, "frame_id", "") or "base_link"
        ).strip().lstrip("/"),
        "pose": (
            [
                float(origin.position.x),
                float(origin.position.y),
                float(origin.position.z),
                *_normalized_quaternion(origin),
            ]
            if origin is not None
            else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        ),
        "tree_id": str(getattr(octomap, "id", "")),
        "resolution_m": resolution,
        "binary": bool(getattr(octomap, "binary", False)),
        "data_bytes": data_bytes,
        "content_sha256": digest,
        "render_status": "metadata_only",
    }


def _serialize_collision_object(
    collision_object,
    *,
    attached_to: str = "",
    max_mesh_vertices: int = 20_000,
    max_mesh_triangles: int = 40_000,
) -> tuple[list[dict[str, object]], int]:
    """Sanitize one MoveIt collision object without passing ROS messages to Qt."""

    geometries: list[dict[str, object]] = []
    truncated = 0
    object_id = str(getattr(collision_object, "id", ""))
    header = getattr(collision_object, "header", None)
    frame_id = str(
        getattr(header, "frame_id", "") or attached_to or "base_link"
    ).strip().lstrip("/")
    gui_managed = object_id.startswith(_GUI_SCENE_OBJECT_PREFIX)
    object_pose = getattr(collision_object, "pose", None)

    primitives = list(getattr(collision_object, "primitives", ()) or ())
    primitive_poses = list(
        getattr(collision_object, "primitive_poses", ()) or ()
    )
    primitive_names = {
        int(getattr(SolidPrimitive, "BOX", 1)): "box",
        int(getattr(SolidPrimitive, "SPHERE", 2)): "sphere",
        int(getattr(SolidPrimitive, "CYLINDER", 3)): "cylinder",
        int(getattr(SolidPrimitive, "CONE", 4)): "cone",
    }
    for index, primitive in enumerate(primitives):
        if index >= len(primitive_poses):
            truncated += 1
            continue
        shape = primitive_names.get(int(getattr(primitive, "type", 0)))
        if shape is None:
            truncated += 1
            continue
        geometries.append(
            {
                "object_id": object_id,
                "frame_id": frame_id,
                "shape": shape,
                "pose": _composed_pose_payload(
                    object_pose,
                    primitive_poses[index],
                ),
                "dimensions": [
                    float(value)
                    for value in getattr(primitive, "dimensions", ())
                ],
                "vertices": [],
                "triangles": [],
                "attached_to": str(attached_to),
                "gui_managed": gui_managed,
            }
        )

    meshes = list(getattr(collision_object, "meshes", ()) or ())
    mesh_poses = list(getattr(collision_object, "mesh_poses", ()) or ())
    for index, mesh in enumerate(meshes):
        if index >= len(mesh_poses):
            truncated += 1
            continue
        raw_vertices = list(getattr(mesh, "vertices", ()) or ())
        raw_triangles = list(getattr(mesh, "triangles", ()) or ())
        if (
            len(raw_vertices) > max_mesh_vertices
            or len(raw_triangles) > max_mesh_triangles
        ):
            truncated += 1
            continue
        vertices = [
            [float(vertex.x), float(vertex.y), float(vertex.z)]
            for vertex in raw_vertices
        ]
        triangles = [
            [int(value) for value in getattr(face, "vertex_indices", ())[:3]]
            for face in raw_triangles
            if len(getattr(face, "vertex_indices", ())) >= 3
        ]
        geometries.append(
            {
                "object_id": object_id,
                "frame_id": frame_id,
                "shape": "mesh",
                "pose": _composed_pose_payload(
                    object_pose,
                    mesh_poses[index],
                ),
                "dimensions": [],
                "vertices": vertices,
                "triangles": triangles,
                "attached_to": str(attached_to),
                "gui_managed": gui_managed,
            }
        )
    planes = list(getattr(collision_object, "planes", ()) or ())
    plane_poses = list(getattr(collision_object, "plane_poses", ()) or ())
    for index, plane in enumerate(
        planes[:_MAX_PLANES_PER_COLLISION_OBJECT]
    ):
        if index >= len(plane_poses):
            truncated += 1
            continue
        try:
            coefficients = _normalized_plane_coefficients(
                getattr(plane, "coef", ())
            )
            pose = _composed_pose_payload(
                object_pose,
                plane_poses[index],
            )
        except (TypeError, ValueError):
            truncated += 1
            continue
        geometries.append(
            {
                "object_id": object_id,
                "frame_id": frame_id,
                "shape": "plane",
                "pose": pose,
                "dimensions": [_MONITORED_PLANE_DISPLAY_SIZE_M],
                "plane_coefficients": coefficients,
                "vertices": [],
                "triangles": [],
                "attached_to": str(attached_to),
                "gui_managed": gui_managed,
            }
        )
    truncated += max(0, len(planes) - _MAX_PLANES_PER_COLLISION_OBJECT)
    return geometries, truncated


def serialize_planning_scene(scene) -> dict[str, object]:
    """Create a deterministic, GUI-safe snapshot of MoveIt's world geometry."""

    geometries: list[dict[str, object]] = []
    truncated = 0
    world = getattr(scene, "world", None)
    for collision_object in getattr(world, "collision_objects", ()) or ():
        items, skipped = _serialize_collision_object(collision_object)
        geometries.extend(items)
        truncated += skipped
    robot_state = getattr(scene, "robot_state", None)
    for attached in getattr(robot_state, "attached_collision_objects", ()) or ():
        items, skipped = _serialize_collision_object(
            attached.object,
            attached_to=str(getattr(attached, "link_name", "")),
        )
        geometries.extend(items)
        truncated += skipped
    return {
        "name": str(getattr(scene, "name", "")),
        "geometries": geometries,
        "truncated_geometry_count": truncated,
        "octomap": _serialize_octomap(world),
    }


def cell_scene_reconciliation(
    observed_ids,
    obstacles: list[dict[str, object]],
) -> tuple[set[str], tuple[str, ...]]:
    """Return desired GUI IDs and stale world IDs that must be removed."""

    known = {
        str(object_id)
        for object_id in observed_ids
        if str(object_id).startswith(_GUI_SCENE_OBJECT_PREFIX)
    }
    desired = {
        f"{_GUI_SCENE_OBJECT_PREFIX}{str(item['object_id'])}"
        for item in obstacles
        if bool(item.get("enabled", True))
    }
    return desired, tuple(sorted(known - desired))


def _parameter_value(value: object) -> ParameterValue:
    parameter_value = ParameterValue()
    if isinstance(value, bool):
        parameter_value.type = ParameterType.PARAMETER_BOOL
        parameter_value.bool_value = bool(value)
    elif isinstance(value, int):
        parameter_value.type = ParameterType.PARAMETER_INTEGER
        parameter_value.integer_value = int(value)
    elif isinstance(value, float):
        parameter_value.type = ParameterType.PARAMETER_DOUBLE
        parameter_value.double_value = float(value)
    elif isinstance(value, str):
        parameter_value.type = ParameterType.PARAMETER_STRING
        parameter_value.string_value = str(value)
    elif isinstance(value, (list, tuple)):
        if all(isinstance(item, int) and not isinstance(item, bool) for item in value):
            parameter_value.type = ParameterType.PARAMETER_INTEGER_ARRAY
            parameter_value.integer_array_value = [int(item) for item in value]
        elif all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in value):
            parameter_value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            parameter_value.double_array_value = [float(item) for item in value]
        else:
            parameter_value.type = ParameterType.PARAMETER_STRING_ARRAY
            parameter_value.string_array_value = [str(item) for item in value]
    else:
        parameter_value.type = ParameterType.PARAMETER_STRING
        parameter_value.string_value = str(value)
    return parameter_value


class _BridgeNode(Node):
    """Internal node owning every subscription / client."""

    def __init__(self, namespace: str = "rebotarm") -> None:
        super().__init__("rebotarm_gui")
        self._ns = namespace.strip().strip("/")
        self._cb_reentrant = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )
        self._diagnostic_lock = threading.RLock()
        self._topic_metrics: dict[str, dict[str, object]] = {}
        self._telemetry_errors: deque[str] = deque(maxlen=20)
        self._last_telemetry_error: tuple[str, float] | None = None

        # ---- Latest state cache (read from the UI thread) ----
        self.joint_state: Optional[JointState] = None
        self.arm_status: Optional[ArmStatus] = None
        self.timing_status: Optional[TimingStatus] = None
        self._timing_status_dirty = False
        self.last_arm_status_stamp: Optional[float] = None  # monotonic; heartbeat
        self.robot_description: Optional[str] = None
        self.trajectory_limits_data: dict[str, object] = {}
        self.speed_override_status: dict[str, object] = {}
        self._speed_override_status_dirty = False
        self.gripper_state: Optional[JointMotorState] = None
        self.joint_detail: dict[str, JointMotorState] = {}
        self.dio_ios: dict[str, DigitalIO] = {}  # id -> latest DigitalIO snapshot
        self.vision_detected: bool = False
        self.vision_pose: Optional[GeometryPose] = None
        self.vision_objects: Optional[SegmentedObjectArray] = None
        self.vision_debug_image = None
        self.vision_raw_image = None
        self._vision_objects_dirty = False
        self._vision_debug_dirty = False
        self._vision_raw_dirty = False
        self._vision_calibration_progress_message: Optional[str] = None
        self._vision_calibration_progress_dirty = False

        # ---- Subscriptions ----
        self.create_subscription(
            JointState,
            f"/{self._ns}/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            ArmStatus,
            f"/{self._ns}/arm_status",
            self._arm_status_cb,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )

        self.create_subscription(
            TimingStatus,
            f"/{self._ns}/timing_status",
            self._timing_status_cb,
            10,
        )

        self.create_subscription(
            JointMotorState,
            f"/{self._ns}/gripper/state",
            self._gripper_state_cb,
            qos_profile_sensor_data,
        )

        # URDF published by robot_state_publisher (latched). Used to derive
        # joint limits and forward kinematics for the UI. Subscribed both at
        # the global scope (real bringup) and inside the arm namespace (the
        # mock_visualize launch namespaces its robot_state_publisher).
        for urdf_topic in ("/robot_description", f"/{self._ns}/robot_description"):
            self.create_subscription(
                String,
                urdf_topic,
                lambda msg, topic=urdf_topic: self._robot_description_cb(msg, topic),
                QoSProfile(
                    depth=1,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                ),
            )

        self.create_subscription(
            String,
            f"/{self._ns}/trajectory_limits",
            self._trajectory_limits_cb,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )
        self.create_subscription(
            String,
            f"/{self._ns}/speed_override/status",
            self._speed_override_status_cb,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )

        self.create_subscription(
            DigitalIOState,
            f"/{self._ns}/dio/state",
            self._digital_io_cb,
            qos_profile_sensor_data,
        )

        # Computer-vision signals (published by an external webcam/vision node
        # on the Raspberry Pi). Subscribed defensively: the topics may not
        # exist yet, in which case the cache keeps its defaults.
        self.create_subscription(
            Bool,
            f"/{self._ns}/vision/detected",
            self._vision_detected_cb,
            10,
        )
        self.create_subscription(
            GeometryPose,
            f"/{self._ns}/vision/part_pose",
            self._vision_pose_cb,
            10,
        )
        self.create_subscription(
            SegmentedObjectArray,
            f"/{self._ns}/vision/objects",
            lambda msg: self._vision_objects_cb(msg, f"/{self._ns}/vision/objects"),
            10,
        )
        self.create_subscription(
            SegmentedObjectArray,
            f"/{self._ns}/vision/objects_2d",
            lambda msg: self._vision_objects_cb(msg, f"/{self._ns}/vision/objects_2d"),
            10,
        )
        self.create_subscription(
            Image,
            f"/{self._ns}/vision/debug_image",
            self._vision_debug_image_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/image_raw",
            self._vision_raw_image_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/rebotarm_vision_calibration/progress",
            self._vision_calibration_progress_cb,
            10,
        )

        # Joint detail topics are created lazily once joint names are known.
        self._joint_detail_created: set[str] = set()

        # ---- Service clients ----
        self.cli_enable = self.create_client(Trigger, f"/{self._ns}/enable")
        self.cli_disable = self.create_client(Trigger, f"/{self._ns}/disable")
        self.cli_safe_home = self.create_client(Trigger, f"/{self._ns}/safe_home")
        self.cli_grav_start = self.create_client(
            Trigger, f"/{self._ns}/gravity_compensation/start"
        )
        self.cli_grav_stop = self.create_client(
            Trigger, f"/{self._ns}/gravity_compensation/stop"
        )
        self.cli_set_zero = self.create_client(SetZero, f"/{self._ns}/set_zero")
        self.cli_move_to_pose_ik = self.create_client(
            MoveToPoseIK, f"/{self._ns}/move_to_pose_ik", callback_group=self._cb_reentrant
        )
        self.cli_gripper_set = self.create_client(
            SetGripper, f"/{self._ns}/gripper/set", callback_group=self._cb_reentrant
        )
        self.cli_gripper_open = self.create_client(
            GripperCommandSrv, f"/{self._ns}/gripper/open", callback_group=self._cb_reentrant
        )
        self.cli_gripper_close = self.create_client(
            GripperCommandSrv, f"/{self._ns}/gripper/close", callback_group=self._cb_reentrant
        )
        self.cli_safety_set_profile = self.create_client(
            SetTorqueProfile,
            f"/{self._ns}/safety/set_torque_profile",
            callback_group=self._cb_reentrant,
        )
        self.cli_safety_set_step = self.create_client(
            SetSafetyStep,
            f"/{self._ns}/safety/set_step",
            callback_group=self._cb_reentrant,
        )
        self.cli_safety_check = self.create_client(
            CheckTorqueProfile,
            f"/{self._ns}/safety/check_torque_profile",
            callback_group=self._cb_reentrant,
        )
        self.cli_safety_reset_fault = self.create_client(
            Trigger,
            f"/{self._ns}/safety/reset_fault",
            callback_group=self._cb_reentrant,
        )
        self.cli_vision_set_parameters = self.create_client(
            SetParameters,
            "/rebotarm_segmentation/set_parameters",
            callback_group=self._cb_reentrant,
        )
        self.cli_vision_pose_set_parameters = self.create_client(
            SetParameters,
            "/rebotarm_object_pose/set_parameters",
            callback_group=self._cb_reentrant,
        )
        self.cli_pick_scheduler_set_parameters = self.create_client(
            SetParameters,
            "/rebotarm_pick_scheduler/set_parameters",
            callback_group=self._cb_reentrant,
        )
        self.cli_vision_process_frame = self.create_client(
            Trigger,
            "/rebotarm_segmentation/process_frame",
            callback_group=self._cb_reentrant,
        )
        self.cli_vision_calibrate = self.create_client(
            CalibrateVision,
            "/rebotarm_vision_calibration/calibrate",
            callback_group=self._cb_reentrant,
        )
        self.cli_dio_set_output = self.create_client(
            SetDigitalOutput,
            f"/{self._ns}/dio/set_output",
            callback_group=self._cb_reentrant,
        )
        self.cli_dio_set_config = self.create_client(
            SetDigitalIOConfig,
            f"/{self._ns}/dio/set_config",
            callback_group=self._cb_reentrant,
        )
        self.cli_apply_planning_scene = (
            self.create_client(
                ApplyPlanningScene,
                "/apply_planning_scene",
                callback_group=self._cb_reentrant,
            )
            if MOVEIT_AVAILABLE
            else None
        )
        self.cli_get_planning_scene = (
            self.create_client(
                GetPlanningScene,
                "/get_planning_scene",
                callback_group=self._cb_reentrant,
            )
            if MOVEIT_AVAILABLE
            else None
        )
        self.cli_get_motion_plan = (
            self.create_client(
                GetMotionPlan,
                "/plan_kinematic_path",
                callback_group=self._cb_reentrant,
            )
            if MOVEIT_AVAILABLE
            else None
        )
        self.cli_get_state_validity = (
            self.create_client(
                GetStateValidity,
                "/check_state_validity",
                callback_group=self._cb_reentrant,
            )
            if MOVEIT_AVAILABLE
            else None
        )
        self.cli_check_trajectory_clearance = self.create_client(
            CheckTrajectoryClearance,
            "/rebotarm/check_trajectory_clearance",
            callback_group=self._cb_reentrant,
        )
        self._cell_scene_object_ids: set[str] = set()
        self._cell_scene_ids_lock = threading.RLock()

        # ---- Action clients ----
        self.ac_move_to_pose = ActionClient(
            self, MoveToPose, f"/{self._ns}/move_to_pose", callback_group=self._cb_reentrant
        )
        self.ac_follow_joint_trajectory = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._ns}/follow_joint_trajectory",
            callback_group=self._cb_reentrant,
        )
        self.ac_gripper_command = ActionClient(
            self,
            GripperCommand,
            f"/{self._ns}/gripper/command",
            callback_group=self._cb_reentrant,
        )
        self.ac_move_group = (
            ActionClient(
                self,
                MoveGroup,
                "/move_action",
                callback_group=self._cb_reentrant,
            )
            if MOVEIT_AVAILABLE
            else None
        )

        # ---- Raw command publishers (created lazily) ----
        self._pubs_mit: dict[str, object] = {}
        self._pubs_posvel: dict[str, object] = {}
        self._pub_vision_test_image = self.create_publisher(Image, "/camera/image_raw", 10)
        self._pub_authorization_lease = self.create_publisher(
            String, f"/{self._ns}/authorization/lease", 10
        )
        self._pub_speed_override = self.create_publisher(
            String,
            f"/{self._ns}/speed_override/command",
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )

        # Stable names are kept alongside the clients so the GUI can expose
        # discovery state without invoking command-line ROS tools.
        self._diagnostic_services = {
            f"/{self._ns}/enable": self.cli_enable,
            f"/{self._ns}/disable": self.cli_disable,
            f"/{self._ns}/safe_home": self.cli_safe_home,
            f"/{self._ns}/gravity_compensation/start": self.cli_grav_start,
            f"/{self._ns}/gravity_compensation/stop": self.cli_grav_stop,
            f"/{self._ns}/set_zero": self.cli_set_zero,
            f"/{self._ns}/move_to_pose_ik": self.cli_move_to_pose_ik,
            f"/{self._ns}/gripper/set": self.cli_gripper_set,
            f"/{self._ns}/gripper/open": self.cli_gripper_open,
            f"/{self._ns}/gripper/close": self.cli_gripper_close,
            f"/{self._ns}/safety/set_torque_profile": self.cli_safety_set_profile,
            f"/{self._ns}/safety/set_step": self.cli_safety_set_step,
            f"/{self._ns}/safety/check_torque_profile": self.cli_safety_check,
            f"/{self._ns}/safety/reset_fault": self.cli_safety_reset_fault,
            "/rebotarm_segmentation/set_parameters": self.cli_vision_set_parameters,
            "/rebotarm_object_pose/set_parameters": self.cli_vision_pose_set_parameters,
            "/rebotarm_pick_scheduler/set_parameters": self.cli_pick_scheduler_set_parameters,
            "/rebotarm_segmentation/process_frame": self.cli_vision_process_frame,
            "/rebotarm_vision_calibration/calibrate": self.cli_vision_calibrate,
            f"/{self._ns}/dio/set_output": self.cli_dio_set_output,
            f"/{self._ns}/dio/set_config": self.cli_dio_set_config,
        }
        if self.cli_apply_planning_scene is not None:
            self._diagnostic_services["/apply_planning_scene"] = (
                self.cli_apply_planning_scene
            )
        if self.cli_get_planning_scene is not None:
            self._diagnostic_services["/get_planning_scene"] = (
                self.cli_get_planning_scene
            )
        if self.cli_get_motion_plan is not None:
            self._diagnostic_services["/plan_kinematic_path"] = (
                self.cli_get_motion_plan
            )
        if self.cli_get_state_validity is not None:
            self._diagnostic_services["/check_state_validity"] = (
                self.cli_get_state_validity
            )
        self._diagnostic_services["/rebotarm/check_trajectory_clearance"] = (
            self.cli_check_trajectory_clearance
        )
        self._diagnostic_actions = {
            f"/{self._ns}/move_to_pose": self.ac_move_to_pose,
            f"/{self._ns}/follow_joint_trajectory": self.ac_follow_joint_trajectory,
            f"/{self._ns}/gripper/command": self.ac_gripper_command,
        }
        if self.ac_move_group is not None:
            self._diagnostic_actions["/move_action"] = self.ac_move_group

        for topic in (
            f"/{self._ns}/joint_states",
            f"/{self._ns}/arm_status",
            f"/{self._ns}/gripper/state",
            "/robot_description",
            f"/{self._ns}/robot_description",
            f"/{self._ns}/dio/state",
            f"/{self._ns}/vision/detected",
            f"/{self._ns}/vision/part_pose",
            f"/{self._ns}/vision/objects",
            f"/{self._ns}/vision/objects_2d",
            f"/{self._ns}/vision/debug_image",
            f"/{self._ns}/trajectory_limits",
            f"/{self._ns}/speed_override/status",
            "/camera/image_raw",
            "/rebotarm_vision_calibration/progress",
        ):
            self._ensure_topic_metric(topic)

    # ------------------------------------------------------------------
    # namespace helpers
    # ------------------------------------------------------------------
    @property
    def namespace(self) -> str:
        return self._ns

    def arm_joint_names(self) -> list[str]:
        """Return only the arm joint names (excluding ``gripper*``)."""
        if self.joint_state is None:
            return []
        return [n for n in self.joint_state.name if not n.startswith("gripper")]

    def all_joint_names(self) -> list[str]:
        return list(self.joint_state.name) if self.joint_state else []

    # ------------------------------------------------------------------
    # diagnostics
    # ------------------------------------------------------------------
    def _ensure_topic_metric(self, topic: str) -> dict[str, object]:
        with self._diagnostic_lock:
            return self._topic_metrics.setdefault(
                str(topic),
                {
                    "samples": deque(maxlen=500),
                    "last_seen": None,
                    "latency_ms": None,
                },
            )

    def _record_topic(self, topic: str, msg: object) -> None:
        """Record receipt time and header latency for one subscribed topic."""
        now = time.monotonic()
        latency_ms: float | None = None
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None)
        sec = int(getattr(stamp, "sec", 0)) if stamp is not None else 0
        nanosec = int(getattr(stamp, "nanosec", 0)) if stamp is not None else 0
        if sec or nanosec:
            message_ns = sec * 1_000_000_000 + nanosec
            latency_ms = (self.get_clock().now().nanoseconds - message_ns) / 1_000_000.0
            # A different clock domain (or a system clock jump) makes transport
            # latency meaningless. Preserve the topic rate but report N/D.
            if abs(latency_ms) > 86_400_000.0:
                latency_ms = None
        metric = self._ensure_topic_metric(topic)
        with self._diagnostic_lock:
            samples = metric["samples"]
            if isinstance(samples, deque):
                samples.append(now)
            metric["last_seen"] = now
            metric["latency_ms"] = latency_ms

    def diagnostic_snapshot(self, heartbeat_timeout_s: float = 1.0) -> dict[str, object]:
        """Return a non-blocking snapshot of topics and endpoint discovery."""
        now = time.monotonic()
        topics: list[dict[str, object]] = []
        with self._diagnostic_lock:
            metrics = list(self._topic_metrics.items())
            for topic, metric in metrics:
                samples = metric["samples"]
                if isinstance(samples, deque):
                    while samples and now - samples[0] > 5.0:
                        samples.popleft()
                    hz = (
                        (len(samples) - 1) / (samples[-1] - samples[0])
                        if len(samples) >= 2 and samples[-1] > samples[0]
                        else None
                    )
                else:  # pragma: no cover - defensive
                    hz = None
                last_seen = metric["last_seen"]
                age_s = now - float(last_seen) if last_seen is not None else None
                topics.append(
                    {
                        "name": topic,
                        "hz": hz,
                        "age_ms": age_s * 1000.0 if age_s is not None else None,
                        "latency_ms": metric["latency_ms"],
                        "received": last_seen is not None,
                    }
                )

        services = [
            {"name": name, "ready": self._client_ready(client, action=False)}
            for name, client in self._diagnostic_services.items()
        ]
        actions = [
            {"name": name, "ready": self._client_ready(client, action=True)}
            for name, client in self._diagnostic_actions.items()
        ]
        heartbeat = next(
            (row for row in topics if row["name"] == f"/{self._ns}/arm_status"),
            {"hz": None, "age_ms": None, "latency_ms": None, "received": False},
        )
        heartbeat_age = heartbeat.get("age_ms")
        heartbeat["alive"] = bool(
            heartbeat_age is not None
            and float(heartbeat_age) < max(0.0, float(heartbeat_timeout_s)) * 1000.0
        )
        heartbeat["timeout_ms"] = float(heartbeat_timeout_s) * 1000.0
        return {
            "connected": True,
            "heartbeat": heartbeat,
            "topics": sorted(topics, key=lambda item: str(item["name"])),
            "services": services,
            "actions": actions,
            "captured_at": time.time(),
        }

    @staticmethod
    def _client_ready(client: object, *, action: bool) -> bool:
        try:
            if action:
                return bool(client.server_is_ready())
            return bool(client.service_is_ready())
        except Exception:
            return False

    # ------------------------------------------------------------------
    # subscription callbacks
    # ------------------------------------------------------------------
    @staticmethod
    def _validate_joint_state(msg: JointState) -> str | None:
        """Return a safety-oriented validation error, or ``None`` when usable."""
        names = list(msg.name)
        if not names:
            return "lista de juntas vazia"
        if any(not str(name).strip() for name in names):
            return "nome de junta vazio"
        if len(set(names)) != len(names):
            return "nomes de juntas duplicados"
        if len(msg.position) != len(names):
            return f"position possui {len(msg.position)} valor(es) para {len(names)} junta(s)"
        for field_name in ("velocity", "effort"):
            values = getattr(msg, field_name)
            if values and len(values) != len(names):
                return f"{field_name} possui {len(values)} valor(es) para {len(names)} junta(s)"
        numeric_values = list(msg.position) + list(msg.velocity) + list(msg.effort)
        if any(not math.isfinite(float(value)) for value in numeric_values):
            return "valor NaN ou infinito"
        return None

    def _queue_telemetry_error(self, topic: str, reason: str) -> None:
        now = time.monotonic()
        key = f"{topic}: {reason}"
        with self._diagnostic_lock:
            previous = self._last_telemetry_error
            if previous is not None and previous[0] == key and now - previous[1] < 1.0:
                return
            self._last_telemetry_error = (key, now)
            self._telemetry_errors.append(key)

    def take_telemetry_errors(self) -> list[str]:
        """Drain validation errors for delivery on the Qt thread boundary."""
        with self._diagnostic_lock:
            errors = list(self._telemetry_errors)
            self._telemetry_errors.clear()
        return errors

    def _joint_state_cb(self, msg: JointState) -> None:
        topic = f"/{self._ns}/joint_states"
        self._record_topic(topic, msg)
        validation_error = self._validate_joint_state(msg)
        if validation_error is not None:
            self._queue_telemetry_error(topic, validation_error)
            return
        self.joint_state = msg
        self._ensure_joint_detail_subs()

    def _arm_status_cb(self, msg: ArmStatus) -> None:
        self._record_topic(f"/{self._ns}/arm_status", msg)
        self.arm_status = msg
        self.last_arm_status_stamp = time.monotonic()

    def _timing_status_cb(self, msg: TimingStatus) -> None:
        self._record_topic(f"/{self._ns}/timing_status", msg)
        self.timing_status = msg
        self._timing_status_dirty = True

    def _robot_description_cb(self, msg: String, topic: str = "/robot_description") -> None:
        self._record_topic(topic, msg)
        if msg.data:
            self.robot_description = str(msg.data)

    def _trajectory_limits_cb(self, msg: String) -> None:
        topic = f"/{self._ns}/trajectory_limits"
        self._record_topic(topic, msg)
        try:
            data = parse_trajectory_limits_payload(msg.data)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self._queue_telemetry_error(topic, "JSON de limites dinâmicos inválido")
            return
        self.trajectory_limits_data = data

    def _speed_override_status_cb(self, msg: String) -> None:
        topic = f"/{self._ns}/speed_override/status"
        self._record_topic(topic, msg)
        try:
            data = json.loads(str(msg.data))
            if int(data.get("version", 0)) != 1 or not bool(
                data.get("supported", False)
            ):
                raise ValueError("versão/capacidade não suportada")
            for key in ("requested", "applied", "minimum", "maximum"):
                value = float(data[key])
                if not math.isfinite(value):
                    raise ValueError(f"{key} não finito")
            if not 0.0 < float(data["minimum"]) <= float(data["maximum"]) <= 1.0:
                raise ValueError("faixa inválida")
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._queue_telemetry_error(topic, f"status inválido: {exc}")
            return
        self.speed_override_status = data
        self._speed_override_status_dirty = True

    def _gripper_state_cb(self, msg: JointMotorState) -> None:
        self._record_topic(f"/{self._ns}/gripper/state", msg)
        self.gripper_state = msg

    def _digital_io_cb(self, msg: DigitalIOState) -> None:
        self._record_topic(f"/{self._ns}/dio/state", msg)
        ios = getattr(msg, "ios", None)
        if not ios:
            return
        dio_ios: dict[str, DigitalIO] = {}
        for io in ios:
            dio_ios[str(io.id)] = io
        self.dio_ios = dio_ios

    def _vision_detected_cb(self, msg: Bool) -> None:
        self._record_topic(f"/{self._ns}/vision/detected", msg)
        self.vision_detected = bool(msg.data)

    def _vision_pose_cb(self, msg: GeometryPose) -> None:
        self._record_topic(f"/{self._ns}/vision/part_pose", msg)
        self.vision_pose = msg

    def _vision_objects_cb(self, msg: SegmentedObjectArray, topic: str) -> None:
        self._record_topic(topic, msg)
        self.vision_objects = msg
        self._vision_objects_dirty = True
        self.vision_detected = bool(msg.objects)
        if msg.objects:
            self.vision_pose = msg.objects[0].pose

    def _vision_debug_image_cb(self, msg: Image) -> None:
        self._record_topic(f"/{self._ns}/vision/debug_image", msg)
        self.vision_debug_image = msg
        self._vision_debug_dirty = True

    def _vision_raw_image_cb(self, msg: Image) -> None:
        self._record_topic("/camera/image_raw", msg)
        self.vision_raw_image = msg
        self._vision_raw_dirty = True

    def _vision_calibration_progress_cb(self, msg: String) -> None:
        self._record_topic("/rebotarm_vision_calibration/progress", msg)
        self._vision_calibration_progress_message = str(msg.data)
        self._vision_calibration_progress_dirty = True

    def _joint_detail_cb_factory(self, joint_name: str):
        def _cb(msg: JointMotorState) -> None:
            self._record_topic(f"/{self._ns}/joints/{joint_name}/state", msg)
            self.joint_detail[joint_name] = msg

        return _cb

    def _ensure_joint_detail_subs(self) -> None:
        if self.joint_state is None:
            return
        for name in self.joint_state.name:
            if name in self._joint_detail_created:
                continue
            self.create_subscription(
                JointMotorState,
                f"/{self._ns}/joints/{name}/state",
                self._joint_detail_cb_factory(name),
                qos_profile_sensor_data,
            )
            self._ensure_topic_metric(f"/{self._ns}/joints/{name}/state")
            self._joint_detail_created.add(name)

    def ensure_raw_publishers(self) -> None:
        """Create raw command publishers for every known joint."""
        if self.joint_state is None:
            return
        for name in self.joint_state.name:
            if name not in self._pubs_mit:
                self._pubs_mit[name] = self.create_publisher(
                    JointMitCmd, f"/{self._ns}/joints/{name}/cmd/mit", 10
                )
            if name not in self._pubs_posvel:
                self._pubs_posvel[name] = self.create_publisher(
                    JointPosVelCmd, f"/{self._ns}/joints/{name}/cmd/pos_vel", 10
                )

    # ------------------------------------------------------------------
    # publish helpers
    # ------------------------------------------------------------------
    def publish_mit(
        self, joint_name: str, pos: float, vel: float, kp: float, kd: float, tau: float
    ) -> bool:
        pub = self._pubs_mit.get(joint_name)
        if pub is None:
            return False
        msg = JointMitCmd(
            pos=float(pos), vel=float(vel), kp=float(kp), kd=float(kd), tau=float(tau)
        )
        msg.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)
        return True

    def publish_posvel(self, joint_name: str, pos: float, vlim: float) -> bool:
        pub = self._pubs_posvel.get(joint_name)
        if pub is None:
            return False
        msg = JointPosVelCmd(pos=float(pos), vlim=float(vlim))
        msg.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)
        return True

    # ------------------------------------------------------------------
    # service-call helpers (non-blocking, return Future)
    # ------------------------------------------------------------------
    def _call_service(self, client, request) -> Optional[Future]:
        if not client.wait_for_service(timeout_sec=0.01):
            return None
        return client.call_async(request)

    def call_enable(self) -> Optional[Future]:
        return self._call_service(self.cli_enable, Trigger.Request())

    def call_disable(self) -> Optional[Future]:
        return self._call_service(self.cli_disable, Trigger.Request())

    def call_safe_home(self) -> Optional[Future]:
        return self._call_service(self.cli_safe_home, Trigger.Request())

    def call_gravity_start(self) -> Optional[Future]:
        return self._call_service(self.cli_grav_start, Trigger.Request())

    def call_gravity_stop(self) -> Optional[Future]:
        return self._call_service(self.cli_grav_stop, Trigger.Request())

    def call_set_zero(self, joint_name: str = "") -> Optional[Future]:
        req = SetZero.Request()
        req.joint_name = joint_name
        return self._call_service(self.cli_set_zero, req)

    def call_move_to_pose_ik(self, pose: Pose) -> Optional[Future]:
        req = MoveToPoseIK.Request()
        req.target_pose = pose
        return self._call_service(self.cli_move_to_pose_ik, req)

    def call_apply_cell_scene(
        self,
        *,
        frame_id: str,
        minimum_clearance_m: float,
        obstacles: list[dict[str, object]],
    ) -> tuple[Optional[Future], int]:
        """Mirror GUI cell boxes into MoveIt's monitored planning scene."""

        client = self.cli_apply_planning_scene
        if client is None:
            return None, 0
        with self._cell_scene_ids_lock:
            desired_ids, stale_ids = cell_scene_reconciliation(
                self._cell_scene_object_ids,
                obstacles,
            )
        scene = PlanningScene(is_diff=True)
        for stale_id in stale_ids:
            scene.world.collision_objects.append(
                CollisionObject(
                    id=stale_id,
                    operation=CollisionObject.REMOVE,
                )
            )
        for item in obstacles:
            if not bool(item.get("enabled", True)):
                continue
            center = [float(value) for value in item["center"]]
            size = [float(value) for value in item["size"]]
            inflation = max(
                0.0,
                float(item.get("margin_m", 0.0))
                + float(minimum_clearance_m),
            )
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                max(1e-6, value + 2.0 * inflation) for value in size
            ]
            pose = make_pose(center[0], center[1], center[2])
            scene.world.collision_objects.append(
                CollisionObject(
                    id=(
                        f"{_GUI_SCENE_OBJECT_PREFIX}"
                        f"{str(item['object_id'])}"
                    ),
                    header=Header(frame_id=str(frame_id)),
                    primitives=[primitive],
                    primitive_poses=[pose],
                    operation=CollisionObject.ADD,
                )
            )
        future = self._call_service(
            client,
            ApplyPlanningScene.Request(scene=scene),
        )
        if future is not None:
            def commit_if_accepted(completed: Future) -> None:
                try:
                    accepted = bool(completed.result().success)
                except Exception:
                    accepted = False
                if accepted:
                    with self._cell_scene_ids_lock:
                        self._cell_scene_object_ids = set(desired_ids)

            future.add_done_callback(commit_if_accepted)
        return future, len(stale_ids)

    def observe_cell_scene_world(self, scene) -> set[str]:
        """Replace known GUI IDs from an authoritative MoveIt world snapshot."""

        world = getattr(scene, "world", None)
        observed = {
            str(getattr(item, "id", ""))
            for item in getattr(world, "collision_objects", ()) or ()
            if str(getattr(item, "id", "")).startswith(
                _GUI_SCENE_OBJECT_PREFIX
            )
        }
        with self._cell_scene_ids_lock:
            self._cell_scene_object_ids = observed
        return set(observed)

    @property
    def cell_scene_object_ids(self) -> set[str]:
        with self._cell_scene_ids_lock:
            return set(self._cell_scene_object_ids)

    def call_get_planning_scene(self) -> Optional[Future]:
        """Read world and attached collision geometry from MoveIt."""

        client = self.cli_get_planning_scene
        if client is None:
            return None
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | getattr(PlanningSceneComponents, "OCTOMAP", 0)
        )
        return self._call_service(client, request)

    def resolve_planning_scene_frames(
        self,
        payload: dict[str, object],
        target_frame: str,
        *,
        timeout_s: float = 0.02,
    ) -> dict[str, object]:
        """Resolve each unique planning-scene frame through the live TF tree."""

        target = str(target_frame).strip().lstrip("/") or "base_link"
        bounded_timeout = min(max(float(timeout_s), 0.0), 0.02)
        source_frames = sorted(
            {
                str(item.get("frame_id", "")).strip().lstrip("/")
                for item in payload.get("geometries", ())
                if isinstance(item, dict)
                and str(item.get("frame_id", "")).strip().lstrip("/")
                and str(item.get("frame_id", "")).strip().lstrip("/") != target
            }
        )
        transforms: dict[str, tuple[float, ...]] = {}
        errors: dict[str, str] = {}
        for source in source_frames[:32]:
            try:
                stamped = self.tf_buffer.lookup_transform(
                    target,
                    source,
                    Time(),
                    timeout=RclpyDuration(seconds=bounded_timeout),
                )
                transform = stamped.transform
                candidate = (
                    float(transform.translation.x),
                    float(transform.translation.y),
                    float(transform.translation.z),
                    float(transform.rotation.x),
                    float(transform.rotation.y),
                    float(transform.rotation.z),
                    float(transform.rotation.w),
                )
                if not all(
                    math.isfinite(value) for value in candidate[:3]
                ):
                    raise ValueError("translação TF não finita")
                _normalized_quaternion_values(candidate[3:])
                transforms[source] = candidate
            except (TransformException, TypeError, ValueError) as exc:
                errors[source] = str(exc)
        for source in source_frames[32:]:
            errors[source] = "limite de 32 frames TF excedido"
        resolved = transform_planning_scene_payload(
            payload,
            target,
            transforms,
        )
        resolved["tf_errors"] = errors
        return resolved

    def call_trajectory_clearance(
        self,
        samples: list[tuple[list[str], list[float]]],
        *,
        group_name: str = "arm",
    ) -> Optional[Future]:
        """Measure full-robot clearance for a bounded batch of joint states."""

        client = self.cli_check_trajectory_clearance
        if client is None or len(samples) > 120:
            return None
        request = CheckTrajectoryClearance.Request()
        request.group_name = str(group_name)
        for names, positions in samples:
            if not names or len(names) != len(positions):
                return None
            state = JointState()
            state.name = [str(name) for name in names]
            state.position = [float(value) for value in positions]
            request.states.append(state)
        return self._call_service(client, request)

    @staticmethod
    def _pose_from_mapping(pose_data: dict[str, object]) -> Pose:
        from .motion.trajectory import rpy_to_quat

        qx, qy, qz, qw = rpy_to_quat(
            float(pose_data.get("roll", 0.0)),
            float(pose_data.get("pitch", 0.0)),
            float(pose_data.get("yaw", 0.0)),
        )
        return make_pose(
            float(pose_data["x"]),
            float(pose_data["y"]),
            float(pose_data["z"]),
            qx,
            qy,
            qz,
            qw,
        )

    @staticmethod
    def _joint_robot_state(
        joint_names: list[str],
        positions: list[float],
    ):
        state = RobotState()
        state.joint_state.name = [str(name) for name in joint_names]
        state.joint_state.position = [float(value) for value in positions]
        # Planned trajectories usually contain only the selected arm group.
        # A diff lets MoveIt retain current gripper/passive-joint values.
        state.is_diff = True
        return state

    def call_motion_plan(
        self,
        pose_data: dict[str, object],
        speed_scale: float,
        *,
        start_joint_names: list[str] | None = None,
        start_positions: list[float] | None = None,
        base_frame: str = "base_link",
        end_link: str = "gripper_tcp",
        group_name: str = "arm",
    ) -> Optional[Future]:
        """Plan one pose without execution through ``/plan_kinematic_path``."""

        client = self.cli_get_motion_plan
        if client is None:
            return None
        target = self._pose_from_mapping(pose_data)
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name = str(group_name)
        plan.num_planning_attempts = 5
        plan.allowed_planning_time = 5.0
        scale = max(0.01, min(1.0, float(speed_scale)))
        plan.max_velocity_scaling_factor = scale
        plan.max_acceleration_scaling_factor = scale
        if start_joint_names and start_positions:
            plan.start_state = self._joint_robot_state(
                start_joint_names,
                start_positions,
            )
        else:
            plan.start_state.is_diff = True

        region = PositionConstraint()
        region.header.frame_id = str(base_frame)
        region.link_name = str(end_link)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.002]
        region.constraint_region.primitives = [primitive]
        region.constraint_region.primitive_poses = [target]
        region.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = str(base_frame)
        orientation.link_name = str(end_link)
        orientation.orientation = target.orientation
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.name = "rebotarm_gui_plan_preview"
        constraints.position_constraints = [region]
        constraints.orientation_constraints = [orientation]
        plan.goal_constraints = [constraints]
        return self._call_service(client, request)

    def call_state_validity(
        self,
        joint_names: list[str],
        positions: list[float],
        *,
        group_name: str = "arm",
    ) -> Optional[Future]:
        """Ask MoveIt to check one sampled full-robot state for collision."""

        client = self.cli_get_state_validity
        if client is None:
            return None
        request = GetStateValidity.Request()
        request.robot_state = self._joint_robot_state(joint_names, positions)
        request.group_name = str(group_name)
        return self._call_service(client, request)

    def call_gripper_set(self, position: float) -> Optional[Future]:
        req = SetGripper.Request()
        req.position = float(position)
        return self._call_service(self.cli_gripper_set, req)

    def call_gripper_open(self, position: float = 0.0, timeout: float = 3.0) -> Optional[Future]:
        req = GripperCommandSrv.Request()
        req.position = float(position)
        req.timeout = float(timeout)
        return self._call_service(self.cli_gripper_open, req)

    def call_gripper_close(self, position: float = 0.0, timeout: float = 3.0) -> Optional[Future]:
        req = GripperCommandSrv.Request()
        req.position = float(position)
        req.timeout = float(timeout)
        return self._call_service(self.cli_gripper_close, req)

    def call_safety_set_profile(
        self,
        profile_path: str,
        mode: str,
        violation_samples: int,
        active_step_id: str,
        derivative_limit: float = 0.0,
    ) -> Optional[Future]:
        req = SetTorqueProfile.Request()
        req.profile_path = str(profile_path)
        req.mode = str(mode)
        req.violation_samples = int(violation_samples)
        req.active_step_id = str(active_step_id)
        req.derivative_limit = float(derivative_limit)
        return self._call_service(self.cli_safety_set_profile, req)

    def call_safety_set_step(self, step_id: str, payload_kg: float = 0.0) -> Optional[Future]:
        req = SetSafetyStep.Request()
        req.step_id = str(step_id)
        req.payload_kg = float(payload_kg)
        return self._call_service(self.cli_safety_set_step, req)

    def call_safety_check(self, step_id: str, phase: float) -> Optional[Future]:
        req = CheckTorqueProfile.Request()
        req.step_id = str(step_id)
        req.phase = float(phase)
        return self._call_service(self.cli_safety_check, req)

    def call_safety_reset_fault(self) -> Optional[Future]:
        return self._call_service(self.cli_safety_reset_fault, Trigger.Request())

    def call_vision_set_parameters(self, values: dict[str, object]) -> Optional[Future]:
        req = SetParameters.Request()
        req.parameters = [self._make_parameter(name, value) for name, value in values.items()]
        return self._call_service(self.cli_vision_set_parameters, req)

    def call_vision_pose_set_parameters(self, values: dict[str, object]) -> Optional[Future]:
        req = SetParameters.Request()
        req.parameters = [self._make_parameter(name, value) for name, value in values.items()]
        return self._call_service(self.cli_vision_pose_set_parameters, req)

    def call_pick_scheduler_set_parameters(self, values: dict[str, object]) -> Optional[Future]:
        req = SetParameters.Request()
        req.parameters = [self._make_parameter(name, value) for name, value in values.items()]
        return self._call_service(self.cli_pick_scheduler_set_parameters, req)

    def call_vision_process_frame(self) -> Optional[Future]:
        return self._call_service(self.cli_vision_process_frame, Trigger.Request())

    def call_vision_calibrate(
        self,
        image: Image,
        prompt: str,
        api_key: str,
        max_iterations: int,
        max_time_s: float,
    ) -> Optional[Future]:
        req = CalibrateVision.Request()
        req.image = image
        req.prompt = str(prompt)
        req.api_key = str(api_key)
        req.max_iterations = int(max_iterations)
        req.max_time_s = float(max_time_s)
        return self._call_service(self.cli_vision_calibrate, req)

    def call_dio_set_output(self, io_id: str, value: bool) -> Optional[Future]:
        req = SetDigitalOutput.Request()
        req.id = str(io_id)
        req.value = bool(value)
        return self._call_service(self.cli_dio_set_output, req)

    def call_dio_set_config(self, io_id: str, tag: str, enabled: bool) -> Optional[Future]:
        req = SetDigitalIOConfig.Request()
        req.id = str(io_id)
        req.tag = str(tag)
        req.enabled = bool(enabled)
        return self._call_service(self.cli_dio_set_config, req)

    def publish_vision_test_image(
        self,
        width: int,
        height: int,
        rgb_data: bytes,
        step: int | None = None,
        frame_id: str = "camera_link",
    ) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(frame_id)
        msg.height = int(height)
        msg.width = int(width)
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = int(step) if step is not None else int(width) * 3
        msg.data = bytes(rgb_data)
        self._pub_vision_test_image.publish(msg)

    def publish_authorization_lease(self, lease: str) -> None:
        self._pub_authorization_lease.publish(String(data=str(lease)))

    def publish_speed_override(self, payload: str) -> None:
        self._pub_speed_override.publish(String(data=str(payload)))

    @staticmethod
    def _make_parameter(name: str, value: object) -> Parameter:
        parameter = Parameter()
        parameter.name = str(name)
        parameter.value = _parameter_value(value)
        return parameter

    # ------------------------------------------------------------------
    # action goal helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _action_ready(client: ActionClient) -> bool:
        """Return action-server discovery state without blocking the Qt thread."""
        try:
            return bool(client.server_is_ready())
        except Exception:
            return False

    def action_server_ready(self, name: str) -> bool:
        clients = {
            "move_to_pose": self.ac_move_to_pose,
            "follow_joint_trajectory": self.ac_follow_joint_trajectory,
            "gripper_command": self.ac_gripper_command,
            "move_group": self.ac_move_group,
        }
        client = clients.get(str(name))
        return self._action_ready(client) if client is not None else False

    def send_follow_joint_trajectory(
        self, joint_names: list[str], positions: list[float], duration: float
    ) -> Optional[Future]:
        if not self._action_ready(self.ac_follow_joint_trajectory):
            return None
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = duration_msg(duration)
        traj.points = [point]
        goal.trajectory = traj
        return self.ac_follow_joint_trajectory.send_goal_async(goal)

    def send_move_to_pose(
        self, pose: Pose, duration: float, feedback_callback=None
    ) -> Optional[Future]:
        if not self._action_ready(self.ac_move_to_pose):
            return None
        goal = MoveToPose.Goal()
        goal.target_pose = pose
        goal.duration = float(duration)
        return self.ac_move_to_pose.send_goal_async(
            goal, feedback_callback=feedback_callback
        )

    def send_collision_planned_pose(
        self,
        pose: Pose,
        speed_scale: float,
        *,
        base_frame: str = "base_link",
        end_link: str = "gripper_tcp",
        group_name: str = "arm",
    ) -> Optional[Future]:
        """Plan and execute one collision-aware pose goal through MoveIt."""
        client = self.ac_move_group
        if client is None or not self._action_ready(client):
            return None

        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = str(group_name)
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        scale = max(0.01, min(1.0, float(speed_scale)))
        request.max_velocity_scaling_factor = scale
        request.max_acceleration_scaling_factor = scale
        request.start_state.is_diff = True

        region = PositionConstraint()
        region.header.frame_id = str(base_frame)
        region.link_name = str(end_link)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.002]
        region.constraint_region.primitives = [primitive]
        region.constraint_region.primitive_poses = [pose]
        region.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = str(base_frame)
        orientation.link_name = str(end_link)
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.name = "rebotarm_collision_aware_pose"
        constraints.position_constraints = [region]
        constraints.orientation_constraints = [orientation]
        request.goal_constraints = [constraints]

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return client.send_goal_async(goal)

    def send_gripper_command(self, position: float, max_effort: float = 0.0) -> Optional[Future]:
        if not self._action_ready(self.ac_gripper_command):
            return None
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        return self.ac_gripper_command.send_goal_async(goal)


class RosBridge(QObject):
    """Qt-facing bridge.

    Runs a dedicated ROS executor inside a ``QThread`` and forwards cached
    state to the UI via Qt signals.
    """

    # ---- Signals (emitted from the ROS thread, consumed in the UI thread) ----
    joint_state_changed = Signal(object)
    arm_status_changed = Signal(object)
    timing_status_changed = Signal(object)
    gripper_state_changed = Signal(object)
    joint_detail_changed = Signal(str, object)
    dio_ios_changed = Signal(object)  # dict[str, DigitalIO] id -> snapshot
    service_result = Signal(str, bool, str)  # (label, success, message)
    action_result = Signal(str, bool, str)  # (label, success, message)
    pose_action_result = Signal(str, bool, str)  # (tag, success, message) - tagged MoveToPose
    safety_check_result = Signal(object)
    # (key, success, message, q_solution) — IK reachability probe result.
    ik_pose_checked = Signal(str, bool, str, object)
    vision_objects_changed = Signal(object)
    vision_debug_image_changed = Signal(object)
    vision_raw_image_changed = Signal(object)
    vision_calibration_progress = Signal(str)
    vision_calibration_result = Signal(str, str)  # (profile_json, report)
    log_message = Signal(str, str)  # (level, text)
    connected = Signal(bool)
    arm_alive = Signal(bool)  # heartbeat of /{ns}/arm_status (False = stale)
    move_feedback = Signal(float, object)  # MoveToPose feedback (progress, current_pose)
    diagnostics_changed = Signal(object)  # read-only ROS health snapshot
    speed_override_changed = Signal(float)  # fraction: 0.05 .. 1.0
    speed_override_status_changed = Signal(object)
    planning_scene_result = Signal(bool, object, str)
    motion_plan_preview_progress = Signal(str, int, int, str)
    motion_plan_preview_result = Signal(str, bool, object, str)

    def __init__(self, namespace: str = "rebotarm") -> None:
        super().__init__()
        self._namespace = namespace
        self._node: Optional[_BridgeNode] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._thread: Optional[QThread] = None
        self._stopping = threading.Event()
        self._timer_interval = 0.08  # seconds
        self._diagnostic_interval = 1.0
        self._action_lock = threading.RLock()
        self._action_seq = 0
        self._action_requests: dict[int, dict[str, object]] = {}
        self.action_accept_timeout_s = 3.0
        self.action_result_grace_s = 5.0
        # The controller may safely stretch a goal beyond the requested time
        # when a joint velocity/acceleration is limiting.  Its own action
        # server enforces the retimed deadline, so the GUI must not cancel a
        # healthy, deliberately slowed trajectory first.
        self.last_reached_pose: Optional[Pose] = None  # TCP pose captured from action results
        self._arm_alive_state = False
        self._speed_override = 1.0
        self._speed_override_session = uuid.uuid4().hex
        self._speed_override_sequence = 0
        self._dynamic_speed_override_supported = False
        self._dynamic_speed_override_status: dict[str, object] = {}
        self._authorization = AuthorizationContext()
        self._recipe_execution_scope = False
        self._preview_lock = threading.RLock()
        self._plan_preview_requests: dict[str, dict[str, object]] = {}
        self._planning_scene_lock = threading.RLock()
        self._planning_scene_raw_payload: dict[str, object] | None = None
        self._planning_scene_target_frame = "base_link"
        self._planning_scene_has_foreign_frames = False
        self._planning_scene_last_tf_refresh = 0.0
        self._planning_scene_tf_sequence = 0
        self._planning_scene_generation = 0
        self._planning_scene_last_tf_error = ""
        self._cell_scene_apply_lock = threading.RLock()
        self._cell_scene_apply_in_progress = False
        self.arm_status_timeout = 1.0  # seconds without arm_status -> not alive

    @property
    def authorization(self) -> AuthorizationContext:
        return self._authorization

    def set_authorization_context(self, context: AuthorizationContext) -> None:
        self._authorization = context

    def set_recipe_execution_scope(self, enabled: bool) -> None:
        """Select the released-recipe capability for subsequent motion commands."""
        self._recipe_execution_scope = bool(enabled)

    def _authorized(self, capability: str, label: str) -> bool:
        allowed, message = self._authorization.require(capability)
        if not allowed:
            self.log_message.emit("error", f"{label}: {message}")
            self.service_result.emit(label, False, message)
        return allowed

    @property
    def node(self) -> Optional[_BridgeNode]:
        return self._node

    @property
    def namespace(self) -> str:
        return self._namespace.strip().strip("/")

    @property
    def speed_override(self) -> float:
        """GUI-wide speed fraction applied to every time-based motion goal."""
        return self._speed_override

    @property
    def dynamic_speed_override_supported(self) -> bool:
        return self._dynamic_speed_override_supported

    @property
    def dynamic_speed_override_status(self) -> dict[str, object]:
        return dict(self._dynamic_speed_override_status)

    def set_speed_override(self, value: float) -> float:
        """Set the global override and return the clamped fraction.

        New controllers apply this target to the common virtual trajectory
        clock, including an already-running goal. Older controllers retain the
        safe duration-scaling fallback for newly dispatched goals.
        """
        clamped = clamp_speed_override(value)
        if clamped > self._speed_override + 1e-9:
            allowed, message = self._authorization.require(CAP_SPEED_OVERRIDE)
            if not allowed:
                self.log_message.emit("error", message)
                return self._speed_override
        if abs(clamped - self._speed_override) > 1e-9:
            self._speed_override = clamped
            self.speed_override_changed.emit(clamped)
        self._publish_speed_override_target()
        return clamped

    def _publish_speed_override_target(self) -> bool:
        node = self._node
        publish = getattr(node, "publish_speed_override", None)
        if not callable(publish):
            return False
        self._speed_override_sequence += 1
        payload = {
            "version": 1,
            "value": float(self._speed_override),
            "session": self._speed_override_session,
            "sequence": self._speed_override_sequence,
            "issued_at": time.time(),
        }
        publish(json.dumps(payload, sort_keys=True, separators=(",", ":")))
        return True

    def effective_motion_duration(self, duration: float) -> float:
        """Scale a command duration using the current global override."""
        nominal = max(0.001, float(duration))
        if self._dynamic_speed_override_supported:
            return nominal
        return nominal / self._speed_override

    def motion_result_timeout(self, requested_duration: float) -> float:
        """Allow headroom for controller-side synchronized retiming."""

        dynamic_budget = (
            float(requested_duration) / 0.05 + self.action_result_grace_s
            if self._dynamic_speed_override_supported
            else 0.0
        )
        return max(
            float(requested_duration) + self.action_result_grace_s,
            float(requested_duration) * 10.0,
            self.action_result_grace_s * 10.0,
            dynamic_budget,
        )

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def start(self) -> None:
        import rclpy

        self._stopping.clear()
        rclpy.init()
        self._node = _BridgeNode(self._namespace)
        self._publish_speed_override_target()
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)

        self._thread = QThread()
        self._thread.run = self._spin_loop
        self._thread.start()
        self.connected.emit(True)
        self._emit_diagnostics()

    def shutdown(self) -> None:
        import rclpy

        self._stopping.set()
        self._clear_action_requests(cancel_handles=False)
        with self._preview_lock:
            self._plan_preview_requests.clear()
        with self._planning_scene_lock:
            self._planning_scene_raw_payload = None
            self._planning_scene_has_foreign_frames = False
        with self._cell_scene_apply_lock:
            self._cell_scene_apply_in_progress = False
        if self._executor is not None:
            try:
                self._executor.wake()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()

        if self._thread is not None:
            self._thread.quit()
            if not self._thread.wait(3000):
                self._thread.terminate()
                self._thread.wait(1000)
            self._thread = None

        if self._executor is not None:
            self._executor.shutdown()
            self._executor = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        self._arm_alive_state = False
        self.connected.emit(False)
        self.diagnostics_changed.emit(self.diagnostic_snapshot())

    def _spin_loop(self) -> None:
        """Spin the ROS executor and periodically emit Qt signals."""
        import rclpy
        from datetime import datetime

        lock = threading.Event()
        last_diagnostic_emit = 0.0
        while not self._stopping.is_set() and rclpy.ok() and self._executor is not None:
            try:
                self._executor.spin_once(timeout_sec=self._timer_interval)
            except Exception as exc:  # pragma: no cover - defensive
                ts = datetime.now().strftime("%H:%M:%S")
                self.log_message.emit("error", f"[{ts}] spin error: {exc}")
                lock.wait(0.2)
                continue

            node = self._node
            if node is None:
                break

            if node.joint_state is not None:
                self.joint_state_changed.emit(node.joint_state)
                node.ensure_raw_publishers()

            stamp = node.last_arm_status_stamp
            alive = stamp is not None and (time.monotonic() - stamp) < self.arm_status_timeout
            if alive != self._arm_alive_state:
                self._arm_alive_state = alive
                if not alive:
                    cancelled = self.cancel_active_actions()
                    if cancelled:
                        self.log_message.emit(
                            "warn",
                            "heartbeat perdido: actions ativas foram canceladas",
                        )
                self.arm_alive.emit(alive)
            if alive and node.arm_status is not None:
                self.arm_status_changed.emit(node.arm_status)

            if node.timing_status is not None and node._timing_status_dirty:
                self.timing_status_changed.emit(node.timing_status)
                node._timing_status_dirty = False

            if (
                node.speed_override_status
                and node._speed_override_status_dirty
            ):
                self._dynamic_speed_override_status = dict(
                    node.speed_override_status
                )
                self._dynamic_speed_override_supported = True
                self.speed_override_status_changed.emit(
                    dict(self._dynamic_speed_override_status)
                )
                node._speed_override_status_dirty = False

            if node.gripper_state is not None:
                self.gripper_state_changed.emit(node.gripper_state)

            if node.dio_ios:
                self.dio_ios_changed.emit(node.dio_ios)

            if node.vision_objects is not None and node._vision_objects_dirty:
                self.vision_objects_changed.emit(node.vision_objects)
                node._vision_objects_dirty = False

            if node.vision_debug_image is not None and node._vision_debug_dirty:
                self.vision_debug_image_changed.emit(node.vision_debug_image)
                node._vision_debug_dirty = False

            if node.vision_raw_image is not None and node._vision_raw_dirty:
                self.vision_raw_image_changed.emit(node.vision_raw_image)
                node._vision_raw_dirty = False

            if node._vision_calibration_progress_dirty:
                self.vision_calibration_progress.emit(
                    node._vision_calibration_progress_message or ""
                )
                node._vision_calibration_progress_dirty = False

            for jn, detail in list(node.joint_detail.items()):
                self.joint_detail_changed.emit(jn, detail)

            for error in node.take_telemetry_errors():
                self.log_message.emit("error", f"telemetria inválida em {error}")

            now = time.monotonic()
            self._refresh_cached_planning_scene_transforms(now)
            if now - last_diagnostic_emit >= self._diagnostic_interval:
                self._emit_diagnostics()
                last_diagnostic_emit = now

    def diagnostic_snapshot(self) -> dict[str, object]:
        """Return current ROS health data without network calls or waiting."""
        node = self._node
        if node is None:
            return {
                "connected": False,
                "heartbeat": {
                    "alive": False,
                    "received": False,
                    "hz": None,
                    "age_ms": None,
                    "latency_ms": None,
                    "timeout_ms": self.arm_status_timeout * 1000.0,
                },
                "topics": [],
                "services": [],
                "actions": [],
                "captured_at": time.time(),
            }
        return node.diagnostic_snapshot(self.arm_status_timeout)

    def _emit_diagnostics(self) -> None:
        try:
            self.diagnostics_changed.emit(self.diagnostic_snapshot())
        except Exception as exc:  # pragma: no cover - defensive
            self.log_message.emit("error", f"falha ao coletar diagnóstico ROS: {exc}")

    # ------------------------------------------------------------------
    # call wrappers (called from the UI thread)
    # ------------------------------------------------------------------
    def _future_done(self, label: str, future: Future, success_key: str = "success") -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.service_result.emit(label, False, str(exc))
            return
        success = bool(getattr(result, success_key, False))
        message = getattr(result, "message", "") or getattr(result, "error_string", "")
        self.service_result.emit(label, success, str(message))

    def _call(self, label: str, future: Optional[Future], success_key: str = "success") -> bool:
        if future is None:
            self.log_message.emit("warn", f"{label}: serviço indisponível")
            self.service_result.emit(label, False, "serviço indisponível")
            return False
        future.add_done_callback(lambda f: self._future_done(label, f, success_key))
        self.log_message.emit("info", f"chamando serviço {label} ...")
        return True

    # Services ---------------------------------------------------------
    def enable(self) -> bool:
        if not self._authorized(CAP_MANUAL_MOTION, "enable"):
            return False
        node = self._node
        return self._call("enable", node.call_enable() if node else None)

    def disable(self) -> bool:
        node = self._node
        return self._call("disable", node.call_disable() if node else None)

    def safe_home(self) -> bool:
        if not self._authorized(CAP_MANUAL_MOTION, "safe_home"):
            return False
        node = self._node
        return self._call("safe_home", node.call_safe_home() if node else None)

    def gravity_start(self) -> bool:
        if not self._authorized(CAP_MANUAL_MOTION, "gravity_start"):
            return False
        node = self._node
        return self._call("gravity_start", node.call_gravity_start() if node else None)

    def gravity_stop(self) -> bool:
        if not self._authorized(CAP_MANUAL_MOTION, "gravity_stop"):
            return False
        node = self._node
        return self._call("gravity_stop", node.call_gravity_stop() if node else None)

    def set_zero(self, joint_name: str = "") -> bool:
        label = f"set_zero({joint_name})" if joint_name else "set_zero(all)"
        if not self._authorized(CAP_RAW_CONTROL, label):
            return False
        node = self._node
        return self._call(label, node.call_set_zero(joint_name) if node else None)

    def move_to_pose_ik(self, pose: Pose) -> bool:
        node = self._node
        return self._call("move_to_pose_ik", node.call_move_to_pose_ik(pose) if node else None)

    def apply_cell_scene(self, scene) -> bool:
        """Reconcile and apply the validated GUI cell scene asynchronously."""

        if not self._authorized(CAP_SAFETY_CONFIG, "apply_cell_scene"):
            return False
        node = self._node
        try:
            validated = scene.validated()
            obstacles = [
                {
                    "object_id": obstacle.object_id,
                    "center": list(obstacle.center),
                    "size": list(obstacle.size),
                    "margin_m": obstacle.margin_m,
                    "enabled": obstacle.enabled,
                }
                for obstacle in validated.obstacles
            ]
        except (AttributeError, KeyError, TypeError, ValueError) as exc:
            self.service_result.emit("apply_cell_scene", False, str(exc))
            return False
        with self._cell_scene_apply_lock:
            if self._cell_scene_apply_in_progress:
                message = "reconciliação da cena já está em andamento"
                self.service_result.emit("apply_cell_scene", False, message)
                return False
            self._cell_scene_apply_in_progress = True
        query = node.call_get_planning_scene() if node is not None else None
        if query is None:
            with self._cell_scene_apply_lock:
                self._cell_scene_apply_in_progress = False
            message = (
                "não foi possível ler a cena do MoveIt para remover "
                "objetos obsoletos"
            )
            self.service_result.emit("apply_cell_scene", False, message)
            self.log_message.emit("warn", f"apply_cell_scene: {message}")
            return False

        def finish(success: bool, message: str) -> None:
            with self._cell_scene_apply_lock:
                self._cell_scene_apply_in_progress = False
            self.service_result.emit(
                "apply_cell_scene",
                bool(success),
                str(message),
            )

        def applied(completed: Future, stale_count: int) -> None:
            try:
                result = completed.result()
                success = bool(getattr(result, "success", False))
                response_message = str(getattr(result, "message", "") or "")
            except Exception as exc:
                finish(False, str(exc))
                return
            if success:
                message = (
                    "cena reconciliada no MoveIt"
                    f"; {stale_count} objeto(s) obsoleto(s) removido(s)"
                )
                if response_message:
                    message += f"; {response_message}"
            else:
                message = response_message or "MoveIt recusou a cena reconciliada"
            finish(success, message)

        def reconciled(completed: Future) -> None:
            try:
                response = completed.result()
                node.observe_cell_scene_world(response.scene)
                apply_future, stale_count = node.call_apply_cell_scene(
                    frame_id=validated.frame_id,
                    minimum_clearance_m=validated.minimum_clearance_m,
                    obstacles=obstacles,
                )
            except Exception as exc:
                finish(False, f"falha ao reconciliar cena: {exc}")
                return
            if apply_future is None:
                finish(False, "serviço /apply_planning_scene indisponível")
                return
            apply_future.add_done_callback(
                lambda future: applied(future, stale_count)
            )

        query.add_done_callback(reconciled)
        self.log_message.emit(
            "info",
            "lendo cena atual do MoveIt antes de aplicar volumes locais ...",
        )
        return True

    def refresh_planning_scene(
        self,
        target_frame: str = "base_link",
    ) -> bool:
        """Refresh the read-only collision geometry snapshot shown by the GUI."""

        node = self._node
        target = str(target_frame).strip().lstrip("/") or "base_link"
        future = node.call_get_planning_scene() if node is not None else None
        if future is None:
            self._clear_planning_scene_cache()
            message = "serviço /get_planning_scene indisponível"
            self.planning_scene_result.emit(False, {}, message)
            self.log_message.emit("warn", message)
            return False

        def done(completed: Future) -> None:
            try:
                response = completed.result()
                node.observe_cell_scene_world(response.scene)
                raw_payload = serialize_planning_scene(response.scene)
                dynamic_enabled, generation = self._cache_planning_scene_snapshot(
                    raw_payload,
                    target,
                )
                payload = node.resolve_planning_scene_frames(
                    raw_payload,
                    target,
                )
                payload.update(
                    self._planning_scene_tf_metadata(
                        dynamic_enabled=dynamic_enabled,
                        dynamic_update=False,
                        sequence=0,
                        generation=generation,
                    )
                )
            except Exception as exc:
                self._clear_planning_scene_cache()
                self.planning_scene_result.emit(False, {}, str(exc))
                return
            skipped = int(payload.get("truncated_geometry_count", 0))
            message = (
                f"{len(payload.get('geometries', ()))} geometria(s) importada(s)"
            )
            if skipped:
                message += f"; {skipped} geometria(s) omitida(s) por limite/formato"
            resolved = int(payload.get("tf_resolved_geometry_count", 0))
            unresolved = int(payload.get("tf_unresolved_geometry_count", 0))
            if resolved:
                message += f"; {resolved} geometria(s) transformada(s) por TF"
            if unresolved:
                message += (
                    f"; {unresolved} geometria(s) sem TF para "
                    f"{payload.get('display_frame_id', target_frame)}"
                )
            self.planning_scene_result.emit(True, payload, message)

        future.add_done_callback(done)
        return True

    @staticmethod
    def _planning_scene_tf_metadata(
        *,
        dynamic_enabled: bool,
        dynamic_update: bool,
        sequence: int,
        generation: int,
    ) -> dict[str, object]:
        """Build bounded-refresh metadata shared with the GUI."""

        return {
            "tf_dynamic_refresh_enabled": bool(dynamic_enabled),
            "tf_dynamic_update": bool(dynamic_update),
            "tf_refresh_rate_hz": (
                _PLANNING_SCENE_TF_REFRESH_HZ if dynamic_enabled else 0.0
            ),
            "tf_refresh_sequence": max(0, int(sequence)),
            "tf_scene_generation": max(0, int(generation)),
            "tf_refreshed_at": time.time(),
        }

    def _cache_planning_scene_snapshot(
        self,
        payload: dict[str, object],
        target_frame: str,
    ) -> tuple[bool, int]:
        """Cache source geometry so TF can move it without another service call."""

        target = str(target_frame).strip().lstrip("/") or "base_link"
        has_foreign_frames = any(
            isinstance(item, dict)
            and bool(str(item.get("frame_id", "")).strip().lstrip("/"))
            and str(item.get("frame_id", "")).strip().lstrip("/") != target
            for item in payload.get("geometries", ())
        )
        with self._planning_scene_lock:
            self._planning_scene_raw_payload = payload
            self._planning_scene_target_frame = target
            self._planning_scene_has_foreign_frames = has_foreign_frames
            self._planning_scene_last_tf_refresh = time.monotonic()
            self._planning_scene_tf_sequence = 0
            self._planning_scene_generation += 1
            generation = self._planning_scene_generation
            self._planning_scene_last_tf_error = ""
        return has_foreign_frames, generation

    def _clear_planning_scene_cache(self) -> None:
        with self._planning_scene_lock:
            self._planning_scene_raw_payload = None
            self._planning_scene_has_foreign_frames = False
            self._planning_scene_tf_sequence = 0

    @staticmethod
    def _transform_update_payload(
        payload: dict[str, object],
    ) -> dict[str, object]:
        """Strip immutable mesh/primitive data from a live TF-only update."""

        updates = []
        for index, item in enumerate(payload.get("geometries", ())):
            if not isinstance(item, dict):
                continue
            updates.append(
                {
                    "geometry_index": index,
                    "object_id": item.get("object_id", ""),
                    "attached_to": item.get("attached_to", ""),
                    "source_frame_id": item.get("source_frame_id", ""),
                    "source_pose": item.get("source_pose", ()),
                    "frame_id": item.get("frame_id", ""),
                    "pose": item.get("pose", ()),
                    "transform_status": item.get(
                        "transform_status",
                        "native",
                    ),
                }
            )
        update_payload = {
            key: value
            for key, value in payload.items()
            if key != "geometries"
        }
        update_payload["transform_updates"] = updates
        return update_payload

    def _refresh_cached_planning_scene_transforms(
        self,
        now: float | None = None,
    ) -> bool:
        """Re-resolve cached geometry at 5 Hz without calling MoveIt again."""

        current_time = time.monotonic() if now is None else float(now)
        with self._planning_scene_lock:
            raw_payload = self._planning_scene_raw_payload
            target = self._planning_scene_target_frame
            generation = self._planning_scene_generation
            if (
                raw_payload is None
                or not self._planning_scene_has_foreign_frames
                or (
                    current_time - self._planning_scene_last_tf_refresh
                    < _PLANNING_SCENE_TF_REFRESH_INTERVAL_S
                )
            ):
                return False
            self._planning_scene_last_tf_refresh = current_time
        node = self._node
        if node is None:
            return False
        try:
            resolved = node.resolve_planning_scene_frames(
                raw_payload,
                target,
                timeout_s=0.0,
            )
        except Exception as exc:  # pragma: no cover - defensive
            error = str(exc)
            with self._planning_scene_lock:
                changed = error != self._planning_scene_last_tf_error
                self._planning_scene_last_tf_error = error
            if changed:
                self.log_message.emit(
                    "warn",
                    f"atualização TF da cena monitorada falhou: {error}",
                )
            return False
        with self._planning_scene_lock:
            if raw_payload is not self._planning_scene_raw_payload:
                return False
            self._planning_scene_tf_sequence += 1
            sequence = self._planning_scene_tf_sequence
            self._planning_scene_last_tf_error = ""
        resolved.update(
            self._planning_scene_tf_metadata(
                dynamic_enabled=True,
                dynamic_update=True,
                sequence=sequence,
                generation=generation,
            )
        )
        compact = self._transform_update_payload(resolved)
        self.planning_scene_result.emit(
            True,
            compact,
            "poses da cena monitorada atualizadas por TF",
        )
        return True

    @staticmethod
    def _serialize_planned_segment(plan_response) -> dict[str, object]:
        trajectory = plan_response.trajectory.joint_trajectory
        points: list[dict[str, object]] = []
        for point in trajectory.points:
            stamp = point.time_from_start
            points.append(
                {
                    "positions": [float(value) for value in point.positions],
                    "velocities": [float(value) for value in point.velocities],
                    "accelerations": [
                        float(value) for value in point.accelerations
                    ],
                    "time_s": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
                }
            )
        return {
            "joint_names": [str(name) for name in trajectory.joint_names],
            "points": points,
            "planning_time_s": float(
                getattr(plan_response, "planning_time", 0.0)
            ),
            "planner_id": str(getattr(plan_response, "planner_id", "")),
        }

    def request_motion_plan_preview(
        self,
        tag: str,
        paths: list[list[dict[str, object]]],
        *,
        speed_scale: float = 1.0,
        required_clearance_m: float = 0.0,
    ) -> bool:
        """Plan bounded program paths without execution and validate samples."""

        request_tag = str(tag)
        try:
            normalized = [
                [dict(pose) for pose in path]
                for path in paths
                if path
            ]
            total = sum(len(path) for path in normalized)
            required_clearance = float(required_clearance_m)
            if not math.isfinite(required_clearance):
                raise ValueError("folga mínima da prévia não finita")
            if len(normalized) > 16 or total > 512:
                raise ValueError(
                    "prévia MoveIt excede 16 caminhos ou 512 alvos"
                )
            for path in normalized:
                for pose in path:
                    for key in ("x", "y", "z"):
                        value = float(pose[key])
                        if not math.isfinite(value):
                            raise ValueError("pose de prévia não finita")
        except (KeyError, TypeError, ValueError) as exc:
            self.motion_plan_preview_result.emit(
                request_tag, False, {}, str(exc)
            )
            return False
        node = self._node
        if node is None or getattr(node, "cli_get_motion_plan", None) is None:
            message = "serviço /plan_kinematic_path indisponível"
            self.motion_plan_preview_result.emit(
                request_tag, False, {}, message
            )
            return False
        state: dict[str, object] = {
            "paths": normalized,
            "speed_scale": max(0.01, min(1.0, float(speed_scale))),
            "required_clearance_m": max(
                0.0, required_clearance
            ),
            "total": total,
            "done": 0,
            "path_index": 0,
            "target_index": 0,
            "start_names": [],
            "start_positions": [],
            "result_paths": [],
            "segments": [],
        }
        with self._preview_lock:
            self._plan_preview_requests[request_tag] = state
        if not normalized:
            self._finish_motion_plan_preview(
                request_tag,
                True,
                {
                    "paths": [],
                    "state_validation": {
                        "available": False,
                        "verified": True,
                        "sample_count": 0,
                        "invalid_count": 0,
                        "contacts": [],
                    },
                },
                "programa sem movimentos cartesianos",
            )
            return True
        self._dispatch_motion_plan_preview(request_tag)
        return True

    def cancel_motion_plan_preview(self, tag: str) -> None:
        """Ignore remaining callbacks for a closed preview dialog."""

        with self._preview_lock:
            state = self._plan_preview_requests.pop(str(tag), None)
        timer = state.get("timeout_timer") if state is not None else None
        if timer is not None:
            timer.cancel()

    def _preview_state(self, tag: str) -> dict[str, object] | None:
        with self._preview_lock:
            return self._plan_preview_requests.get(str(tag))

    def _arm_preview_timeout(
        self,
        tag: str,
        seconds: float,
        stage: str,
    ) -> None:
        state = self._preview_state(tag)
        if state is None:
            return
        previous = state.get("timeout_timer")
        if previous is not None:
            previous.cancel()
        timer = threading.Timer(
            max(0.1, float(seconds)),
            lambda: self._finish_motion_plan_preview(
                tag,
                False,
                {},
                f"timeout aguardando {stage}",
            ),
        )
        timer.daemon = True
        state["timeout_timer"] = timer
        timer.start()

    def _cancel_preview_timeout(self, state: dict[str, object]) -> None:
        timer = state.pop("timeout_timer", None)
        if timer is not None:
            timer.cancel()

    def _dispatch_motion_plan_preview(self, tag: str) -> None:
        state = self._preview_state(tag)
        if state is None:
            return
        paths = state["paths"]
        path_index = int(state["path_index"])
        target_index = int(state["target_index"])
        if path_index >= len(paths):
            self._start_preview_state_validation(tag)
            return
        path = paths[path_index]
        if target_index >= len(path):
            state["result_paths"].append(
                {"path_index": path_index, "segments": state["segments"]}
            )
            state["path_index"] = path_index + 1
            state["target_index"] = 0
            state["start_names"] = []
            state["start_positions"] = []
            state["segments"] = []
            self._dispatch_motion_plan_preview(tag)
            return
        node = self._node
        future = (
            node.call_motion_plan(
                path[target_index],
                float(state["speed_scale"]),
                start_joint_names=list(state["start_names"]),
                start_positions=list(state["start_positions"]),
            )
            if node is not None
            else None
        )
        if future is None:
            self._finish_motion_plan_preview(
                tag,
                False,
                {},
                "serviço /plan_kinematic_path ficou indisponível",
            )
            return
        self.motion_plan_preview_progress.emit(
            tag,
            int(state["done"]),
            int(state["total"]),
            f"planejando caminho {path_index + 1}, alvo {target_index + 1}",
        )
        future.add_done_callback(
            lambda completed, request_tag=tag: self._on_motion_plan_done(
                request_tag, completed
            )
        )
        self._arm_preview_timeout(tag, 8.0, "/plan_kinematic_path")

    def _on_motion_plan_done(self, tag: str, future: Future) -> None:
        state = self._preview_state(tag)
        if state is None:
            return
        self._cancel_preview_timeout(state)
        try:
            response = future.result()
            plan_response = response.motion_plan_response
            error_value = int(plan_response.error_code.val)
            success_value = int(getattr(MoveItErrorCodes, "SUCCESS", 1))
            if error_value != success_value:
                raise RuntimeError(f"MoveIt recusou o plano (código {error_value})")
            segment = self._serialize_planned_segment(plan_response)
            points = segment["points"]
            if not segment["joint_names"] or not points:
                raise RuntimeError("MoveIt retornou trajetória vazia")
        except Exception as exc:
            self._finish_motion_plan_preview(tag, False, {}, str(exc))
            return
        state["segments"].append(segment)
        final = points[-1]
        state["start_names"] = list(segment["joint_names"])
        state["start_positions"] = list(final["positions"])
        state["target_index"] = int(state["target_index"]) + 1
        state["done"] = int(state["done"]) + 1
        self._dispatch_motion_plan_preview(tag)

    @staticmethod
    def _sample_planned_states(
        result_paths: list[dict[str, object]],
        limit: int = 120,
    ) -> list[tuple[list[str], list[float]]]:
        samples: list[tuple[list[str], list[float]]] = []
        for path in result_paths:
            for segment in path["segments"]:
                names = list(segment["joint_names"])
                for point in segment["points"]:
                    samples.append((names, list(point["positions"])))
        if len(samples) <= limit:
            return samples
        indexes = {
            round(index * (len(samples) - 1) / (limit - 1))
            for index in range(limit)
        }
        return [samples[index] for index in sorted(indexes)]

    def _start_preview_state_validation(self, tag: str) -> None:
        state = self._preview_state(tag)
        if state is None:
            return
        result_paths = list(state["result_paths"])
        samples = self._sample_planned_states(result_paths)
        state["sampled_states"] = samples
        node = self._node
        if not samples:
            self._finish_motion_plan_preview(
                tag,
                True,
                {
                    "paths": result_paths,
                    "state_validation": {
                        "available": True,
                        "verified": True,
                        "sample_count": 0,
                        "invalid_count": 0,
                        "contacts": [],
                    },
                    "full_robot_clearance": {
                        "available": True,
                        "verified": True,
                        "sample_count": 0,
                        "required_clearance_m": float(
                            state["required_clearance_m"]
                        ),
                    },
                },
                "plano sem estados amostráveis",
            )
            return
        if (
            node is None
            or getattr(node, "cli_get_state_validity", None) is None
        ):
            self._start_preview_clearance(
                tag,
                {
                    "available": False,
                    "verified": False,
                    "sample_count": 0,
                    "invalid_count": 0,
                    "contacts": [],
                },
                status_message=(
                    "plano obtido; /check_state_validity indisponível"
                ),
            )
            return
        futures: list[Future] = []
        for names, positions in samples:
            future = node.call_state_validity(names, positions)
            if future is None:
                self._start_preview_clearance(
                    tag,
                    {
                        "available": False,
                        "verified": False,
                        "sample_count": 0,
                        "invalid_count": 0,
                        "contacts": [],
                    },
                    status_message=(
                        "plano obtido; /check_state_validity ficou "
                        "indisponível"
                    ),
                )
                return
            futures.append(future)
        state["validity_pending"] = len(futures)
        state["validity_total"] = len(futures)
        state["invalid_count"] = 0
        state["contacts"] = []
        self.motion_plan_preview_progress.emit(
            tag,
            int(state["done"]),
            int(state["total"]),
            f"validando {len(futures)} estado(s) do robô completo",
        )
        for future in futures:
            future.add_done_callback(
                lambda completed, request_tag=tag: self._on_state_validity_done(
                    request_tag, completed
                )
            )
        self._arm_preview_timeout(tag, 8.0, "/check_state_validity")

    def _on_state_validity_done(self, tag: str, future: Future) -> None:
        state = self._preview_state(tag)
        if state is None:
            return
        try:
            response = future.result()
            valid = bool(response.valid)
            contacts = list(getattr(response, "contacts", ()) or ())
        except Exception as exc:
            self._finish_motion_plan_preview(
                tag,
                False,
                {},
                f"falha ao validar estados planejados: {exc}",
            )
            return
        if not valid:
            state["invalid_count"] = int(state["invalid_count"]) + 1
            known = state["contacts"]
            for contact in contacts:
                pair = (
                    str(getattr(contact, "contact_body_1", "")),
                    str(getattr(contact, "contact_body_2", "")),
                )
                if pair not in known and len(known) < 20:
                    known.append(pair)
        state["validity_pending"] = int(state["validity_pending"]) - 1
        if int(state["validity_pending"]) > 0:
            return
        self._cancel_preview_timeout(state)
        invalid = int(state["invalid_count"])
        self._start_preview_clearance(
            tag,
            {
                "available": True,
                "verified": True,
                "sample_count": int(state["validity_total"]),
                "invalid_count": invalid,
                "contacts": list(state["contacts"]),
            },
            status_message=(
                f"{invalid} estado(s) planejado(s) em colisão"
                if invalid
                else "trajetória planejada e estados amostrados válidos"
            ),
        )

    def _start_preview_clearance(
        self,
        tag: str,
        validation: dict[str, object],
        *,
        status_message: str,
    ) -> None:
        state = self._preview_state(tag)
        if state is None:
            return
        state["state_validation"] = validation
        state["validation_message"] = str(status_message)
        samples = list(state.get("sampled_states", ()))
        node = self._node
        if (
            node is None
            or getattr(node, "cli_check_trajectory_clearance", None) is None
        ):
            self._finish_preview_without_clearance(
                tag, "/rebotarm/check_trajectory_clearance indisponível"
            )
            return
        future = node.call_trajectory_clearance(samples, group_name="arm")
        if future is None:
            self._finish_preview_without_clearance(
                tag,
                "/rebotarm/check_trajectory_clearance ficou indisponível",
            )
            return
        self.motion_plan_preview_progress.emit(
            tag,
            int(state["done"]),
            int(state["total"]),
            f"medindo folga de {len(samples)} estado(s) do robô completo",
        )
        future.add_done_callback(
            lambda completed, request_tag=tag: self._on_clearance_done(
                request_tag, completed
            )
        )
        self._arm_preview_timeout(
            tag, 8.0, "/rebotarm/check_trajectory_clearance"
        )

    def _finish_preview_without_clearance(
        self,
        tag: str,
        reason: str,
    ) -> None:
        state = self._preview_state(tag)
        if state is None:
            return
        validation = dict(state.get("state_validation", {}))
        invalid = int(validation.get("invalid_count", 0))
        payload = {
            "paths": list(state["result_paths"]),
            "state_validation": validation,
            "full_robot_clearance": {
                "available": False,
                "verified": False,
                "sample_count": 0,
                "required_clearance_m": float(
                    state["required_clearance_m"]
                ),
            },
        }
        self._finish_motion_plan_preview(
            tag,
            invalid == 0,
            payload,
            f"{state.get('validation_message', '')}; {reason}",
        )

    def _on_clearance_done(self, tag: str, future: Future) -> None:
        state = self._preview_state(tag)
        if state is None:
            return
        self._cancel_preview_timeout(state)
        try:
            response = future.result()
            if not bool(response.success):
                self._finish_preview_without_clearance(
                    tag,
                    str(response.message)
                    or "serviço de folga recusou a análise",
                )
                return
            samples = list(state.get("sampled_states", ()))
            available = list(response.available)
            distances = [float(value) for value in response.distance_m]
            collisions = [bool(value) for value in response.collision]
            body_1 = [str(value) for value in response.body_1]
            body_2 = [str(value) for value in response.body_2]
            sources = [str(value) for value in response.source]
            expected = len(samples)
            arrays = (
                available,
                distances,
                collisions,
                body_1,
                body_2,
                sources,
            )
            if any(len(values) != expected for values in arrays):
                raise ValueError("resposta de folga com tamanho inconsistente")
            measured = [
                index
                for index, is_available in enumerate(available)
                if is_available and math.isfinite(distances[index])
            ]
            if len(measured) != expected:
                raise ValueError(
                    "backend não produziu distância para todos os estados"
                )
            limiting_index = min(measured, key=distances.__getitem__)
            minimum = distances[limiting_index]
            required = float(state["required_clearance_m"])
            collision_count = sum(collisions)
            safe = collision_count == 0 and minimum + 1e-9 >= required
            points_1 = list(response.nearest_point_1)
            points_2 = list(response.nearest_point_2)
            nearest_1 = (
                [
                    float(points_1[limiting_index].x),
                    float(points_1[limiting_index].y),
                    float(points_1[limiting_index].z),
                ]
                if len(points_1) == expected
                else []
            )
            nearest_2 = (
                [
                    float(points_2[limiting_index].x),
                    float(points_2[limiting_index].y),
                    float(points_2[limiting_index].z),
                ]
                if len(points_2) == expected
                else []
            )
        except Exception as exc:
            self._finish_preview_without_clearance(
                tag, f"falha ao interpretar medição de folga: {exc}"
            )
            return

        validation = dict(state.get("state_validation", {}))
        invalid = int(validation.get("invalid_count", 0))
        payload = {
            "paths": list(state["result_paths"]),
            "state_validation": validation,
            "full_robot_clearance": {
                "available": True,
                "verified": True,
                "sample_count": len(samples),
                "minimum_distance_m": minimum,
                "required_clearance_m": required,
                "collision_count": collision_count,
                "safe": safe,
                "limiting_sample": limiting_index,
                "body_1": body_1[limiting_index],
                "body_2": body_2[limiting_index],
                "source": sources[limiting_index],
                "nearest_point_1": nearest_1,
                "nearest_point_2": nearest_2,
            },
        }
        pair = f"{body_1[limiting_index]} ↔ {body_2[limiting_index]}"
        self._finish_motion_plan_preview(
            tag,
            invalid == 0 and safe,
            payload,
            (
                f"folga mínima do robô completo {minimum * 1000:.1f} mm "
                f"em {pair} (requerida {required * 1000:.1f} mm)"
            ),
        )

    def _finish_motion_plan_preview(
        self,
        tag: str,
        success: bool,
        payload: dict[str, object],
        message: str,
    ) -> None:
        with self._preview_lock:
            existed = self._plan_preview_requests.pop(str(tag), None)
        if existed is None:
            return
        self._cancel_preview_timeout(existed)
        self.motion_plan_preview_result.emit(
            str(tag),
            bool(success),
            payload,
            str(message),
        )

    def check_pose_reachable(self, key: str, pose_dict: dict) -> bool:
        """Probe IK reachability for one pose; emit ``ik_pose_checked``.

        ``pose_dict`` uses the GUI convention (x, y, z, roll, pitch, yaw in
        metres / radians). The full service response (including ``q_solution``)
        is forwarded via :attr:`ik_pose_checked` so callers can additionally
        inspect joint limits. Returns False (and emits nothing) when the
        service is not available, so the caller can count it as skipped.
        """
        node = self._node
        if node is None:
            return False
        try:
            from .motion.trajectory import rpy_to_quat
            qx, qy, qz, qw = rpy_to_quat(
                float(pose_dict.get("roll", 0.0)),
                float(pose_dict.get("pitch", 0.0)),
                float(pose_dict.get("yaw", 0.0)),
            )
        except Exception:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        pose = make_pose(
            float(pose_dict.get("x", 0.0)),
            float(pose_dict.get("y", 0.0)),
            float(pose_dict.get("z", 0.0)),
            qx, qy, qz, qw,
        )
        future = node.call_move_to_pose_ik(pose)
        if future is None:
            return False
        future.add_done_callback(lambda f, k=str(key): self._on_ik_done(k, f))
        return True

    def _on_ik_done(self, key: str, future: Future) -> None:
        """Forward the full IK response (with q_solution) to the UI thread."""
        try:
            result = future.result()
        except Exception as exc:
            self.ik_pose_checked.emit(key, False, str(exc), [])
            return
        success = bool(getattr(result, "success", False))
        message = str(getattr(result, "message", "") or "")
        q_solution = list(getattr(result, "q_solution", []) or [])
        self.ik_pose_checked.emit(key, success, message, q_solution)

    def set_dio_output(self, io_id: str, value: bool) -> bool:
        label = f"dio_set_output({io_id}={'ON' if value else 'OFF'})"
        if not self._authorized(CAP_IO_WRITE, label):
            return False
        return self._call(label, self._node.call_dio_set_output(io_id, value))

    def set_dio_config(self, io_id: str, tag: str, enabled: bool) -> bool:
        label = f"dio_set_config({io_id})"
        if not self._authorized(CAP_SAFETY_CONFIG, label):
            return False
        return self._call(label, self._node.call_dio_set_config(io_id, tag, enabled))

    def gripper_set(self, position: float) -> bool:
        return self._call(
            "gripper_set", self._node.call_gripper_set(position), success_key="success"
        )

    def gripper_open(self, position: float = 0.0, timeout: float = 3.0) -> bool:
        return self._call(
            "gripper_open",
            self._node.call_gripper_open(position, timeout),
            success_key="success",
        )

    def gripper_close(self, position: float = 0.0, timeout: float = 3.0) -> bool:
        return self._call(
            "gripper_close",
            self._node.call_gripper_close(position, timeout),
            success_key="success",
        )

    def safety_set_profile(
        self,
        profile_path: str,
        mode: str,
        violation_samples: int,
        active_step_id: str = "",
        derivative_limit: float = 0.0,
    ) -> bool:
        if not self._authorized(CAP_SAFETY_CONFIG, "safety_set_profile"):
            return False
        return self._call(
            "safety_set_profile",
            self._node.call_safety_set_profile(
                profile_path,
                mode,
                violation_samples,
                active_step_id,
                derivative_limit,
            ),
        )

    def safety_set_step(self, step_id: str, payload_kg: float = 0.0) -> bool:
        label = f"safety_set_step({step_id or 'default'})"
        if not self._authorized(CAP_EXECUTE_RELEASED, label):
            return False
        return self._call(
            label,
            self._node.call_safety_set_step(step_id, payload_kg),
        )

    def safety_reset_fault(self) -> bool:
        if not self._authorized(CAP_SAFETY_RESET, "safety_reset_fault"):
            return False
        return self._call(
            "safety_reset_fault",
            self._node.call_safety_reset_fault(),
        )

    def safety_check(self, step_id: str, phase: float) -> bool:
        future = self._node.call_safety_check(step_id, phase)
        if future is None:
            self.log_message.emit("warn", "safety_check: serviço indisponível")
            self.service_result.emit("safety_check", False, "serviço indisponível")
            return False
        future.add_done_callback(self._safety_check_done)
        self.log_message.emit("info", "verificando perfil de torque ...")
        return True

    def _safety_check_done(self, future: Future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.service_result.emit("safety_check", False, str(exc))
            return
        self.safety_check_result.emit(result)

    def vision_set_parameters(self, values: dict[str, object]) -> bool:
        if self._node is None:
            return False
        future = self._node.call_vision_set_parameters(values)
        if future is None:
            self.log_message.emit("warn", "vision: nó rebotarm_segmentation indisponível")
            self.service_result.emit("vision_set_parameters", False, "nó indisponível")
            return False
        future.add_done_callback(self._vision_params_done)
        self.log_message.emit("info", "aplicando parâmetros de visão ...")
        return True

    def vision_pose_set_parameters(self, values: dict[str, object]) -> bool:
        if self._node is None:
            return False
        future = self._node.call_vision_pose_set_parameters(values)
        if future is None:
            self.log_message.emit("warn", "vision: nó rebotarm_object_pose indisponível")
            self.service_result.emit("vision_pose_set_parameters", False, "nó indisponível")
            return False
        future.add_done_callback(self._vision_pose_params_done)
        self.log_message.emit("info", "aplicando correção XY da mesa ...")
        return True

    def pick_scheduler_set_parameters(self, values: dict[str, object]) -> bool:
        if self._node is None:
            return False
        future = self._node.call_pick_scheduler_set_parameters(values)
        if future is None:
            self.log_message.emit("warn", "visão: nó rebotarm_pick_scheduler indisponível")
            self.service_result.emit("pick_scheduler_set_parameters", False, "nó indisponível")
            return False
        future.add_done_callback(self._pick_scheduler_params_done)
        self.log_message.emit("info", "aplicando planejamento da rota de pega ...")
        return True

    def vision_process_frame(self) -> bool:
        if self._node is None:
            return False
        future = self._node.call_vision_process_frame()
        if future is None:
            self.log_message.emit("warn", "vision: serviço process_frame indisponível")
            self.service_result.emit("vision_process_frame", False, "serviço indisponível")
            return False
        future.add_done_callback(self._vision_process_done)
        self.log_message.emit("info", "processando um frame de visão ...")
        return True

    def vision_calibrate(
        self,
        image: Image,
        prompt: str,
        api_key: str = "",
        max_iterations: int = 5,
        max_time_s: float = 300.0,
    ) -> bool:
        if self._node is None:
            return False
        future = self._node.call_vision_calibrate(
            image,
            prompt,
            api_key,
            max_iterations,
            max_time_s,
        )
        if future is None:
            self.log_message.emit("warn", "vision: serviço de calibração indisponível")
            self.service_result.emit("vision_calibrate", False, "serviço indisponível")
            return False
        future.add_done_callback(self._vision_calibrate_done)
        self.log_message.emit("info", "iniciando calibração de visão por IA ...")
        return True

    def _vision_params_done(self, future: Future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.service_result.emit("vision_set_parameters", False, str(exc))
            return
        results = list(getattr(response, "results", []))
        success = bool(results) and all(bool(getattr(item, "successful", False)) for item in results)
        reasons = [str(getattr(item, "reason", "")) for item in results if getattr(item, "reason", "")]
        message = "parâmetros aplicados" if success else "; ".join(reasons) or "falha ao aplicar"
        self.service_result.emit("vision_set_parameters", success, message)

    def _vision_pose_params_done(self, future: Future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.service_result.emit("vision_pose_set_parameters", False, str(exc))
            return
        results = list(getattr(response, "results", []))
        success = bool(results) and all(bool(getattr(item, "successful", False)) for item in results)
        reasons = [str(getattr(item, "reason", "")) for item in results if getattr(item, "reason", "")]
        message = "correção XY aplicada" if success else "; ".join(reasons) or "falha ao aplicar"
        self.service_result.emit("vision_pose_set_parameters", success, message)

    def _pick_scheduler_params_done(self, future: Future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.service_result.emit("pick_scheduler_set_parameters", False, str(exc))
            return
        results = list(getattr(response, "results", []))
        success = bool(results) and all(bool(getattr(item, "successful", False)) for item in results)
        reasons = [str(getattr(item, "reason", "")) for item in results if getattr(item, "reason", "")]
        message = "rota de pega aplicada" if success else "; ".join(reasons) or "falha ao aplicar"
        self.service_result.emit("pick_scheduler_set_parameters", success, message)

    def _vision_process_done(self, future: Future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.service_result.emit("vision_process_frame", False, str(exc))
            return
        success = bool(getattr(response, "success", False))
        message = str(getattr(response, "message", ""))
        self.service_result.emit("vision_process_frame", success, message)

    def _vision_calibrate_done(self, future: Future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.service_result.emit("vision_calibrate", False, str(exc))
            return
        success = bool(getattr(response, "success", False))
        message = str(getattr(response, "message", ""))
        profile_json = str(getattr(response, "profile_json", "{}"))
        report = str(getattr(response, "report", ""))
        self.service_result.emit("vision_calibrate", success, message)
        if success:
            self.vision_calibration_result.emit(profile_json, report)

    def publish_vision_test_image(
        self,
        width: int,
        height: int,
        rgb_data: bytes,
        step: int | None = None,
        frame_id: str = "camera_link",
    ) -> bool:
        if self._node is None:
            return False
        self._node.publish_vision_test_image(width, height, rgb_data, step, frame_id)
        self.log_message.emit("info", f"imagem de teste publicada em /camera/image_raw ({width}x{height})")
        return True

    def publish_authorization_lease(self) -> bool:
        if self._node is None or not self._authorization.identity.authenticated:
            return False
        try:
            lease = signed_lease(
                self._authorization.identity,
                production_mode=self._authorization.production_mode,
            )
            self._node.publish_authorization_lease(lease)
        except OSError as exc:
            self.log_message.emit("error", f"falha ao assinar autorização: {exc}")
            return False
        return True

    def current_tcp_pose(self) -> Optional[Pose]:
        return self.last_reached_pose

    # Actions ----------------------------------------------------------
    def _emit_action_result(
        self, request: dict[str, object], success: bool, message: str
    ) -> None:
        """Route a result to the regular or tagged Qt signal."""
        tag = request.get("tag")
        if tag is None:
            self.action_result.emit(str(request["label"]), success, str(message))
        else:
            self.pose_action_result.emit(str(tag), success, str(message))

    def _new_action_request(
        self,
        label: str,
        future: Future,
        *,
        result_timeout_s: float,
        tag: str | None = None,
    ) -> int:
        """Track goal acceptance and result deadlines for one action request."""
        with self._action_lock:
            self._action_seq += 1
            request_id = self._action_seq
            request: dict[str, object] = {
                "label": str(label),
                "tag": tag,
                "future": future,
                "handle": None,
                "result_timeout_s": max(0.1, float(result_timeout_s)),
                "accept_timer": None,
                "result_timer": None,
            }
            timer = threading.Timer(
                self.action_accept_timeout_s,
                self._on_action_accept_timeout,
                args=(request_id,),
            )
            timer.daemon = True
            request["accept_timer"] = timer
            self._action_requests[request_id] = request
            timer.start()
        future.add_done_callback(lambda f, rid=request_id: self._action_goal_done(rid, f))
        return request_id

    @staticmethod
    def _cancel_timer(timer: object) -> None:
        if isinstance(timer, threading.Timer):
            timer.cancel()

    def _pop_action_request(self, request_id: int) -> dict[str, object] | None:
        with self._action_lock:
            request = self._action_requests.pop(request_id, None)
        if request is not None:
            self._cancel_timer(request.get("accept_timer"))
            self._cancel_timer(request.get("result_timer"))
        return request

    def _on_action_accept_timeout(self, request_id: int) -> None:
        request = self._pop_action_request(request_id)
        if request is None or self._stopping.is_set():
            return
        label = str(request["label"])
        self.log_message.emit("warn", f"{label}: timeout aguardando aceitação do goal")
        self._emit_action_result(request, False, "timeout aguardando aceitação do goal")

    def _on_action_result_timeout(self, request_id: int) -> None:
        request = self._pop_action_request(request_id)
        if request is None or self._stopping.is_set():
            return
        handle = request.get("handle")
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # pragma: no cover - defensive
                self.log_message.emit("warn", f"cancel após timeout falhou: {exc}")
        label = str(request["label"])
        self.log_message.emit("warn", f"{label}: timeout aguardando resultado")
        self._emit_action_result(request, False, "timeout aguardando resultado da action")

    def _action_goal_done(self, request_id: int, future: Future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            request = self._pop_action_request(request_id)
            if request is not None:
                self._emit_action_result(request, False, str(exc))
            return

        with self._action_lock:
            request = self._action_requests.get(request_id)
            if request is not None:
                self._cancel_timer(request.get("accept_timer"))
                request["accept_timer"] = None

        # The request may have timed out or been cancelled while discovery was
        # still pending. If the server accepts it later, cancel it immediately.
        if request is None:
            if goal_handle is not None and getattr(goal_handle, "accepted", False):
                try:
                    goal_handle.cancel_goal_async()
                    self.log_message.emit(
                        "info",
                        "goal aceito após parada; cancelamento imediato solicitado",
                    )
                except Exception:  # pragma: no cover - defensive
                    pass
            return
        if goal_handle is None or not goal_handle.accepted:
            self._pop_action_request(request_id)
            self._emit_action_result(request, False, "goal rejeitado")
            return

        timer = threading.Timer(
            float(request["result_timeout_s"]),
            self._on_action_result_timeout,
            args=(request_id,),
        )
        timer.daemon = True
        with self._action_lock:
            current = self._action_requests.get(request_id)
            if current is not request:
                current = None
            else:
                request["handle"] = goal_handle
                request["result_timer"] = timer
                timer.start()
        if current is None:
            try:
                goal_handle.cancel_goal_async()
            except Exception:  # pragma: no cover - defensive
                pass
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, rid=request_id: self._action_result_done(rid, f)
        )

    @staticmethod
    def _parse_action_result(label: str, wrapped_result) -> tuple[bool, str, object | None]:
        result = wrapped_result.result
        status = int(getattr(wrapped_result, "status", GoalStatus.STATUS_UNKNOWN))
        if status != GoalStatus.STATUS_SUCCEEDED:
            status_names = {
                GoalStatus.STATUS_CANCELED: "goal cancelado",
                GoalStatus.STATUS_ABORTED: "goal abortado",
            }
            return False, status_names.get(status, f"action terminou com status={status}"), None

        error_code = getattr(result, "error_code", None)
        if error_code is not None:
            # Standard control actions encode SUCCESS as integer 0. MoveIt
            # wraps its code in MoveItErrorCodes and uses SUCCESS=1.
            if hasattr(error_code, "val"):
                value = int(error_code.val)
                success = value == 1
                message = (
                    getattr(error_code, "message", "")
                    or getattr(result, "error_string", "")
                    or f"moveit_error_code={value}"
                )
            else:
                value = int(error_code)
                success = value == 0
                message = getattr(result, "error_string", "") or f"error_code={value}"
        elif hasattr(result, "reached_goal"):
            success = bool(result.reached_goal)
            if success:
                message = "objetivo do gripper atingido"
            elif bool(getattr(result, "stalled", False)):
                message = "gripper bloqueado antes de atingir o objetivo"
            else:
                message = "gripper não atingiu o objetivo"
        else:
            success = bool(getattr(result, "success", True))
            message = getattr(result, "message", "")
        final_pose = getattr(result, "final_pose", None) if success else None
        return success, str(message), final_pose

    def _action_result_done(self, request_id: int, future: Future) -> None:
        request = self._pop_action_request(request_id)
        if request is None:
            return
        try:
            wrapped_result = future.result()
            success, message, final_pose = self._parse_action_result(
                str(request["label"]), wrapped_result
            )
        except Exception as exc:
            self._emit_action_result(request, False, str(exc))
            return
        if success and final_pose is not None and hasattr(final_pose, "position"):
            self.last_reached_pose = final_pose
        self._emit_action_result(request, success, message)

    def _send_action(
        self,
        label: str,
        future: Optional[Future],
        *,
        result_timeout_s: float,
        tag: str | None = None,
    ) -> bool:
        if future is None:
            self.log_message.emit("warn", f"{label}: action server indisponível")
            request = {"label": label, "tag": tag}
            self._emit_action_result(request, False, "action server indisponível")
            return False
        self._new_action_request(
            label,
            future,
            result_timeout_s=result_timeout_s,
            tag=tag,
        )
        self.log_message.emit("info", f"enviando goal {label} ...")
        return True

    def follow_joint_trajectory(
        self,
        joint_names: list[str],
        positions: list[float],
        duration: float,
        *,
        recipe_execution: bool = False,
    ) -> bool:
        capability = (
            CAP_EXECUTE_RELEASED
            if recipe_execution or self._recipe_execution_scope
            else CAP_MANUAL_MOTION
        )
        allowed, message = self._authorization.require(capability)
        if not allowed:
            self.log_message.emit("error", message)
            self.action_result.emit("follow_joint_trajectory", False, message)
            return False
        node = self._node
        effective_duration = self.effective_motion_duration(duration)
        return self._send_action(
            "follow_joint_trajectory",
            node.send_follow_joint_trajectory(joint_names, positions, effective_duration)
            if node else None,
            result_timeout_s=self.motion_result_timeout(effective_duration),
        )

    def move_to_pose(
        self, pose: Pose, duration: float, *, recipe_execution: bool = False
    ) -> bool:
        capability = (
            CAP_EXECUTE_RELEASED
            if recipe_execution or self._recipe_execution_scope
            else CAP_MANUAL_MOTION
        )
        allowed, message = self._authorization.require(capability)
        if not allowed:
            self.log_message.emit("error", message)
            self.action_result.emit("move_to_pose", False, message)
            return False
        node = self._node
        effective_duration = self.effective_motion_duration(duration)
        return self._send_action(
            "move_to_pose",
            node.send_move_to_pose(pose, effective_duration, self._on_move_feedback)
            if node else None,
            result_timeout_s=self.motion_result_timeout(effective_duration),
        )

    def _on_move_feedback(self, feedback_msg) -> None:
        feedback = getattr(feedback_msg, "feedback", None)
        if feedback is None:
            return
        progress = float(getattr(feedback, "progress", 0.0))
        current_pose = getattr(feedback, "current_pose", None)
        self.move_feedback.emit(progress, current_pose)

    # ---- Tagged MoveToPose (for sequential motion programs) ----
    def move_to_pose_tagged(
        self,
        tag: str,
        pose: Pose,
        duration: float,
        *,
        recipe_execution: bool = False,
    ) -> bool:
        """Send a MoveToPose goal whose result is reported via ``pose_action_result``.

        The ``tag`` is passed back unchanged so a caller running a sequence can
        correlate the result with the goal it sent. Only the most recent goal
        handle is kept for cancellation.
        """
        capability = (
            CAP_EXECUTE_RELEASED
            if recipe_execution or self._recipe_execution_scope
            else CAP_MANUAL_MOTION
        )
        allowed, message = self._authorization.require(capability)
        if not allowed:
            self.log_message.emit("error", message)
            self.pose_action_result.emit(str(tag), False, message)
            return False
        node = self._node
        effective_duration = self.effective_motion_duration(duration)
        future = (
            node.send_move_to_pose(pose, effective_duration, self._on_move_feedback)
            if node is not None
            else None
        )
        return self._send_action(
            "move_to_pose",
            future,
            result_timeout_s=self.motion_result_timeout(effective_duration),
            tag=tag,
        )

    def move_to_pose_planned_tagged(
        self,
        tag: str,
        pose: Pose,
        duration: float,
        *,
        program_speed_scale: float = 1.0,
        recipe_speed_ceiling: float = 1.0,
    ) -> bool:
        """Collision-aware MoveIt plan+execute used by production recipes."""
        allowed, message = self._authorization.require(CAP_EXECUTE_RELEASED)
        if not allowed:
            self.pose_action_result.emit(str(tag), False, message)
            self.log_message.emit("error", message)
            return False
        node = self._node
        effective_duration = self.effective_motion_duration(duration)
        future = (
            node.send_collision_planned_pose(
                pose,
                min(
                    1.0,
                    (
                        1.0
                        if self._dynamic_speed_override_supported
                        else self._speed_override
                    )
                    * max(0.01, float(program_speed_scale))
                    * max(0.01, min(1.0, float(recipe_speed_ceiling))),
                ),
            )
            if node is not None
            else None
        )
        return self._send_action(
            "move_group",
            future,
            result_timeout_s=max(30.0, self.motion_result_timeout(effective_duration)),
            tag=tag,
        )

    def collision_planner_ready(self) -> bool:
        node = self._node
        return bool(
            MOVEIT_AVAILABLE
            and node is not None
            and node.action_server_ready("move_group")
        )

    def cancel_move_to_pose(self) -> bool:
        """Cancel every pending or active MoveToPose goal."""
        return self.cancel_active_actions({"move_to_pose", "move_group"}) > 0

    def _clear_action_requests(self, *, cancel_handles: bool) -> int:
        """Drop tracked actions and optionally request cancellation for accepted goals."""
        with self._action_lock:
            requests = list(self._action_requests.values())
            self._action_requests.clear()
        for request in requests:
            self._cancel_timer(request.get("accept_timer"))
            self._cancel_timer(request.get("result_timer"))
            handle = request.get("handle")
            if cancel_handles and handle is not None:
                try:
                    future = handle.cancel_goal_async()
                    future.add_done_callback(self._on_move_cancel_done)
                except Exception as exc:  # pragma: no cover - defensive
                    self.log_message.emit("warn", f"cancel action falhou: {exc}")
        return len(requests)

    def cancel_active_actions(self, labels: set[str] | None = None) -> int:
        """Cancel pending and accepted actions, optionally filtered by interface label.

        Pending goal requests are removed immediately. If a server accepts one
        after this call, its completion callback notices that it is no longer
        tracked and cancels the late goal before it can continue unattended.
        """
        wanted = {str(label) for label in labels} if labels is not None else None
        with self._action_lock:
            selected = {
                request_id: request
                for request_id, request in self._action_requests.items()
                if wanted is None or str(request["label"]) in wanted
            }
            for request_id in selected:
                self._action_requests.pop(request_id, None)
        for request in selected.values():
            self._cancel_timer(request.get("accept_timer"))
            self._cancel_timer(request.get("result_timer"))
            handle = request.get("handle")
            if handle is not None:
                try:
                    future = handle.cancel_goal_async()
                    future.add_done_callback(self._on_move_cancel_done)
                except Exception as exc:  # pragma: no cover - defensive
                    self.log_message.emit("warn", f"cancel action falhou: {exc}")
        if selected:
            labels_text = ", ".join(sorted({str(r["label"]) for r in selected.values()}))
            self.log_message.emit(
                "info", f"cancelamento solicitado para {len(selected)} action(s): {labels_text}"
            )
        return len(selected)

    def _on_move_cancel_done(self, future: Future) -> None:
        try:
            response = future.result()
            goals_canceling = getattr(response, "goals_canceling", None) or []
            if goals_canceling:
                self.log_message.emit("info", "cancelamento de action aceito pelo servidor")
            else:
                self.log_message.emit("warn", "servidor não confirmou o cancelamento da action")
        except Exception as exc:  # pragma: no cover - defensive
            self.log_message.emit("warn", f"cancel action callback: {exc}")

    def gripper_command(
        self,
        position: float,
        max_effort: float = 0.0,
        *,
        recipe_execution: bool = False,
    ) -> bool:
        capability = (
            CAP_EXECUTE_RELEASED
            if recipe_execution or self._recipe_execution_scope
            else CAP_MANUAL_MOTION
        )
        allowed, message = self._authorization.require(capability)
        if not allowed:
            self.log_message.emit("error", message)
            self.action_result.emit("gripper_command", False, message)
            return False
        node = self._node
        return self._send_action(
            "gripper_command",
            node.send_gripper_command(position, max_effort) if node else None,
            result_timeout_s=10.0,
        )

    # Raw commands -----------------------------------------------------
    def publish_mit(
        self, joint_name: str, pos: float, vel: float, kp: float, kd: float, tau: float
    ) -> bool:
        if not self._authorized(CAP_RAW_CONTROL, f"mit({joint_name})"):
            return False
        if self._node is None:
            return False
        self._node.ensure_raw_publishers()
        return self._node.publish_mit(joint_name, pos, vel, kp, kd, tau)

    def publish_posvel(self, joint_name: str, pos: float, vlim: float) -> bool:
        if not self._authorized(CAP_RAW_CONTROL, f"posvel({joint_name})"):
            return False
        if self._node is None:
            return False
        self._node.ensure_raw_publishers()
        return self._node.publish_posvel(joint_name, pos, vlim)

    # ------------------------------------------------------------------
    # live-state accessors (ConditionContext for the motion-program VM)
    # ------------------------------------------------------------------
    @property
    def state_machine(self) -> str:
        node = self._node
        if self._arm_alive_state and node is not None and node.arm_status is not None:
            return str(node.arm_status.state_machine or "")
        return ""

    @property
    def is_arm_enabled(self) -> bool:
        node = self._node
        if self._arm_alive_state and node is not None and node.arm_status is not None:
            return bool(node.arm_status.enabled)
        return False

    @property
    def is_connected(self) -> bool:
        return self._node is not None

    def action_server_ready(self, name: str) -> bool:
        """Return discovery state for a named action server without blocking."""
        node = self._node
        return node.action_server_ready(name) if node is not None else False

    def ik_service_ready(self, timeout_sec: float = 0.05) -> bool:
        """Non-blocking discovery check for the ``move_to_pose_ik`` service."""
        node = self._node
        client = getattr(node, "cli_move_to_pose_ik", None) if node is not None else None
        if client is None:
            return False
        try:
            return bool(client.wait_for_service(timeout_sec=float(timeout_sec)))
        except Exception:
            return False

    def motion_readiness(self, action_name: str = "move_to_pose") -> tuple[bool, str]:
        """Central readiness policy used by every motion entry point."""
        if self._node is None:
            return False, "Não conectado ao ROS 2."
        if not self._arm_alive_state:
            return False, "Controlador sem heartbeat recente em arm_status."
        if not self.is_arm_enabled:
            return False, "Braço não está habilitado (enable)."
        state = self.state_machine.upper()
        if state in {"PROTECTIVE_STOP", "ERROR", "FAULT", "EMERGENCY_STOP"}:
            return False, f"Braço está em estado {state}."
        if not self.action_server_ready(action_name):
            return False, f"Action server {action_name} indisponível."
        return True, ""

    def signal_by_id(self, io_id: str) -> int:
        """Current level of a digital input by its logical id (0/1)."""
        node = self._node
        if node is None:
            return 0
        io = node.dio_ios.get(str(io_id))
        if io is None or not io.enabled:
            return 0
        return 1 if bool(io.value) else 0

    def dio_ios_snapshot(self) -> dict:
        """Return a copy of the latest id -> DigitalIO map (empty if no node)."""
        node = self._node
        return dict(node.dio_ios) if node is not None else {}

    @staticmethod
    def _dio_default_ids(direction: str) -> list[str]:
        if direction == "input":
            return [f"IN{i}" for i in range(1, 6)]
        return [f"OUT{i}" for i in range(1, 6)]

    def dio_input_ids(self) -> list[str]:
        node = self._node
        if node is None:
            return self._dio_default_ids("input")
        ids = sorted(i.id for i in node.dio_ios.values()
                     if int(i.direction) == DigitalIO.DIR_INPUT)
        return ids or self._dio_default_ids("input")

    def dio_output_ids(self) -> list[str]:
        node = self._node
        if node is None:
            return self._dio_default_ids("output")
        ids = sorted(i.id for i in node.dio_ios.values()
                     if int(i.direction) == DigitalIO.DIR_OUTPUT)
        return ids or self._dio_default_ids("output")

    def joint_position(self, joint: str) -> Optional[float]:
        node = self._node
        if node is None or node.joint_state is None or not joint:
            return None
        names = node.joint_state.name
        if joint in names:
            return float(node.joint_state.position[names.index(joint)])
        return None

    @property
    def gripper_position(self) -> Optional[float]:
        node = self._node
        if node is not None and node.gripper_state is not None:
            try:
                return float(node.gripper_state.position)
            except (TypeError, ValueError):
                return None
        return None

    def vision_detected(self) -> bool:
        node = self._node
        return bool(node.vision_detected) if node is not None else False

    def vision_pose(self, axis: str) -> Optional[float]:
        node = self._node
        if node is None or node.vision_pose is None:
            return None
        p = node.vision_pose.position
        return {"x": p.x, "y": p.y, "z": p.z}.get(str(axis).lower())

    def joint_names(self) -> list[str]:
        node = self._node
        return node.arm_joint_names() if node is not None else []

    def current_arm_joint_positions(self) -> list[float]:
        """Measured arm positions ordered like :meth:`joint_names`."""

        node = self._node
        if node is None or node.joint_state is None:
            return []
        values = dict(zip(node.joint_state.name, node.joint_state.position))
        names = node.arm_joint_names()
        try:
            return [float(values[name]) for name in names]
        except (KeyError, TypeError, ValueError):
            return []

    def trajectory_limits(self) -> dict[str, object]:
        """Authoritative dynamic limits published by the active controller."""

        node = self._node
        if node is None or not node.trajectory_limits_data:
            return {}
        data = node.trajectory_limits_data
        return {
            "joint_names": list(data.get("joint_names", [])),
            "max_velocity": list(data.get("max_velocity", [])),
            "max_acceleration": list(data.get("max_acceleration", [])),
            "safety_factor": data.get("safety_factor"),
        }

    # ------------------------------------------------------------------
    # emergency stop / heartbeat / URDF helpers
    # ------------------------------------------------------------------
    def emergency_stop(self) -> None:
        """Software stop: cancel every tracked action and request disable.

        This is a software convenience only — it does NOT replace a hardware
        emergency stop, safety PLC or risk assessment of the cell.
        """
        cancelled = self.cancel_active_actions()
        self.log_message.emit(
            "warn",
            "Parada de software: "
            f"cancelamento solicitado para {cancelled} action(s); solicitando disable",
        )
        self.disable()

    @property
    def is_arm_alive(self) -> bool:
        """True while ``/{ns}/arm_status`` keeps arriving (see arm_status_timeout)."""
        return self._arm_alive_state

    def joint_limits(self) -> Dict[str, Tuple[float, float]]:
        """Per-joint (lower, upper) position limits parsed from the URDF.

        Returns an empty dict while ``/robot_description`` is unavailable.
        """
        node = self._node
        urdf = node.robot_description if node is not None else None
        if not urdf:
            return {}
        try:
            return kinematics.joint_limits(urdf)
        except Exception:  # pragma: no cover - defensive
            return {}

    def tcp_pose_fk(self) -> Optional[Pose]:
        """TCP pose from forward kinematics over the latest joint_states.

        Uses the URDF chain base_link -> gripper_link. Returns None when the
        URDF or joint states are not available yet.
        """
        node = self._node
        if node is None or node.robot_description is None or node.joint_state is None:
            return None
        try:
            chain = kinematics.load_chain(node.robot_description)
            if not chain:
                return None
            q = {
                name: float(pos)
                for name, pos in zip(node.joint_state.name, node.joint_state.position)
            }
            transform = kinematics.tcp_transform(chain, q)
            x, y, z = (float(v) for v in transform[:3, 3])
            qx, qy, qz, qw = kinematics.rotation_to_quaternion(transform[:3, :3])
            return make_pose(x, y, z, qx, qy, qz, qw)
        except Exception:  # pragma: no cover - defensive
            return None
