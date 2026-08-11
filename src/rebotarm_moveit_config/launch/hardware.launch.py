import os
from importlib.machinery import SourceFileLoader
from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

moveit_parameters = SourceFileLoader(
    "moveit_launch_common",
    os.path.join(os.path.dirname(__file__), "moveit_launch_common.py"),
).load_module().moveit_parameters


def generate_launch_description():
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file in rebotarm_moveit_config/launch",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz with the MoveIt motion planning plugin",
    )
    arm_namespace_arg = DeclareLaunchArgument(
        "arm_namespace",
        default_value="rebotarm",
        description="Namespace used by an already-running reBotArmController",
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="dm",
        description="Robot model to load: dm or rs",
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            use_rviz_arg,
            arm_namespace_arg,
            model_arg,
            OpaqueFunction(function=_launch_setup),
        ]
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    MoveItConfigsBuilder = _load_moveit_configs_builder()
    _require_package(
        "moveit_ros_move_group",
        "The current ROS Lyrical apt repository does not publish this package; "
        "install MoveIt from source or use a ROS distro with the full MoveIt stack.",
    )
    _require_clearance_service()

    model = LaunchConfiguration("model").perform(context).strip().lower()
    is_rs = model == "rs"
    arm_namespace = LaunchConfiguration("arm_namespace")

    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(
            file_path=(
                "config/rebotarm_rs.urdf.xacro"
                if is_rs
                else "config/rebotarm.urdf.xacro"
            )
        )
        .robot_description_semantic(
            file_path="config/rebotarm_rs.srdf" if is_rs else "config/rebotarm.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_hardware_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    moveit_params = moveit_parameters(moveit_config)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
        remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
    )

    clearance_service_node = Node(
        package="rebotarm_moveit_config",
        executable="clearance_service_node",
        name="rebotarm_clearance_service",
        output="screen",
        parameters=[
            moveit_params,
            {
                "joint_states_topic": "/joint_states",
                "planning_scene_topic": "/monitored_planning_scene",
                "get_planning_scene_service": "/get_planning_scene",
            },
        ],
        remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("rebotarm_moveit_config"),
            "launch",
            LaunchConfiguration("rviz_config"),
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[moveit_params],
        remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
        remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
    )

    return [
        static_tf_node,
        robot_state_publisher_node,
        move_group_node,
        clearance_service_node,
        rviz_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=move_group_node,
                on_exit=[
                    EmitEvent(event=Shutdown(reason="move_group exited"))
                ],
            )
        ),
    ]


def _load_moveit_configs_builder():
    try:
        from moveit_configs_utils import MoveItConfigsBuilder
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Missing moveit_configs_utils. Install ros-lyrical-moveit-configs-utils "
            "or source a workspace that provides it before launching MoveIt."
        ) from exc
    return MoveItConfigsBuilder


def _require_package(package_name, hint):
    try:
        get_package_prefix(package_name)
    except PackageNotFoundError as exc:
        raise RuntimeError(f"Missing required ROS package '{package_name}'. {hint}") from exc


def _require_clearance_service():
    executable = os.path.join(
        get_package_prefix("rebotarm_moveit_config"),
        "lib",
        "rebotarm_moveit_config",
        "clearance_service_node",
    )
    if not os.path.exists(executable):
        raise RuntimeError(
            "clearance_service_node was not built. MoveIt C++ packages "
            "moveit_core and moveit_ros_planning are unavailable in this environment, "
            "so trajectory clearance checking cannot run."
        )
