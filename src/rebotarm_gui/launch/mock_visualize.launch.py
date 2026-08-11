"""Launch RViz + robot_state_publisher para visualizar o mock do braço.

Uso:
    ros2 launch rebotarm_gui mock_visualize.launch.py

Requisitos:
    - mock_arm rodando (ros2 run rebotarm_gui mock_arm)
    - pacote rebotarm_bringup no workspace (contém o URDF e as malhas)
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_urdf_path() -> Path:
    """Resolve the mock URDF from the installed rebotarm_bringup share dir."""
    candidate = Path(
        get_package_share_directory("rebotarm_bringup"),
        "description",
        "urdf",
        "reBot_B601_DM_with_gripper.urdf",
    )
    if candidate.is_file():
        return candidate
    # Fallback para execucao a partir do source tree (symlink-install).
    return Path(__file__).parent.parent.parent / "rebotarm_bringup" / "description" / "urdf" / "reBot_B601_DM_with_gripper.urdf"


def generate_launch_description():
    urdf_path = _resolve_urdf_path()
    robot_description = urdf_path.read_text(encoding="utf-8")

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="rebotarm",
        description="Namespace do braço (deve bater com o mock_arm)",
    )
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            namespace_arg,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=namespace,
                parameters=[{"robot_description": robot_description}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(Path(__file__).parent / "mock_visualize.rviz")],
                parameters=[{"use_sim_time": False}],
            ),
        ]
    )
