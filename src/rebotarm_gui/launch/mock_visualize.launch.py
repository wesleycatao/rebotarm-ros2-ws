"""Launch RViz + robot_state_publisher para visualizar o mock do braço.

Uso:
    ros2 launch rebotarm_gui mock_visualize.launch.py

Requisitos:
    - mock_arm rodando (ros2 run rebotarm_gui mock_arm)
    - pacote rebotarm_bringup no workspace (contém o URDF e as malhas)
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = Path(__file__).parent.parent.parent / "rebotarm_bringup" / "description" / "urdf" / "reBot_B601_DM_with_gripper.urdf"
    # Fallback caso o launch esteja sendo usado de outro lugar.
    if not urdf_path.is_file():
        urdf_path = Path("/home/wesley/ros2_ws/src/rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf")

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
