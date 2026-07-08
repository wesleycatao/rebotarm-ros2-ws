"""Launch file for the reBotArm GUI."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the reBotArm GUI node."""
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="rebotarm",
        description="Namespace do braço reBotarm.",
    )

    gui_node = Node(
        package="rebotarm_gui",
        executable="rebotarm_gui",
        name="rebotarm_gui",
        output="screen",
        arguments=["--namespace", LaunchConfiguration("namespace")],
    )

    return LaunchDescription([namespace_arg, gui_node])