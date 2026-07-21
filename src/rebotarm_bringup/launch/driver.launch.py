from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    hardware_config = LaunchConfiguration("hardware_config")
    model = LaunchConfiguration("model")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    arm_namespace = LaunchConfiguration("arm_namespace")
    disable_after_safe_home = LaunchConfiguration("disable_after_safe_home")
    torque_profile_path = LaunchConfiguration("torque_profile_path")
    torque_profile_mode = LaunchConfiguration("torque_profile_mode")
    torque_profile_step_id = LaunchConfiguration("torque_profile_step_id")
    torque_profile_violation_samples = LaunchConfiguration(
        "torque_profile_violation_samples"
    )
    torque_profile_derivative_limit = LaunchConfiguration(
        "torque_profile_derivative_limit"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "rebotarm_hardware.yaml"]
                ),
            ),
            DeclareLaunchArgument("model", default_value=""),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("disable_after_safe_home", default_value="true"),
            DeclareLaunchArgument("torque_profile_path", default_value=""),
            DeclareLaunchArgument("torque_profile_mode", default_value="off"),
            DeclareLaunchArgument("torque_profile_step_id", default_value="default"),
            DeclareLaunchArgument(
                "torque_profile_violation_samples",
                default_value="3",
            ),
            DeclareLaunchArgument(
                "torque_profile_derivative_limit",
                default_value="0.0",
            ),
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                parameters=[
                    {
                        "hardware_config": hardware_config,
                        "model": model,
                        "channel": channel,
                        "joint_state_rate": joint_state_rate,
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                        "disable_after_safe_home": ParameterValue(
                            disable_after_safe_home,
                            value_type=bool,
                        ),
                        "torque_profile_path": torque_profile_path,
                        "torque_profile_mode": torque_profile_mode,
                        "torque_profile_step_id": torque_profile_step_id,
                        "torque_profile_violation_samples": ParameterValue(
                            torque_profile_violation_samples,
                            value_type=int,
                        ),
                        "torque_profile_derivative_limit": ParameterValue(
                            torque_profile_derivative_limit,
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
