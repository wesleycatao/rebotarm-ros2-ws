from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
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
    use_rviz = LaunchConfiguration("use_rviz")
    frame_id = LaunchConfiguration("frame_id")
    ee_frame_id = LaunchConfiguration("ee_frame_id")
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
    torque_feedback_timeout = LaunchConfiguration("torque_feedback_timeout")
    safety_reset_max_velocity = LaunchConfiguration("safety_reset_max_velocity")
    safety_reset_quiet_samples = LaunchConfiguration("safety_reset_quiet_samples")
    torque_payload_tolerance_kg = LaunchConfiguration("torque_payload_tolerance_kg")
    deadline_watchdog_enabled = LaunchConfiguration("deadline_watchdog_enabled")
    control_loop_deadline_ms = LaunchConfiguration("control_loop_deadline_ms")
    control_loop_deadline_violations = LaunchConfiguration(
        "control_loop_deadline_violations"
    )
    safety_monitor_deadline_ms = LaunchConfiguration("safety_monitor_deadline_ms")
    timing_diagnostics_rate = LaunchConfiguration("timing_diagnostics_rate")
    authorization_required = LaunchConfiguration("authorization_required")
    authorization_secret_path = LaunchConfiguration("authorization_secret_path")

    urdf_file = PathJoinSubstitution(
        [
            bringup_share,
            "description",
            "urdf",
            PythonExpression(
                [
                    "'00-arm-rs_asm-v3.urdf' if '",
                    model,
                    "'.lower() == 'rs' else 'reBot-DevArm_fixend.urdf'",
                ]
            ),
        ]
    )
    rviz_config = PathJoinSubstitution([bringup_share, "rviz", "rebotarm.rviz"])
    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)

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
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("ee_frame_id", default_value="end_link"),
            DeclareLaunchArgument("disable_after_safe_home", default_value="true"),
            DeclareLaunchArgument("torque_profile_path", default_value=""),
            DeclareLaunchArgument("torque_profile_mode", default_value="off"),
            DeclareLaunchArgument("torque_profile_step_id", default_value="default"),
            DeclareLaunchArgument(
                "torque_profile_violation_samples", default_value="3"
            ),
            DeclareLaunchArgument(
                "torque_profile_derivative_limit", default_value="0.0"
            ),
            DeclareLaunchArgument("torque_feedback_timeout", default_value="0.10"),
            DeclareLaunchArgument("safety_reset_max_velocity", default_value="0.03"),
            DeclareLaunchArgument("safety_reset_quiet_samples", default_value="5"),
            DeclareLaunchArgument("torque_payload_tolerance_kg", default_value="0.10"),
            DeclareLaunchArgument("deadline_watchdog_enabled", default_value="true"),
            DeclareLaunchArgument("control_loop_deadline_ms", default_value="6.0"),
            DeclareLaunchArgument(
                "control_loop_deadline_violations", default_value="3"
            ),
            DeclareLaunchArgument("safety_monitor_deadline_ms", default_value="30.0"),
            DeclareLaunchArgument("timing_diagnostics_rate", default_value="1.0"),
            DeclareLaunchArgument("authorization_required", default_value="false"),
            DeclareLaunchArgument("authorization_secret_path", default_value=""),
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                on_exit=Shutdown(reason="reBotArmController exited"),
                parameters=[
                    {
                        "hardware_config": hardware_config,
                        "model": model,
                        "channel": channel,
                        "joint_state_rate": joint_state_rate,
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                        "frame_id": frame_id,
                        "ee_frame_id": ee_frame_id,
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
                        "torque_feedback_timeout": ParameterValue(
                            torque_feedback_timeout,
                            value_type=float,
                        ),
                        "safety_reset_max_velocity": ParameterValue(
                            safety_reset_max_velocity,
                            value_type=float,
                        ),
                        "safety_reset_quiet_samples": ParameterValue(
                            safety_reset_quiet_samples,
                            value_type=int,
                        ),
                        "torque_payload_tolerance_kg": ParameterValue(
                            torque_payload_tolerance_kg,
                            value_type=float,
                        ),
                        "deadline_watchdog_enabled": ParameterValue(
                            deadline_watchdog_enabled,
                            value_type=bool,
                        ),
                        "control_loop_deadline_ms": ParameterValue(
                            control_loop_deadline_ms,
                            value_type=float,
                        ),
                        "control_loop_deadline_violations": ParameterValue(
                            control_loop_deadline_violations,
                            value_type=int,
                        ),
                        "safety_monitor_deadline_ms": ParameterValue(
                            safety_monitor_deadline_ms,
                            value_type=float,
                        ),
                        "timing_diagnostics_rate": ParameterValue(
                            timing_diagnostics_rate,
                            value_type=float,
                        ),
                        "authorization_required": ParameterValue(
                            authorization_required,
                            value_type=bool,
                        ),
                        "authorization_secret_path": authorization_secret_path,
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
