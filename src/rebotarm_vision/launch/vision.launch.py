from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vision_share = FindPackageShare("rebotarm_vision")
    config = LaunchConfiguration("config")
    use_camera = LaunchConfiguration("use_camera")
    use_static_image = LaunchConfiguration("use_static_image")
    use_executor = LaunchConfiguration("use_executor")
    use_calibration = LaunchConfiguration("use_calibration")
    static_image_path = LaunchConfiguration("static_image_path")
    static_image_rate = LaunchConfiguration("static_image_rate")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [vision_share, "config", "vision.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument("use_static_image", default_value="false"),
            DeclareLaunchArgument(
                "static_image_path",
                default_value="",
                description=(
                    "Imagem de teste. Vazio usa test_images/default_vision_test.png "
                    "quando disponível."
                ),
            ),
            DeclareLaunchArgument("static_image_rate", default_value="1.0"),
            DeclareLaunchArgument("use_executor", default_value="true"),
            DeclareLaunchArgument("use_calibration", default_value="false"),
            Node(
                package="rebotarm_vision",
                executable="camera_node",
                name="rebotarm_webcam",
                output="screen",
                parameters=[config],
                condition=IfCondition(use_camera),
            ),
            Node(
                package="rebotarm_vision",
                executable="image_file_node",
                name="rebotarm_image_file",
                output="screen",
                parameters=[
                    config,
                    {
                        "image_path": static_image_path,
                        "publish_rate": static_image_rate,
                    },
                ],
                condition=IfCondition(use_static_image),
            ),
            Node(
                package="rebotarm_vision",
                executable="segmentation_node",
                name="rebotarm_segmentation",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="rebotarm_vision",
                executable="object_pose_node",
                name="rebotarm_object_pose",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="rebotarm_vision",
                executable="pick_scheduler_node",
                name="rebotarm_pick_scheduler",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="rebotarm_vision",
                executable="vision_pick_executor",
                name="rebotarm_vision_pick_executor",
                output="screen",
                parameters=[config],
                condition=IfCondition(use_executor),
            ),
            Node(
                package="rebotarm_vision",
                executable="vision_calibration_node",
                name="rebotarm_vision_calibration",
                output="screen",
                parameters=[
                    config,
                    {
                        "api_key": EnvironmentVariable(
                            "OPENAI_API_KEY", default_value=""
                        ),
                    },
                ],
                condition=IfCondition(use_calibration),
            ),
        ]
    )
