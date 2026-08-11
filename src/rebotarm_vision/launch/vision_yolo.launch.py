"""Launch the reBotArm vision pipeline using a trained YOLO model.

Intended for the Raspberry Pi 5, where the model was trained on a PC and
copied over (e.g. via SCP).

Example::

    ros2 launch rebotarm_vision vision_yolo.launch.py \
        model_path:=/home/pi/rebotarm_modelos/deploy/best_ncnn_model \
        image_size:=320 \
        camera_index:=0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config_file = PathJoinSubstitution(
        [FindPackageShare("rebotarm_vision"), "config", "vision_yolo.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value="",
                description="Path to trained YOLO/NCNN model. Required for YOLO backend.",
            ),
            DeclareLaunchArgument(
                "image_size",
                default_value="320",
                description="YOLO input size (320 for Pi 5, 416 for better accuracy).",
            ),
            DeclareLaunchArgument(
                "camera_index",
                default_value="0",
                description="V4L2 camera index.",
            ),
            DeclareLaunchArgument(
                "confidence_threshold",
                default_value="0.35",
                description="Minimum YOLO confidence to keep a detection.",
            ),
            DeclareLaunchArgument(
                "process_mode",
                default_value="manual",
                description="'manual' waits for /rebotarm_segmentation/process_frame; "
                "'continuous' runs capped by max_processing_fps.",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="Path to the vision pipeline YAML configuration file.",
            ),
            Node(
                package="rebotarm_vision",
                executable="camera_node",
                name="rebotarm_webcam",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"camera_index": LaunchConfiguration("camera_index")},
                ],
            ),
            Node(
                package="rebotarm_vision",
                executable="segmentation_node",
                name="rebotarm_segmentation",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "image_size": LaunchConfiguration("image_size"),
                        "confidence_threshold": LaunchConfiguration("confidence_threshold"),
                        "process_mode": LaunchConfiguration("process_mode"),
                    },
                ],
            ),
            Node(
                package="rebotarm_vision",
                executable="object_pose_node",
                name="rebotarm_object_pose",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
            Node(
                package="rebotarm_vision",
                executable="pick_scheduler_node",
                name="rebotarm_pick_scheduler",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
