from glob import glob

from setuptools import setup


package_name = "rebotarm_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.json")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/test_images", glob("test_images/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Webcam vision MVP for reBotArm object segmentation and pick suggestions.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_node = rebotarm_vision.camera_node:main",
            "image_file_node = rebotarm_vision.image_file_node:main",
            "segmentation_node = rebotarm_vision.segmentation_node:main",
            "object_pose_node = rebotarm_vision.object_pose_node:main",
            "pick_scheduler_node = rebotarm_vision.pick_scheduler_node:main",
            "vision_pick_executor = rebotarm_vision.vision_pick_executor:main",
            "vision_calibration_node = rebotarm_vision.vision_calibration_node:main",
        ],
    },
)
