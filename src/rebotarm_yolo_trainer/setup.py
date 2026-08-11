from glob import glob

from setuptools import setup


package_name = "rebotarm_yolo_trainer"

setup(
    name=package_name,
    version="0.3.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/example_dataset", glob("example_dataset/*")),
    ],
    install_requires=["setuptools", "ultralytics>=8.3.0", "PyYAML"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Simple YOLO training/export utilities for reBotArm object detection.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "yolo_train = rebotarm_yolo_trainer.train:main",
            "yolo_export = rebotarm_yolo_trainer.export:main",
            "yolo_inspect = rebotarm_yolo_trainer.inspect:main",
        ],
    },
)
