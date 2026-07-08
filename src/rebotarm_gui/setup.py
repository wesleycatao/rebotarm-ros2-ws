from setuptools import find_packages, setup

package_name = "rebotarm_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/styles", ["rebotarm_gui/styles/dark.qss"]),
        (f"share/{package_name}/launch", ["launch/gui.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Graphical user interface for the reBotArm B601 controller (ROS 2).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rebotarm_gui = rebotarm_gui.main:main",
            "mock_arm = rebotarm_gui.mock_arm:main",
        ],
    },
)