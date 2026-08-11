from pathlib import Path
from setuptools import find_packages, setup

package_name = "rebotarm_gui"

_icon_files = [str(p) for p in (Path(__file__).parent / "rebotarm_gui" / "icons").glob("*.svg")]
_locale_files = [
    str(p) for p in (Path(__file__).parent / "rebotarm_gui" / "locale").glob("*.json")
]

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (
            f"share/{package_name}",
            [
                "package.xml",
                "README.md",
                "GUI_UX_REDESIGN.md",
                "ACCESSIBILITY_USABILITY.md",
                "AUDIT_LOG.md",
            ],
        ),
        (f"share/{package_name}/styles", [
            "rebotarm_gui/styles/dark.qss",
            "rebotarm_gui/styles/light.qss",
        ]),
        (f"share/{package_name}/icons", _icon_files),
        (f"share/{package_name}/locale", _locale_files),
        (f"share/{package_name}/launch", ["launch/gui.launch.py", "launch/mock_visualize.launch.py", "launch/mock_visualize.rviz"]),
    ],
    install_requires=["setuptools", "PyYAML", "matplotlib", "numpy"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Graphical user interface for the reBotArm B601 controller (ROS 2).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rebotarm_gui = rebotarm_gui.main:main",
            "mock_arm = rebotarm_gui.mock_arm:main",
            "plot_mock_joints = rebotarm_gui.scripts.plot_mock_joints:main",
        ],
    },
)
