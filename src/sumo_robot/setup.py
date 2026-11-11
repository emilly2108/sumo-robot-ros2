from glob import glob
import os

from setuptools import find_packages, setup


package_name = "sumo_robot"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.xml"),
        ),
    ],
    package_data={package_name: [
        "center_calibration.json",
        "hfsm_brain/robot_settings.yaml",
    ]},
    include_package_data=True,
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="mars",
    maintainer_email="mars@todo.todo",
    description="ROS 2 control stack for the SteadyWin four-wheel sumo robot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vision_node = sumo_robot.vision_node:main",
            "hfsm_brain_node = sumo_robot.hfsm_brain.node:main",
            "motor_node = sumo_robot.motor_node:main",
            "switch_node = sumo_robot.switch_node:main",
            "color_node = sumo_robot.color_node:main",
            "color_recovery_supervisor = sumo_robot.color_recovery_supervisor:main",
        ],
    },
)
