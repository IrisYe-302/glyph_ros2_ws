from glob import glob
import os

from setuptools import find_packages, setup


package_name = "go2_unitree_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ming",
    maintainer_email="maintainer@example.com",
    description="ROS 2 bridge for Unitree Go2 using official unitree_ros2 topics.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "go2_unitree_bridge_node = go2_unitree_bridge.bridge_node:main",
            "go2_mujoco_bridge_node = go2_unitree_bridge.mujoco_bridge_node:main",
        ],
    },
)
