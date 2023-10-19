import os
from glob import glob
from setuptools import setup

package_name = "nturt_nms1_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),

        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.rviz"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),

        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),

        (os.path.join("share", package_name, "maps"), glob(os.path.join("maps", "*.png"))),
        (os.path.join("share", package_name, "maps"), glob(os.path.join("maps", "*.yaml"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Quantumspawner",
    maintainer_email="jet22854111@gmail.com",
    description="Package with simulating environment based on f1tenth gym for nano model simplification, mark 1.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
                "f1tenth_gym_ros_bridge = nturt_nms1_sim.f1tenth_gym_ros_bridge:main"
        ],
    },
)
