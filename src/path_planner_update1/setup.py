from setuptools import setup
from setuptools import find_packages

package_name = "path_planner_update1"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/full_system.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotics_user",
    maintainer_email="you@example.com",
    description="Advanced Path Smoothing and Trajectory Tracking for TurtleBot3 using ROS 2 Humble",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "follower_node = path_planner_update1.follower_node:main",
            "visualizer_node = path_planner_update1.visualizer_node:main",
        ],
    },
)

