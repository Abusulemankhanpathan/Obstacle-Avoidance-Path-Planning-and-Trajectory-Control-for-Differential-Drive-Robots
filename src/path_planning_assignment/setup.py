# setup.py
from setuptools import setup

package_name = "path_planning_assignment"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Your Name",
    author_email="you@example.com",
    description="Path planning assignment package",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "follower_node = path_planning_assignment.follower_node:main",
            "visualizer = path_planning_assignment.visualizer:main",
        ],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["path_planning_assignment/launch/full_system.launch.py"]),
    ],
)

