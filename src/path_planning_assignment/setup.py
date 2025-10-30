#!/usr/bin/env python3
"""
Setup file for path_planning_assignment ROS2 package
"""
from setuptools import setup
import os
from glob import glob

package_name = "path_planning_assignment"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        # Register package with ROS2
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        
        # Install package.xml
        ("share/" + package_name, ["package.xml"]),
        
        # Install launch files
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("path_planning_assignment", "launch", "*.launch.py"))
        ),
        
        # Install config files (YAML)
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("path_planning_assignment", "config", "*.yaml"))
        ),
        
        # Install RViz config files
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("path_planning_assignment", "rviz", "*.rviz"))
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="Path planning assignment with trajectory following and YAML configuration",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "follower_node = path_planning_assignment.follower_node:main",
            "visualizer = path_planning_assignment.visualizer:main",
        ],
    },
)

