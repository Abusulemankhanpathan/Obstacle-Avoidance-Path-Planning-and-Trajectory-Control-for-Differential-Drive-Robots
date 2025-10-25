#!/usr/bin/env python3
"""
Launch file to start the complete system:
- Gazebo simulation with TurtleBot3 world
- Trajectory follower node
- Path visualizer node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Set TURTLEBOT3 model
    set_tb3 = SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger")

    # Get path to the TurtleBot3 Gazebo world launch file
    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, "launch", "turtlebot3_world.launch.py")
        )
    )

    # Your follower and visualizer nodes
    follower = Node(
        package="path_planner_update1",
        executable="follower_node",
        name="trajectory_follower",
        output="screen",
    )

    visualizer = Node(
        package="path_planner_update1",
        executable="visualizer_node",
        name="path_visualizer",
        output="screen",
    )

    return LaunchDescription([set_tb3, gazebo_launch, follower, visualizer])
