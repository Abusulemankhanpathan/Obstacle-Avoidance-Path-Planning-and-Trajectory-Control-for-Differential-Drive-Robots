#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set TurtleBot3 model
    set_tb3 = SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger")

    # Gazebo launch
    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, "launch", "turtlebot3_world.launch.py")
        )
    )

    # RViz node with exact path
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", "/root/ros2_ws/src/path_planning_assignment/path_planning_assignment/rviz/path_planner_config.rviz"],
        output="screen",
    )

    # Follower node
    follower_node = Node(
        package="path_planning_assignment",
        executable="follower_node",
        name="trajectory_follower",
        output="screen",
    )

    # Visualizer node
    visualizer_node = Node(
        package="path_planning_assignment",
        executable="visualizer",
        name="path_visualizer",
        output="screen",
    )

    # Delay follower + visualizer to ensure Gazebo is ready
    delayed_nodes = TimerAction(
        period=15.0,
        actions=[follower_node, visualizer_node]
    )

    return LaunchDescription([set_tb3, gazebo_launch, rviz_node, delayed_nodes])
