#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    print("\n" + "="*60)
    print("LAUNCH FILE DEBUG INFO")
    print("="*60)
    
    # Set TurtleBot3 model
    set_tb3 = SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger")
    
    # Try to get package directory with error handling
    try:
        pkg_dir = get_package_share_directory("path_planning_assignment")
        print(f"✓ Package found at: {pkg_dir}")
    except Exception as e:
        print(f"✗ ERROR: Cannot find package 'path_planning_assignment'")
        print(f"  Error: {e}")
        print("\nTroubleshooting:")
        print("  1. Check if package is built: colcon build --packages-select path_planning_assignment")
        print("  2. Source workspace: source install/setup.bash")
        print("  3. Verify package: ros2 pkg list | grep path_planning")
        print("="*60 + "\n")
        raise
    
    # Path to config file
    config_file = os.path.join(pkg_dir, "config", "controller_params.yaml")
    
    # Check if config file exists
    if os.path.exists(config_file):
        print(f"✓ Config file found: {config_file}")
    else:
        print(f"✗ WARNING: Config file not found: {config_file}")
        print("  Will use default parameters from nodes")
    
    # Gazebo launch
    try:
        tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
        print(f"✓ TurtleBot3 Gazebo found at: {tb3_gazebo_dir}")
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_dir, "launch", "turtlebot3_world.launch.py")
            )
        )
    except Exception as e:
        print(f"✗ ERROR: Cannot find turtlebot3_gazebo package: {e}")
        print("  Install with: sudo apt install ros-humble-turtlebot3-gazebo")
        raise
    
    # RViz config path
    rviz_config = os.path.join(pkg_dir, "rviz", "path_planner_config.rviz")
    if os.path.exists(rviz_config):
        print(f"✓ RViz config found: {rviz_config}")
    else:
        print(f"✗ WARNING: RViz config not found: {rviz_config}")
        rviz_config = ""  # Launch RViz without config
    
    # RViz node
    if rviz_config:
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )
    else:
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        )
    
    # Follower node with YAML parameters
    if os.path.exists(config_file):
        follower_node = Node(
            package="path_planning_assignment",
            executable="follower_node",
            name="trajectory_follower",
            output="screen",
            parameters=[config_file]
        )
        print("✓ Follower node will use config file")
    else:
        follower_node = Node(
            package="path_planning_assignment",
            executable="follower_node",
            name="trajectory_follower",
            output="screen"
        )
        print("✓ Follower node will use default parameters")
    
    # Visualizer node with YAML parameters
    if os.path.exists(config_file):
        visualizer_node = Node(
            package="path_planning_assignment",
            executable="visualizer",
            name="path_visualizer",
            output="screen",
            parameters=[config_file]
        )
        print("✓ Visualizer node will use config file")
    else:
        visualizer_node = Node(
            package="path_planning_assignment",
            executable="visualizer",
            name="path_visualizer",
            output="screen"
        )
        print("✓ Visualizer node will use default parameters")
    
    print("="*60)
    print("Starting launch in 2 seconds...")
    print("="*60 + "\n")
    
    # Delay follower + visualizer to ensure Gazebo is ready
    delayed_nodes = TimerAction(
        period=15.0,
        actions=[follower_node, visualizer_node]
    )
    
    return LaunchDescription([
        set_tb3, 
        gazebo_launch, 
        rviz_node, 
        delayed_nodes
    ])
