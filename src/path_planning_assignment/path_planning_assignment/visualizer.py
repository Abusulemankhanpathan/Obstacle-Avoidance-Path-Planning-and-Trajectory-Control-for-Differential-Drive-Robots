#!/usr/bin/env python3
"""
Enhanced Path Visualizer for RViz - YAML Parameter Support
Subscribes to /path_to_visualize (trajectory from follower) and /odom
All visualization parameters configurable via YAML file.
"""

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import math

class PathVisualizer(Node):
    def __init__(self):
        super().__init__("path_visualizer")

        # Declare parameters with defaults
        self.declare_parameter('trail_min_distance', 0.02)
        self.declare_parameter('planned_path_width', 0.07)
        self.declare_parameter('actual_path_width', 0.15)
        self.declare_parameter('waypoint_radius', 0.08)
        
        # Get parameters
        self.trail_min_distance = self.get_parameter('trail_min_distance').value
        self.planned_path_width = self.get_parameter('planned_path_width').value
        self.actual_path_width = self.get_parameter('actual_path_width').value
        self.waypoint_radius = self.get_parameter('waypoint_radius').value

        # Subscriber to receive trajectory information from follower_node
        self.create_subscription(
            MarkerArray, "/path_to_visualize", self.trajectory_callback, 10
        )

        # Subscriber to receive odometry data
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Publisher for combined visualization
        self.marker_pub = self.create_publisher(
            MarkerArray, "/path_visualization", 10
        )

        # Timer for publishing visualization
        self.timer = self.create_timer(0.05, self.publish_visualization)  # 20 Hz

        # Storage
        self.full_trajectory = []  # Full list of (x, y) waypoints
        self.current_waypoint_idx = 0  # Current waypoint index from follower
        self.actual_path_points = []  # Stores (x, y, z) for actual robot path
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.start_point = None
        self.end_point = None

        self.get_logger().info(
            "PathVisualizer started with YAML parameters 🎯\n"
            f"- Trail min distance: {self.trail_min_distance}m\n"
            f"- Planned path width: {self.planned_path_width}m\n"
            f"- Actual path width: {self.actual_path_width}m\n"
            f"- Waypoint radius: {self.waypoint_radius}m\n"
            "- Add '/path_visualization' MarkerArray in RViz\n"
            "- Fixed frame: 'odom'"
        )

    def trajectory_callback(self, msg: MarkerArray):
        """Extract trajectory information from follower node markers."""
        if not msg.markers:
            self.get_logger().warn("No markers in trajectory message.")
            return

        for marker in msg.markers:
            if marker.ns == "trajectory" and len(marker.points) > 0:
                # Extract all waypoints from the trajectory line
                if not self.full_trajectory:
                    self.full_trajectory = [(pt.x, pt.y) for pt in marker.points]
                    if self.full_trajectory:
                        self.start_point = self.full_trajectory[0]
                        self.end_point = self.full_trajectory[-1]
            
            elif marker.ns == "waypoints":
                # Count waypoints to determine current index
                total_waypoints_in_msg = sum(1 for m in msg.markers if m.ns == "waypoints")
                if self.full_trajectory:
                    self.current_waypoint_idx = len(self.full_trajectory) - total_waypoints_in_msg

    def odom_callback(self, msg: Odometry):
        """Update robot position and build actual path trail."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = yaw
        
        # Add to actual path trail
        if not self.actual_path_points or \
           math.hypot(x - self.actual_path_points[-1][0], 
                     y - self.actual_path_points[-1][1]) > self.trail_min_distance:
            self.actual_path_points.append((x, y, 0.03))

    def publish_visualization(self):
        """Publish comprehensive visualization."""
        ma = MarkerArray()

        # === 1. PLANNED PATH (Cyan, thin, semi-transparent) ===
        if len(self.full_trajectory) > 1:
            planned_path = Marker()
            planned_path.header.frame_id = "odom"
            planned_path.header.stamp = self.get_clock().now().to_msg()
            planned_path.ns = "planned_path"
            planned_path.id = 1000
            planned_path.type = Marker.LINE_STRIP
            planned_path.action = Marker.ADD
            planned_path.scale.x = self.planned_path_width
            planned_path.pose.orientation.w = 1.0
            
            # Cyan with moderate transparency
            planned_path.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.5)
            
            for (x, y) in self.full_trajectory:
                pt = Point()
                pt.x = float(x)
                pt.y = float(y)
                pt.z = 0.02
                planned_path.points.append(pt)
            
            ma.markers.append(planned_path)

        # === 2. ACTUAL PATH (Dark purple, thick, fully opaque) ===
        if len(self.actual_path_points) > 1:
            actual_path = Marker()
            actual_path.header.frame_id = "odom"
            actual_path.header.stamp = self.get_clock().now().to_msg()
            actual_path.ns = "actual_path"
            actual_path.id = 2000
            actual_path.type = Marker.LINE_STRIP
            actual_path.action = Marker.ADD
            actual_path.scale.x = self.actual_path_width
            actual_path.pose.orientation.w = 1.0
            
            # Dark purple, fully opaque - DOMINANT
            actual_path.color = ColorRGBA(r=0.4, g=0.0, b=0.6, a=1.0)
            
            for (x, y, z) in self.actual_path_points:
                pt = Point()
                pt.x = float(x)
                pt.y = float(y)
                pt.z = float(z)
                actual_path.points.append(pt)
            
            ma.markers.append(actual_path)

        # === 3. START POINT (Green sphere) ===
        if self.start_point:
            start_marker = Marker()
            start_marker.header.frame_id = "odom"
            start_marker.header.stamp = self.get_clock().now().to_msg()
            start_marker.ns = "start_point"
            start_marker.id = 3000
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.scale.x = 0.2
            start_marker.scale.y = 0.2
            start_marker.scale.z = 0.2
            
            # Bright green
            start_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            
            start_marker.pose.position.x = float(self.start_point[0])
            start_marker.pose.position.y = float(self.start_point[1])
            start_marker.pose.position.z = 0.1
            start_marker.pose.orientation.w = 1.0
            
            ma.markers.append(start_marker)

        # === 4. END POINT (Red sphere) ===
        if self.end_point:
            end_marker = Marker()
            end_marker.header.frame_id = "odom"
            end_marker.header.stamp = self.get_clock().now().to_msg()
            end_marker.ns = "end_point"
            end_marker.id = 3001
            end_marker.type = Marker.SPHERE
            end_marker.action = Marker.ADD
            end_marker.scale.x = 0.2
            end_marker.scale.y = 0.2
            end_marker.scale.z = 0.2
            
            # Red color for end point
            end_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            
            end_marker.pose.position.x = float(self.end_point[0])
            end_marker.pose.position.y = float(self.end_point[1])
            end_marker.pose.position.z = 0.1
            end_marker.pose.orientation.w = 1.0
            
            ma.markers.append(end_marker)

        # === 5. WAYPOINTS ===
        for i, (x, y) in enumerate(self.full_trajectory):
            waypoint = Marker()
            waypoint.header.frame_id = "odom"
            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.ns = "waypoints"
            waypoint.id = 4000 + i
            waypoint.type = Marker.SPHERE
            waypoint.action = Marker.ADD
            waypoint.scale.x = self.waypoint_radius
            waypoint.scale.y = self.waypoint_radius
            waypoint.scale.z = self.waypoint_radius

            # Determine color based on waypoint status
            if i < self.current_waypoint_idx:
                waypoint.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)  # Passed (Green)
            elif i == self.current_waypoint_idx:
                waypoint.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Current (Orange)
            else:
                waypoint.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)  # Upcoming (Yellow)

            waypoint.pose.position.x = float(x)
            waypoint.pose.position.y = float(y)
            waypoint.pose.position.z = 0.04
            waypoint.pose.orientation.w = 1.0
            
            ma.markers.append(waypoint)

        # Publish markers
        self.marker_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()








