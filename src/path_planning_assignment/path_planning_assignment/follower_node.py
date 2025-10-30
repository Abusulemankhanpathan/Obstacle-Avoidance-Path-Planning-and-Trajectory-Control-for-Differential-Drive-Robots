#!/usr/bin/env python3
"""
Pure Pursuit Trajectory Follower for TurtleBot3 - YAML Parameter Support
Subscribes to /odom and follows a given trajectory.
Publishes cmd_vel commands and trajectory visualization.
All parameters configurable via YAML file.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')

        # Declare all parameters with defaults
        self.declare_parameter('lookahead_distance', 0.1)
        self.declare_parameter('max_linear_vel', 0.15)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('kp_ang', 2.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('waypoint_tolerance', 0.1)
        
        # Advanced controller parameters
        self.declare_parameter('use_advanced_controller', False)
        self.declare_parameter('advanced_lookahead', 0.6)
        self.declare_parameter('advanced_v_max', 0.22)
        self.declare_parameter('advanced_kp', 1.8)
        self.declare_parameter('advanced_ki', 0.0)
        self.declare_parameter('advanced_kd', 0.04)
        self.declare_parameter('advanced_max_w', 1.2)
        self.declare_parameter('advanced_alpha', 0.45)
        self.declare_parameter('advanced_max_dv_per_sec', 0.6)
        
        # Path smoothing parameters
        self.declare_parameter('smoothness', 0.35)
        self.declare_parameter('num_smooth_points', 400)
        
        # Waypoints as flattened list [x1, y1, x2, y2, ...]
        self.declare_parameter('waypoints_flat', [
            -2.0, -0.5,
            -1.5, -0.5,
            -1.0, -0.45,
            -0.5, -0.48,
            0.0, -0.5,
            0.5, -0.5,
            1.0, -0.45,
            1.5, -0.48,
            2.0, -0.5,
            1.5, 0.0,
            2.0, 0.5,
            1.0, 0.5,
            0.0, 0.5,
            -0.5, 0.48,
            -1.0, 0.45,
            -1.5, 0.5,
            -2.0, 0.5,
            -2.0, 0.0,
            -2.0, -0.5
        ])

        # Get parameters
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.kp_ang = self.get_parameter('kp_ang').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        
        self.use_advanced_controller = self.get_parameter('use_advanced_controller').value
        self.smoothness = self.get_parameter('smoothness').value
        self.num_smooth_points = self.get_parameter('num_smooth_points').value

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Load trajectory from flattened YAML parameters
        waypoints_flat = self.get_parameter('waypoints_flat').value
        self.trajectory = []
        
        # Convert flattened list to list of tuples
        for i in range(0, len(waypoints_flat), 2):
            if i + 1 < len(waypoints_flat):
                self.trajectory.append((float(waypoints_flat[i]), float(waypoints_flat[i+1])))
        
        # Current waypoint index
        self.current_waypoint_idx = 0
        self.goal_reached = False
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_vis_pub = self.create_publisher(MarkerArray, '/path_to_visualize', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        # Timer for visualization publishing
        self.vis_timer = self.create_timer(0.1, self.publish_trajectory_visualization)  # 10 Hz

        self.get_logger().info(
            f"TrajectoryFollower initialized with YAML parameters\n"
            f"- Waypoints: {len(self.trajectory)}\n"
            f"- Lookahead: {self.lookahead_distance}m\n"
            f"- Max Linear Vel: {self.max_linear_vel} m/s\n"
            f"- Max Angular Vel: {self.max_angular_vel} rad/s\n"
            f"- Kp Angular: {self.kp_ang}\n"
            f"- Goal Tolerance: {self.goal_tolerance}m\n"
            f"- Waypoint Tolerance: {self.waypoint_tolerance}m\n"
            f"- Use Advanced Controller: {self.use_advanced_controller}"
        )

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def publish_trajectory_visualization(self):
        """Publish the planned trajectory for visualization."""
        ma = MarkerArray()
        
        # 1. Trajectory line (remaining path)
        if self.current_waypoint_idx < len(self.trajectory):
            traj_marker = Marker()
            traj_marker.header.frame_id = "odom"
            traj_marker.header.stamp = self.get_clock().now().to_msg()
            traj_marker.ns = "trajectory"
            traj_marker.id = 0
            traj_marker.type = Marker.LINE_STRIP
            traj_marker.action = Marker.ADD
            traj_marker.scale.x = 0.03
            traj_marker.pose.orientation.w = 1.0
            
            # Semi-transparent white for planned path
            traj_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.5)
            
            # Add remaining waypoints
            for i in range(self.current_waypoint_idx, len(self.trajectory)):
                pt = Point()
                pt.x = float(self.trajectory[i][0])
                pt.y = float(self.trajectory[i][1])
                pt.z = 0.05
                traj_marker.points.append(pt)
            
            ma.markers.append(traj_marker)
        
        # 2. Waypoint markers (small spheres for remaining waypoints)
        for i in range(self.current_waypoint_idx, len(self.trajectory)):
            wp_marker = Marker()
            wp_marker.header.frame_id = "odom"
            wp_marker.header.stamp = self.get_clock().now().to_msg()
            wp_marker.ns = "waypoints"
            wp_marker.id = i
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            
            # Smaller size for waypoints
            wp_marker.scale.x = 0.06
            wp_marker.scale.y = 0.06
            wp_marker.scale.z = 0.06
            
            # Light blue for waypoints
            if i == self.current_waypoint_idx:
                # Current target - brighter
                wp_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8)
            else:
                # Future waypoints - dimmer
                wp_marker.color = ColorRGBA(r=0.5, g=0.8, b=1.0, a=0.4)
            
            wp_marker.pose.position.x = float(self.trajectory[i][0])
            wp_marker.pose.position.y = float(self.trajectory[i][1])
            wp_marker.pose.position.z = 0.05
            wp_marker.pose.orientation.w = 1.0
            
            ma.markers.append(wp_marker)
        
        # 3. Current target indicator
        if self.current_waypoint_idx < len(self.trajectory):
            target = self.trajectory[self.current_waypoint_idx]
            target_marker = Marker()
            target_marker.header.frame_id = "odom"
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = "current_target"
            target_marker.id = 1000
            target_marker.type = Marker.CYLINDER
            target_marker.action = Marker.ADD
            target_marker.scale.x = 0.15
            target_marker.scale.y = 0.15
            target_marker.scale.z = 0.01
            target_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.5)
            target_marker.pose.position.x = float(target[0])
            target_marker.pose.position.y = float(target[1])
            target_marker.pose.position.z = 0.01
            target_marker.pose.orientation.w = 1.0
            ma.markers.append(target_marker)
        
        # 4. Goal marker (final waypoint)
        if len(self.trajectory) > 0:
            goal = self.trajectory[-1]
            goal_marker = Marker()
            goal_marker.header.frame_id = "odom"
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = "goal"
            goal_marker.id = 2000
            goal_marker.type = Marker.CYLINDER
            goal_marker.action = Marker.ADD
            goal_marker.scale.x = 0.25
            goal_marker.scale.y = 0.25
            goal_marker.scale.z = 0.02
            
            if self.goal_reached:
                goal_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)  # Green when reached
            else:
                goal_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)  # Orange when approaching
            
            goal_marker.pose.position.x = float(goal[0])
            goal_marker.pose.position.y = float(goal[1])
            goal_marker.pose.position.z = 0.01
            goal_marker.pose.orientation.w = 1.0
            ma.markers.append(goal_marker)
        
        # Publish
        self.path_vis_pub.publish(ma)

    def control_loop(self):
        if self.goal_reached:
            # Stop the robot
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        if not self.trajectory:
            return

        # Update current waypoint if close enough
        if self.current_waypoint_idx < len(self.trajectory):
            current_wp = self.trajectory[self.current_waypoint_idx]
            dist_to_wp = math.hypot(current_wp[0] - self.x, current_wp[1] - self.y)
            
            if dist_to_wp < self.waypoint_tolerance:
                self.current_waypoint_idx += 1
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx}/{len(self.trajectory)}")
                
                # Check if reached final goal
                if self.current_waypoint_idx >= len(self.trajectory):
                    self.goal_reached = True
                    self.get_logger().info("Goal reached! Stopping robot.")
                    cmd = Twist()
                    self.cmd_pub.publish(cmd)
                    return

        # Find lookahead point from remaining waypoints
        lookahead_point = None
        for i in range(self.current_waypoint_idx, len(self.trajectory)):
            px, py = self.trajectory[i]
            dist = math.hypot(px - self.x, py - self.y)
            
            if dist >= self.lookahead_distance:
                lookahead_point = (px, py)
                break
    
        if lookahead_point is None:
            lookahead_point = self.trajectory[-1]

        # Compute heading error and trajectory adjustments
        dx = lookahead_point[0] - self.x
        dy = lookahead_point[1] - self.y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)

        # Normalize heading error to [-pi, pi]
        heading_error = target_angle - self.yaw
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Control law - reduce speed when large heading error
        if abs(heading_error) > math.pi / 4:  # 45 degrees
            linear_vel = 0.05  # Slow down for sharp turns
        else:
            linear_vel = min(self.max_linear_vel, distance * 0.5)
        
        angular_vel = self.kp_ang * heading_error
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))

        # Publish cmd_vel
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_pub.publish(cmd)

        # Logging
        self.get_logger().info(
            f"WP:{self.current_waypoint_idx+1}/{len(self.trajectory)} | "
            f"Pos:({self.x:.2f},{self.y:.2f}) | "
            f"Goal:({lookahead_point[0]:.2f},{lookahead_point[1]:.2f}) | "
            f"Dist:{distance:.2f} | HeadErr:{math.degrees(heading_error):.1f}° | "
            f"V:{linear_vel:.2f} W:{angular_vel:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






