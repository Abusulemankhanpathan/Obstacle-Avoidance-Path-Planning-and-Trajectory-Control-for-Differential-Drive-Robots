#!/usr/bin/env python3
"""
Trajectory follower with improved narrow corridor handling:
- Dynamic trajectory regeneration
- Dynamic waypoint skipping
- Smooth obstacle avoidance with scaled speed
- Controlled recovery and wiggle logic
"""

import math
import time
from collections import deque
import statistics

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

from .controller import AdvancedController
from .path_smoothing import smooth_path
from .trajectory_generator import generate_trajectory


class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__("trajectory_follower")

        # Hard-coded waypoints
        self.waypoints = [
            [-2.0, -0.5],
            [-1.5, -0.5],
            [-1.0, -0.45],
            [-0.5, -0.48],
            [0.0, -0.5],
            [0.20, -0.5],
            [0.5, -0.5],
            [1.0, -0.45],
            [1.5, -0.48],
            [0.0, -0.5],
            [-0.5, -0.5],
            [-1.0,-0.5],
            [-1.5, -0.5]
        ]

        # Parameters
        self.declare_parameter("v_max", 0.22)
        self.declare_parameter("a_max", 0.6)
        self.declare_parameter("lookahead", 0.6)
        self.declare_parameter("control_rate", 15.0)
        self.declare_parameter("obstacle_distance_threshold", 0.25)  # increased for narrow corridor

        self.v_max = float(self.get_parameter("v_max").value)
        self.a_max = float(self.get_parameter("a_max").value)
        self.lookahead = float(self.get_parameter("lookahead").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.obstacle_threshold = float(self.get_parameter("obstacle_distance_threshold").value)

        # Recovery / wiggle parameters
        self.stuck_timeout = 2.5
        self.recovery_max_time = 2.0
        self.recovery_total_max_time = 6.0
        self.min_forward_when_blocked = 0.05
        self.avoid_gain = 0.35
        self.hard_reverse_time = 0.5
        self.log_debug_period = 0.8

        # Frontal sector
        self.frontal_half_angle = math.radians(20)  # narrower front
        self.scan_buffer_len = 6
        self.obstacle_enter_mult = 0.95
        self.obstacle_exit_mult = 1.12

        # Initialize trajectory
        self.regenerate_trajectory(initial=True)

        # Controller
        self.controller = AdvancedController(lookahead=self.lookahead, v_max=self.v_max)

        # ROS publishers/subscribers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)

        # State
        self.pose = (0.0, 0.0, 0.0)
        self.obs_min = float("inf")
        self.obs_angle = 0.0
        self.frontal_min_buffer = deque(maxlen=self.scan_buffer_len)

        # Recovery
        self.last_nonzero_v_time = time.time()
        self.in_recovery = False
        self.recovery_start_time = None
        self.recovery_total_start = None
        self.wiggle_phase = 0
        self.wiggle_start_time = None
        self.in_hard_escape = False
        self._was_blocked = False
        self._just_regen = False

        # Dynamic waypoint skipping
        self.current_waypoint_idx = 0
        self.waypoint_skip_distance = 0.15  # meters

        # Timer
        self.timer = self.create_timer(1.0 / max(1.0, self.control_rate), self.control_loop)
        self.last_log_time = time.time()
        self.get_logger().info("TrajectoryFollower initialized with narrow-corridor adjustments.")

    # --- ROS callbacks ---
    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.pose = (p.x, p.y, yaw)

    def scan_cb(self, msg: LaserScan):
        ranges = msg.ranges
        angle = msg.angle_min
        frontal = []
        global_min = float('inf')
        global_min_angle = 0.0
        for r in ranges:
            if r is None or math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue
            if r < global_min:
                global_min = r
                global_min_angle = angle
            if abs(angle) <= self.frontal_half_angle:
                frontal.append((angle, r))
            angle += msg.angle_increment

        if frontal:
            min_ang, min_r = min(frontal, key=lambda x: x[1])
            total_weight = sum(max(0.001, rng) for _, rng in frontal)
            weighted_angle = sum(ang * max(0.001, rng) for ang, rng in frontal)
            avg_angle = weighted_angle / total_weight if total_weight > 0 else min_ang
            self.obs_min = float(min_r)
            self.obs_angle = float(avg_angle)
            self.frontal_min_buffer.append(self.obs_min)
        else:
            self.obs_min = float(global_min)
            self.obs_angle = float(global_min_angle)
            self.frontal_min_buffer.append(self.obs_min)

    # --- Recovery helpers ---
    def enter_recovery(self):
        if not self.in_recovery:
            self.in_recovery = True
            self.recovery_start_time = time.time()
            self.recovery_total_start = self.recovery_total_start or self.recovery_start_time
            self._just_regen = True
            self.get_logger().warning("ENTER recovery mode.")

    def exit_recovery(self):
        if self.in_recovery:
            self.in_recovery = False
            self.recovery_start_time = None
            self.wiggle_phase = 0
            self.wiggle_start_time = None
            self.in_hard_escape = False
            self.get_logger().info("EXIT recovery mode.")

    def start_wiggle(self):
        self.wiggle_phase = 1
        self.wiggle_start_time = time.time()
        self.recovery_total_start = self.recovery_total_start or self.wiggle_start_time
        self.get_logger().info("WIGGLE start.")

    # --- Dynamic trajectory regeneration ---
    def regenerate_trajectory(self, initial=False):
        if initial:
            path = [[x, y] for x, y in self.waypoints]
        else:
            remaining_waypoints = self.waypoints[self.current_waypoint_idx:]
            path = [[self.pose[0], self.pose[1]]] + remaining_waypoints
        path_k = smooth_path(path, smoothness=0.35, num_points=350)
        self.traj = generate_trajectory(path_k, v_max=self.v_max, a_max=self.a_max)
        self.get_logger().info("Trajectory regenerated dynamically.")

    # --- Waypoint skipping ---
    def update_current_waypoint(self):
        while self.current_waypoint_idx < len(self.waypoints):
            wp = self.waypoints[self.current_waypoint_idx]
            dist = math.hypot(self.pose[0] - wp[0], self.pose[1] - wp[1])
            if dist < self.waypoint_skip_distance:
                self.current_waypoint_idx += 1
            else:
                break

    # --- Main control loop ---
    def control_loop(self):
        self.update_current_waypoint()
        now = time.time()
        v_ctrl, w_ctrl, status = self.controller.step(self.pose, self.traj, now)

        # Smooth frontal min
        smooth_min = float(statistics.median(self.frontal_min_buffer)) if self.frontal_min_buffer else self.obs_min

        # Scale velocity based on distance (0.05 m → threshold)
        scale = min(1.0, max(0.0, (smooth_min - 0.05)/(self.obstacle_threshold - 0.05)))
        v = v_ctrl * scale + self.min_forward_when_blocked * (1.0 - scale)
        w = w_ctrl * scale + (-math.copysign(self.avoid_gain, self.obs_angle) * (1.0 - scale) if smooth_min < self.obstacle_threshold else 0.0)

        # Track last nonzero forward
        if abs(v) > 0.03:
            self.last_nonzero_v_time = now
            self.recovery_total_start = None

        # Stuck detection
        blocked_duration = now - self.last_nonzero_v_time
        if blocked_duration > self.stuck_timeout and not self.in_recovery:
            self.enter_recovery()

        # Recovery / Wiggle (gentler)
        if self.in_recovery:
            elapsed = now - (self.recovery_start_time or now)
            w_rec = -math.copysign(0.6, self.obs_angle) if math.isfinite(self.obs_min) and self.obs_min < self.obstacle_threshold else 0.3
            v = 0.0
            w = w_rec
            if elapsed >= self.recovery_max_time:
                self.exit_recovery()
                self.start_wiggle()

        # Publish
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

        # Logging
        if now - self.last_log_time > self.log_debug_period:
            self.get_logger().info(
                f"[DEBUG] pose=({self.pose[0]:.3f},{self.pose[1]:.3f},{self.pose[2]:.3f}) "
                f"v={v:.3f} w={w:.3f} status={status} obs={self.obs_min:.3f} "
                f"in_recovery={self.in_recovery} wiggle={self.wiggle_phase} waypoint_idx={self.current_waypoint_idx}"
            )
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TrajectoryFollower.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
