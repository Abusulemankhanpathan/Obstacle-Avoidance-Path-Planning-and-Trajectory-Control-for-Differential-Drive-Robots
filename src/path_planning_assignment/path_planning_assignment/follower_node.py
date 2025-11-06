#!/usr/bin/env python3
"""
Integrated Trajectory Follower for TurtleBot3 (ROS 2)
- YAML-configurable
- Two modes:
    1) Pure Pursuit (default)
    2) Advanced Controller (curvature feedforward + PID) with smoothed, time-parameterized trajectory
- Publishes visualization markers compatible with visualizer.py
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# Project modules
from controller import AdvancedController                 # controller.py
from path_smoothing import smooth_path                    # path_smoothing.py
from trajectory_generator import generate_trajectory      # trajectory_generator.py


class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')

        # --------------------
        # Parameters (YAML)
        # --------------------
        # Pure Pursuit / common
        self.declare_parameter('lookahead_distance', 0.1)
        self.declare_parameter('max_linear_vel', 0.15)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('kp_ang', 2.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('waypoint_tolerance', 0.1)

        # Advanced controller toggle & tuning
        self.declare_parameter('use_advanced_controller', False)
        self.declare_parameter('advanced_lookahead', 0.6)
        self.declare_parameter('advanced_v_max', 0.22)
        self.declare_parameter('advanced_kp', 1.8)
        self.declare_parameter('advanced_ki', 0.0)
        self.declare_parameter('advanced_kd', 0.04)
        self.declare_parameter('advanced_max_w', 1.2)
        self.declare_parameter('advanced_alpha', 0.45)
        self.declare_parameter('advanced_max_dv_per_sec', 0.6)

        # Path smoothing & sampling
        self.declare_parameter('smoothness', 0.35)
        self.declare_parameter('num_smooth_points', 400)

        # Waypoints in two formats: prefer 'waypoints' (pairs); fallback to 'waypoints_flat'
        self.declare_parameter('waypoints', [])
        self.declare_parameter('waypoints_flat', [])

        # --------------------
        # Read parameters
        # --------------------
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.kp_ang = float(self.get_parameter('kp_ang').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)

        self.use_advanced = bool(self.get_parameter('use_advanced_controller').value)
        self.smoothness = float(self.get_parameter('smoothness').value)
        self.num_smooth_points = int(self.get_parameter('num_smooth_points').value)

        # Advanced-only params
        self.adv_lookahead = float(self.get_parameter('advanced_lookahead').value)
        self.adv_v_max = float(self.get_parameter('advanced_v_max').value)
        self.adv_kp = float(self.get_parameter('advanced_kp').value)
        self.adv_ki = float(self.get_parameter('advanced_ki').value)
        self.adv_kd = float(self.get_parameter('advanced_kd').value)
        self.adv_max_w = float(self.get_parameter('advanced_max_w').value)
        self.adv_alpha = float(self.get_parameter('advanced_alpha').value)
        self.adv_max_dv_per_sec = float(self.get_parameter('advanced_max_dv_per_sec').value)

        # --------------------
        # Robot state
        # --------------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # --------------------
        # Build trajectory(s)
        # --------------------
        # Prefer nested 'waypoints' pairs; fallback to 'waypoints_flat'
        pairs = self.get_parameter('waypoints').value or []
        self.raw_waypoints = []
        if pairs:
            for p in pairs:
                if isinstance(p, (list, tuple)) and len(p) == 2:
                    self.raw_waypoints.append((float(p[0]), float(p[1])))
        else:
            flat = self.get_parameter('waypoints_flat').value or []
            for i in range(0, len(flat), 2):
                if i + 1 < len(flat):
                    self.raw_waypoints.append((float(flat[i]), float(flat[i + 1])))

        # For visualization & progression
        self.vis_traj_xy = []   # list[(x,y)] remaining path the visualizer expects
        self.current_idx = 0
        self.goal_reached = False

        # Controller & trajectory storage
        self.adv_controller = None
        self.traj_for_control = []  # list of dicts for advanced OR pure pursuit

        if self.use_advanced:
            # Smooth → curvature → time-param trajectory
            smoothed = smooth_path(
                self.raw_waypoints,
                smoothness=self.smoothness,
                num_points=self.num_smooth_points
            )  # [(x,y,kappa_signed)]

            rich = generate_trajectory(
                smoothed,
                v_max=self.adv_v_max,
                a_max=self.adv_max_dv_per_sec  # reuse as accel budget
            )  # [{"x","y","kappa","s","v","t"}, ...]

            # Build controller trajectory (SIGNED kappa + v)
            self.traj_for_control = [
                {"x": float(x), "y": float(y), "kappa": float(kappa), "v": float(pt["v"])}
                for (x, y, kappa), pt in zip(smoothed, rich)
            ]

            # Visualization uses the same dense path (remaining points)
            self.vis_traj_xy = [(float(p["x"]), float(p["y"])) for p in rich]

            # Init controller
            self.adv_controller = AdvancedController(
                lookahead=self.adv_lookahead,
                v_max=self.adv_v_max,
                angular_pid=(self.adv_kp, self.adv_ki, self.adv_kd),
                max_w=self.adv_max_w,
                alpha=self.adv_alpha,
                max_dv_per_sec=self.adv_max_dv_per_sec
            )

        else:
            # Pure pursuit mode: use raw waypoints directly
            self.traj_for_control = [{"x": x, "y": y} for (x, y) in self.raw_waypoints]
            self.vis_traj_xy = list(self.raw_waypoints)

        # --------------------
        # ROS I/O
        # --------------------
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_vis_pub = self.create_publisher(MarkerArray, '/path_to_visualize', 10)

        # Timers
        self.create_timer(0.05, self.control_loop)        # 20 Hz
        self.create_timer(0.10, self.publish_markers)     # 10 Hz

        self.get_logger().info(
            "TrajectoryFollower initialized\n"
            f"- Mode: {'AdvancedController' if self.use_advanced else 'Pure Pursuit'}\n"
            f"- Waypoints loaded: {len(self.raw_waypoints)}\n"
            f"- Visualization points: {len(self.vis_traj_xy)}\n"
            f"- Lookahead: {self.adv_lookahead if self.use_advanced else self.lookahead_distance} m\n"
            f"- V_max: {self.adv_v_max if self.use_advanced else self.max_linear_vel} m/s\n"
        )

    # --------------------
    # Callbacks
    # --------------------
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    # --------------------
    # Visualization
    # --------------------
    def publish_markers(self):
        ma = MarkerArray()

        # 1) Trajectory line (remaining points)
        if self.current_idx < len(self.vis_traj_xy):
            traj = Marker()
            traj.header.frame_id = "odom"
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.ns = "trajectory"
            traj.id = 0
            traj.type = Marker.LINE_STRIP
            traj.action = Marker.ADD
            traj.scale.x = 0.03
            traj.pose.orientation.w = 1.0
            traj.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.5)  # semi-transparent white

            for i in range(self.current_idx, len(self.vis_traj_xy)):
                pxy = self.vis_traj_xy[i]
                p = Point()
                p.x = float(pxy[0]); p.y = float(pxy[1]); p.z = 0.05
                traj.points.append(p)

            ma.markers.append(traj)

        # 2) Waypoint spheres (remaining)
        for i in range(self.current_idx, len(self.vis_traj_xy)):
            wp = Marker()
            wp.header.frame_id = "odom"
            wp.header.stamp = self.get_clock().now().to_msg()
            wp.ns = "waypoints"
            wp.id = i
            wp.type = Marker.SPHERE
            wp.action = Marker.ADD
            wp.scale.x = 0.06; wp.scale.y = 0.06; wp.scale.z = 0.06
            wp.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8) if i == self.current_idx \
                       else ColorRGBA(r=0.5, g=0.8, b=1.0, a=0.4)
            wp.pose.position.x = float(self.vis_traj_xy[i][0])
            wp.pose.position.y = float(self.vis_traj_xy[i][1])
            wp.pose.position.z = 0.05
            wp.pose.orientation.w = 1.0
            ma.markers.append(wp)

        # 3) Current target indicator
        if self.current_idx < len(self.vis_traj_xy):
            tgt_xy = self.vis_traj_xy[self.current_idx]
            tgt = Marker()
            tgt.header.frame_id = "odom"
            tgt.header.stamp = self.get_clock().now().to_msg()
            tgt.ns = "current_target"
            tgt.id = 1000
            tgt.type = Marker.CYLINDER
            tgt.action = Marker.ADD
            tgt.scale.x = 0.15; tgt.scale.y = 0.15; tgt.scale.z = 0.01
            tgt.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.5)
            tgt.pose.position.x = float(tgt_xy[0])
            tgt.pose.position.y = float(tgt_xy[1])
            tgt.pose.position.z = 0.01
            tgt.pose.orientation.w = 1.0
            ma.markers.append(tgt)

        # 4) Goal marker (final)
        if len(self.vis_traj_xy) > 0:
            goal_xy = self.vis_traj_xy[-1]
            goal = Marker()
            goal.header.frame_id = "odom"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.ns = "goal"
            goal.id = 2000
            goal.type = Marker.CYLINDER
            goal.action = Marker.ADD
            goal.scale.x = 0.25; goal.scale.y = 0.25; goal.scale.z = 0.02
            goal.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7) if self.goal_reached \
                         else ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)
            goal.pose.position.x = float(goal_xy[0])
            goal.pose.position.y = float(goal_xy[1])
            goal.pose.position.z = 0.01
            goal.pose.orientation.w = 1.0
            ma.markers.append(goal)

        self.path_vis_pub.publish(ma)

    # --------------------
    # Control loop
    # --------------------
    def control_loop(self):
        if self.goal_reached or len(self.traj_for_control) == 0:
            self.cmd_pub.publish(Twist())
            return

        if self.use_advanced:
            # ----- Advanced controller path -----
            v, w, status = self.adv_controller.step(
                (self.x, self.y, self.yaw),
                self.traj_for_control,
                now=time.monotonic()
            )

            # Progress vis index by proximity
            if self.current_idx < len(self.vis_traj_xy):
                tx, ty = self.vis_traj_xy[self.current_idx]
                if math.hypot(tx - self.x, ty - self.y) < self.waypoint_tolerance:
                    self.current_idx += 1
                    self.get_logger().info(f"ADV reached {self.current_idx}/{len(self.vis_traj_xy)}")
                    if self.current_idx >= len(self.vis_traj_xy):
                        self.goal_reached = True
                        self.cmd_pub.publish(Twist())
                        self.get_logger().info("ADV goal reached. Stopping.")
                        return

            if status == "finished":
                self.goal_reached = True
                self.cmd_pub.publish(Twist())
                self.get_logger().info("ADV finished by distance threshold. Stopping.")
                return

            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                f"ADV | idx {self.current_idx}/{len(self.vis_traj_xy)} | "
                f"Pos({self.x:.2f},{self.y:.2f}) | V:{v:.2f} W:{w:.2f} | {status}"
            )

        else:
            # ----- Pure Pursuit path -----
            # Advance waypoint if close
            if self.current_idx < len(self.traj_for_control):
                cwp = self.traj_for_control[self.current_idx]
                if math.hypot(cwp["x"] - self.x, cwp["y"] - self.y) < self.waypoint_tolerance:
                    self.current_idx += 1
                    self.get_logger().info(f"PP reached {self.current_idx}/{len(self.traj_for_control)}")
                    if self.current_idx >= len(self.traj_for_control):
                        self.goal_reached = True
                        self.cmd_pub.publish(Twist())
                        self.get_logger().info("PP goal reached. Stopping.")
                        return

            # Find lookahead point
            lookahead_pt = None
            for i in range(self.current_idx, len(self.traj_for_control)):
                px, py = self.traj_for_control[i]["x"], self.traj_for_control[i]["y"]
                if math.hypot(px - self.x, py - self.y) >= self.lookahead_distance:
                    lookahead_pt = (px, py); break
            if lookahead_pt is None:
                last = self.traj_for_control[-1]
                lookahead_pt = (last["x"], last["y"])

            # Geometry
            dx = lookahead_pt[0] - self.x
            dy = lookahead_pt[1] - self.y
            distance = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)

            # Heading error in [-pi, pi]
            heading_error = target_angle - self.yaw
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi

            # Control law
            if abs(heading_error) > (math.pi / 4):
                linear_vel = 0.05
            else:
                linear_vel = min(self.max_linear_vel, distance * 0.5)

            angular_vel = self.kp_ang * heading_error
            angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))

            cmd = Twist()
            cmd.linear.x = float(linear_vel)
            cmd.angular.z = float(angular_vel)
            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                f"PP | idx {self.current_idx+1}/{len(self.traj_for_control)} | "
                f"Pos({self.x:.2f},{self.y:.2f}) | "
                f"Goal({lookahead_pt[0]:.2f},{lookahead_pt[1]:.2f}) | "
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







