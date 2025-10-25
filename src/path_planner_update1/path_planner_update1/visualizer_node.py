#!/usr/bin/env python3
"""
Publishes MarkerArray for RViz: trajectory line, start/goal, and odom trail.
Topic: /path_visualization (MarkerArray)
"""

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# import helper modules
from .path_smoothing import smooth_path
from .trajectory_generator import generate_trajectory


class PathVisualizer(Node):
    def __init__(self):
        super().__init__("path_visualizer")

        # ✅ Hard-coded waypoints (no parameter type issues)
        # Replace these with pillar 1 → 9 positions as per your map
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
                  [-1.5, -0.5]     # Return down to bottom-center
        ]

        # Other tunable parameters
        smoothness = 0.35
        num_points = 350
        v_max = 0.22
        a_max = 0.6

        # Generate smooth path and trajectory
        self.get_logger().info("Generating smoothed trajectory...")
        path_k = smooth_path(
            self.waypoints, smoothness=smoothness, num_points=num_points
        )
        self.traj = generate_trajectory(path_k, v_max=v_max, a_max=a_max)

        if self.traj:
            self.get_logger().info(
                f"Trajectory generated with {len(self.traj)} points."
            )
        else:
            self.get_logger().warn("Empty trajectory generated!")

        # Publisher and subscriber
        self.marker_pub = self.create_publisher(MarkerArray, "/path_visualization", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.odom_trail = []

        # Timer to publish markers periodically
        self.timer = self.create_timer(0.5, self.publish_markers)
        self.get_logger().info(
            "PathVisualizer started. Open RViz and add '/path_visualization' MarkerArray."
        )

    # --- Odometry callback ---
    def odom_cb(self, msg: Odometry):
        """Collects odometry trail points."""
        p = msg.pose.pose.position
        self.odom_trail.append((p.x, p.y))
        if len(self.odom_trail) > 300:
            self.odom_trail.pop(0)

    # --- Marker helper ---
    def make_marker(self, ns, idn, mtype, pts, color_rgba, scale=(0.02,)):
        """Helper to create a marker."""
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = idn
        m.type = mtype
        m.action = Marker.ADD
        m.scale.x = scale[0]
        m.scale.y = scale[0] if len(scale) == 1 else scale[1]
        m.scale.z = 0.02
        m.color = ColorRGBA(
            r=float(color_rgba[0]),
            g=float(color_rgba[1]),
            b=float(color_rgba[2]),
            a=float(color_rgba[3]),
        )
        m.pose.orientation.w = 1.0

        # Add points
        for x, y, z in pts:
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = float(z)
            m.points.append(pt)
        return m

    # --- Publish visualization markers ---
    def publish_markers(self):
        """Publishes the trajectory line, start/goal points, and odometry
        trail."""
        ma = MarkerArray()

        # Trajectory line
        if self.traj:
            pts = [(pt["x"], pt["y"], 0.02) for pt in self.traj]
            ma.markers.append(
                self.make_marker(
                    "trajectory",
                    0,
                    Marker.LINE_STRIP,
                    pts,
                    (0.0, 1.0, 1.0, 0.9),  # cyan
                    (0.02,),
                )
            )

            # Start (green) and Goal (red)
            start = self.traj[0]
            goal = self.traj[-1]
            ma.markers.append(
                self.make_marker(
                    "points",
                    1,
                    Marker.SPHERE,
                    [(start["x"], start["y"], 0.05)],
                    (0, 1, 0, 0.9),
                    (0.12,),
                )
            )
            ma.markers.append(
                self.make_marker(
                    "points",
                    2,
                    Marker.SPHERE,
                    [(goal["x"], goal["y"], 0.05)],
                    (1, 0, 0, 0.9),
                    (0.12,),
                )
            )

        # Odometry trail
        if self.odom_trail:
            trail_pts = [(x, y, 0.01) for x, y in self.odom_trail]
            ma.markers.append(
                self.make_marker(
                    "trail",
                    10,
                    Marker.LINE_STRIP,
                    trail_pts,
                    (0.0, 0.0, 1.0, 0.8),  # blue
                    (0.02,),
                )
            )

        # Publish to RViz
        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PathVisualizer.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
