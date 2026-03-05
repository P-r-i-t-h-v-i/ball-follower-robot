#!/usr/bin/env python3
"""
Path Tracker Node — Records and visualizes the robot's traveled path

Subscribes to /odom and accumulates the robot positions into a nav_msgs/Path
for RViz visualization. The path shows everywhere the robot has been.

Subscribes:
    /odom (nav_msgs/Odometry)

Publishes:
    /path (nav_msgs/Path) — accumulated path for RViz
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math


class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')

        # ---------- Parameters ----------
        self.declare_parameter('min_distance', 0.02)     # meters - min movement to record
        self.declare_parameter('publish_rate', 2.0)      # Hz
        self.declare_parameter('max_path_points', 5000)   # limit memory usage

        self.min_distance = self.get_parameter('min_distance').value
        publish_rate = self.get_parameter('publish_rate').value
        self.max_points = self.get_parameter('max_path_points').value

        # ---------- Subscriber ----------
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 50)

        # ---------- Publisher ----------
        self.path_pub = self.create_publisher(Path, '/path', 10)

        # ---------- State ----------
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.last_x = None
        self.last_y = None

        # ---------- Timer ----------
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_path)

        self.get_logger().info('Path tracker started')

    def odom_callback(self, msg: Odometry):
        """Add position to path if robot has moved enough."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Check if we've moved enough to record a new point
        if self.last_x is not None:
            dist = math.sqrt((x - self.last_x) ** 2 + (y - self.last_y) ** 2)
            if dist < self.min_distance:
                return

        self.last_x = x
        self.last_y = y

        # Create pose stamped
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'odom'
        pose.pose = msg.pose.pose

        self.path.poses.append(pose)

        # Limit path length
        if len(self.path.poses) > self.max_points:
            self.path.poses = self.path.poses[-self.max_points:]

        self.get_logger().debug(
            f'Path: {len(self.path.poses)} points, '
            f'pos=({x:.3f}, {y:.3f})')

    def publish_path(self):
        """Publish the accumulated path."""
        if self.path.poses:
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
