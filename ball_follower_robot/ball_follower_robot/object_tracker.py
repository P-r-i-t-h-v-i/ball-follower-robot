#!/usr/bin/env python3
"""
Object Tracker Node — Yellow Ball Detection using OpenCV

Captures frames from USB webcam, detects yellow objects using HSV color
filtering, and publishes Twist commands to steer the robot toward the ball.

Publishes:
    /target_cmd (geometry_msgs/Twist) — forward speed + turn direction
    /camera/image_raw (sensor_msgs/Image) — annotated camera feed
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')

        # ---------- Parameters ----------
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        # HSV range for yellow (tune these for your ball & lighting)
        self.declare_parameter('h_low', 20)
        self.declare_parameter('s_low', 100)
        self.declare_parameter('v_low', 100)
        self.declare_parameter('h_high', 35)
        self.declare_parameter('s_high', 255)
        self.declare_parameter('v_high', 255)
        self.declare_parameter('min_radius', 20)        # px — ignore smaller detections
        self.declare_parameter('max_linear_speed', 0.5)  # m/s equivalent scale
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('target_radius', 120)     # px — "close enough" radius
        self.declare_parameter('show_preview', True)

        # ---------- Read parameters ----------
        cam_idx = self.get_parameter('camera_index').value
        w = self.get_parameter('frame_width').value
        h = self.get_parameter('frame_height').value
        self.h_low = self.get_parameter('h_low').value
        self.s_low = self.get_parameter('s_low').value
        self.v_low = self.get_parameter('v_low').value
        self.h_high = self.get_parameter('h_high').value
        self.s_high = self.get_parameter('s_high').value
        self.v_high = self.get_parameter('v_high').value
        self.min_radius = self.get_parameter('min_radius').value
        self.max_lin = self.get_parameter('max_linear_speed').value
        self.max_ang = self.get_parameter('max_angular_speed').value
        self.target_radius = self.get_parameter('target_radius').value
        self.show_preview = self.get_parameter('show_preview').value

        # ---------- Camera ----------
        self.cap = cv2.VideoCapture(cam_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera index {cam_idx}')
            raise RuntimeError('Camera not available')
        self.get_logger().info(f'Camera opened: index={cam_idx}, {w}x{h}')

        # ---------- Publishers ----------
        self.cmd_pub = self.create_publisher(Twist, '/target_cmd', 10)
        self.img_pub = self.create_publisher(Image, '/camera/image_raw', 5)
        self.bridge = CvBridge()

        # ---------- Timer (30 Hz) ----------
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab frame')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Build HSV mask for yellow
        lower = np.array([self.h_low, self.s_low, self.v_low])
        upper = np.array([self.h_high, self.s_high, self.v_high])
        mask = cv2.inRange(hsv, lower, upper)

        # Morphological cleanup
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()
        h_frame, w_frame = frame.shape[:2]
        center_x = w_frame // 2

        if contours:
            # Pick the largest contour
            largest = max(contours, key=cv2.contourArea)
            ((cx, cy), radius) = cv2.minEnclosingCircle(largest)

            if radius >= self.min_radius:
                cx, cy, radius = int(cx), int(cy), int(radius)

                # Draw detection on frame
                cv2.circle(frame, (cx, cy), radius, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(frame, f'r={radius}', (cx - 30, cy - radius - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if radius < self.target_radius:
                    # Scale: 1.0 when far, 0.0 when at target radius
                    forward_ratio = 1.0 - (radius / self.target_radius)
                    twist.linear.x = forward_ratio * self.max_lin
                    
                    # Compute steering only when moving toward ball
                    # Horizontal error: negative = ball is left, positive = right
                    error_x = (cx - center_x) / center_x  # normalized -1..1
                    twist.angular.z = -error_x * self.max_ang
                else:
                    twist.linear.x = 0.0  # close enough, stop forward
                    twist.angular.z = 0.0 # close enough, stop steering

                self.get_logger().debug(
                    f'Ball at ({cx},{cy}) r={radius} -> '
                    f'lin={twist.linear.x:.2f} ang={twist.angular.z:.2f}')
            else:
                # Ball too small / noise — do not move
                twist.angular.z = 0.0
        else:
            # No ball detected — do not move
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

        # Publish camera image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.img_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')

        # Optional preview window
        if self.show_preview:
            cv2.imshow('Ball Tracker', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
