#!/usr/bin/env python3
"""
Motor Controller Node — Serial Bridge to Arduino

Subscribes to /target_cmd (Twist) from the object tracker, converts to
differential drive speeds, sends motor commands to Arduino via serial,
and publishes basic odometry for path tracking.

Subscribes:
    /target_cmd (geometry_msgs/Twist)

Publishes:
    /odom (nav_msgs/Odometry)

Serial to Arduino:
    Sends: M,leftSpeed,rightSpeed\n   (values -255 to 255)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import serial
import math
import time


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # ---------- Parameters ----------
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_pwm', 200)          # max PWM value (0-255)
        self.declare_parameter('wheel_base', 0.15)      # meters between wheels
        self.declare_parameter('wheel_radius', 0.033)   # meters
        self.declare_parameter('max_speed', 0.5)        # m/s at max PWM
        self.declare_parameter('cmd_rate', 10.0)        # Hz for sending commands

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_speed = self.get_parameter('max_speed').value
        cmd_rate = self.get_parameter('cmd_rate').value

        # ---------- Serial Connection ----------
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2)  # Wait for Arduino reset
            self.get_logger().info(f'Serial connected: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial port {port}: {e}')
            self.get_logger().warn('Running in DRY-RUN mode (no Arduino)')

        # ---------- Subscriber ----------
        self.cmd_sub = self.create_subscription(
            Twist, '/target_cmd', self.cmd_callback, 10)

        # ---------- Odometry Publisher ----------
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---------- State ----------
        self.latest_twist = Twist()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # ---------- Timer to send commands at fixed rate ----------
        self.cmd_timer = self.create_timer(1.0 / cmd_rate, self.send_cmd)

    def cmd_callback(self, msg: Twist):
        """Store the latest velocity command."""
        self.latest_twist = msg

    def send_cmd(self):
        """Convert Twist to motor PWM and send to Arduino + publish odom."""
        linear = self.latest_twist.linear.x
        angular = self.latest_twist.angular.z

        # Differential drive kinematics
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)

        # Convert velocity to PWM (-255 to 255)
        pwm_left = int(self._vel_to_pwm(v_left))
        pwm_right = int(self._vel_to_pwm(v_right))

        # Clamp
        pwm_left = max(-self.max_pwm, min(self.max_pwm, pwm_left))
        pwm_right = max(-self.max_pwm, min(self.max_pwm, pwm_right))

        # Send serial command to Arduino
        cmd = f'M,{pwm_left},{pwm_right}\n'
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd.encode())
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')

        # Update odometry (dead reckoning)
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt > 0 and dt < 1.0:
            self._update_odom(linear, angular, dt)

    def _vel_to_pwm(self, velocity):
        """Map velocity (m/s) to PWM value."""
        if self.max_speed == 0:
            return 0.0
        return (velocity / self.max_speed) * self.max_pwm

    def _update_odom(self, linear, angular, dt):
        """Dead-reckoning odometry update."""
        # Update pose
        self.theta += angular * dt
        self.x += linear * math.cos(self.theta) * dt
        self.y += linear * math.sin(self.theta) * dt

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        now = self.get_clock().now()

        # Publish TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        # Quaternion from yaw
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular

        self.odom_pub.publish(odom)

    def destroy_node(self):
        # Stop motors before shutting down
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'STOP\n')
                time.sleep(0.1)
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
