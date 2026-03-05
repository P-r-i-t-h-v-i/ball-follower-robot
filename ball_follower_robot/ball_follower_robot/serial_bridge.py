#!/usr/bin/env python3
"""
Serial Bridge Node — Single serial connection shared between motor and scanner

Since both motor_controller and ultrasonic_mapper need the same Arduino serial
port, this node manages the single serial connection and:
  - Subscribes to /target_cmd and forwards motor commands to Arduino
  - Reads scan data (S,angle,distance) and publishes /scan
  - Publishes /odom from dead reckoning

This replaces running motor_controller + ultrasonic_mapper separately.

Subscribes:
    /target_cmd (geometry_msgs/Twist)

Publishes:
    /scan (sensor_msgs/LaserScan)
    /odom (nav_msgs/Odometry)
    TF: odom -> base_link, base_link -> laser_frame
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import Odometry
import tf2_ros
import serial
import math
import threading
import time


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # ---------- Parameters ----------
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_pwm', 200)
        self.declare_parameter('wheel_base', 0.15)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('num_readings', 37)
        self.declare_parameter('cmd_rate', 10.0)
        self.declare_parameter('scan_rate', 2.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.num_readings = self.get_parameter('num_readings').value
        cmd_rate = self.get_parameter('cmd_rate').value
        scan_rate = self.get_parameter('scan_rate').value

        # ---------- Serial Connection ----------
        self.ser = None
        self.serial_lock = threading.Lock()
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2)  # Wait for Arduino reset
            self.get_logger().info(f'Serial bridge connected: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial port: {e}')
            self.get_logger().warn('Running in DRY-RUN mode')

        # ---------- Subscribers ----------
        self.cmd_sub = self.create_subscription(
            Twist, '/target_cmd', self.cmd_callback, 10)

        # ---------- Publishers ----------
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---------- State ----------
        self.latest_twist = Twist()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.servo_angle_rad = 0.0  # current servo angle in radians
        self.last_time = self.get_clock().now()
        self.scan_data = [float('inf')] * self.num_readings
        self.scan_lock = threading.Lock()

        # ---------- Serial reader thread ----------
        self.running = True
        if self.ser:
            self.read_thread = threading.Thread(
                target=self._serial_reader, daemon=True)
            self.read_thread.start()

        # ---------- Timers ----------
        self.cmd_timer = self.create_timer(1.0 / cmd_rate, self.send_motor_cmd)
        self.scan_timer = self.create_timer(1.0 / scan_rate, self.publish_scan)

    # ==================== MOTOR SIDE ====================

    def cmd_callback(self, msg: Twist):
        self.latest_twist = msg

    def send_motor_cmd(self):
        linear = self.latest_twist.linear.x
        angular = self.latest_twist.angular.z

        # Differential drive
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)

        pwm_left = int((v_left / self.max_speed) * self.max_pwm) if self.max_speed else 0
        pwm_right = int((v_right / self.max_speed) * self.max_pwm) if self.max_speed else 0
        pwm_left = max(-self.max_pwm, min(self.max_pwm, pwm_left))
        pwm_right = max(-self.max_pwm, min(self.max_pwm, pwm_right))

        cmd = f'M,{pwm_left},{pwm_right}\n'
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(cmd.encode())
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')

        # Odometry
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if 0 < dt < 1.0:
            self._update_odom(linear, angular, dt)

    def _update_odom(self, linear, angular, dt):
        self.theta += angular * dt
        self.x += linear * math.cos(self.theta) * dt
        self.y += linear * math.sin(self.theta) * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Update wheel positions from differential drive kinematics
        wheel_radius = 0.0275  # matches URDF
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)
        self.left_wheel_pos += (v_left / wheel_radius) * dt
        self.right_wheel_pos += (v_right / wheel_radius) * dt

        now = self.get_clock().now()

        # TF: odom -> base_footprint (match URDF root frame)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        self.odom_pub.publish(odom)

        # Publish joint states
        self._publish_joint_states(now, v_left / wheel_radius, v_right / wheel_radius)

    def _publish_joint_states(self, now, left_vel=0.0, right_vel=0.0):
        """Publish wheel and servo joint positions."""
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint', 'servo_joint']
        js.position = [self.left_wheel_pos, self.right_wheel_pos, self.servo_angle_rad]
        js.velocity = [left_vel, right_vel, 0.0]
        js.effort = []
        self.joint_pub.publish(js)

    # ==================== SCAN SIDE ====================

    def _serial_reader(self):
        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(0.1)
                continue
            try:
                with self.serial_lock:
                    if self.ser.in_waiting:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    else:
                        line = ''
                if line.startswith('S,'):
                    parts = line.split(',')
                    if len(parts) == 3:
                        angle = int(parts[1])
                        dist_m = float(parts[2]) / 100.0
                        # Update servo angle: Arduino 0-180° → URDF -π/2 to +π/2 rad
                        self.servo_angle_rad = math.radians(angle - 90)
                        idx = angle // 5
                        if 0 <= idx < self.num_readings:
                            with self.scan_lock:
                                if self.min_range <= dist_m <= self.max_range:
                                    self.scan_data[idx] = dist_m
                                else:
                                    self.scan_data[idx] = float('inf')
                else:
                    time.sleep(0.005)
            except Exception as e:
                self.get_logger().debug(f'Serial read: {e}')
                time.sleep(0.01)

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi / 2.0
        scan.angle_max = math.pi / 2.0
        scan.angle_increment = math.pi / (self.num_readings - 1)
        scan.time_increment = 0.03
        scan.scan_time = scan.time_increment * self.num_readings
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        with self.scan_lock:
            scan.ranges = list(self.scan_data)
        self.scan_pub.publish(scan)

    def destroy_node(self):
        self.running = False
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
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
