#!/usr/bin/env python3
"""
Ultrasonic Mapper Node — Converts servo-swept HC-SR04 data to LaserScan

Reads scan data from Arduino via serial (S,angle,distance_cm) and converts
it into a ROS LaserScan message for use with SLAM/mapping tools.

The servo sweeps 0°-180° on the robot front. We map this to a LaserScan
where 0° servo = -π/2 (right), 90° servo = 0 (forward), 180° = +π/2 (left).

Publishes:
    /scan (sensor_msgs/LaserScan) — for slam_toolbox / gmapping
    
Also publishes TF: base_link -> laser_frame
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros
import serial
import math
import threading
import time


class UltrasonicMapper(Node):
    def __init__(self):
        super().__init__('ultrasonic_mapper')

        # ---------- Parameters ----------
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_range', 4.0)        # meters (HC-SR04 max ~4m)
        self.declare_parameter('min_range', 0.02)       # meters (HC-SR04 min ~2cm)
        self.declare_parameter('num_readings', 37)      # 180° / 5° steps = 36 + 1
        self.declare_parameter('scan_publish_rate', 2.0) # Hz

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.num_readings = self.get_parameter('num_readings').value
        publish_rate = self.get_parameter('scan_publish_rate').value

        # ---------- Serial Connection ----------
        # NOTE: If motor_controller already opened this port, they share
        # the same Arduino. We can either:
        #   a) Use a single serial port and have motor_controller forward scan data
        #   b) Use separate ports if Arduino has multiple serial
        # Here, we share by reading from the SAME serial object.
        # In practice, a serial_bridge node or multiplexer should be used.
        # For simplicity, this node reads from the serial port directly.
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2)
            self.get_logger().info(f'Ultrasonic serial connected: {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial: {e}')
            self.get_logger().warn(
                'If motor_controller already has this port, use the '
                'serial_bridge approach (see SETUP_GUIDE.md)')

        # ---------- Publisher ----------
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Static TF: base_link -> laser_frame
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_tf()

        # ---------- Scan data buffer ----------
        self.scan_data = [float('inf')] * self.num_readings
        self.scan_lock = threading.Lock()
        self.sweep_complete = False

        # ---------- Serial reader thread ----------
        self.running = True
        if self.ser:
            self.serial_thread = threading.Thread(
                target=self._serial_reader, daemon=True)
            self.serial_thread.start()

        # ---------- Timer to publish scans ----------
        self.scan_timer = self.create_timer(
            1.0 / publish_rate, self.publish_scan)

    def _publish_static_tf(self):
        """Publish static transform: base_link -> laser_frame."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'
        # Sensor mounted on top of robot, centered
        t.transform.translation.x = 0.05  # 5cm forward of center
        t.transform.translation.z = 0.10  # 10cm above base
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    def _serial_reader(self):
        """Background thread: reads scan data from Arduino serial."""
        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(0.1)
                continue

            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('S,'):
                    parts = line.split(',')
                    if len(parts) == 3:
                        angle = int(parts[1])
                        distance_cm = float(parts[2])
                        distance_m = distance_cm / 100.0

                        # Map servo angle (0-180) to array index
                        index = angle // 5  # 5° steps
                        if 0 <= index < self.num_readings:
                            with self.scan_lock:
                                if distance_m < self.min_range:
                                    self.scan_data[index] = float('inf')
                                elif distance_m > self.max_range:
                                    self.scan_data[index] = float('inf')
                                else:
                                    self.scan_data[index] = distance_m

                            # Detect end of sweep
                            if angle == 0 or angle == 180:
                                self.sweep_complete = True

            except (serial.SerialException, ValueError) as e:
                self.get_logger().debug(f'Serial read error: {e}')
                time.sleep(0.01)

    def publish_scan(self):
        """Publish the current scan data as a LaserScan message."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        # Servo: 0° = right, 180° = left
        # LaserScan convention: angle_min to angle_max
        # Map: servo 0° -> -π/2, servo 180° -> +π/2
        scan.angle_min = -math.pi / 2.0
        scan.angle_max = math.pi / 2.0
        scan.angle_increment = math.pi / (self.num_readings - 1)

        # Time between measurements (approximate)
        scan.time_increment = 0.03  # ~30ms per step
        scan.scan_time = scan.time_increment * self.num_readings

        scan.range_min = self.min_range
        scan.range_max = self.max_range

        with self.scan_lock:
            scan.ranges = list(self.scan_data)
            # Reset for next sweep
            if self.sweep_complete:
                self.sweep_complete = False

        self.scan_pub.publish(scan)
        self.get_logger().debug(
            f'Published scan: {sum(1 for r in scan.ranges if r < self.max_range)} valid readings')

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
