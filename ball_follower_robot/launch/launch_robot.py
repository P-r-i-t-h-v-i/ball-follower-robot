#!/usr/bin/env python3
"""
Launch file for the Ball Follower Robot (REAL ROBOT on Raspberry Pi).

Launches:
  - robot_state_publisher: publishes TF from URDF
  - joint_state_publisher: publishes servo joint state
  - serial_bridge: combined motor control + ultrasonic scanning (single serial)
  - object_tracker: yellow ball detection + steering commands
  - path_tracker: accumulates and publishes path
  - slam_toolbox: builds map from /scan + /odom
  - rviz2: visualization

Usage:
    ros2 launch ball_follower_robot launch_robot.py
    ros2 launch ball_follower_robot launch_robot.py serial_port:=/dev/ttyACM0
    ros2 launch ball_follower_robot launch_robot.py show_preview:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    # ---- Load URDF and RViz config ----
    pkg_share = FindPackageShare('ball_follower_robot').find('ball_follower_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ---- Launch arguments ----
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial port for Arduino')

    camera_index_arg = DeclareLaunchArgument(
        'camera_index', default_value='0',
        description='USB camera device index')

    show_preview_arg = DeclareLaunchArgument(
        'show_preview', default_value='true',
        description='Show OpenCV preview window')

    # ---- Robot State Publisher (publishes TF from URDF) ----
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )



    # ---- Serial Bridge (motor + ultrasonic on single serial port) ----
    serial_bridge_node = Node(
        package='ball_follower_robot',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200,
            'max_pwm': 200,
            'wheel_base': 0.215,
            'max_speed': 0.25,
            'max_range': 4.0,
            'min_range': 0.02,
            'num_readings': 37,
            'cmd_rate': 10.0,
            'scan_rate': 2.0,
        }]
    )

    # ---- Object Tracker (camera + yellow ball detection) ----
    object_tracker_node = Node(
        package='ball_follower_robot',
        executable='object_tracker',
        name='object_tracker',
        output='screen',
        parameters=[{
            'camera_index': LaunchConfiguration('camera_index'),
            'frame_width': 640,
            'frame_height': 480,
            'h_low': 20,
            's_low': 100,
            'v_low': 100,
            'h_high': 35,
            's_high': 255,
            'v_high': 255,
            'min_radius': 20,
            'max_linear_speed': 0.4,
            'max_angular_speed': 1.0,
            'target_radius': 120,
            'show_preview': LaunchConfiguration('show_preview'),
        }]
    )

    # ---- Path Tracker ----
    path_tracker_node = Node(
        package='ball_follower_robot',
        executable='path_tracker',
        name='path_tracker',
        output='screen',
        parameters=[{
            'min_distance': 0.02,
            'publish_rate': 2.0,
            'max_path_points': 5000,
        }]
    )

    # ---- SLAM Toolbox ----
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'resolution': 0.05,
            'max_laser_range': 4.0,
            'minimum_travel_distance': 0.1,
            'minimum_travel_heading': 0.1,
            'map_update_interval': 2.0,
        }]
    )

    # ---- RViz2 ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        serial_port_arg,
        camera_index_arg,
        show_preview_arg,
        robot_state_publisher_node,
        serial_bridge_node,
        object_tracker_node,
        path_tracker_node,
        slam_node,
        rviz_node,
    ])
