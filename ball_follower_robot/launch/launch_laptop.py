#!/usr/bin/env python3
"""
LAPTOP LAUNCH FILE — runs on your laptop to visualize the robot remotely.

Only launches RViz2. It connects to the Raspberry Pi's ROS2 topics
over the network (WiFi/Ethernet) using ROS2 DDS discovery.

IMPORTANT: Both laptop and Pi must be on the SAME network and use
the same ROS_DOMAIN_ID (default is 0).

Usage on Laptop:
    ros2 launch ball_follower_robot launch_laptop.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    # ---- Load package share paths ----
    pkg_share = FindPackageShare('ball_follower_robot').find('ball_follower_robot')
    rviz_config = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ---- Robot State Publisher (loads URDF locally for RViz) ----
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

    # ---- RViz2 (visualization) ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
    ])
