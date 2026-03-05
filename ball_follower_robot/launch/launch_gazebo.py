#!/usr/bin/env python3
"""
Gazebo Simulation Launch File (ROS2 Jazzy + Gazebo Harmonic)

Spawns the ball_follower_robot in Gazebo Harmonic with:
  - robot_state_publisher + joint_state_publisher
  - ros_gz_bridge for topic bridging
  - path_tracker, SLAM, and RViz

Usage:
    ros2 launch ball_follower_robot launch_gazebo.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('ball_follower_robot').find('ball_follower_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ---- Start Gazebo Harmonic ----
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen',
    )

    # ---- Spawn Robot in Gazebo ----
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name', 'ball_follower_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',
        ]
    )

    # ---- Robot State Publisher ----
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # ---- Joint State Publisher (for servo joint) ----
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ---- ROS-Gazebo Bridge ----
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': True}],
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
            'use_sim_time': True,
        }]
    )

    # ---- SLAM Toolbox ----
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
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

    # ---- RViz ----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_robot,
        bridge_node,
        path_tracker_node,
        slam_node,
        rviz_node,
    ])
