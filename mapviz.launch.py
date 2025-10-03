#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the directory where this launch file is located
    launch_dir = os.path.dirname(os.path.abspath(__file__))

    # Path to mapviz config file
    mapviz_config = os.path.join(launch_dir, 'mapviz.mvc')

    return LaunchDescription([
        # Start initialize_origin node to set origin from first GPS fix
        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            output='screen',
            parameters=[{
                'local_xy_frame': 'map',
                'local_xy_origin': 'auto',
                'local_xy_navsatfix_topic': '/fix'
            }]
        ),

        # Static TF: map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'map', '--child-frame-id', 'odom']
        ),

        # Static TF: odom -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'odom', '--child-frame-id', 'base_link']
        ),

        # Static TF: base_link -> gps_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_gps_link',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'gps_link']
        ),

        # Start mapviz
        ExecuteProcess(
            cmd=['ros2', 'run', 'mapviz', 'mapviz', '--load', mapviz_config],
            output='screen'
        ),
    ])
