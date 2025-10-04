#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the directory where this launch file is located
    launch_dir = os.path.dirname(os.path.abspath(__file__))

    # Path to mapviz config file
    mapviz_config = os.path.join(launch_dir, 'mapviz.mvc')

    # Osaka Station coordinates
    osaka_station_lat = 34.702485
    osaka_station_lon = 135.495951
    osaka_station_alt = 0.0

    # Remove cached mapviz config file to ensure fresh config load
    mapviz_cache_file = os.path.expanduser('~/.mapviz_config')
    if os.path.exists(mapviz_cache_file):
        os.remove(mapviz_cache_file)

    return LaunchDescription([
        # Start initialize_origin node with fixed Osaka Station coordinates
        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            output='screen',
            parameters=[{
                'local_xy_frame': 'map',
                'local_xy_origin': 'osaka_station',
                'local_xy_origins': str([{
                    'name': 'osaka_station',
                    'latitude': osaka_station_lat,
                    'longitude': osaka_station_lon,
                    'altitude': osaka_station_alt,
                    'heading': 0.0
                }])
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
        Node(
            package='mapviz',
            executable='mapviz',
            name='mapviz',
            output='screen',
            parameters=[{
                'config': mapviz_config,
                'autosave': False
            }]
        ),
    ])
