#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='radar_node',
            executable='radar_node_node',
            name='radar_publisher',
            output='screen',
        ),
        Node(
            package='drwig_adas',
            executable='drwig_eba',
            name='drwig_eba',
            output='screen',
        ),
    ])
