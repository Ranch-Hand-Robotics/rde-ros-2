#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rde_py',
            executable='rde_publisher',
            name='rde_publisher',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='rde_py',
            executable='rde_subscriber',
            name='rde_subscriber',
            output='screen',
            emulate_tty=True,
        )
    ])
