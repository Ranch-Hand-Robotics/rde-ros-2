#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            output='screen'
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='listener',
            output='screen'
        ),
    ])
