#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='demo_nodes_cpp',
            executable='talker',
            name='lc_talker',
            namespace='',
            output='screen'
        ),
    ])
