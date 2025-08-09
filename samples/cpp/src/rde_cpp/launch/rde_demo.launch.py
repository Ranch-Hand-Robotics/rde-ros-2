#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rde_cpp',
            executable='rde_publisher',
            name='rde_publisher',
            output='screen',
            parameters=[]
        ),
        Node(
            package='rde_cpp',
            executable='rde_subscriber',
            name='rde_subscriber',
            output='screen',
            parameters=[]
        )
    ])
