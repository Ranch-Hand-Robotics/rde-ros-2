#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='rde_lifecycle_cpp',
        description='Name of the lifecycle node'
    )

    # Create the lifecycle node
    lifecycle_node = LifecycleNode(
        package='rde_lifecycle_cpp',
        executable='rde_lifecycle_cpp',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[]
    )

    return LaunchDescription([
        node_name_arg,
        lifecycle_node
    ])
