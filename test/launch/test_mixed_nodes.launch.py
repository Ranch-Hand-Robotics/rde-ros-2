#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    # Regular node
    regular_node = Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='regular_talker',
        output='screen'
    )
    
    # Lifecycle node
    lifecycle_node = LifecycleNode(
        package='lifecycle',
        executable='lifecycle_talker',
        name='lifecycle_talker',
        namespace='/test',
        output='screen'
    )
    
    # Another regular node
    listener_node = Node(
        package='demo_nodes_py',
        executable='listener',
        name='listener',
        output='screen'
    )
    
    return LaunchDescription([
        regular_node,
        lifecycle_node,
        listener_node
    ])
