#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    # Regular node
    listener = Node(
        package='demo_nodes_py',
        executable='listener',
        name='listener',
        output='screen'
    )
    
    # Lifecycle node (simple example without complex event handling)
    lifecycle_talker = LifecycleNode(
        package='lifecycle',
        executable='lifecycle_talker',
        name='lc_talker',
        namespace='test',
        output='screen'
    )
    
    return LaunchDescription([
        listener,
        lifecycle_talker,
    ])
