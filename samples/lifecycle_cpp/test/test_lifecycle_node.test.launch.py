#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions


def generate_test_description():
    """
    Launch file for testing rde_lifecycle_cpp node.
    This launches the lifecycle node and runs integration tests against it.
    """
    
    # Start the lifecycle node
    lifecycle_node = Node(
        package='rde_lifecycle_cpp',
        executable='rde_lifecycle_cpp',
        name='rde_lifecycle_cpp',
        output='screen'
    )
    
    return LaunchDescription([
        lifecycle_node,
        launch_testing.actions.ReadyToTest()
    ])
