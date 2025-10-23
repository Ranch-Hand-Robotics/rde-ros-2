#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions


def generate_test_description():
    """
    Launch file for testing rde_publisher Python node.
    This launches the publisher node and runs integration tests against it.
    """
    
    # Start the publisher node
    rde_publisher_node = Node(
        package='rde_py',
        executable='rde_publisher',
        name='rde_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        rde_publisher_node,
        launch_testing.actions.ReadyToTest()
    ])
