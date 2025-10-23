#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions


def generate_test_description():
    """
    Launch file for testing rde_subscriber Python node.
    This launches both publisher and subscriber nodes for integration testing.
    """
    
    # Start the publisher node to generate messages
    rde_publisher_node = Node(
        package='rde_py',
        executable='rde_publisher',
        name='rde_publisher',
        output='screen'
    )
    
    # Start the subscriber node
    rde_subscriber_node = Node(
        package='rde_py',
        executable='rde_subscriber',
        name='rde_subscriber',
        output='screen'
    )
    
    return LaunchDescription([
        rde_publisher_node,
        rde_subscriber_node,
        launch_testing.actions.ReadyToTest()
    ])
