"""
Simple test launch file for launch tree viewer testing.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a simple launch description for testing."""
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_node',
            namespace='/test',
            parameters=[{
                'use_sim_time': True,
                'topic_name': 'chatter'
            }],
            remappings=[
                ('/chatter', '/test/chatter')
            ]
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener_node',
            parameters=[{
                'use_sim_time': True
            }]
        )
    ])
