from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch both publisher and subscriber nodes."""
    return LaunchDescription([
        Node(
            package='rde_rust',
            executable='minimal_publisher',
            name='publisher',
            output='screen'
        ),
        Node(
            package='rde_rust',
            executable='minimal_subscriber',
            name='subscriber',
            output='screen'
        ),
    ])
