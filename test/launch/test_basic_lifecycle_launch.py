#!/usr/bin/env python3

import launch
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return launch.LaunchDescription([
        LifecycleNode(
            package='lifecycle',
            executable='lifecycle_talker',
            name='basic_lc_talker'
        )
    ])
