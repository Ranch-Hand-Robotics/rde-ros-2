#!/usr/bin/env python3
"""Test Python launch file with IncludeLaunchDescription"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    # This is an example of how Python launch files include other files
    # The link provider should work with XML launch files that have similar patterns
    
    return LaunchDescription([
        # Example include using FindPackageShare (similar to find-pkg-share in XML)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('some_package'),
        #             'launch',
        #             'some_file.launch.py'
        #         ])
        #     ])
        # )
    ])
