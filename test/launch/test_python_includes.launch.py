#!/usr/bin/env python3
"""Test Python launch file with IncludeLaunchDescription"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
from pathlib import Path

def generate_launch_description():
    # Get the directory where this launch file is located
    launch_file_dir = Path(__file__).parent
    
    # Include Python launch files using relative paths
    # This allows testing of Ctrl+Click navigation to included files
    # Note: We use PythonLaunchDescriptionSource which is available in all ROS 2 distros
    
    # Include a Python launch file
    included_py_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_file_dir / 'simple_lifecycle_launch.py')
        )
    )
    
    # Include another Python launch file
    included_py_launch2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_file_dir / 'test_launch.py')
        )
    )
    
    # Example of how to include using FindPackageShare (similar to find-pkg-share in XML)
    # This would be used in a real ROS package:
    # included_package_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('some_package'),
    #             'launch',
    #             'some_file.launch.py'
    #         ])
    #     ])
    # )
    
    # Note: For XML launch files, use XMLLaunchDescriptionSource from launch_xml package
    # This requires the launch_xml package which is available in ROS 2 Humble and later:
    #
    # from launch.launch_description_sources import XMLLaunchDescriptionSource  # Requires launch_xml
    # included_xml_launch = IncludeLaunchDescription(
    #     XMLLaunchDescriptionSource(
    #         str(launch_file_dir / 'included_by_python.launch.xml')
    #     )
    # )
    
    return LaunchDescription([
        included_py_launch,
        included_py_launch2,
    ])

