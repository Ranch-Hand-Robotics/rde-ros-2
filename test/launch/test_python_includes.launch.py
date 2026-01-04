#!/usr/bin/env python3
"""Test Python launch file with IncludeLaunchDescription"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
from pathlib import Path

def generate_launch_description():
    # Get the directory where this launch file is located
    launch_file_dir = Path(__file__).parent
    
    # Include an XML launch file using a relative path
    # This allows testing of Ctrl+Click navigation to included files
    included_xml_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            str(launch_file_dir / 'included_by_python.launch.xml')
        )
    )
    
    # Include another Python launch file using a relative path
    included_py_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_file_dir / 'simple_lifecycle_launch.py')
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
    
    return LaunchDescription([
        included_xml_launch,
        included_py_launch,
    ])
