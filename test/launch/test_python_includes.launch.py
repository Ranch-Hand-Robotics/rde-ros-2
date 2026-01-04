#!/usr/bin/env python3
"""Test Python launch file with IncludeLaunchDescription"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
from pathlib import Path

# Try to import XMLLaunchDescriptionSource (available in ROS 2 Humble and later)
try:
    from launch.launch_description_sources import XMLLaunchDescriptionSource
    HAS_XML_LAUNCH_SOURCE = True
except ImportError:
    HAS_XML_LAUNCH_SOURCE = False

def generate_launch_description():
    # Get the directory where this launch file is located
    launch_file_dir = Path(__file__).parent
    
    # List to collect all launch actions
    launch_actions = []
    
    # Include Python launch files using relative paths
    # This allows testing of Ctrl+Click navigation to included files
    # Note: PythonLaunchDescriptionSource is available in all ROS 2 distros
    
    # Include a Python launch file
    included_py_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_file_dir / 'simple_lifecycle_launch.py')
        )
    )
    launch_actions.append(included_py_launch)
    
    # Include another Python launch file
    included_py_launch2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_file_dir / 'test_launch.py')
        )
    )
    launch_actions.append(included_py_launch2)
    
    # Conditionally include XML launch file if XMLLaunchDescriptionSource is available
    # This requires the launch_xml package (available in ROS 2 Humble and later)
    if HAS_XML_LAUNCH_SOURCE:
        included_xml_launch = IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                str(launch_file_dir / 'included_by_python.launch.xml')
            )
        )
        launch_actions.append(included_xml_launch)
    
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
    
    return LaunchDescription(launch_actions)


