#!/usr/bin/env python3
"""Test Python launch file with IncludeLaunchDescription
This demonstrates real-world ROS 2 launch file patterns including FindPackageShare usage.
"""

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
    
    # Method 1: Include using relative paths
    # This is useful for local testing and when files are in the same package
    included_py_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_file_dir / 'simple_lifecycle_launch.py')
        )
    )
    launch_actions.append(included_py_launch)
    
    # Method 2: Include using PathJoinSubstitution with absolute paths
    # This is useful when you know the exact location
    included_py_launch2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_file_dir / 'test_launch.py')
        )
    )
    launch_actions.append(included_py_launch2)
    
    # Method 3: Include XML launch file (distro-sensitive)
    # This requires the launch_xml package (available in ROS 2 Humble and later)
    if HAS_XML_LAUNCH_SOURCE:
        included_xml_launch = IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                str(launch_file_dir / 'included_by_python.launch.xml')
            )
        )
        launch_actions.append(included_xml_launch)
    
    # Method 4: Include using FindPackageShare (REAL ROS package pattern)
    # This is the standard approach for including launch files from other packages
    # Example: Including a launch file from another ROS package
    included_package_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([
             PathJoinSubstitution([
                 FindPackageShare('rde_launch_examples'),  # Replace with actual package name
                 'launch',
                 'simple_lifecycle_launch.py'  # Replace with actual launch file
             ])
         ])
     )
    launch_actions.append(included_package_launch)

    # For XML launch files from packages (requires launch_xml):
    if HAS_XML_LAUNCH_SOURCE:
        included_package_xml = IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rde_launch_examples'),
                    'launch',
                    'included_by_python.launch.xml'
                ])
            ])
        )
        launch_actions.append(included_package_xml)
    
    return LaunchDescription(launch_actions)



