"""Exercise launch logging without starting any processes."""

import atexit
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


print("module diagnostic {not JSON}")
atexit.register(lambda: print("shutdown diagnostic {not JSON}"))


def expand(context):
    print("opaque diagnostic {not JSON}")
    return [
        LogInfo(msg="🚀 Launching as Normal ROS Node"),
        LogInfo(msg="conditional diagnostic", condition=IfCondition(LaunchConfiguration("show_log"))),
        ExecuteProcess(cmd=[sys.executable, "-c", "print('must not execute')"]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("show_log", default_value="true"),
        OpaqueFunction(function=expand),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(os.path.dirname(__file__), "log_info_child.launch.py"))),
    ])