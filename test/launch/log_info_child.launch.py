"""Nested launch logging and substitution fixture."""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=["nested diagnostic: ", LaunchConfiguration("show_log")]),
    ])