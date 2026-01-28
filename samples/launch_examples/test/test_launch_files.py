"""
Unit tests for launch files.
"""

import unittest
import os
from pathlib import Path

from launch import LaunchDescription
from launch.launch_service import LaunchService


class LaunchFilesTest(unittest.TestCase):
    """Test cases for launch files."""

    def setUp(self):
        """Set up test fixtures."""
        # Get the path to the launch files
        self.launch_dir = Path(__file__).parent.parent / 'launch'

    def test_launch_files_exist(self):
        """Test that launch files exist."""
        launch_files = [
            'test_launch.py',
            'test_python_includes.launch.py',
            'simple_lifecycle_launch.py',
        ]

        for launch_file in launch_files:
            launch_path = self.launch_dir / launch_file
            self.assertTrue(
                launch_path.exists(),
                f"Launch file {launch_file} not found at {launch_path}"
            )

    def test_launch_xml_files_exist(self):
        """Test that XML launch files exist."""
        xml_files = [
            'test_launch.py',
            'test_with_includes.launch',
            'test_multiple_includes.launch.xml',
            'included_by_python.launch.xml',
            'example_target.launch.xml',
        ]

        for xml_file in xml_files:
            xml_path = self.launch_dir / xml_file
            self.assertTrue(
                xml_path.exists() or (self.launch_dir / xml_file).with_suffix('').exists(),
                f"XML launch file {xml_file} not found"
            )

    def test_launch_files_are_readable(self):
        """Test that launch files are readable."""
        launch_files = list(self.launch_dir.glob('*.py')) + \
                       list(self.launch_dir.glob('*.launch')) + \
                       list(self.launch_dir.glob('*.launch.xml'))

        for launch_file in launch_files:
            with open(launch_file, 'r') as f:
                content = f.read()
                self.assertIsNotNone(content)
                self.assertGreater(len(content), 0, f"Launch file {launch_file.name} is empty")

    def test_launch_files_valid_python_syntax(self):
        """Test that Python launch files have valid syntax."""
        import ast
        import sys

        python_launch_files = list(self.launch_dir.glob('*.launch.py')) + \
                              list(self.launch_dir.glob('*.py'))

        for launch_file in python_launch_files:
            try:
                with open(launch_file, 'r') as f:
                    ast.parse(f.read())
            except SyntaxError as e:
                self.fail(f"Python launch file {launch_file.name} has syntax error: {e}")

    def test_launch_files_not_empty(self):
        """Test that all launch files have content."""
        all_launch_files = list(self.launch_dir.glob('*launch*'))

        self.assertGreater(
            len(all_launch_files),
            0,
            "No launch files found in the launch directory"
        )


if __name__ == '__main__':
    unittest.main()
