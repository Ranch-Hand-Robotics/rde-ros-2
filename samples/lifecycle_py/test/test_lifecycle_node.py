"""
Unit tests for the ROS 2 lifecycle node example (Python).
"""

import unittest
from unittest.mock import patch, MagicMock

import rclpy
from rclpy.lifecycle import State, TransitionCallbackReturn

# Import the lifecycle node class
import sys
import os

# Add the source directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from rde_lifecycle_py.rde_lifecycle_py import RdeLifecycleNode


class TestRdeLifecycleNode(unittest.TestCase):
    """Test cases for RdeLifecycleNode."""

    def setUp(self):
        """Set up test fixtures."""
        rclpy.init()

    def tearDown(self):
        """Clean up after tests."""
        rclpy.shutdown()

    def test_node_instantiation(self):
        """Test that the lifecycle node can be instantiated."""
        node = RdeLifecycleNode('test_node')
        self.assertIsNotNone(node)
        self.assertEqual(node.get_name(), 'test_node')

    def test_on_configure_returns_success(self):
        """Test that on_configure returns SUCCESS."""
        node = RdeLifecycleNode('test_node')
        state = State(id=1, label='')
        result = node.on_configure(state)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

    def test_on_activate_returns_success(self):
        """Test that on_activate returns SUCCESS."""
        node = RdeLifecycleNode('test_node')
        state = State(id=1, label='')
        result = node.on_activate(state)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

    def test_on_deactivate_returns_success(self):
        """Test that on_deactivate returns SUCCESS."""
        node = RdeLifecycleNode('test_node')
        state = State(id=1, label='')
        result = node.on_deactivate(state)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

    def test_on_cleanup_returns_success(self):
        """Test that on_cleanup returns SUCCESS."""
        node = RdeLifecycleNode('test_node')
        state = State(id=1, label='')
        result = node.on_cleanup(state)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

    def test_on_shutdown_returns_success(self):
        """Test that on_shutdown returns SUCCESS."""
        node = RdeLifecycleNode('test_node')
        state = State(id=1, label='')
        result = node.on_shutdown(state)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

    def test_on_error_returns_success(self):
        """Test that on_error returns SUCCESS."""
        node = RdeLifecycleNode('test_node')
        state = State(id=1, label='')
        result = node.on_error(state)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

    def test_lifecycle_transitions(self):
        """Test a series of lifecycle transitions."""
        node = RdeLifecycleNode('test_node')
        state = State(id=1, label='')

        # Test configure -> activate -> deactivate -> cleanup sequence
        result = node.on_configure(state)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

        result = node.on_activate(state)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

        result = node.on_deactivate(state)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

        result = node.on_cleanup(state)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)

    def test_get_logger(self):
        """Test that the node has a valid logger."""
        node = RdeLifecycleNode('test_node')
        logger = node.get_logger()
        self.assertIsNotNone(logger)


if __name__ == '__main__':
    unittest.main()
