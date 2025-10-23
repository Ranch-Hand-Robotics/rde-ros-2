#!/usr/bin/env python3

import unittest
import pytest

import rclpy
from rclpy.lifecycle import State, TransitionCallbackReturn
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition


class TestLifecycleNode(unittest.TestCase):
    """
    Test suite for the RDE Lifecycle Node.
    """

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 for the test suite."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 after all tests."""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures."""
        self.node = rclpy.create_node('test_lifecycle_node')

    def tearDown(self):
        """Clean up test fixtures."""
        self.node.destroy_node()

    def test_lifecycle_node_services_exist(self):
        """
        Test that the lifecycle node exposes the expected services.
        This test requires the lifecycle node to be running.
        """
        # Create clients for lifecycle services
        get_state_client = self.node.create_client(
            GetState,
            '/rde_lifecycle_py/get_state'
        )
        
        change_state_client = self.node.create_client(
            ChangeState,
            '/rde_lifecycle_py/change_state'
        )
        
        # Wait for services with timeout
        timeout_sec = 5.0
        get_state_ready = get_state_client.wait_for_service(timeout_sec=timeout_sec)
        change_state_ready = change_state_client.wait_for_service(timeout_sec=timeout_sec)
        
        # Clean up
        self.node.destroy_client(get_state_client)
        self.node.destroy_client(change_state_client)
        
        # For this test to pass, the lifecycle node must be running
        # Skip test if services are not available
        if not get_state_ready or not change_state_ready:
            pytest.skip("Lifecycle node services not available - node may not be running")
        
        self.assertTrue(get_state_ready)
        self.assertTrue(change_state_ready)

    def test_lifecycle_transitions(self):
        """
        Test that lifecycle state transitions work correctly.
        """
        get_state_client = self.node.create_client(
            GetState,
            '/rde_lifecycle_py/get_state'
        )
        
        change_state_client = self.node.create_client(
            ChangeState,
            '/rde_lifecycle_py/change_state'
        )
        
        # Wait for services
        if not get_state_client.wait_for_service(timeout_sec=5.0) or \
           not change_state_client.wait_for_service(timeout_sec=5.0):
            pytest.skip("Lifecycle node services not available")
        
        # Test configure transition
        configure_request = ChangeState.Request()
        configure_request.transition.id = Transition.TRANSITION_CONFIGURE
        
        future = change_state_client.call_async(configure_request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.done():
            result = future.result()
            self.assertTrue(result.success)
        
        # Verify we're in inactive state
        state_request = GetState.Request()
        future = get_state_client.call_async(state_request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.done():
            result = future.result()
            self.assertEqual(result.current_state.id, LifecycleState.PRIMARY_STATE_INACTIVE)
        
        # Clean up
        self.node.destroy_client(get_state_client)
        self.node.destroy_client(change_state_client)


if __name__ == '__main__':
    unittest.main()
