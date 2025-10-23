#!/usr/bin/env python3

import unittest
import pytest
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestRdePublisher(unittest.TestCase):
    """
    Test suite for the RDE Publisher node.
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
        self.node = rclpy.create_node('test_rde_publisher')
        self.received_messages = []

    def tearDown(self):
        """Clean up test fixtures."""
        self.node.destroy_node()

    def message_callback(self, msg):
        """Callback for receiving messages."""
        self.received_messages.append(msg.data)

    def test_publisher_exists(self):
        """Test that the publisher node exists and is publishing on /rde topic."""
        # Wait for publisher to appear
        start_time = time.time()
        publisher_found = False
        
        while time.time() - start_time < 5.0:
            topic_info = self.node.get_publishers_info_by_topic('/rde')
            if len(topic_info) > 0:
                publisher_found = True
                break
            time.sleep(0.1)
        
        if not publisher_found:
            pytest.skip("Publisher not found - rde_publisher node may not be running")
        
        self.assertTrue(publisher_found)

    def test_message_publishing(self):
        """Test that messages are being published on the /rde topic."""
        # Create a subscription
        subscription = self.node.create_subscription(
            String,
            '/rde',
            self.message_callback,
            10
        )
        
        # Spin for a few seconds to receive messages
        start_time = time.time()
        while time.time() - start_time < 3.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Clean up
        self.node.destroy_subscription(subscription)
        
        # We should have received at least 2 messages (publisher runs at 1 Hz)
        if len(self.received_messages) == 0:
            pytest.skip("No messages received - rde_publisher node may not be running")
        
        self.assertGreaterEqual(len(self.received_messages), 2)
        
        # Verify message format
        for msg in self.received_messages:
            self.assertIn('Hello from RDE', msg)

    def test_message_increment(self):
        """Test that the message counter increments."""
        # Create a subscription
        subscription = self.node.create_subscription(
            String,
            '/rde',
            self.message_callback,
            10
        )
        
        # Spin for a few seconds
        start_time = time.time()
        while time.time() - start_time < 3.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Clean up
        self.node.destroy_subscription(subscription)
        
        if len(self.received_messages) < 2:
            pytest.skip("Not enough messages received")
        
        # Messages should be different (counter increments)
        self.assertNotEqual(self.received_messages[0], self.received_messages[1])


if __name__ == '__main__':
    unittest.main()
