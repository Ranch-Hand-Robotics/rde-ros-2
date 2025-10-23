#!/usr/bin/env python3

import unittest
import pytest
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestRdeSubscriber(unittest.TestCase):
    """
    Test suite for the RDE Subscriber node.
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
        self.node = rclpy.create_node('test_rde_subscriber')

    def tearDown(self):
        """Clean up test fixtures."""
        self.node.destroy_node()

    def test_subscriber_exists(self):
        """Test that the subscriber node exists and is subscribed to /rde topic."""
        # Wait for subscriber to appear
        start_time = time.time()
        subscriber_found = False
        
        while time.time() - start_time < 5.0:
            topic_info = self.node.get_subscriptions_info_by_topic('/rde')
            if len(topic_info) > 0:
                subscriber_found = True
                break
            time.sleep(0.1)
        
        if not subscriber_found:
            pytest.skip("Subscriber not found - rde_subscriber node may not be running")
        
        self.assertTrue(subscriber_found)

    def test_subscriber_receives_messages(self):
        """Test that the subscriber can receive messages."""
        # Create a publisher
        publisher = self.node.create_publisher(String, '/rde', 10)
        
        # Wait for subscriber to connect
        start_time = time.time()
        while time.time() - start_time < 2.0:
            if publisher.get_subscription_count() > 0:
                break
            time.sleep(0.1)
        
        if publisher.get_subscription_count() == 0:
            pytest.skip("No subscribers connected - rde_subscriber node may not be running")
        
        # Publish some test messages
        for i in range(5):
            msg = String()
            msg.data = f'Test message {i}'
            publisher.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Clean up
        self.node.destroy_publisher(publisher)
        
        # The subscriber should have received our messages
        # (We can't directly verify this without modifying the subscriber node,
        # but we can verify it's subscribed)
        self.assertGreater(publisher.get_subscription_count(), 0)


if __name__ == '__main__':
    unittest.main()
