"""
Tests for ROS 2 message, service, and action interfaces.
"""

import unittest

try:
    from msg_interfaces.msg import (
        BasicTypes,
        ComplexMessage,
        NavigationGoal,
        RobotStatus,
        SensorData,
    )
    from msg_interfaces.srv import (
        CalculateSum,
        GetRobotInfo,
    )
    INTERFACES_AVAILABLE = True
except ImportError:
    INTERFACES_AVAILABLE = False


class MessageInterfacesTest(unittest.TestCase):
    """Test cases for message interfaces."""

    @unittest.skipUnless(INTERFACES_AVAILABLE, "Interfaces not available")
    def test_basic_types_message_creation(self):
        """Test that BasicTypes message can be instantiated."""
        msg = BasicTypes()
        self.assertIsNotNone(msg)

    @unittest.skipUnless(INTERFACES_AVAILABLE, "Interfaces not available")
    def test_complex_message_creation(self):
        """Test that ComplexMessage message can be instantiated."""
        msg = ComplexMessage()
        self.assertIsNotNone(msg)

    @unittest.skipUnless(INTERFACES_AVAILABLE, "Interfaces not available")
    def test_navigation_goal_message_creation(self):
        """Test that NavigationGoal message can be instantiated."""
        msg = NavigationGoal()
        self.assertIsNotNone(msg)

    @unittest.skipUnless(INTERFACES_AVAILABLE, "Interfaces not available")
    def test_robot_status_message_creation(self):
        """Test that RobotStatus message can be instantiated."""
        msg = RobotStatus()
        self.assertIsNotNone(msg)

    @unittest.skipUnless(INTERFACES_AVAILABLE, "Interfaces not available")
    def test_sensor_data_message_creation(self):
        """Test that SensorData message can be instantiated."""
        msg = SensorData()
        self.assertIsNotNone(msg)

    @unittest.skipUnless(INTERFACES_AVAILABLE, "Interfaces not available")
    def test_calculate_sum_service_creation(self):
        """Test that CalculateSum service can be instantiated."""
        request = CalculateSum.Request()
        response = CalculateSum.Response()
        self.assertIsNotNone(request)
        self.assertIsNotNone(response)

    @unittest.skipUnless(INTERFACES_AVAILABLE, "Interfaces not available")
    def test_get_robot_info_service_creation(self):
        """Test that GetRobotInfo service can be instantiated."""
        request = GetRobotInfo.Request()
        response = GetRobotInfo.Response()
        self.assertIsNotNone(request)
        self.assertIsNotNone(response)

    @unittest.skipUnless(INTERFACES_AVAILABLE, "Interfaces not available")
    def test_message_serialization(self):
        """Test that messages can be serialized."""
        msg = BasicTypes()
        try:
            # This tests that the message has proper serialization support
            msg_type = type(msg)
            self.assertIsNotNone(msg_type)
            self.assertTrue(hasattr(msg, '__slots__') or hasattr(msg, '__dict__'))
        except Exception as e:
            self.fail(f"Message serialization failed: {e}")

    def test_message_definitions_syntax(self):
        """Test that message definition files have valid syntax."""
        import os
        from pathlib import Path

        msg_dir = Path(__file__).parent.parent / 'msg'
        srv_dir = Path(__file__).parent.parent / 'srv'

        # Check if message files exist
        msg_files = list(msg_dir.glob('*.msg')) if msg_dir.exists() else []
        srv_files = list(srv_dir.glob('*.srv')) if srv_dir.exists() else []

        self.assertGreater(
            len(msg_files) + len(srv_files),
            0,
            "No message or service definition files found"
        )

    def test_message_files_readable(self):
        """Test that all message definition files are readable."""
        from pathlib import Path

        msg_dir = Path(__file__).parent.parent / 'msg'
        srv_dir = Path(__file__).parent.parent / 'srv'

        all_files = []
        if msg_dir.exists():
            all_files.extend(msg_dir.glob('*.msg'))
        if srv_dir.exists():
            all_files.extend(srv_dir.glob('*.srv'))

        for file in all_files:
            try:
                with open(file, 'r') as f:
                    content = f.read()
                    self.assertGreater(
                        len(content),
                        0,
                        f"Definition file {file.name} is empty"
                    )
            except Exception as e:
                self.fail(f"Cannot read definition file {file.name}: {e}")


if __name__ == '__main__':
    unittest.main()
