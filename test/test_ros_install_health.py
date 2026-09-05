"""Health probe unit tests; never activate ROS or contact a real middleware."""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import types
import unittest
from unittest.mock import Mock, patch


PROBE = Path(__file__).resolve().parents[1] / "assets/scripts/ros_install_health.py"
SPEC = importlib.util.spec_from_file_location("ros_install_health", PROBE)
health = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(health)


class HealthProbeTests(unittest.TestCase):
    def setUp(self):
        self.prefix = str(Path(__file__).resolve().parent / "fake_ros_prefix")
        self.environment = {
            "ROS_VERSION": "2", "ROS_DISTRO": "jazzy", "AMENT_PREFIX_PATH": self.prefix,
            "ROS_DISCOVERY_SERVER": "robot.example:11811", "ROS_DOMAIN_ID": "0",
            "FASTRTPS_DEFAULT_PROFILES_FILE": "/robot/profile.xml",
            "CYCLONEDDS_URI": "robot",
        }
        self.context = Mock()
        self.node = Mock()
        self.executor = Mock()
        self.rclpy = types.ModuleType("rclpy")
        self.rclpy.__file__ = str(Path(self.prefix) / "lib/rclpy/__init__.py")
        self.rclpy.init = Mock()
        self.rclpy.create_node = Mock(return_value=self.node)
        self.modules = {
            "rclpy": self.rclpy,
            "rclpy.context": types.SimpleNamespace(Context=Mock(return_value=self.context)),
            "rclpy.utilities": types.SimpleNamespace(get_rmw_implementation_identifier=lambda: "rmw_fastrtps_cpp"),
            "rclpy.executors": types.SimpleNamespace(SingleThreadedExecutor=Mock(return_value=self.executor)),
            "ament_index_python.packages": types.SimpleNamespace(get_package_prefix=lambda name: self.prefix),
            "std_msgs.msg": types.SimpleNamespace(
                __file__=str(Path(self.prefix) / "lib/std_msgs/msg/__init__.py"),
                String=types.SimpleNamespace,
            ),
        }
        self.callback = None
        self.node.create_subscription.side_effect = self.subscribe
        self.node.create_publisher.return_value.publish.side_effect = lambda msg: self.callback(msg)

    def subscribe(self, message_type, topic, callback, queue):
        self.callback = callback

    def run_probe(self, cli_error=None, distro="jazzy", kind="setup"):
        with patch.dict(os.environ, self.environment, clear=True), \
                patch.dict(sys.modules, self.modules), \
                patch.object(health.shutil, "which", return_value=str(Path(self.prefix) / "bin/ros2")), \
                patch.object(health.subprocess, "run", return_value=types.SimpleNamespace(returncode=0),
                             side_effect=cli_error):
            result = health.inspect_installation(distro, self.prefix, kind)
            self.assertNotIn("ROS_DISCOVERY_SERVER", os.environ)
            self.assertNotIn("RMW_IMPLEMENTATION", os.environ)
            self.assertNotIn("FASTRTPS_DEFAULT_PROFILES_FILE", os.environ)
            self.assertEqual(os.environ["ROS_LOCALHOST_ONLY"], "1")
            return {check["id"]: check for check in result["checks"]}

    def test_successful_local_roundtrip_and_cleanup(self):
        checks = self.run_probe()
        self.assertTrue(all(check["status"] == "passed" for check in checks.values()))
        self.context.try_shutdown.assert_called_once()
        self.node.destroy_node.assert_called_once()
        self.executor.shutdown.assert_called_once()
        kwargs = self.rclpy.init.call_args.kwargs
        self.assertIs(kwargs["context"], self.context)
        self.assertTrue(20 <= kwargs["domain_id"] < 100)
        self.assertFalse(self.rclpy.create_node.call_args.kwargs["enable_rosout"])
        self.assertFalse(self.rclpy.create_node.call_args.kwargs["start_parameter_services"])

    def test_distribution_mismatch_never_initializes_rmw(self):
        checks = self.run_probe(distro="humble")
        self.assertEqual(checks["environment"]["status"], "failed")
        self.assertIn("Expected ROS_DISTRO=humble", checks["environment"]["detail"])
        self.rclpy.init.assert_not_called()

    def test_cli_timeout_and_native_initialization_error_are_explicit(self):
        self.rclpy.init.side_effect = RuntimeError("missing native RMW library")
        checks = self.run_probe(cli_error=subprocess.TimeoutExpired("ros2", 8))
        self.assertIn("TimeoutExpired", checks["ros2-cli"]["detail"])
        self.assertIn("missing native RMW", checks["rmw-init"]["detail"])
        self.assertEqual(checks["pub-sub"]["status"], "failed")
        self.context.try_shutdown.assert_called_once()

    def test_python_and_package_prefixes_cannot_resolve_to_another_installation(self):
        self.rclpy.__file__ = "/different/ros/rclpy/__init__.py"
        checks = self.run_probe()
        self.assertIn("outside the requested prefix", checks["rclpy-import"]["detail"])
        self.rclpy.init.assert_not_called()
        checks = self.run_probe(kind="pixi")
        self.assertIn("Python resolved outside", checks["prefix"]["detail"])

    def test_windows_pixi_uses_library_as_the_ros_prefix(self):
        self.environment["AMENT_PREFIX_PATH"] = str(Path(self.prefix) / "Library")
        self.environment["PIXI_ENVIRONMENT_NAME"] = "jazzy"
        with patch.object(health.sys, "platform", "win32"), \
                patch.object(health.sys, "executable", str(Path(self.prefix) / "python.exe")):
            checks = self.run_probe(kind="pixi")
        self.assertEqual(checks["prefix"]["status"], "passed")
        self.assertTrue(all(check["status"] == "passed" for check in checks.values()))

    def test_pubsub_failure_still_cleans_up_every_resource(self):
        self.executor.spin_once.side_effect = RuntimeError("spin failed")
        self.node.create_publisher.return_value.publish.side_effect = None
        checks = self.run_probe()
        self.assertIn("spin failed", checks["pub-sub"]["detail"])
        self.node.destroy_node.assert_called_once()
        self.context.try_shutdown.assert_called_once()
        self.executor.shutdown.assert_called_once()

    def test_missing_pubsub_message_has_a_deadline(self):
        self.node.create_publisher.return_value.publish.side_effect = None
        with patch.object(health.time, "monotonic", side_effect=range(0, 1000, 10)):
            checks = self.run_probe()
        self.assertIn("within 8 seconds", checks["pub-sub"]["detail"])
        self.context.try_shutdown.assert_called_once()

    def test_packages_must_be_from_the_requested_prefix(self):
        self.modules["ament_index_python.packages"].get_package_prefix = lambda name: "/other/ros"
        checks = self.run_probe()
        self.assertIn("outside requested prefix", checks["package-discovery"]["detail"])
        self.rclpy.init.assert_not_called()

    def test_unknown_middleware_is_not_initialized(self):
        self.modules["rclpy.utilities"].get_rmw_implementation_identifier = lambda: "rmw_zenoh_cpp"
        checks = self.run_probe()
        self.assertIn("does not support middleware", checks["rmw-init"]["detail"])
        self.rclpy.init.assert_not_called()

    def test_cleanup_failure_cannot_pass(self):
        self.node.destroy_node.side_effect = RuntimeError("destroy failed")
        checks = self.run_probe()
        self.assertEqual(checks["pub-sub"]["status"], "failed")
        self.assertIn("cleanup failed", checks["pub-sub"]["detail"])
        self.context.try_shutdown.assert_called_once()

    def test_prefix_containment_is_not_a_string_prefix_comparison(self):
        self.assertFalse(health.within(self.prefix + "_other/bin/ros2", self.prefix))
        self.assertTrue(health.within(str(Path(self.prefix) / "bin/ros2"), self.prefix))


if __name__ == "__main__":
    unittest.main()
