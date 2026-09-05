#!/usr/bin/env python3
"""Read-only installation checks, not a C++/GPU/network or extension-detection test."""

import argparse
import importlib
import json
import os
from pathlib import Path
import secrets
import shutil
import subprocess
import sys
import time


SENTINEL = "RDE_ROS_HEALTH:"
CHECK_IDS = (
    "environment", "prefix", "ros2-cli", "package-discovery",
    "rclpy-import", "rmw-init", "pub-sub",
)


def within(filename, prefix):
    try:
        Path(filename).resolve().relative_to(Path(prefix).resolve())
        return True
    except (ValueError, TypeError):
        return False


def require(condition, detail):
    if not condition:
        raise RuntimeError(detail)


def local_environment():
    # Set again after activation: a setup script must not redirect discovery to a robot.
    for key in list(os.environ):
        if (key.startswith(("ROS_SECURITY_", "FASTDDS_", "FASTRTPS_", "CYCLONEDDS_"))
                or key in ("ROS_DISCOVERY_SERVER", "ROS_STATIC_PEERS", "ROS_DOMAIN_ID",
                           "ZENOH_CONFIG_OVERRIDE",
                           "RMW_ZENOH_CONFIG_URI", "ZENOH_ROUTER_CONFIG_URI",
                           "ZENOH_SESSION_CONFIG_URI")):
            os.environ.pop(key, None)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["ROS_STATIC_PEERS"] = ""
    os.environ["ROS2CLI_NO_DAEMON"] = "1"
    # Prevent Fast DDS from loading a profile from the working directory.
    os.environ["SKIP_DEFAULT_XML"] = "1"


def inspect_installation(distro, prefix, kind):
    checks = []
    state = {}

    def check(check_id, action, dependencies=()):
        started = time.monotonic()
        try:
            for dependency in dependencies:
                require(state.get(dependency), "Not run: prerequisite {} failed".format(dependency))
            detail = action()
            status = "passed"
        except Exception as error:
            detail = "{}: {}".format(type(error).__name__, error)
            status = "failed"
        state[check_id] = status == "passed"
        checks.append({
            "id": check_id, "status": status, "detail": str(detail)[:8192],
            "durationMs": round((time.monotonic() - started) * 1000, 3),
        })

    def environment():
        require(os.environ.get("ROS_VERSION") == "2",
                "Expected ROS_VERSION=2, got {!r}".format(os.environ.get("ROS_VERSION")))
        require(os.environ.get("ROS_DISTRO") == distro,
                "Expected ROS_DISTRO={}, got {!r}".format(distro, os.environ.get("ROS_DISTRO")))
        return "ROS_VERSION=2; ROS_DISTRO={}".format(distro)

    def prefix_check():
        prefixes = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
        ros_prefix = Path(prefix) / "Library" if kind == "pixi" and sys.platform == "win32" else Path(prefix)
        require(any(p and Path(p).resolve() == ros_prefix.resolve() for p in prefixes),
                "Requested prefix is absent from AMENT_PREFIX_PATH: {}".format(prefix))
        if kind == "pixi":
            require(within(sys.executable, prefix),
                    "Python resolved outside the requested Pixi environment: {}".format(sys.executable))
            require(os.environ.get("PIXI_ENVIRONMENT_NAME") == distro,
                    "Pixi did not activate the requested named environment")
        ros2 = shutil.which("ros2")
        require(ros2 and within(ros2, prefix),
                "ros2 executable is missing or outside the requested prefix: {}".format(ros2))
        state["ros2"] = ros2
        return "prefix={}; python={}; ros2={}".format(prefix, sys.executable, ros2)

    def cli():
        # Outer supervisor bounds captured output and kills this process group on timeout.
        result = subprocess.run([state["ros2"], "--help"], timeout=8, check=False)
        require(result.returncode == 0, "ros2 --help exited with code {}".format(result.returncode))
        return "ros2 --help exited with code 0"

    def packages():
        from ament_index_python.packages import get_package_prefix
        for name in ("rclpy", "std_msgs"):
            resolved = get_package_prefix(name)
            require(within(resolved, prefix),
                    "{} package resolved outside requested prefix: {}".format(name, resolved))
        return "ament package discovery found rclpy and std_msgs in the requested prefix"

    def python_import():
        module = importlib.import_module("rclpy")
        require(within(module.__file__, prefix),
                "rclpy imported from outside the requested prefix: {}".format(module.__file__))
        messages = importlib.import_module("std_msgs.msg")
        require(within(messages.__file__, prefix),
                "std_msgs imported from outside the requested prefix: {}".format(messages.__file__))
        state["rclpy"] = module
        state["String"] = messages.String
        return "Imported rclpy and std_msgs from {}".format(prefix)

    context = None
    node = None
    executor = None

    def rmw():
        nonlocal context
        from rclpy.context import Context
        from rclpy.utilities import get_rmw_implementation_identifier
        implementation = get_rmw_implementation_identifier()
        # The standard localhost controls have defined support for these DDS backends.
        # Do not initialize arbitrary transports (e.g. Zenoh routers) in a local-only test.
        require(implementation in ("rmw_fastrtps_cpp", "rmw_fastrtps_dynamic_cpp", "rmw_cyclonedds_cpp"),
                "Local-only health probe does not support middleware {}".format(implementation))
        context = Context()
        domain = 20 + secrets.randbelow(80)
        state["rclpy"].init(args=[], context=context, domain_id=domain)
        state["domain"] = domain
        return "Native RMW initialized: {}; isolated localhost domain {}".format(implementation, domain)

    def pubsub():
        nonlocal node, executor
        from rclpy.executors import SingleThreadedExecutor
        token = secrets.token_hex(12)
        topic = "/rde_install_health_{}".format(token)
        node = state["rclpy"].create_node(
            "rde_install_health_{}".format(token), context=context,
            enable_rosout=False, start_parameter_services=False,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        received = []
        node.create_subscription(state["String"], topic, lambda msg: received.append(msg.data), 1)
        publisher = node.create_publisher(state["String"], topic, 1)
        message = state["String"]()
        message.data = token
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            publisher.publish(message)
            executor.spin_once(timeout_sec=0.1)
            if token in received:
                return "std_msgs/String round-trip on a unique localhost topic (domain {})".format(state["domain"])
        raise RuntimeError("No matching local pub/sub message received within 8 seconds")

    local_environment()
    check("environment", environment)
    check("prefix", prefix_check, ("environment",))
    check("ros2-cli", cli, ("prefix",))
    check("package-discovery", packages, ("prefix",))
    check("rclpy-import", python_import, ("package-discovery",))
    try:
        check("rmw-init", rmw, ("rclpy-import",))
        check("pub-sub", pubsub, ("rmw-init",))
    finally:
        cleanup_errors = []
        for resource, method in ((executor, "shutdown"), (node, "destroy_node"), (context, "try_shutdown")):
            if resource is not None:
                try:
                    getattr(resource, method)()
                except Exception as error:
                    cleanup_errors.append(str(error))
        if cleanup_errors:
            checks[-1]["status"] = "failed"
            checks[-1]["detail"] += "; cleanup failed: " + "; ".join(cleanup_errors)[:2048]
    return {"version": 1, "distro": distro, "checks": checks}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--distro", required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--kind", choices=("setup", "pixi"), required=True)
    args = parser.parse_args()
    result = inspect_installation(args.distro, args.prefix, args.kind)
    print("\n" + SENTINEL + json.dumps(result), flush=True)
    return 0 if all(item["status"] == "passed" for item in result["checks"]) else 1


if __name__ == "__main__":
    sys.exit(main())
