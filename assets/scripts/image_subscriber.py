# Copyright (c) Ranch Hand Robotics. All rights reserved.
# Licensed under the MIT License.

"""Stream ROS images as bounded JSON/base64 frames, not byte-by-byte YAML."""

import argparse
import base64
import json
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


MAX_IMAGE_BYTES = 32 * 1024 * 1024


def valid_rate(value):
    rate = float(value)
    if not math.isfinite(rate) or not 1 <= rate <= 30:
        raise ValueError("Image refresh rate must be between 1 and 30 Hz")
    return rate


def encode_image(message):
    if len(message.data) > MAX_IMAGE_BYTES:
        raise ValueError("Image exceeds the 32 MiB preview limit")
    result = {
        "header": {
            "stamp": {"sec": message.header.stamp.sec, "nanosec": message.header.stamp.nanosec},
            "frame_id": message.header.frame_id,
        },
        "data": base64.b64encode(bytes(message.data)).decode("ascii"),
    }
    if isinstance(message, CompressedImage):
        result["format"] = message.format
    else:
        result.update(width=message.width, height=message.height, encoding=message.encoding,
                      step=message.step, is_bigendian=message.is_bigendian)
    return result


class ImageStream:
    def __init__(self, output, rate):
        self.output = output
        self.rate = valid_rate(rate)
        self.next_frame = 0.0

    def receive(self, message):
        now = time.monotonic()
        if now < self.next_frame:
            return
        self.next_frame = now + 1.0 / self.rate
        # Synchronous writes provide pipe backpressure. DDS keeps only the latest
        # frame while the consumer is busy; there is no Python frame queue.
        self.output.write(json.dumps(encode_image(message), separators=(",", ":")) + "\n")
        self.output.flush()


def read_controls(stream, stopped):
    try:
        while not stopped.is_set():
            line = sys.stdin.readline(4096)
            if not line:
                break  # Extension exited or closed the subscription.
            try:
                control = json.loads(line)
                stream.rate = valid_rate(control["rateHz"])
                stream.next_frame = 0.0
            except (ValueError, TypeError, KeyError):
                print("Ignored invalid image refresh control", file=sys.stderr)
    finally:
        stopped.set()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--type", required=True, choices=["sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"])
    parser.add_argument("--rate", type=valid_rate, default=5.0)
    args = parser.parse_args()
    output = sys.stdout
    sys.stdout = sys.stderr  # Only ImageStream may write to the protocol pipe.
    stream = ImageStream(output, args.rate)
    stopped = threading.Event()
    node = None
    rclpy.init()
    try:
        node = rclpy.create_node("_rde_image_" + str(os.getpid()))
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        message_type = Image if args.type == "sensor_msgs/msg/Image" else CompressedImage
        node.create_subscription(message_type, args.topic, stream.receive, qos)
        threading.Thread(target=read_controls, args=(stream, stopped), daemon=True).start()
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException, BrokenPipeError):
        pass
    finally:
        stopped.set()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()