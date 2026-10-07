# Copyright (c) Ranch Hand Robotics. All rights reserved.
# Licensed under the MIT License.

"""Stream ROS images and point clouds as bounded metadata + raw binary frames."""

import argparse
import json
import math
import os
import struct
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, PointCloud2


MAX_IMAGE_BYTES = 32 * 1024 * 1024
MAX_POINT_CLOUD_BYTES = 32 * 1024 * 1024
POINT_CLOUD_BYTES_PER_SECOND = 8 * 1024 * 1024
MAX_METADATA_BYTES = 64 * 1024
MAX_PAYLOAD_BYTES = 32 * 1024 * 1024
FRAME_HEADER = struct.Struct("<4sII")


def write_frame(output, metadata, data):
    """RDEB + uint32 LE lengths + UTF-8 JSON (no data) + opaque ROS bytes.

    Keep the native uint8 array alive through synchronous writes. No payload
    encoding or copy is needed; short pipe writes advance a memoryview only.
    """
    payload = memoryview(data)
    if payload.nbytes > MAX_PAYLOAD_BYTES:
        raise ValueError("Payload exceeds the 32 MiB preview limit")
    if not isinstance(metadata, dict) or "data" in metadata:
        raise ValueError("Invalid subscriber metadata")
    encoded = json.dumps(metadata, separators=(",", ":"), ensure_ascii=False,
                         allow_nan=False).encode("utf-8")
    if len(encoded) > MAX_METADATA_BYTES:
        raise ValueError("Metadata exceeds the 64 KiB transport limit")
    header = FRAME_HEADER.pack(b"RDEB", len(encoded), payload.nbytes)
    for part in (memoryview(header), memoryview(encoded), payload):
        while part:
            written = output.write(part)
            if not isinstance(written, int) or written <= 0 or written > part.nbytes:
                raise BrokenPipeError("Incomplete subscriber frame write")
            part = part[written:]
    output.flush()


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
    }
    if isinstance(message, CompressedImage):
        result["format"] = message.format
    else:
        result.update(width=message.width, height=message.height, encoding=message.encoding,
                      step=message.step, is_bigendian=message.is_bigendian)
    return result


def valid_cloud_rate(value):
    rate = float(value)
    if not math.isfinite(rate) or not 0.2 <= rate <= 5:
        raise ValueError("PointCloud2 refresh rate must be between 0.2 and 5 Hz")
    return rate


def encode_point_cloud(message):
    if len(message.data) > MAX_POINT_CLOUD_BYTES:
        raise ValueError("PointCloud2 exceeds the 32 MiB preview limit")
    return {
        "header": {
            "stamp": {"sec": message.header.stamp.sec, "nanosec": message.header.stamp.nanosec},
            "frame_id": message.header.frame_id,
        },
        "width": message.width,
        "height": message.height,
        "fields": [{"name": field.name, "offset": field.offset,
                    "datatype": field.datatype, "count": field.count}
                   for field in message.fields],
        "is_bigendian": message.is_bigendian,
        "point_step": message.point_step,
        "row_step": message.row_step,
        "is_dense": message.is_dense,
    }


class PointCloudStream:
    def __init__(self, output, rate=1.0):
        self.output = output
        self.rate = valid_cloud_rate(rate)
        self.next_frame = 0.0

    def receive(self, message):
        now = time.monotonic()
        if now < self.next_frame:
            return
        size = len(message.data)
        if size > MAX_POINT_CLOUD_BYTES:
            raise ValueError("PointCloud2 exceeds the 32 MiB preview limit")
        # Charge the raw payload before metadata serialization or pipe writes. Controls must
        # never reset this deadline, even when the user repeatedly changes rate.
        self.next_frame = now + max(1.0 / self.rate, size / POINT_CLOUD_BYTES_PER_SECOND)
        # Blocking pipe writes and DDS KEEP_LAST(1) avoid an application queue.
        write_frame(self.output, encode_point_cloud(message), message.data)


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
        write_frame(self.output, encode_image(message), message.data)


def read_controls(stream, stopped):
    cloud = isinstance(stream, PointCloudStream)
    validate = valid_cloud_rate if cloud else valid_rate
    try:
        while not stopped.is_set():
            line = sys.stdin.readline(4096)
            if not line:
                break  # Extension exited or closed the subscription.
            try:
                control = json.loads(line)
                stream.rate = validate(control["rateHz"])
                if not cloud:
                    stream.next_frame = 0.0
            except (ValueError, TypeError, KeyError):
                kind = "PointCloud2" if cloud else "image"
                print(f"Ignored invalid {kind} refresh control", file=sys.stderr)
    finally:
        stopped.set()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--type", required=True, choices=["sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage",
                                                       "sensor_msgs/msg/PointCloud2"])
    parser.add_argument("--rate")
    args = parser.parse_args()
    cloud = args.type == "sensor_msgs/msg/PointCloud2"
    validate = valid_cloud_rate if cloud else valid_rate
    try:
        rate = validate(args.rate if args.rate is not None else (1.0 if cloud else 5.0))
    except ValueError as error:
        parser.error(str(error))
    output = sys.stdout.buffer
    sys.stdout = sys.stderr  # Only the stream may write to the protocol pipe.
    stream = PointCloudStream(output, rate) if cloud else ImageStream(output, rate)
    stopped = threading.Event()
    node = None
    rclpy.init()
    try:
        node = rclpy.create_node(("_rde_point_cloud_" if cloud else "_rde_image_") + str(os.getpid()))
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        message_type = PointCloud2 if cloud else (Image if args.type == "sensor_msgs/msg/Image" else CompressedImage)
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