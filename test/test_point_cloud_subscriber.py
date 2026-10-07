"""PointCloud2 transport tests using native sensor_msgs; no live nodes needed."""

import importlib.util
import io
import json
from pathlib import Path
import struct
import threading
import unittest
from unittest.mock import Mock, call, patch

from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField


HELPER = Path(__file__).resolve().parents[1] / "assets/scripts/image_subscriber.py"
SPEC = importlib.util.spec_from_file_location("cloud_image_subscriber", HELPER)
subscriber = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(subscriber)


def parse_frame(raw):
  magic, metadata_length, payload_length = struct.unpack("<4sII", raw[:12])
  assert magic == b"RDEB"
  assert 0 < metadata_length <= 65536
  assert payload_length <= 32 * 1024 * 1024
  assert len(raw) == 12 + metadata_length + payload_length
  metadata = json.loads(raw[12:12 + metadata_length].decode("utf-8"))
  assert "data" not in metadata
  return metadata, raw[12 + metadata_length:]


def cloud_frame(message):
  output = io.BytesIO()
  subscriber.PointCloudStream(output).receive(message)
  return parse_frame(output.getvalue())


class EncodePointCloudTests(unittest.TestCase):
  def test_arbitrary_fields_endianness_organized_rows_and_padding_are_lossless(self):
    # Non-XYZ field order, vector fields, point padding, and row padding are opaque.
    raw = bytes(range(144))
    fields = [PointField(name="normal", offset=8, datatype=PointField.FLOAT32, count=3),
              PointField(name="intensity", offset=0, datatype=PointField.UINT16, count=1),
              PointField(name="label", offset=24, datatype=PointField.INT8, count=4)]
    for bigendian in (False, True):
      for dense in (False, True):
        with self.subTest(bigendian=bigendian, dense=dense):
          message = PointCloud2(width=2, height=2, fields=fields,
                                point_step=32, row_step=72, is_bigendian=bigendian,
                                is_dense=dense, data=raw)
          message.header.frame_id = "lidar 🚀"
          message.header.stamp.sec = -12
          message.header.stamp.nanosec = 345
          result, payload = cloud_frame(message)
          self.assertEqual(result, {
            "header": {"stamp": {"sec": -12, "nanosec": 345}, "frame_id": "lidar 🚀"},
            "width": 2, "height": 2,
            "fields": [{"name": f.name, "offset": f.offset, "datatype": f.datatype,
                        "count": f.count} for f in fields],
            "is_bigendian": bigendian, "point_step": 32, "row_step": 72,
            "is_dense": dense,
          })
          self.assertEqual(result, subscriber.encode_point_cloud(message))
          self.assertEqual(payload, raw)
          self.assertEqual(bytes(message.data), raw)

  def test_all_native_field_datatypes_and_empty_cloud_are_preserved(self):
    fields = [PointField(name=f"field_{datatype}", offset=datatype * 8,
                         datatype=datatype, count=2) for datatype in range(1, 9)]
    message = PointCloud2(fields=fields)
    result, payload = cloud_frame(message)
    self.assertEqual(payload, b"")
    self.assertEqual(result["width"], 0)
    self.assertEqual(result["height"], 0)
    self.assertEqual(result["fields"], [
      {"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count}
      for f in fields])

  def test_32_mib_cap_and_boundary_before_conversion(self):
    self.assertEqual(subscriber.MAX_POINT_CLOUD_BYTES, 32 * 1024 * 1024)
    with patch.object(subscriber, "MAX_POINT_CLOUD_BYTES", 3):
      self.assertEqual(cloud_frame(PointCloud2(data=b"abc"))[1], b"abc")
      message = PointCloud2(data=b"abcd")
      with patch.object(subscriber, "bytes", create=True) as convert:
        with self.assertRaisesRegex(ValueError, "32 MiB preview limit"):
          subscriber.encode_point_cloud(message)
        convert.assert_not_called()


class PointCloudStreamTests(unittest.TestCase):
  def test_default_and_valid_rates(self):
    self.assertEqual(subscriber.PointCloudStream(io.BytesIO()).rate, 1)
    for rate in (0.2, "0.2", 0.5, "1", 2.5, 5, "5"):
      with self.subTest(rate=rate):
        self.assertEqual(subscriber.valid_cloud_rate(rate), float(rate))
        self.assertEqual(subscriber.PointCloudStream(io.BytesIO(), rate).rate, float(rate))

  def test_invalid_rates(self):
    for rate in ("nan", "inf", "-inf", float("nan"), float("inf"),
                 -float("inf"), -1, 0, 0.199, 5.001, 30, "", "invalid", None):
      with self.subTest(rate=rate):
        with self.assertRaises((ValueError, TypeError)):
          subscriber.valid_cloud_rate(rate)
        with self.assertRaises((ValueError, TypeError)):
          subscriber.PointCloudStream(io.BytesIO(), rate)

  def test_throttle_precedes_metadata_and_pipe_writes_and_accepts_deadline(self):
    output = Mock(wraps=io.BytesIO())
    stream = subscriber.PointCloudStream(output, 2)
    message = PointCloud2(data=b"abc")
    with patch.object(subscriber.time, "monotonic", return_value=10):
      stream.receive(message)
    self.assertEqual(stream.next_frame, 10.5)
    output.reset_mock()
    with patch.object(subscriber.time, "monotonic", return_value=10.49), \
         patch.object(subscriber, "bytes", create=True) as convert, \
         patch.object(subscriber, "write_frame") as write, \
         patch.object(subscriber.json, "dumps") as dumps, \
         patch.object(subscriber, "encode_point_cloud") as frame:
      stream.receive(message)
      for operation in (convert, write, dumps, frame):
        operation.assert_not_called()
      self.assertEqual(output.mock_calls, [])
    with patch.object(subscriber.time, "monotonic", return_value=10.5):
      stream.receive(message)
    self.assertEqual(stream.next_frame, 11)
    self.assertEqual(output.write.call_count, 3)

  def test_large_frames_obey_raw_eight_mib_per_second_budget(self):
    self.assertEqual(subscriber.POINT_CLOUD_BYTES_PER_SECOND, 8 * 1024 * 1024)
    for size, rate, interval in ((16 * 1024 * 1024, 5, 2),
                                 (32 * 1024 * 1024, 5, 4),
                                 (16 * 1024 * 1024, 0.2, 5)):
      with self.subTest(size=size, rate=rate):
        message = PointCloud2(data=b"\xff" * size)
        stream = subscriber.PointCloudStream(io.BytesIO(), rate)
        with patch.object(subscriber.time, "monotonic", side_effect=[10, 10 + interval - 0.01,
                                                                    10 + interval]), \
             patch.object(subscriber, "encode_point_cloud", return_value={}) as encode:
          stream.receive(message)
          self.assertEqual(stream.next_frame, 10 + interval)
          stream.receive(message)
          encode.assert_called_once_with(message)
          stream.receive(message)
          self.assertEqual(encode.call_count, 2)
          self.assertEqual(stream.next_frame, 10 + 2 * interval)

  def test_repeated_rate_controls_cannot_reset_budget_deadline(self):
    message = PointCloud2(data=b"\x00" * (16 * 1024 * 1024))
    stream = subscriber.PointCloudStream(io.BytesIO(), 5)
    with patch.object(subscriber.time, "monotonic", return_value=10), \
         patch.object(subscriber, "encode_point_cloud", return_value={}):
      stream.receive(message)
    self.assertEqual(stream.next_frame, 12)
    for rate in (0.2, 5, 1, 5):
      stopped = threading.Event()
      with patch.object(subscriber.sys, "stdin", io.StringIO(json.dumps({"rateHz": rate}) + "\n")):
        subscriber.read_controls(stream, stopped)
      self.assertEqual(stream.rate, rate)
      self.assertEqual(stream.next_frame, 12)
      self.assertTrue(stopped.is_set())
      with patch.object(subscriber.time, "monotonic", return_value=11.9), \
           patch.object(subscriber, "encode_point_cloud") as encode:
        stream.receive(message)
        encode.assert_not_called()
    with patch.object(subscriber.time, "monotonic", return_value=12), \
         patch.object(subscriber, "encode_point_cloud", return_value={}) as encode:
      stream.receive(message)
      encode.assert_called_once_with(message)
    self.assertEqual(stream.next_frame, 14)

  def test_oversize_is_rejected_before_encoding_or_output(self):
    output = Mock()
    stream = subscriber.PointCloudStream(output)
    message = PointCloud2(data=b"abcd")
    with patch.object(subscriber, "MAX_POINT_CLOUD_BYTES", 3), \
         patch.object(subscriber, "encode_point_cloud") as encode:
      with self.assertRaisesRegex(ValueError, "preview limit"):
        stream.receive(message)
      encode.assert_not_called()
      self.assertEqual(output.mock_calls, [])

  def test_synchronous_binary_write_and_flush_no_queue_or_payload_copy(self):
    buffer = io.BytesIO()
    output = Mock(wraps=buffer)
    message = PointCloud2(data=b"\x00\xff")
    subscriber.PointCloudStream(output).receive(message)
    self.assertEqual(parse_frame(buffer.getvalue()),
             (subscriber.encode_point_cloud(message), b"\x00\xff"))
    writes = output.write.call_args_list
    self.assertEqual(len(writes), 3)
    self.assertEqual(len(writes[0].args[0]), 12)
    self.assertIsInstance(writes[2].args[0], memoryview)
    self.assertIs(writes[2].args[0].obj, message.data)
    self.assertEqual(output.mock_calls[-1], call.flush())
    output = Mock()
    output.write.side_effect = BrokenPipeError
    with self.assertRaises(BrokenPipeError):
      subscriber.PointCloudStream(output).receive(message)
    output.flush.assert_not_called()

  def test_invalid_controls_keep_rate_and_deadline_and_continue(self):
    for line in ("not json", "{}", "null", "[]", '{"rateHz": null}',
                 '{"rateHz": "nan"}', '{"rateHz": "inf"}',
                 '{"rateHz": 0.19}', '{"rateHz": 5.01}', '{"rateHz": 30}'):
      for suffix in ("", '{"rateHz": 0.2}\n'):
        with self.subTest(line=line, suffix=suffix):
          stream = subscriber.PointCloudStream(io.BytesIO())
          stream.next_frame = 100
          stopped = threading.Event()
          with patch.object(subscriber.sys, "stdin", io.StringIO(line + "\n" + suffix)), \
               patch.object(subscriber.sys, "stderr", io.StringIO()) as errors:
            subscriber.read_controls(stream, stopped)
          self.assertEqual((stream.rate, stream.next_frame), (0.2 if suffix else 1, 100))
          self.assertIn("Ignored invalid PointCloud2 refresh control", errors.getvalue())
          self.assertTrue(stopped.is_set())


class SubscriberCliTests(unittest.TestCase):
  def test_source_type_defaults_explicit_rates_and_latest_only_qos(self):
    for message_type, stream_type, default in (
        (PointCloud2, subscriber.PointCloudStream, 1),
        (Image, subscriber.ImageStream, 5),
        (CompressedImage, subscriber.ImageStream, 5)):
      for explicit in (None, "0.2" if message_type is PointCloud2 else "30"):
        with self.subTest(message_type=message_type.__name__, rate=explicit):
          args = [str(HELPER), "--topic", "/sensor", "--type", f"sensor_msgs/msg/{message_type.__name__}"]
          if explicit is not None:
            args += ["--rate", explicit]
          output = io.TextIOWrapper(io.BytesIO(), encoding="utf-8")
          errors = io.StringIO()
          node = Mock()
          with patch.object(subscriber.sys, "argv", args), \
               patch.object(subscriber.sys, "stdout", output), \
               patch.object(subscriber.sys, "stderr", errors), \
               patch.object(subscriber.rclpy, "init", side_effect=lambda: print("ROS diagnostic")) as initialize, \
               patch.object(subscriber.rclpy, "create_node", return_value=node), \
               patch.object(subscriber.rclpy, "ok", side_effect=[True, False, True]), \
               patch.object(subscriber.rclpy, "spin_once") as spin, \
               patch.object(subscriber.rclpy, "shutdown") as shutdown, \
               patch.object(subscriber.threading, "Thread") as thread:
            subscriber.main()
          initialize.assert_called_once_with()
          selected_type, topic, callback, qos = node.create_subscription.call_args.args
          self.assertIs(selected_type, message_type)
          self.assertEqual(topic, "/sensor")
          self.assertIsInstance(callback.__self__, stream_type)
          self.assertIs(callback.__self__.output, output.buffer)
          self.assertEqual(errors.getvalue(), "ROS diagnostic\n")
          self.assertEqual(output.buffer.getvalue(), b"")
          self.assertEqual(callback.__self__.rate, default if explicit is None else float(explicit))
          self.assertEqual(qos.history, subscriber.HistoryPolicy.KEEP_LAST)
          self.assertEqual(qos.depth, 1)
          self.assertEqual(qos.reliability, subscriber.ReliabilityPolicy.BEST_EFFORT)
          self.assertEqual(qos.durability, subscriber.DurabilityPolicy.VOLATILE)
          spin.assert_called_once_with(node, timeout_sec=0.2)
          self.assertIs(thread.call_args.kwargs["target"], subscriber.read_controls)
          self.assertTrue(thread.call_args.kwargs["daemon"])
          thread.return_value.start.assert_called_once_with()
          node.destroy_node.assert_called_once_with()
          shutdown.assert_called_once_with()

  def test_cli_rejects_wrong_type_and_type_specific_invalid_rates_before_ros_init(self):
    for message_type, rate in (("PointCloud2", "0.19"), ("PointCloud2", "5.1"),
                               ("PointCloud2", "nan"), ("PointCloud2", "inf"),
                               ("PointCloud2", "invalid"), ("Image", "0.2"),
                               ("Image", "31"), ("CompressedImage", "0.2"),
                               ("PointCloud", "1")):
      with self.subTest(message_type=message_type, rate=rate):
        args = [str(HELPER), "--rate", rate, "--topic", "/sensor",
                "--type", f"sensor_msgs/msg/{message_type}"]
        with patch.object(subscriber.sys, "argv", args), \
             patch.object(subscriber.sys, "stderr", io.StringIO()), \
             patch.object(subscriber.rclpy, "init") as initialize:
          with self.assertRaises(SystemExit) as error:
            subscriber.main()
          self.assertEqual(error.exception.code, 2)
          initialize.assert_not_called()


if __name__ == "__main__":
  unittest.main()