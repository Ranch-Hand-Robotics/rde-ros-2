"""Image stream tests for an activated native ROS environment; no nodes needed."""

import base64
import importlib.util
import io
import json
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock, call, patch

from sensor_msgs.msg import CompressedImage, Image


HELPER = Path(__file__).resolve().parents[1] / "assets/scripts/image_subscriber.py"
SPEC = importlib.util.spec_from_file_location("image_subscriber", HELPER)
subscriber = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(subscriber)


class EncodeImageTests(unittest.TestCase):
  def test_rgb_and_bgr_preserve_metadata_and_exact_bytes(self):
    raw = bytes([0, 127, 255, 3, 2, 1])
    for encoding in ("rgb8", "bgr8"):
      with self.subTest(encoding=encoding):
        message = Image(width=2, height=1, encoding=encoding, step=6,
                        is_bigendian=0, data=raw)
        message.header.stamp.sec = 12
        message.header.stamp.nanosec = 345
        message.header.frame_id = "camera_color"
        self.assertEqual(subscriber.encode_image(message), {
          "header": {"stamp": {"sec": 12, "nanosec": 345},
                     "frame_id": "camera_color"},
          "width": 2, "height": 1, "encoding": encoding,
          "step": 6, "is_bigendian": 0,
          "data": base64.b64encode(raw).decode("ascii"),
        })

  def test_padded_big_endian_depth_preserves_layout_and_padding(self):
    raw = b"\x00\x01\x01\x00\xaa\xbb\x12\x34\xff\xff\xcc\xdd"
    message = Image(width=2, height=2, encoding="16UC1", step=6,
                    is_bigendian=1, data=raw)
    result = subscriber.encode_image(message)
    self.assertEqual((result["width"], result["height"], result["encoding"],
                      result["step"], result["is_bigendian"]),
                     (2, 2, "16UC1", 6, 1))
    self.assertEqual(result["data"], base64.b64encode(raw).decode("ascii"))
    self.assertEqual(base64.b64decode(result["data"]), raw)

  def test_compressed_preserves_format_header_and_bytes(self):
    raw = b"\xff\xd8\x00\x80\xff\xd9"
    message = CompressedImage(format="rgb8; jpeg compressed bgr8", data=raw)
    message.header.frame_id = "camera_compressed"
    message.header.stamp.sec = 7
    message.header.stamp.nanosec = 99
    self.assertEqual(subscriber.encode_image(message), {
      "header": {"stamp": {"sec": 7, "nanosec": 99},
                 "frame_id": "camera_compressed"},
      "format": message.format,
      "data": base64.b64encode(raw).decode("ascii"),
    })

  def test_zero_length_data(self):
    for message_type in (Image, CompressedImage):
      with self.subTest(message_type=message_type.__name__):
        self.assertEqual(subscriber.encode_image(message_type(data=b""))["data"], "")

  def test_size_cap_accepts_boundary_and_rejects_overflow_before_base64(self):
    with patch.object(subscriber, "MAX_IMAGE_BYTES", 3):
      for message_type in (Image, CompressedImage):
        with self.subTest(message_type=message_type.__name__):
          self.assertEqual(subscriber.encode_image(message_type(data=b"abc"))["data"],
                           "YWJj")
          with patch.object(subscriber.base64, "b64encode") as encode:
            with self.assertRaisesRegex(ValueError, "preview limit"):
              subscriber.encode_image(message_type(data=b"abcd"))
            encode.assert_not_called()


class ImageStreamTests(unittest.TestCase):
  def test_valid_rates_including_boundaries(self):
    for value in (1, "1", 2.5, "5.5", 30, "30"):
      with self.subTest(value=value):
        self.assertEqual(subscriber.valid_rate(value), float(value))
        self.assertEqual(subscriber.ImageStream(io.StringIO(), value).rate, float(value))

  def test_invalid_rates(self):
    for value in ("nan", "inf", "-inf", float("nan"), float("inf"),
                  -float("inf"), -1, 0, 0.99, 30.01, 31, "", "invalid"):
      with self.subTest(value=value):
        with self.assertRaises(ValueError):
          subscriber.valid_rate(value)
        with self.assertRaises(ValueError):
          subscriber.ImageStream(io.StringIO(), value)

  def test_throttle_happens_before_encode_and_accepts_deadline(self):
    output = Mock()
    stream = subscriber.ImageStream(output, 2)
    message = Image()
    with patch.object(subscriber.time, "monotonic", side_effect=[10, 10.49, 10.5]), \
         patch.object(subscriber, "encode_image", return_value={}) as encode:
      stream.receive(message)
      encode.assert_called_once_with(message)
      self.assertEqual(stream.next_frame, 10.5)
      encode.reset_mock()
      output.reset_mock()
      stream.receive(message)
      encode.assert_not_called()
      self.assertEqual(output.mock_calls, [])
      self.assertEqual(stream.next_frame, 10.5)
      stream.receive(message)
      encode.assert_called_once_with(message)
      self.assertEqual(stream.next_frame, 11)

  def test_stdout_protocol_writes_json_line_then_flushes(self):
    buffer = io.StringIO()
    output = Mock(wraps=buffer)
    message = Image(data=b"\x00\xff")
    with patch.object(subscriber.sys, "stdout", output), \
         patch.object(subscriber.time, "monotonic", return_value=1):
      subscriber.ImageStream(subscriber.sys.stdout, 5).receive(message)
    line = buffer.getvalue()
    self.assertEqual(json.loads(line), subscriber.encode_image(message))
    self.assertTrue(line.endswith("\n"))
    self.assertEqual(len(line.splitlines()), 1)
    self.assertEqual(output.mock_calls, [call.write(line), call.flush()])

  def test_refresh_updates_rate_resets_throttle_and_eof_stops(self):
    stream = subscriber.ImageStream(io.StringIO(), 5)
    stream.next_frame = 100
    stopped = threading.Event()
    with patch.object(subscriber.sys, "stdin", io.StringIO('{"rateHz": 12.5}\n')):
      subscriber.read_controls(stream, stopped)
    self.assertEqual(stream.rate, 12.5)
    self.assertEqual(stream.next_frame, 0)
    self.assertTrue(stopped.is_set())

  def test_malformed_controls_preserve_state_and_continue(self):
    for line in ('not json', '{}', 'null', '[]', '{"rateHz": null}',
                 '{"rateHz": "nan"}', '{"rateHz": "inf"}', '{"rateHz": 31}'):
      for suffix in ("", '{"rateHz": 1}\n'):
        with self.subTest(line=line, suffix=suffix):
          stream = subscriber.ImageStream(io.StringIO(), 5)
          stream.next_frame = 100
          stopped = threading.Event()
          with patch.object(subscriber.sys, "stdin", io.StringIO(line + "\n" + suffix)), \
               patch.object(subscriber.sys, "stderr", io.StringIO()) as errors:
            subscriber.read_controls(stream, stopped)
          self.assertEqual((stream.rate, stream.next_frame), (1, 0) if suffix else (5, 100))
          self.assertIn("Ignored invalid image refresh control", errors.getvalue())
          self.assertTrue(stopped.is_set())


if __name__ == "__main__":
  unittest.main()