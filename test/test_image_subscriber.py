"""Image stream tests for an activated native ROS environment; no nodes needed."""

import importlib.util
import io
import json
from pathlib import Path
import struct
import threading
import unittest
from unittest.mock import Mock, call, patch

from sensor_msgs.msg import CompressedImage, Image


HELPER = Path(__file__).resolve().parents[1] / "assets/scripts/image_subscriber.py"
SPEC = importlib.util.spec_from_file_location("image_subscriber", HELPER)
subscriber = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(subscriber)


def parse_frames(raw):
  frames = []
  while raw:
    magic, metadata_length, payload_length = struct.unpack("<4sII", raw[:12])
    assert magic == b"RDEB"
    assert 0 < metadata_length <= 64 * 1024
    assert payload_length <= 32 * 1024 * 1024
    end = 12 + metadata_length + payload_length
    assert end <= len(raw)
    metadata = json.loads(raw[12:12 + metadata_length].decode("utf-8"))
    assert "data" not in metadata
    frames.append((metadata, raw[12 + metadata_length:end]))
    raw = raw[end:]
  return frames


def image_frame(message):
  output = io.BytesIO()
  subscriber.ImageStream(output, 5).receive(message)
  return parse_frames(output.getvalue())[0]


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
        metadata, payload = image_frame(message)
        self.assertEqual(metadata, {
          "header": {"stamp": {"sec": 12, "nanosec": 345},
                     "frame_id": "camera_color"},
          "width": 2, "height": 1, "encoding": encoding,
          "step": 6, "is_bigendian": 0,
        })
        self.assertEqual(metadata, subscriber.encode_image(message))
        self.assertEqual(payload, raw)

  def test_padded_big_endian_depth_preserves_layout_and_padding(self):
    raw = b"\x00\x01\x01\x00\xaa\xbb\x12\x34\xff\xff\xcc\xdd"
    message = Image(width=2, height=2, encoding="16UC1", step=6,
                    is_bigendian=1, data=raw)
    result, payload = image_frame(message)
    self.assertEqual((result["width"], result["height"], result["encoding"],
                      result["step"], result["is_bigendian"]),
                     (2, 2, "16UC1", 6, 1))
    self.assertEqual(payload, raw)

  def test_compressed_preserves_format_header_and_bytes(self):
    raw = b"\xff\xd8\x00\x80\xff\xd9"
    message = CompressedImage(format="rgb8; jpeg compressed bgr8", data=raw)
    message.header.frame_id = "camera_compressed"
    message.header.stamp.sec = 7
    message.header.stamp.nanosec = 99
    metadata, payload = image_frame(message)
    self.assertEqual(metadata, {
      "header": {"stamp": {"sec": 7, "nanosec": 99},
                 "frame_id": "camera_compressed"},
      "format": message.format,
    })
    self.assertEqual(payload, raw)

  def test_zero_length_data(self):
    for message_type in (Image, CompressedImage):
      with self.subTest(message_type=message_type.__name__):
        metadata, payload = image_frame(message_type(data=b""))
        self.assertNotIn("data", metadata)
        self.assertEqual(payload, b"")

  def test_size_cap_accepts_boundary_and_rejects_overflow_before_output(self):
    with patch.object(subscriber, "MAX_IMAGE_BYTES", 3):
      for message_type in (Image, CompressedImage):
        with self.subTest(message_type=message_type.__name__):
          self.assertEqual(image_frame(message_type(data=b"abc"))[1], b"abc")
          with patch.object(subscriber, "write_frame") as write:
            with self.assertRaisesRegex(ValueError, "preview limit"):
              subscriber.ImageStream(io.BytesIO(), 5).receive(message_type(data=b"abcd"))
            write.assert_not_called()


class ImageStreamTests(unittest.TestCase):
  def test_valid_rates_including_boundaries(self):
    for value in (1, "1", 2.5, "5.5", 30, "30"):
      with self.subTest(value=value):
        self.assertEqual(subscriber.valid_rate(value), float(value))
        self.assertEqual(subscriber.ImageStream(io.BytesIO(), value).rate, float(value))

  def test_invalid_rates(self):
    for value in ("nan", "inf", "-inf", float("nan"), float("inf"),
                  -float("inf"), -1, 0, 0.99, 30.01, 31, "", "invalid"):
      with self.subTest(value=value):
        with self.assertRaises(ValueError):
          subscriber.valid_rate(value)
        with self.assertRaises(ValueError):
          subscriber.ImageStream(io.BytesIO(), value)

  def test_throttle_happens_before_encode_and_accepts_deadline(self):
    output = Mock(wraps=io.BytesIO())
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

  def test_stdout_protocol_writes_header_metadata_and_native_memoryview_then_flushes(self):
    buffer = io.BytesIO()
    output = Mock(wraps=buffer)
    message = Image(data=b"\x00\xff")
    with patch.object(subscriber.time, "monotonic", return_value=1):
      subscriber.ImageStream(output, 5).receive(message)
    self.assertEqual(parse_frames(buffer.getvalue()), [(subscriber.encode_image(message), b"\x00\xff")])
    writes = output.write.call_args_list
    self.assertEqual(len(writes), 3)
    self.assertEqual(len(writes[0].args[0]), 12)
    self.assertIsInstance(writes[2].args[0], memoryview)
    self.assertIs(writes[2].args[0].obj, message.data, "Payload must not be copied")
    self.assertEqual(output.mock_calls[-1], call.flush())

  def test_refresh_updates_rate_resets_throttle_and_eof_stops(self):
    stream = subscriber.ImageStream(io.BytesIO(), 5)
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
          stream = subscriber.ImageStream(io.BytesIO(), 5)
          stream.next_frame = 100
          stopped = threading.Event()
          with patch.object(subscriber.sys, "stdin", io.StringIO(line + "\n" + suffix)), \
               patch.object(subscriber.sys, "stderr", io.StringIO()) as errors:
            subscriber.read_controls(stream, stopped)
          self.assertEqual((stream.rate, stream.next_frame), (1, 0) if suffix else (5, 100))
          self.assertIn("Ignored invalid image refresh control", errors.getvalue())
          self.assertTrue(stopped.is_set())


class BinaryFrameTests(unittest.TestCase):
  def test_unicode_metadata_byte_lengths_and_multiple_frames(self):
    message = Image(data=b"RDEB\x00\xff\n")
    message.header.frame_id = "camera 🚀 雪"
    output = io.BytesIO()
    stream = subscriber.ImageStream(output, 5)
    with patch.object(subscriber.time, "monotonic", side_effect=[1, 2]):
      stream.receive(message)
      stream.receive(message)
    raw = output.getvalue()
    _, metadata_length, payload_length = struct.unpack("<4sII", raw[:12])
    self.assertEqual(payload_length, len(message.data))
    self.assertIn("🚀".encode("utf-8"), raw[12:12 + metadata_length])
    self.assertEqual(parse_frames(raw), [(subscriber.encode_image(message), bytes(message.data))] * 2)

  def test_metadata_and_payload_caps_reject_before_any_write(self):
    self.assertEqual(subscriber.MAX_METADATA_BYTES, 65536)
    self.assertEqual(subscriber.MAX_PAYLOAD_BYTES, 32 * 1024 * 1024)
    for metadata in ({"frame": "x" * 65536}, {"data": "forbidden"}, [], {"value": float("nan")}):
      output = Mock()
      with self.assertRaises(ValueError):
        subscriber.write_frame(output, metadata, b"abc")
      self.assertEqual(output.mock_calls, [])
    output = Mock()
    with patch.object(subscriber, "MAX_PAYLOAD_BYTES", 2), \
         patch.object(subscriber.json, "dumps") as dumps:
      with self.assertRaisesRegex(ValueError, "preview limit"):
        subscriber.write_frame(output, {}, b"abc")
      dumps.assert_not_called()
      self.assertEqual(output.mock_calls, [])

  def test_exact_metadata_cap_and_empty_payload(self):
    metadata = {"padding": "x" * (65536 - len('{"padding":""}'))}
    output = io.BytesIO()
    subscriber.write_frame(output, metadata, b"")
    self.assertEqual(len(output.getvalue()), 12 + 65536)
    self.assertEqual(parse_frames(output.getvalue()), [(metadata, b"")])

  def test_short_writes_preserve_bytes_without_payload_conversion(self):
    buffer = io.BytesIO()
    output = Mock()
    output.write.side_effect = lambda part: buffer.write(part[:1])
    message = Image(data=b"\x00\xff\x80RDEB\n")
    subscriber.ImageStream(output, 5).receive(message)
    self.assertEqual(parse_frames(buffer.getvalue()), [(subscriber.encode_image(message), bytes(message.data))])
    for invocation in output.write.call_args_list[-len(message.data):]:
      self.assertIs(invocation.args[0].obj, message.data)
    output.flush.assert_called_once_with()

  def test_failed_writes_never_flush_or_spin(self):
    for result in (0, None, -1, 999):
      output = Mock()
      output.write.return_value = result
      with self.assertRaises(BrokenPipeError):
        subscriber.write_frame(output, {}, b"abc")
      output.write.assert_called_once()
      output.flush.assert_not_called()


if __name__ == "__main__":
  unittest.main()