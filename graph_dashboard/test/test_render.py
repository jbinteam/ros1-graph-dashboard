# Copyright 2026 JB
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Tests for the tap strip's message renderers.

No ROS is involved: the renderers take any object with the message's
fields, so synthetic payloads built with OpenCV exercise them directly.
Skipped where OpenCV/numpy are absent — they are only needed by the tap,
never by the scanner or the server's static half.
"""
import struct

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from graph_dashboard.server import _render_compressed_image, _render_message  # noqa: E402


class _Compressed:
    """Stand-in for sensor_msgs/CompressedImage (format + data is all we read)."""

    def __init__(self, fmt, data):
        self.format = fmt
        self.data = data


def _jpeg(width=640, height=480):
    img = np.zeros((height, width, 3), np.uint8)
    img[:, : width // 2] = (0, 0, 255)  # BGR red on the left half
    ok, buf = cv2.imencode(".jpg", img)
    assert ok
    return buf.tobytes()


def _compressed_depth(fmt, quant_a=0.0, quant_b=0.0, quantized=None, raw_mm=None):
    payload = raw_mm if quantized is None else quantized
    ok, buf = cv2.imencode(".png", payload)
    assert ok
    return _Compressed(fmt, struct.pack("<iff", 0, quant_a, quant_b) + buf.tobytes())


def test_jpeg_decodes_downscales_and_keeps_colour():
    out = _render_compressed_image(_Compressed("bgr8; jpeg compressed bgr8", _jpeg()))
    assert out["kind"] == "image"
    assert out["source_size"].startswith("640x480")
    thumb = cv2.imdecode(
        np.frombuffer(__import__("base64").b64decode(out["jpeg_b64"]), np.uint8),
        cv2.IMREAD_COLOR)
    assert thumb.shape[1] == 480  # downscaled from 640
    b, g, r = thumb[10, 10]
    assert r > 200 and b < 60  # the red half survived, channels not swapped


def test_png_mono_renders():
    ok, buf = cv2.imencode(".png", np.full((100, 200), 128, np.uint8))
    assert ok
    out = _render_compressed_image(_Compressed("mono8; png compressed", buf.tobytes()))
    assert out["kind"] == "image" and out["source_size"].startswith("200x100")


def test_compressed_depth_16uc1_reports_millimetre_range():
    depth = np.zeros((64, 64), np.uint16)
    depth[20:40, 20:40] = 1500  # 1.5 m
    out = _render_compressed_image(
        _compressed_depth("16UC1; compressedDepth png", raw_mm=depth))
    assert out["kind"] == "image"
    assert out["depth_meta"] == {"depth_min_m": 1.5, "depth_max_m": 1.5}


def test_compressed_depth_32fc1_is_dequantized_before_colorizing():
    # The subtle one: a 32FC1 source is stored as uint16 quantized by
    # quant_a / (pixel - quant_b). Reading those values as raw millimetres
    # would report 0.5 m for a 2.0 m scene — wrong by 4x.
    quant_a, quant_b = 1000.0, 0.0
    meters = np.zeros((64, 64), np.float32)
    meters[10:50, 10:50] = 2.0
    quantized = np.zeros_like(meters, np.uint16)
    nz = meters > 0
    quantized[nz] = (quant_a / meters[nz] + quant_b).astype(np.uint16)
    out = _render_compressed_image(_compressed_depth(
        "32FC1; compressedDepth png", quant_a=quant_a, quant_b=quant_b, quantized=quantized))
    assert out["kind"] == "image"
    assert out["depth_meta"]["depth_min_m"] == pytest.approx(2.0, abs=0.01)
    assert out["depth_meta"]["depth_max_m"] == pytest.approx(2.0, abs=0.01)


@pytest.mark.parametrize("fmt,data,expected", [
    ("theora", b"\x00\x01", "theora is a video stream"),
    ("jpeg", b"", "empty compressed payload"),
    ("jpeg", b"definitely not an image", "could not decode"),
    ("16UC1; compressedDepth png", b"\x00" * 4, "payload too short"),
])
def test_undecodable_payloads_explain_themselves(fmt, data, expected):
    out = _render_compressed_image(_Compressed(fmt, data))
    assert out["kind"] == "note" and expected in out["note"]


def test_dispatch_and_failure_containment():
    msg = _Compressed("bgr8; jpeg compressed bgr8", _jpeg(64, 48))
    assert _render_message(msg, "sensor_msgs/CompressedImage")["kind"] == "image"

    class Broken:
        format = "jpeg"

        @property
        def data(self):
            raise RuntimeError("deserialization went wrong")

    # A bad message must degrade to a note, never take the poll down.
    out = _render_message(Broken(), "sensor_msgs/CompressedImage")
    assert out["kind"] == "note" and "compressed render failed" in out["note"]
