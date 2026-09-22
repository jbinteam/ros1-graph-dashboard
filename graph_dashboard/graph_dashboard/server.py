# Copyright 2026 JB
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""serve — local web view of the static ROS graph on 127.0.0.1:8092.

Endpoints:
  GET /            the rqt_graph-style view (vendored vis-network, offline)
  GET /api/graph   graph JSON, RE-SCANNED on every request so a browser
                   reload always reflects the current source tree — no
                   stale graph.json coordination needed.
  GET /api/ego?id=<element>   signed-distance ego-graph of one element
                   (negative = upstream inputs, positive = downstream
                   outputs) for the focus panel; recomputed per request
                   from a fresh scan like /api/graph. Elements the static
                   scan never saw (a topic/node only visible live — e.g. one
                   a nodelet creates at runtime) still get a minimal
                   one-hop ego built from the live master-API sample,
                   marked "live_only": true, instead of a 404.
  GET /api/tap?id=topic:<name>   on-demand live tap of ONE topic (Hz,
                   bandwidth, latest message rendered as thumbnail or field
                   tree). Poll-driven lifecycle: the first poll creates the
                   tap, and a tap unpolled for ~5 s is dropped — no explicit
                   close needed, so a closed browser tab releases the
                   subscription by itself. At most 3 concurrent taps.
                   Accepts a topic the static scan declared OR one only
                   visible in the live sample — the 404 gate still blocks
                   arbitrary names.
  GET /api/live    snapshot of the RUNNING ROS graph, sampled every ~2 s by
                   polling the ROS 1 master's system state over XML-RPC —
                   no live node needs to be created to do this (unlike
                   ROS 2's DDS introspection), so nothing shows up in
                   `rosnode list` as a side effect. Degrades honestly: where
                   `rosgraph` is not importable (ROS not sourced) or no
                   master is reachable, the payload says {"available":
                   false, "reason": ...} and the static view keeps working
                   untouched — rosgraph is imported lazily inside the probe
                   thread, never at module import time.

Port 8092 deliberately: 8091 belongs to the sibling ROS 2 ros2-graph-
dashboard, so both can run side by side on the same machine. Binds
loopback only — this is a lab-local introspection tool, not a service.
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import sys
import threading
import time
import urllib.parse

from graph_dashboard.scanner import _find_src_root, compute_ego, scan_workspace

_PROBE_PERIOD_S = 2.0
_MASTER_CALLER_ID = "/graph_dashboard_probe"


class LiveProber:
    """Background sampler of the live ROS 1 graph via the master's XML-RPC API.

    Read-only introspection: `rosgraph.Master.getSystemState()` /
    `getTopicTypes()` are pure queries against roscore — no node is ever
    created, nothing is published or subscribed here (that's TapManager's
    job, on demand). All rosgraph use lives inside the daemon thread so the
    HTTP server works identically where ROS is absent.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._state = {"available": False, "reason": "probe starting"}
        self._host_cache = {}  # node full name -> {"uri", "host", "ip"}
        self._ip_cache = {}  # hostname -> ip string (or None once it fails)
        self._thread = threading.Thread(target=self._run, daemon=True, name="live-probe")
        self._thread.start()

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def _set(self, state):
        with self._lock:
            self._state = state

    def _resolve_ip(self, host):
        """Hostname -> IP, cached (including failures, to stop retry storms)."""
        if host in self._ip_cache:
            return self._ip_cache[host]
        import socket

        try:
            ip = socket.gethostbyname(host)
        except Exception:
            ip = None  # unresolvable from here — the hostname is still useful
        self._ip_cache[host] = ip
        return ip

    def _node_host(self, master, full_name):
        """Where a node runs: its XML-RPC URI host, plus that host's IP.

        `lookupNode` is one round trip per node, so results are cached by
        node name: a node's URI is fixed for its lifetime, and re-asking
        every 2 s would put a per-node XML-RPC call on the master forever.
        Entries for nodes that leave the graph are dropped by the caller.
        """
        cached = self._host_cache.get(full_name)
        if cached is not None:
            return cached
        try:
            uri = master.lookupNode(full_name)
        except Exception:
            return None  # node vanished between getSystemState and now
        host = None
        try:
            host = urllib.parse.urlparse(uri).hostname
        except Exception:
            pass
        if not host:
            return None
        info = {"uri": uri, "host": host, "ip": self._resolve_ip(host)}
        self._host_cache[full_name] = info
        return info

    def _sample(self, master):
        pubs, subs, _services = master.getSystemState()
        types = dict(master.getTopicTypes())
        node_names = set()
        topics = []
        edges = []
        pub_map = {t: names for t, names in pubs}
        sub_map = {t: names for t, names in subs}
        for tname in sorted(set(pub_map) | set(sub_map)):
            pub_names = pub_map.get(tname, [])
            sub_names = sub_map.get(tname, [])
            node_names.update(pub_names)
            node_names.update(sub_names)
            ttype = types.get(tname)
            topics.append(
                {
                    "name": tname,
                    "types": [ttype] if ttype else [],
                    "pub_count": len(pub_names),
                    "sub_count": len(sub_names),
                }
            )
            for n in pub_names:
                edges.append({"node": n, "topic": tname, "kind": "pub"})
            for n in sub_names:
                edges.append({"node": n, "topic": tname, "kind": "sub"})
        nodes = []
        for n in sorted(node_names):
            entry = {
                "name": n.rsplit("/", 1)[-1],
                "namespace": n.rsplit("/", 1)[0] or "/",
                "full": n,
            }
            info = self._node_host(master, n)
            if info:
                entry.update(info)
            nodes.append(entry)
        # Forget nodes that left the graph, so the cache tracks the system
        # rather than growing for the life of the process (and so a node
        # restarted on another machine is looked up afresh).
        for gone in set(self._host_cache) - node_names:
            self._host_cache.pop(gone, None)
        return {
            "available": True,
            "sampled_at": time.time(),
            "nodes": nodes,
            "topics": sorted(topics, key=lambda t: t["name"]),
            "edges": edges,
        }

    def _run(self):
        try:
            import rosgraph
        except Exception as exc:  # ROS not sourced — static view must survive
            self._set({"available": False, "reason": "rosgraph not importable: {}".format(exc)})
            return
        master = rosgraph.Master(_MASTER_CALLER_ID)
        while True:
            try:
                self._set(self._sample(master))
            except Exception as exc:
                master_uri = getattr(master, "master_uri", "?")
                self._set(
                    {
                        "available": False,
                        "reason": "ROS master not reachable at {}: {}".format(master_uri, exc),
                    }
                )
            time.sleep(_PROBE_PERIOD_S)


class _Tap:
    """One tapped topic.

    Counters are written by the rospy callback; everything else is
    read/written only at poll time.
    """

    def __init__(self, topic):
        self.topic = topic
        self.sub = None
        self.msg_class = None
        self.msg_type = None
        self.last_poll = time.monotonic()
        self.count = 0
        self.bytes = 0
        self.latest_raw = None
        self.latest_mono = None
        self.prev = None  # (count, bytes, monotonic) at previous poll
        self.render_cache = None
        self.render_mono = 0.0


def _truncate_tree(obj, max_items=32):
    """Bound a message dict for display: long arrays elided, bytes summarized."""
    if isinstance(obj, dict):
        return {k: _truncate_tree(v, max_items) for k, v in obj.items()}
    if isinstance(obj, (bytes, bytearray)):
        return "bytes[{}]".format(len(obj))
    if isinstance(obj, (list, tuple)) or (hasattr(obj, "tolist") and hasattr(obj, "__len__")):
        seq = list(obj)
        out = [_truncate_tree(v, max_items) for v in seq[:max_items]]
        if len(seq) > max_items:
            out.append("… {} total".format(len(seq)))
        return out
    if isinstance(obj, (bool, int, str)) or obj is None:
        return obj
    if isinstance(obj, float):
        return round(obj, 6)
    try:
        return round(float(obj), 6)  # numpy scalars
    except (TypeError, ValueError):
        return str(obj)


def _msg_to_dict(msg, max_items=32):
    """Recursively convert a genpy message to a plain dict via __slots__.

    ROS 1 has no bundled equivalent of rosidl_runtime_py's
    message_to_ordereddict; every genpy-generated message exposes
    `__slots__` (its field names, in declaration order) so this needs no
    extra dependency (`rospy_message_converter` is a separate, not always
    installed, package). Nested messages/lists recurse; anything else
    falls through to `_truncate_tree`'s scalar/sequence handling.
    """
    slots = getattr(msg, "__slots__", None)
    if slots is None:
        return _truncate_tree(msg, max_items)
    out = {}
    for name in slots:
        value = getattr(msg, name)
        if hasattr(value, "__slots__"):
            out[name] = _msg_to_dict(value, max_items)
        elif isinstance(value, (list, tuple)) and value and hasattr(value[0], "__slots__"):
            seq = [_msg_to_dict(v, max_items) for v in value[:max_items]]
            if len(value) > max_items:
                seq.append("… {} total".format(len(value)))
            out[name] = seq
        else:
            out[name] = _truncate_tree(value, max_items)
    return out


_DEPTH_Z16_SCALE_M = 0.001  # realsense Z16: raw uint16 is millimeters
_GRAY_ENCODINGS = {"mono8", "8uc1", "y8"}
_DEPTH_U16_ENCODINGS = {"16uc1", "mono16"}
_DEPTH_F32_ENCODINGS = {"32fc1"}


def _reshape_row_major(data, height, step, row_nbytes, dtype):
    """Reshape a flat Image payload to (height, cols), honoring row padding.

    `step` is the message's declared row stride in bytes; `row_nbytes` is
    the actual pixel payload per row (may be smaller when step pads rows).
    """
    import numpy as np

    itemsize = np.dtype(dtype).itemsize
    arr = np.frombuffer(data, dtype)
    cols = row_nbytes // itemsize
    if step == row_nbytes:
        return arr.reshape(height, cols)
    step_items = step // itemsize
    return arr.reshape(height, step_items)[:, :cols]


def _colorize_depth(depth, valid_mask, is_float_meters):
    """Percentile-normalize depth to a perceptual colormap.

    Zeros / invalid pixels render black. Returns (bgr uint8 array, meta
    dict with depth_min_m/depth_max_m, or Nones if nothing was valid).
    """
    import cv2
    import numpy as np

    meters = depth.astype(np.float32)
    if not is_float_meters:
        meters = meters * _DEPTH_Z16_SCALE_M  # Z16/mono16 raw -> meters

    valid = meters[valid_mask]
    if valid.size == 0:
        return np.zeros(depth.shape + (3,), np.uint8), {
            "depth_min_m": None,
            "depth_max_m": None,
        }

    lo, hi = np.percentile(valid, [1, 99])
    if hi <= lo:
        hi = lo + 1e-6
    # Invalid pixels (zeros, NaN/inf) get a neutral fill before normalizing
    # so the cast to uint8 never sees NaN/inf — they're zeroed to black via
    # valid_mask right after anyway.
    safe_meters = np.where(valid_mask, meters, lo)
    norm8 = (np.clip((safe_meters - lo) / (hi - lo), 0.0, 1.0) * 255).astype(np.uint8)
    colormap = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
    color = cv2.applyColorMap(norm8, colormap)
    color[~valid_mask] = 0
    return color, {
        "depth_min_m": round(float(valid.min()), 4),
        "depth_max_m": round(float(valid.max()), 4),
    }


def _thumbnail_result(bgr, source_size, meta=None):
    """Downscale a BGR frame to <=480 wide and JPEG-encode it for the panel."""
    import base64

    import cv2

    height, width = bgr.shape[:2]
    if width > 480:
        scale = 480.0 / width
        bgr = cv2.resize(bgr, (480, max(1, int(height * scale))))
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    if not ok:
        return {"kind": "note", "note": "jpeg encode failed"}
    result = {
        "kind": "image",
        "jpeg_b64": base64.b64encode(buf.tobytes()).decode("ascii"),
        "source_size": source_size,
    }
    if meta is not None:
        result["depth_meta"] = meta
    return result


def _render_image(msg):
    """Render a sensor_msgs/Image to a JPEG thumbnail.

    Color (rgb8/bgr8) and grayscale (mono8) pass through mostly as before.
    Depth encodings (16UC1/mono16 raw millimeters, 32FC1 meters) are
    percentile-normalized over valid (nonzero / finite) pixels and mapped
    through a perceptual colormap (TURBO, falling back to JET on older
    OpenCV), with zero/invalid pixels rendered black and the true depth
    range (meters) reported in `depth_meta` for the data panel.
    """
    import cv2

    encoding = msg.encoding
    enc_lower = encoding.lower()
    height, width, step = msg.height, msg.width, msg.step
    # msg.data deserializes as a Python bytes/bytearray, which already
    # exposes the buffer protocol — np.frombuffer (inside
    # _reshape_row_major) reads it directly with zero copies.
    data = msg.data
    meta = None

    if encoding in ("rgb8", "bgr8"):
        arr = _reshape_row_major(data, height, step, width * 3, "uint8")
        bgr = arr.reshape(height, width, 3)
        if encoding == "rgb8":
            bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)
    elif enc_lower in _GRAY_ENCODINGS:
        arr = _reshape_row_major(data, height, step, width, "uint8")
        bgr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    elif enc_lower in _DEPTH_U16_ENCODINGS:
        arr = _reshape_row_major(data, height, step, width * 2, "<u2")
        bgr, meta = _colorize_depth(arr, arr != 0, is_float_meters=False)
    elif enc_lower in _DEPTH_F32_ENCODINGS:
        import numpy as np

        arr = _reshape_row_major(data, height, step, width * 4, "<f4")
        bgr, meta = _colorize_depth(arr, np.isfinite(arr) & (arr > 0), is_float_meters=True)
    else:
        return {"kind": "note", "note": "unsupported image encoding: {}".format(encoding)}

    return _thumbnail_result(bgr, "{}x{} {}".format(width, height, encoding), meta)


# compressed_depth_image_transport prefixes its PNG/RVL payload with a
# ConfigHeader: an int format enum plus two float quantization params.
_COMPRESSED_DEPTH_HEADER_BYTES = 12


def _render_compressed_image(msg):
    """Render a sensor_msgs/CompressedImage to a JPEG thumbnail.

    Covers what image_transport actually puts on the wire: `jpeg`/`png`
    color and mono frames, decoded by OpenCV, and `compressedDepth`, whose
    payload is NOT a bare image — it carries a 12-byte ConfigHeader that
    imdecode chokes on, so it is stripped first.

    Depth quantization matters for correctness, not just looks: a 16UC1
    source is PNG-encoded as raw millimeters, but a 32FC1 source is
    quantized to uint16 as `quant_a / (pixel - quant_b)` with the two
    params living in that header. Colorizing the quantized values directly
    would report a plausible-looking but wrong meter range, so the inverse
    is applied before `_colorize_depth` sees them.

    `theora` (and RVL-coded depth) are not decodable here — they need the
    codec's streaming state, not a self-contained frame — and say so
    instead of failing obscurely.
    """
    import struct

    import cv2
    import numpy as np

    fmt = (getattr(msg, "format", "") or "").strip()
    fmt_key = fmt.lower().replace(" ", "")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size == 0:
        return {"kind": "note", "note": "empty compressed payload (format: {})".format(fmt)}
    wire_kb = round(raw.size / 1024.0, 1)

    if "compresseddepth" in fmt_key:
        if raw.size <= _COMPRESSED_DEPTH_HEADER_BYTES:
            return {"kind": "note", "note": "compressedDepth payload too short"}
        quant_a, quant_b = struct.unpack_from(
            "<ff", raw[:_COMPRESSED_DEPTH_HEADER_BYTES].tobytes(), 4)
        decoded = cv2.imdecode(raw[_COMPRESSED_DEPTH_HEADER_BYTES:], cv2.IMREAD_UNCHANGED)
        if decoded is None:
            return {"kind": "note",
                    "note": "compressedDepth not decodable (format: {}) — RVL coding "
                            "needs the transport plugin".format(fmt)}
        if "32fc1" in fmt_key and quant_a:
            valid = decoded > 0
            meters = np.zeros(decoded.shape, np.float32)
            meters[valid] = quant_a / (decoded[valid].astype(np.float32) - quant_b)
            bgr, meta = _colorize_depth(meters, valid, is_float_meters=True)
        elif decoded.dtype == np.uint16:
            bgr, meta = _colorize_depth(decoded, decoded != 0, is_float_meters=False)
        else:
            return {"kind": "note",
                    "note": "unsupported compressedDepth payload: {}".format(decoded.dtype)}
        height, width = decoded.shape[:2]
        return _thumbnail_result(
            bgr, "{}x{} {} ({} KB wire)".format(width, height, fmt, wire_kb), meta)

    if "theora" in fmt_key:
        return {"kind": "note",
                "note": "theora is a video stream, not a self-contained frame — "
                        "tap the raw image topic instead"}

    bgr = cv2.imdecode(raw, cv2.IMREAD_COLOR)
    if bgr is None:
        return {"kind": "note",
                "note": "could not decode compressed image (format: {})".format(fmt)}
    height, width = bgr.shape[:2]
    return _thumbnail_result(
        bgr, "{}x{} {} ({} KB wire)".format(width, height, fmt, wire_kb))


_PC2_MAX_POINTS = 30000  # payload budget: keeps xyz+rgb base64 well under ~2 MB

# sensor_msgs/PointField.datatype -> (numpy dtype letter, size in bytes).
# Hardcoded rather than imported: these are frozen wire constants from the
# PointField message definition, not something that changes per-message.
_PC2_DTYPE_INFO = {
    1: ("i1", 1),  # INT8
    2: ("u1", 1),  # UINT8
    3: ("i2", 2),  # INT16
    4: ("u2", 2),  # UINT16
    5: ("i4", 4),  # INT32
    6: ("u4", 4),  # UINT32
    7: ("f4", 4),  # FLOAT32
    8: ("f8", 8),  # FLOAT64
}


def _pc2_points_2d(data, height, width, point_step, row_step):
    """Reshape a flat PointCloud2 payload to (n_points, point_step) bytes.

    `data` is whatever the deserialized message hands us for its `uint8[]`
    field (bytes in genpy), passed straight to np.frombuffer, which reads
    any buffer-protocol object with zero copies.

    Handles row padding (organized clouds where `row_step` exceeds the
    tight `width * point_step`) the same way `_reshape_row_major` does for
    images — but only when `row_step` is actually trustworthy. Confirmed
    real-world quirk (realsense driver, unorganized cloud, only
    valid-depth points serialized): `row_step` is stale, left over from the
    full sensor grid (e.g. 640 wide) rather than the reported `width`
    (e.g. ~198k, ~65% of 640x480), which would corrupt this reshape if
    trusted. Rule: only honor `row_step` for genuinely organized clouds
    (`height > 1`) where it's at least the tight per-row size and accounts
    for the whole buffer; otherwise fall back to a flat reshape sized from
    the real buffer length — i.e. trust `len(data) == point_step * n`,
    never `row_step`, exactly as measured on real hardware.
    """
    import numpy as np

    flat = np.frombuffer(data, dtype=np.uint8)
    row_nbytes = width * point_step
    row_step_trustworthy = (
        height > 1 and row_step >= row_nbytes and height * row_step <= len(flat)
    )
    if not row_step_trustworthy:
        n_points = len(flat) // point_step
        return flat[: n_points * point_step].reshape(n_points, point_step)
    usable = flat[: height * row_step].reshape(height, row_step)
    return usable[:, :row_nbytes].reshape(height * width, point_step)


def _pc2_field_array(points_2d, field, is_bigendian, as_dtype=None):
    """Extract one PointCloud2 field as a 1-D numpy array.

    `points_2d` is (n_points, point_step) uint8. `as_dtype`, if given,
    overrides the field's declared dtype — used for bit-packed rgb/rgba,
    which convention declares FLOAT32 but which is never a real float: it
    must be reinterpreted as a raw uint32 bit pattern, never cast.
    """
    import numpy as np

    np_kind, size = as_dtype if as_dtype else _PC2_DTYPE_INFO[field.datatype]
    dt = np.dtype((">" if is_bigendian else "<") + np_kind)
    raw = points_2d[:, field.offset: field.offset + size]
    return np.ascontiguousarray(raw).view(dt).reshape(-1)


def _render_pointcloud(msg):
    """Render a sensor_msgs/PointCloud2 as a compact downsampled 3D payload.

    Parses the field layout from the message itself (name/offset/datatype
    per field) — never assumes a fixed sensor layout. Packed as raw
    little-endian float32 xyz (+ optional uint8 rgb) bytes, base64-encoded
    — not JSON floats, for bandwidth. Color comes from a packed rgb/rgba
    field (PCL convention: 4 bytes holding a bit-packed 0x00RRGGBB,
    declared FLOAT32 but reinterpreted as uint32) or from separate r/g/b
    fields; clouds with neither render uncolored.

    Performance: the stride downsample to at most `_PC2_MAX_POINTS`
    happens FIRST, as a cheap strided VIEW over the raw (n_points,
    point_step) byte rows — before any field is extracted. Every per-field
    step that actually costs CPU then only ever runs over the sampled
    rows, not the full raw point count.

    Trade-off, intentional and documented rather than silently changed:
    NaN/invalid filtering now happens only on the sampled points, not a
    full scan of the message beforehand — `total_valid` is the raw
    serialized point count (pre-stride, pre-NaN-check), not an exact
    whole-message valid-point count.

    Raises on missing x/y/z fields — the caller falls back to the generic
    field-tree render.
    """
    import numpy as np

    fields = {f.name: f for f in msg.fields}
    if not {"x", "y", "z"} <= fields.keys():
        raise ValueError("no x/y/z fields")

    has_color = "rgb" in fields or "rgba" in fields or {"r", "g", "b"} <= fields.keys()
    points = _pc2_points_2d(msg.data, msg.height, msg.width, msg.point_step, msg.row_step)
    n_points = points.shape[0]
    if n_points == 0:
        return {
            "kind": "pointcloud", "point_count": 0, "total_valid": 0,
            "has_color": has_color, "frame_id": msg.header.frame_id,
            "bbox": None, "xyz_b64": "", "rgb_b64": "",
        }

    # Cheap strided VIEW (no copy) — everything field-extraction-related
    # below only ever touches this reduced row set.
    stride = -(-n_points // _PC2_MAX_POINTS) if n_points > _PC2_MAX_POINTS else 1
    sampled = points[::stride]

    x = _pc2_field_array(sampled, fields["x"], msg.is_bigendian).astype(np.float32)
    y = _pc2_field_array(sampled, fields["y"], msg.is_bigendian).astype(np.float32)
    z = _pc2_field_array(sampled, fields["z"], msg.is_bigendian).astype(np.float32)
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)

    rgb = None
    if "rgb" in fields or "rgba" in fields:
        packed_field = fields.get("rgb", fields.get("rgba"))
        packed = _pc2_field_array(sampled, packed_field, msg.is_bigendian, as_dtype=("u4", 4))
        rgb = np.stack(
            [((packed >> 16) & 0xFF), ((packed >> 8) & 0xFF), (packed & 0xFF)], axis=1
        ).astype(np.uint8)
    elif {"r", "g", "b"} <= fields.keys():
        rgb = np.stack(
            [_pc2_field_array(sampled, fields[c], msg.is_bigendian) for c in "rgb"], axis=1
        ).astype(np.uint8)

    xyz = np.stack([x[valid], y[valid], z[valid]], axis=1).astype("<f4")
    if xyz.shape[0] == 0:
        return {
            "kind": "pointcloud", "point_count": 0, "total_valid": n_points,
            "has_color": rgb is not None, "frame_id": msg.header.frame_id,
            "bbox": None, "xyz_b64": "", "rgb_b64": "",
        }
    bbox = {
        "min": [round(float(v), 4) for v in xyz.min(axis=0)],
        "max": [round(float(v), 4) for v in xyz.max(axis=0)],
    }

    import base64

    rgb_b64 = base64.b64encode(rgb[valid].tobytes()).decode("ascii") if rgb is not None else ""
    result = {
        "kind": "pointcloud",
        "point_count": int(xyz.shape[0]),
        "total_valid": n_points,
        "has_color": rgb is not None,
        "frame_id": msg.header.frame_id,
        "bbox": bbox,
        "xyz_b64": base64.b64encode(xyz.tobytes()).decode("ascii"),
        "rgb_b64": rgb_b64,
    }
    return result


def _render_message(msg, msg_type):
    """Render a deserialized message for the tap strip.

    JPEG thumbnail for images — raw or image_transport-compressed, color,
    grayscale or colorized depth — a compact 3D point payload for
    PointCloud2, truncated field tree otherwise. Runs at poll time
    (<= ~2 Hz), never in the subscriber callback.
    """
    if msg_type == "sensor_msgs/Image":
        return _render_image(msg)
    if msg_type == "sensor_msgs/CompressedImage":
        try:
            return _render_compressed_image(msg)
        except Exception as exc:
            return {"kind": "note", "note": "compressed render failed: {}".format(exc)}
    if msg_type == "sensor_msgs/PointCloud2":
        try:
            return _render_pointcloud(msg)
        except Exception:
            pass  # fall through to the generic field-tree render below
    return {"kind": "fields", "tree": _msg_to_dict(msg)}


class TapManager:
    """On-demand single-topic live taps (oscilloscope-probe principle).

    Safety rules baked in: a best-effort `rospy.AnyMsg` raw subscription;
    the callback only counts bytes and stashes the newest serialized
    buffer; deserialization + rendering happen at poll time (browser polls
    ~2 Hz) with a rate guard. Byte counts are exact wire payload sizes
    (`AnyMsg._buff` length). Taps expire ~5 s after the last poll.

    Unlike the ROS 2/rclpy version, no executor/spin thread is needed:
    once `rospy.init_node()` has run, rospy dispatches every subscriber
    callback on its own background threads automatically. Only a small
    reaper loop is kept, to expire idle taps.
    """

    MAX_TAPS = 3
    EXPIRE_S = 5.0
    RENDER_MIN_INTERVAL_S = 0.45
    # PointCloud2 rendering is far more expensive per call than an image
    # thumbnail (field extraction + NaN mask + rgb unpack over tens of
    # thousands of points vs. a JPEG encode) — refresh it at a slower 1 Hz
    # rather than the general ~2.2 Hz. The Hz/bandwidth counters are
    # unaffected: they come from the subscriber callback's count/bytes
    # tally (every message, always), never from the render step.
    RENDER_MIN_INTERVAL_POINTCLOUD_S = 1.0

    def __init__(self):
        self._lock = threading.Lock()
        self._taps = {}
        self._node_ready = False
        self._reaper_started = False

    def _ensure_node(self):
        """Init the rospy client library lazily; False if rospy is down."""
        try:
            import rospy
        except Exception:
            return False
        with self._lock:
            if not self._node_ready:
                try:
                    rospy.init_node(
                        "graph_dashboard_tap", anonymous=True,
                        disable_signals=True, log_level=rospy.ERROR,
                    )
                except Exception:
                    return False
                self._node_ready = True
            if not self._reaper_started:
                self._reaper_started = True
                threading.Thread(target=self._reap_loop, daemon=True, name="tap-reap").start()
        return True

    def _reap_loop(self):
        while True:
            time.sleep(0.5)
            self._reap()

    def _reap(self):
        now = time.monotonic()
        with self._lock:
            for topic, tap in list(self._taps.items()):
                if now - tap.last_poll > self.EXPIRE_S:
                    if tap.sub is not None:
                        try:
                            tap.sub.unregister()
                        except Exception:
                            pass
                    del self._taps[topic]

    def _try_subscribe(self, tap, live):
        """Resolve the live type and attach the raw AnyMsg subscription.

        Returns a status string on failure, None on success.
        """
        entry = next((t for t in live.get("topics", []) if t["name"] == tap.topic), None)
        if entry is None or not entry.get("types"):
            return "type_unavailable"
        if not self._ensure_node():
            return "live_unavailable"
        import rospy

        tap.msg_type = entry["types"][0]
        try:
            import roslib.message

            tap.msg_class = roslib.message.get_message_class(tap.msg_type)
        except Exception as exc:
            return "type_unavailable: {}".format(exc)
        if tap.msg_class is None:
            return "type_unavailable: {} not found".format(tap.msg_type)

        def _cb(any_msg, tap=tap):
            # Oscilloscope probe: count, measure, stash the reference. Zero
            # deserialization or copying here.
            tap.count += 1
            tap.bytes += len(any_msg._buff)
            tap.latest_raw = any_msg._buff
            tap.latest_mono = time.monotonic()

        tap.sub = rospy.Subscriber(tap.topic, rospy.AnyMsg, _cb, queue_size=1)
        return None

    def poll(self, topic, live):
        now = time.monotonic()
        with self._lock:
            tap = self._taps.get(topic)
            if tap is None:
                if len(self._taps) >= self.MAX_TAPS:
                    return {
                        "status": "tap_limit",
                        "detail": "at most {} concurrent taps — close another "
                                  "focus panel first".format(self.MAX_TAPS),
                    }
                tap = _Tap(topic)
                self._taps[topic] = tap
            tap.last_poll = now
        if not live.get("available"):
            return {"status": "live_unavailable", "reason": live.get("reason", "")}
        if tap.sub is None:
            failure = self._try_subscribe(tap, live)
            if failure:
                return {"status": failure, "topic": topic}

        count, nbytes = tap.count, tap.bytes
        hz = bw = 0.0
        if tap.prev is not None:
            dt = now - tap.prev[2]
            if dt > 0.05:
                hz = (count - tap.prev[0]) / dt
                bw = (nbytes - tap.prev[1]) / dt
        tap.prev = (count, nbytes, now)

        if count == 0:
            entry = next((t for t in live.get("topics", []) if t["name"] == topic), None)
            if entry is not None and entry.get("pub_count", 0) == 0:
                return {"status": "no_live_publisher", "topic": topic,
                        "msg_type": tap.msg_type}

        render = tap.render_cache
        raw = tap.latest_raw
        render_interval = (
            self.RENDER_MIN_INTERVAL_POINTCLOUD_S
            if tap.msg_type == "sensor_msgs/PointCloud2"
            else self.RENDER_MIN_INTERVAL_S
        )
        if raw is not None and now - tap.render_mono >= render_interval:
            try:
                msg = tap.msg_class()
                msg.deserialize(raw)
                render = _render_message(msg, tap.msg_type)
            except Exception as exc:
                render = {"kind": "note", "note": "render failed: {}".format(exc)}
            tap.render_cache = render
            tap.render_mono = now
        age = None if tap.latest_mono is None else round(now - tap.latest_mono, 2)
        return {
            "status": "ok",
            "topic": topic,
            "msg_type": tap.msg_type,
            "hz": round(hz, 2),
            "bandwidth_bps": round(bw, 1),
            "msgs_total": count,
            "last_msg_age_s": age,
            "render": render,
        }


def _live_only_ego(elem, live):
    """Minimal one-hop ego for an element the static scan never saw.

    Live-only elements are common wherever the C++ heuristic scanner can't
    resolve something (nodelets, macro-built names) or a topic is created
    dynamically at runtime. Built entirely from the LiveProber sample: a
    topic center's neighbors are the live nodes that publish/subscribe it
    (via the per-node edge list); a node center's neighbors are the live
    topics it publishes/subscribes. Directions mirror `compute_ego`'s
    convention (negative = upstream/inputs, positive = downstream/outputs).
    Returns None if `elem` is present in neither the static graph nor the
    live sample (truly unknown — the 404 case).
    """
    if not live.get("available"):
        return None
    edges = live.get("edges", [])
    levels = {elem: 0}
    if elem.startswith("topic:"):
        tname = elem[len("topic:"):]
        if not any(t["name"] == tname for t in live.get("topics", [])):
            return None
        for e in edges:
            if e["topic"] == tname:
                levels["live:" + e["node"]] = -1 if e["kind"] == "pub" else 1
    elif elem.startswith("live:"):
        full = elem[len("live:"):]
        if not any(n["full"] == full for n in live.get("nodes", [])):
            return None
        for e in edges:
            if e["node"] == full:
                levels["topic:" + e["topic"]] = 1 if e["kind"] == "pub" else -1
    else:
        return None
    return {"center": elem, "levels": levels, "dual": [], "live_only": True}


class _ScanCache:
    """Short-lived cache for scan_workspace(), used by /api/tap and /api/ego.

    /api/graph deliberately re-scans on every request — its documented
    contract is that a browser reload always reflects the current source
    tree — and keeps calling scan_workspace() directly, uncached. But
    /api/tap and /api/ego are polled automatically every ~0.5-2 s for as
    long as a focus panel stays open, and re-running the full AST + C++
    heuristic scan on every single poll is the dominant server-side cost
    by far on any workspace of real size. A couple of seconds of staleness
    on "is this topic/node in the static graph" is a clearly acceptable
    trade for not re-parsing the whole workspace source tree twice a
    second; the live master-API sample (which changes far faster and
    matters far more for a live-only element) is never cached.
    """

    # 3 s: the source tree realistically doesn't change mid-inspection-
    # session, and /api/graph remains the always-fresh path for anyone who
    # actually wants to confirm a just-edited file (a reload re-scans
    # unconditionally). Matches the same order of staleness already deemed
    # acceptable elsewhere in this file (TapManager.EXPIRE_S = 5 s).
    TTL_S = 3.0

    def __init__(self):
        self._lock = threading.Lock()
        self._src_root = None
        self._graph = None
        self._mono = 0.0

    def get(self, src_root):
        now = time.monotonic()
        with self._lock:
            if (
                self._graph is not None
                and self._src_root == src_root
                and now - self._mono < self.TTL_S
            ):
                return self._graph
        graph = scan_workspace(src_root)
        with self._lock:
            self._graph = graph
            self._src_root = src_root
            self._mono = now
        return graph


_WEB_DIR = Path(__file__).parent / "web"


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


class _Handler(BaseHTTPRequestHandler):
    src_root = None  # set by main() before serving
    prober = None  # set by main() before serving
    tap_manager = None  # set by main() before serving
    scan_cache = None  # set by main() before serving

    def _send(self, status, content_type, body):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):  # noqa: N802 — http.server API name
        if self.path == "/api/live":
            body = json.dumps(self.prober.snapshot()).encode("utf-8")
            self._send(200, "application/json", body)
        elif self.path.startswith("/api/tap"):
            query = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            elem = (query.get("id") or [""])[0]
            graph = self.scan_cache.get(self.src_root)
            live = self.prober.snapshot()
            # A topic is tappable if the static scan declared it OR the live
            # master-API graph actually has it right now (e.g. a topic a
            # nodelet creates at runtime, never visible to the AST scan or
            # the C++ heuristic). Either way the 404 gate still blocks
            # arbitrary names.
            known = {"topic:" + t["name"] for t in graph["topics"]}
            known.update("topic:" + t["name"] for t in live.get("topics", []))
            if elem not in known:
                self._send(404, "application/json",
                           json.dumps({"error": "unknown topic", "id": elem}).encode("utf-8"))
            else:
                result = self.tap_manager.poll(elem[len("topic:"):], live)
                self._send(200, "application/json", json.dumps(result).encode("utf-8"))
        elif self.path.startswith("/api/ego"):
            query = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            elem = (query.get("id") or [""])[0]
            graph = self.scan_cache.get(self.src_root)
            known = {n["id"] for n in graph["nodes"]}
            known.update("topic:" + t["name"] for t in graph["topics"])
            if elem in known:
                # Advisor-scoped radius: direct neighborhood only. A node
                # shows its topics (±1) and the before/after nodes (±2);
                # a topic shows just its publishers/subscribers (±1).
                depth = 1 if elem.startswith("topic:") else 2
                body = json.dumps(
                    compute_ego(graph["edges"], elem, max_depth=depth)
                ).encode("utf-8")
                self._send(200, "application/json", body)
            else:
                # Not in the static graph — try a live-only ego (a topic or
                # node the live master-API sample knows about but source
                # scanning never could) before giving up with a 404.
                live_ego = _live_only_ego(elem, self.prober.snapshot())
                if live_ego is None:
                    body = json.dumps({"error": "unknown element", "id": elem})
                    self._send(404, "application/json", body.encode("utf-8"))
                else:
                    self._send(200, "application/json", json.dumps(live_ego).encode("utf-8"))
        elif self.path.split("?", 1)[0] in ("/", "/index.html"):
            body = (_WEB_DIR / "index.html").read_bytes()
            self._send(200, "text/html; charset=utf-8", body)
        elif self.path == "/vendor/vis-network.min.js":
            body = (_WEB_DIR / "vis-network.min.js").read_bytes()
            self._send(200, "application/javascript", body)
        elif self.path == "/api/graph":
            graph = scan_workspace(self.src_root)
            body = json.dumps(graph).encode("utf-8")
            self._send(200, "application/json", body)
        else:
            self._send(404, "text/plain; charset=utf-8", b"not found\n")

    def log_message(self, fmt, *args):
        # One quiet line per request instead of BaseHTTPRequestHandler's
        # stderr chatter with timestamps.
        print("  {} {}".format(self.address_string(), fmt % args))


def main(argv=None):
    parser = argparse.ArgumentParser(description="Serve the static ROS-graph dashboard.")
    parser.add_argument(
        "--src",
        type=Path,
        default=None,
        help="workspace src directory (default: auto-detect a src/ with packages upward from cwd)",
    )
    parser.add_argument("--host", default="127.0.0.1", help="bind address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8092, help="port (default: 8092)")
    args = parser.parse_args(argv)

    src_root = args.src or _find_src_root(Path.cwd())
    if src_root is None or not Path(src_root).is_dir():
        print(
            "error: could not find a catkin src/ directory with packages by "
            "walking up from the current directory; pass --src /path/to/ws/src",
            file=sys.stderr,
        )
        return 2

    _Handler.src_root = Path(src_root).resolve()
    _Handler.prober = LiveProber()
    _Handler.tap_manager = TapManager()
    _Handler.scan_cache = _ScanCache()
    server = ThreadingHTTPServer((args.host, args.port), _Handler)

    # SIGTERM shuts down as cleanly as Ctrl-C: raising KeyboardInterrupt in
    # the main thread breaks serve_forever() and runs the same finally path
    # (matters for scripted/supervised runs, where SIGINT may be ignored).
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    print(
        "graph_dashboard: serving {} on http://{}:{}/ (Ctrl-C to stop)".format(
            _Handler.src_root, args.host, args.port
        )
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\ngraph_dashboard: shutting down")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
