# Copyright 2026 JB
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit + HTTP-level tests for live-only element handling.

A topic (or node) created dynamically at runtime — never visible to the
static AST/C++-heuristic scan — must still be tappable and focusable, using
only the LiveProber sample. No real ROS/roscore is needed: a fake
prober/tap manager stand in for the real rosgraph/rospy-backed classes,
since `_Handler` only ever calls `.snapshot()` / `.poll()` on whatever
object it's given.
"""
from http.server import ThreadingHTTPServer
import json
import threading
import urllib.error
import urllib.parse
import urllib.request

from graph_dashboard import server as srv

_LIVE_SAMPLE = {
    "available": True,
    "sampled_at": 0.0,
    "nodes": [{"name": "driver", "namespace": "/", "full": "/driver"}],
    "topics": [
        {"name": "/live/only/points", "types": ["sensor_msgs/PointCloud2"],
         "pub_count": 1, "sub_count": 0},
    ],
    "edges": [{"node": "/driver", "topic": "/live/only/points", "kind": "pub"}],
}


class _FakeProber:
    def __init__(self, snapshot):
        self._snapshot = snapshot

    def snapshot(self):
        return self._snapshot


class _FakeTapManager:
    def poll(self, topic, live):
        return {"status": "ok", "topic": topic, "polled_with_live": live is not None}


# --------------------------------------------------------- _live_only_ego
def test_live_only_ego_topic_center():
    ego = srv._live_only_ego("topic:/live/only/points", _LIVE_SAMPLE)
    assert ego == {
        "center": "topic:/live/only/points",
        "levels": {"topic:/live/only/points": 0, "live:/driver": -1},
        "dual": [],
        "live_only": True,
    }


def test_live_only_ego_node_center():
    ego = srv._live_only_ego("live:/driver", _LIVE_SAMPLE)
    assert ego == {
        "center": "live:/driver",
        "levels": {"live:/driver": 0, "topic:/live/only/points": 1},
        "dual": [],
        "live_only": True,
    }


def test_live_only_ego_unknown_returns_none():
    assert srv._live_only_ego("topic:/nothing/here", _LIVE_SAMPLE) is None
    assert srv._live_only_ego("live:/nothing/here", _LIVE_SAMPLE) is None


def test_live_only_ego_probe_unavailable_returns_none():
    unavailable = {"available": False, "reason": "x"}
    assert srv._live_only_ego("topic:/live/only/points", unavailable) is None


# --------------------------------------------------------- _ScanCache
def test_scan_cache_hits_within_ttl_then_refreshes_on_expiry_or_src_change(monkeypatch):
    calls = []

    def fake_scan(src_root):
        calls.append(src_root)
        return {"n": len(calls), "src_root": src_root}

    monkeypatch.setattr(srv, "scan_workspace", fake_scan)
    clock = [0.0]
    monkeypatch.setattr(srv.time, "monotonic", lambda: clock[0])

    cache = srv._ScanCache()
    g1 = cache.get("/a")
    assert g1["n"] == 1 and len(calls) == 1

    clock[0] += 0.5  # well within TTL_S — must be a cache hit
    g2 = cache.get("/a")
    assert g2 is g1 and len(calls) == 1

    clock[0] += srv._ScanCache.TTL_S + 0.1  # past TTL — must re-scan
    g3 = cache.get("/a")
    assert g3["n"] == 2 and len(calls) == 2

    g4 = cache.get("/b")  # different src_root — always a miss, TTL or not
    assert g4["n"] == 3 and len(calls) == 3


# ------------------------------------------------------- HTTP-level gates
def _boot(src_root, prober, tap_manager):
    srv._Handler.src_root = src_root
    srv._Handler.prober = prober
    srv._Handler.tap_manager = tap_manager
    srv._Handler.scan_cache = srv._ScanCache()
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), srv._Handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd, thread


def _get(base, path):
    url = base + path
    try:
        with urllib.request.urlopen(url, timeout=5) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def test_tap_and_ego_accept_live_only_but_still_404_unknown(tmp_path):
    (tmp_path / "package.xml").write_text("<package/>", encoding="utf-8")
    httpd, thread = _boot(tmp_path, _FakeProber(_LIVE_SAMPLE), _FakeTapManager())
    base = "http://127.0.0.1:{}".format(httpd.server_address[1])
    try:
        # Known to neither the (empty) static scan nor the live sample.
        status, body = _get(base, "/api/tap?id=" + urllib.parse.quote("topic:/nope"))
        assert status == 404
        status, body = _get(base, "/api/ego?id=" + urllib.parse.quote("topic:/nope"))
        assert status == 404

        # Live-only topic: unknown to the static scan, present in the live
        # sample — must be accepted (200), not gated out as unknown.
        tid = urllib.parse.quote("topic:/live/only/points")
        status, body = _get(base, "/api/tap?id=" + tid)
        assert status == 200
        assert body["status"] == "ok"
        assert body["polled_with_live"] is True

        status, body = _get(base, "/api/ego?id=" + tid)
        assert status == 200
        assert body["live_only"] is True
        assert body["levels"] == {"topic:/live/only/points": 0, "live:/driver": -1}

        # Live-only node id, same treatment on the ego side.
        status, body = _get(base, "/api/ego?id=" + urllib.parse.quote("live:/driver"))
        assert status == 200
        assert body["live_only"] is True
    finally:
        httpd.shutdown()
        httpd.server_close()
        thread.join(timeout=5)
