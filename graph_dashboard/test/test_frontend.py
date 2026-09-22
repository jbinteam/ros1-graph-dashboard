# Copyright 2026 JB
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Filter-rule tests that run the REAL page script under Node.

The page's filter logic reads perfectly and still shipped a bug: hiding a
machine hid its nodes but left its topics on screen, because the
"hide a topic once every node touching it is hidden" rule only knew about
statically-scanned edges. Nothing short of running the actual code against
an actual multi-machine sample caught it, hence this harness.

Skipped where Node is unavailable; the dashboard itself never needs it.
"""
import json
from pathlib import Path
import shutil
import subprocess

import pytest

_HARNESS = Path(__file__).parent / "frontend_harness.js"
_PAGE = Path(__file__).parents[1] / "graph_dashboard" / "web" / "index.html"


def _run(payload, host_filter=None):
    node = shutil.which("node") or shutil.which("nodejs")
    if node is None:
        pytest.skip("node not installed — frontend harness cannot run")
    body = dict(payload)
    body["_host_filter"] = host_filter
    out = subprocess.run(
        [node, str(_HARNESS)],
        input=json.dumps(body), capture_output=True, text=True, timeout=60,
        env={"PAGE": str(_PAGE), "PATH": "/usr/bin:/bin"},
    )
    assert out.returncode == 0, "harness failed:\n" + out.stderr
    return json.loads(out.stdout)


def _node(name, ip, host):
    # ROS 1 gets the real address from the master, so the key is the IP.
    return {"name": name, "namespace": "/", "full": "/" + name,
            "host": host, "ip": ip, "uri": "http://{}:11311/".format(host)}


# Two machines, one topic private to each, one they both touch.
_TWO_MACHINES = {
    "available": True,
    "sampled_at": 0.0,
    "nodes": [
        _node("jetson_camera", "10.0.0.5", "robot-pc"),
        _node("jetson_detector", "10.0.0.5", "robot-pc"),
        _node("nuc_lidar", "10.0.0.9", "desk-pc"),
    ],
    "topics": [
        {"name": "/jetson/image", "types": ["sensor_msgs/Image"],
         "pub_count": 1, "sub_count": 1},
        {"name": "/nuc/scan", "types": ["sensor_msgs/LaserScan"],
         "pub_count": 1, "sub_count": 0},
        {"name": "/diagnostics", "types": ["diagnostic_msgs/DiagnosticArray"],
         "pub_count": 2, "sub_count": 0},
    ],
    "edges": [
        {"node": "/jetson_camera", "topic": "/jetson/image", "kind": "pub"},
        {"node": "/jetson_detector", "topic": "/jetson/image", "kind": "sub"},
        {"node": "/nuc_lidar", "topic": "/nuc/scan", "kind": "pub"},
        {"node": "/jetson_camera", "topic": "/diagnostics", "kind": "pub"},
        {"node": "/nuc_lidar", "topic": "/diagnostics", "kind": "pub"},
    ],
}


def test_machines_are_discovered_from_the_sample():
    assert _run(_TWO_MACHINES)["machines"] == ["10.0.0.5", "10.0.0.9"]


def test_no_filter_shows_everything():
    out = _run(_TWO_MACHINES)
    assert out["hidden"] == []
    assert "live:/nuc_lidar" in out["visible"]
    assert "topic:/nuc/scan" in out["visible"]


def test_host_filter_hides_the_other_machines_topics_too():
    # The regression: filtering to one machine used to hide only nodes,
    # leaving the other machine's topics floating on screen.
    out = _run(_TWO_MACHINES, host_filter="10.0.0.5")
    assert "live:/nuc_lidar" in out["hidden"]
    assert "topic:/nuc/scan" in out["hidden"]
    assert "live:/jetson_camera" in out["visible"]
    assert "topic:/jetson/image" in out["visible"]


def test_topic_touched_by_both_machines_survives_a_filter():
    # /diagnostics has a publisher on each machine, so it must stay while
    # either one is shown — hiding it would misreport the graph.
    for key in ("10.0.0.5", "10.0.0.9"):
        assert "topic:/diagnostics" in _run(_TWO_MACHINES, host_filter=key)["visible"]


def test_filtering_the_other_way_is_symmetric():
    out = _run(_TWO_MACHINES, host_filter="10.0.0.9")
    assert {"live:/jetson_camera", "live:/jetson_detector",
            "topic:/jetson/image"} <= set(out["hidden"])
    assert "live:/nuc_lidar" in out["visible"]
    assert "topic:/nuc/scan" in out["visible"]


def test_unknown_host_filter_hides_every_node():
    out = _run(_TWO_MACHINES, host_filter="10.0.0.99")
    assert not [i for i in out["visible"] if i.startswith("live:")]


def test_live_unavailable_clears_the_machine_list():
    out = _run({"available": False, "reason": "rosgraph not importable"})
    assert out["machines"] == [] and out["elements"] == []
