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
from copy import deepcopy
from pathlib import Path
import shutil
import subprocess
from urllib.parse import parse_qs, urlparse

import pytest

_HARNESS = Path(__file__).parent / "frontend_harness.js"
_PAGE = Path(__file__).parents[1] / "graph_dashboard" / "web" / "index.html"


def _run(payload, host_filter=None, graph=None, url="", click_types=(), next_live=None,
         actions=(), egos=None):
    node = shutil.which("node") or shutil.which("nodejs")
    if node is None:
        pytest.skip("node not installed — frontend harness cannot run")
    body = dict(payload)
    if host_filter is not None:
        body["_host_filter"] = host_filter
    body.update(_graph=graph, _url=url, _click_types=click_types, _next_live=next_live)
    body.update(_actions=actions, _egos=egos)
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


_STATIC_GRAPH = {
    "nodes": [{"id": "node:vision/camera", "node_name": "jetson_camera",
               "package": "vision", "source_file": "camera.py"}],
    "topics": [
        {"name": "/jetson/image", "msg_type": "sensor_msgs/Image | sensor_msgs/CompressedImage"},
        {"name": "/idle/image", "msg_type": "sensor_msgs/Image"},
        {"name": "/unknown", "msg_type": ""},
    ],
    "edges": [
        {"source": "node:vision/camera", "target": "topic:" + topic, "kind": "pub"}
        for topic in ("/jetson/image", "/idle/image", "/unknown")
    ],
    "summary": {"node_count": 1, "topic_count": 3, "edge_count": 3,
                "dynamic_topic_count": 0, "files_scanned": 1},
}
_NO_LIVE = {"available": False, "reason": "no master"}


def test_type_buttons_include_static_live_and_unknown_topics():
    out = _run(_TWO_MACHINES, graph=_STATIC_GRAPH)
    buttons = {b["type"]: b for b in out["type_buttons"]}
    assert buttons[""]["pressed"] == "true"
    assert buttons["sensor_msgs/Image"]["label"] == "sensor_msgs/Image (2)"
    assert buttons["sensor_msgs/LaserScan"]["label"] == "sensor_msgs/LaserScan (1)"
    assert buttons["(unknown)"]["label"] == "(unknown) (1)"
    # The master knows the actual type of a running, statically declared topic.
    assert "sensor_msgs/CompressedImage" not in buttons


def test_type_click_filters_topics_and_edges_and_updates_url():
    out = _run(_TWO_MACHINES, click_types=["sensor_msgs/Image"])
    assert "topic:/jetson/image" in out["visible"]
    assert {"topic:/nuc/scan", "topic:/diagnostics"} <= set(out["hidden"])
    assert "live:/nuc_lidar" in out["visible"]
    assert all(e["hidden"] for e in out["edges"] if e["to"] == "topic:/nuc/scan")
    assert parse_qs(urlparse(out["url"]).query)["types"] == ["sensor_msgs/Image"]


def test_multiple_type_selections_match_any_selected_type():
    out = _run(_TWO_MACHINES, click_types=["sensor_msgs/Image", "sensor_msgs/LaserScan"])
    assert {"topic:/jetson/image", "topic:/nuc/scan"} <= set(out["visible"])
    assert "topic:/diagnostics" in out["hidden"]
    assert {b["type"] for b in out["type_buttons"] if b["pressed"] == "true"} == {
        "sensor_msgs/Image", "sensor_msgs/LaserScan"}


@pytest.mark.parametrize("clicks", [
    ["sensor_msgs/Image", "sensor_msgs/Image"],
    ["sensor_msgs/Image", "sensor_msgs/LaserScan", ""],
])
def test_all_types_reset_restores_topics_and_removes_url_parameter(clicks):
    out = _run(_TWO_MACHINES, click_types=clicks)
    assert out["hidden"] == []
    assert "types" not in parse_qs(urlparse(out["url"]).query)
    assert out["type_buttons"][0]["pressed"] == "true"


def test_static_multi_type_topic_matches_without_ros():
    out = _run(_NO_LIVE, graph=_STATIC_GRAPH, url="?types=sensor_msgs%2FCompressedImage")
    assert "topic:/jetson/image" in out["visible"]
    assert {"topic:/idle/image", "topic:/unknown"} <= set(out["hidden"])
    assert "node:vision/camera" in out["visible"]


def test_unknown_type_is_selectable():
    out = _run(_NO_LIVE, graph=_STATIC_GRAPH, click_types=["(unknown)"])
    assert "topic:/unknown" in out["visible"]
    assert {"topic:/idle/image", "topic:/jetson/image"} <= set(out["hidden"])


def test_message_types_compose_with_host_and_package_filters():
    out = _run(_TWO_MACHINES, url="?host=10.0.0.9&types=sensor_msgs/Image")
    assert {"topic:/jetson/image", "topic:/nuc/scan", "topic:/diagnostics"} <= set(out["hidden"])
    assert "live:/nuc_lidar" in out["visible"]
    out = _run(_NO_LIVE, graph=_STATIC_GRAPH, url="?hide=vision", click_types=["sensor_msgs/Image"])
    assert set(out["hidden"]) == set(out["elements"])
    assert parse_qs(urlparse(out["url"]).query)["hide"] == ["vision"]


def test_type_changes_on_existing_live_topics_refresh_filter():
    changed = deepcopy(_TWO_MACHINES)
    changed["topics"][0]["types"] = ["sensor_msgs/LaserScan"]
    out = _run(_TWO_MACHINES, click_types=["sensor_msgs/Image"], next_live=changed)
    assert "topic:/jetson/image" in out["hidden"]
    assert any(b["label"] == "sensor_msgs/Image (0)" and b["pressed"] == "true"
               for b in out["type_buttons"])


def test_live_unavailable_restores_declared_message_types():
    out = _run(_TWO_MACHINES, graph=_STATIC_GRAPH,
               url="?types=sensor_msgs/CompressedImage", next_live=_NO_LIVE)
    assert "topic:/jetson/image" in out["visible"]
    assert "topic:/idle/image" in out["hidden"]
    assert any(b["label"] == "sensor_msgs/CompressedImage (1)" for b in out["type_buttons"])


def test_selected_type_remains_clearable_when_its_live_topic_disappears():
    changed = deepcopy(_TWO_MACHINES)
    changed["topics"] = [t for t in changed["topics"] if t["name"] != "/nuc/scan"]
    changed["edges"] = [e for e in changed["edges"] if e["topic"] != "/nuc/scan"]
    out = _run(_TWO_MACHINES, click_types=["sensor_msgs/LaserScan"], next_live=changed)
    assert "topic:/nuc/scan" not in out["elements"]
    assert "topic:/jetson/image" in out["hidden"]
    assert any(b["label"] == "sensor_msgs/LaserScan (0)" and b["pressed"] == "true"
               for b in out["type_buttons"])


_CAMERA_ID = "node:vision/camera"
_PATH_FOCUS = "node:demo/focus"
_PATH_LINKS = [
    ("node:demo/root_a", "topic:/first", "pub"),
    ("topic:/first", "node:demo/relay", "sub"),
    ("node:demo/relay", "topic:/input", "pub"),
    ("topic:/input", _PATH_FOCUS, "sub"),
    ("node:demo/root_b", "topic:/second", "pub"),
    ("topic:/second", _PATH_FOCUS, "sub"),
    (_PATH_FOCUS, "topic:/output", "pub"),
    ("topic:/output", "node:demo/sink", "sub"),
    ("node:demo/root_a", "topic:/side", "pub"),
    ("topic:/side", "node:demo/sibling", "sub"),
]
_PATH_GRAPH = {
    "nodes": [{"id": "node:demo/" + name, "node_name": name,
               "package": "demo", "source_file": name + ".py"}
              for name in ("root_a", "root_b", "relay", "focus", "sink", "sibling", "unused")],
    "topics": [{"name": "/" + name, "msg_type": "std_msgs/String"}
               for name in ("first", "input", "second", "output", "side")],
    "edges": [{"source": source, "target": target, "kind": kind}
              for source, target, kind in _PATH_LINKS],
    "summary": {"node_count": 7, "topic_count": 5, "edge_count": 10,
                "dynamic_topic_count": 0, "files_scanned": 7},
}


def _bold_paths(out):
    return {(e["from"], e["to"]) for e in out["edges"]
            if e["width"] >= 3 and e["color"] == "#1565c0"}


def test_focus_bolds_all_upstream_roots_and_downstream_paths_without_moving_nodes():
    before = _run(_NO_LIVE, graph=_PATH_GRAPH)
    out = _run(_NO_LIVE, graph=_PATH_GRAPH, actions=[{"kind": "focus", "id": _PATH_FOCUS}])
    assert _bold_paths(out) == {(source, target) for source, target, _ in _PATH_LINKS[:8]}
    for name in ("root_a", "root_b", "relay", "focus", "sink"):
        assert out["node_styles"]["node:demo/" + name]["border_width"] >= 3
    assert out["node_styles"][_PATH_FOCUS]["border_color"] == "#1565c0"
    assert out["node_styles"]["node:demo/sibling"]["border_width"] == 1
    assert out["node_styles"]["node:demo/unused"]["border_width"] == 1
    assert out["hidden"] == []
    assert out["rendered_positions"] == before["rendered_positions"]


@pytest.mark.parametrize("blur", [False, True])
def test_focus_paths_stay_bold_when_hovering_elsewhere_or_leaving_the_upper_graph(blur):
    actions = [{"kind": "focus", "id": _PATH_FOCUS},
               {"kind": "hover", "id": "node:demo/sibling"}]
    if blur:
        actions.append({"kind": "blur", "id": "node:demo/sibling"})
    out = _run(_NO_LIVE, graph=_PATH_GRAPH, actions=actions)
    assert _bold_paths(out) == {(source, target) for source, target, _ in _PATH_LINKS[:8]}


def test_closing_focus_restores_normal_graph_styling():
    out = _run(_NO_LIVE, graph=_PATH_GRAPH, actions=[
        {"kind": "focus", "id": _PATH_FOCUS},
        {"kind": "close"},
    ])
    assert not _bold_paths(out)
    assert all(e["width"] == 1 and e["color"] == "#78909c" for e in out["edges"])
    assert all(st["border_width"] == 1 for st in out["node_styles"].values())


def test_focus_paths_include_feedback_cycles():
    graph = deepcopy(_PATH_GRAPH)
    graph["topics"].append({"name": "/feedback", "msg_type": "std_msgs/String"})
    feedback = [("node:demo/sink", "topic:/feedback"), ("topic:/feedback", _PATH_FOCUS)]
    graph["edges"].extend({"source": source, "target": target, "kind": kind}
                          for (source, target), kind in zip(feedback, ("pub", "sub")))
    graph["summary"].update(topic_count=6, edge_count=12)
    out = _run(_NO_LIVE, graph=graph, actions=[{"kind": "focus", "id": _PATH_FOCUS}])
    expected = {(source, target) for source, target, _ in _PATH_LINKS[:8]} | set(feedback)
    assert _bold_paths(out) == expected


def test_walking_the_focus_panel_updates_the_bold_paths():
    target = "topic:/idle/image"
    out = _run(_NO_LIVE, graph=_STATIC_GRAPH, egos={
        _CAMERA_ID: {"center": _CAMERA_ID, "levels": {_CAMERA_ID: 0, target: 1}, "dual": []},
    }, actions=[
        {"kind": "focus", "id": _CAMERA_ID},
        {"kind": "focus-click", "id": target},
    ])
    assert out["focus_id"] == target
    assert _bold_paths(out) == {(_CAMERA_ID, target)}


def test_focus_paths_extend_from_static_nodes_through_live_only_connections():
    out = _run(_TWO_MACHINES, graph=_STATIC_GRAPH,
               actions=[{"kind": "focus", "id": _CAMERA_ID}])
    assert _bold_paths(out) == {
        (_CAMERA_ID, "topic:/jetson/image"), (_CAMERA_ID, "topic:/idle/image"),
        (_CAMERA_ID, "topic:/unknown"), (_CAMERA_ID, "topic:/diagnostics"),
        ("topic:/jetson/image", "live:/jetson_detector"),
    }
    assert len({(e["from"], e["to"]) for e in out["edges"]}) == len(out["edges"])
    edge = next(e for e in out["edges"] if e["to"] == "live:/jetson_detector")
    assert edge["dashes"] == [3, 3]
    assert out["node_styles"]["live:/jetson_detector"]["border_width"] >= 3
    assert out["node_styles"]["live:/nuc_lidar"]["border_width"] == 1


def test_focus_paths_follow_live_endpoint_changes_without_rearranging_existing_nodes():
    changed = deepcopy(_TWO_MACHINES)
    changed["edges"][1]["topic"] = "/diagnostics"
    changed["topics"][0]["sub_count"] = 0
    changed["topics"][2]["sub_count"] = 1
    focus = [{"kind": "focus", "id": "topic:/jetson/image"}]
    before = _run(_TWO_MACHINES, actions=focus)
    out = _run(_TWO_MACHINES, actions=focus + [{"kind": "live", "live": changed}])
    pairs = {(e["from"], e["to"]) for e in out["edges"]}
    assert ("topic:/jetson/image", "live:/jetson_detector") not in pairs
    assert ("topic:/diagnostics", "live:/jetson_detector") in pairs
    assert _bold_paths(out) == {("live:/jetson_camera", "topic:/jetson/image")}
    assert out["node_styles"]["live:/jetson_detector"]["border_width"] == 1
    assert out["elements"] == before["elements"]
    assert out["rendered_positions"] == before["rendered_positions"]


def test_declared_focus_paths_remain_bold_when_live_discovery_becomes_unavailable():
    out = _run(_TWO_MACHINES, graph=_STATIC_GRAPH, actions=[
        {"kind": "focus", "id": _CAMERA_ID},
        {"kind": "live", "live": _NO_LIVE},
    ])
    assert _bold_paths(out) == {(_CAMERA_ID, "topic:" + name)
                                for name in ("/jetson/image", "/idle/image", "/unknown")}
    assert "live:/jetson_detector" not in out["elements"]


def test_hover_still_previews_paths_when_the_focus_panel_is_closed():
    out = _run(_NO_LIVE, graph=_PATH_GRAPH, actions=[{"kind": "hover", "id": _PATH_FOCUS}])
    assert out["focus_id"] is None
    assert _bold_paths(out) == {(source, target) for source, target, _ in _PATH_LINKS[:8]}
    cleared = _run(_NO_LIVE, graph=_PATH_GRAPH, actions=[
        {"kind": "hover", "id": _PATH_FOCUS}, {"kind": "blur", "id": _PATH_FOCUS},
    ])
    assert not _bold_paths(cleared)


def test_focus_deep_link_bolds_its_paths_on_load():
    out = _run(_NO_LIVE, graph=_PATH_GRAPH, url="?focus=focus")
    assert out["focus_id"] == _PATH_FOCUS
    assert _bold_paths(out) == {(source, target) for source, target, _ in _PATH_LINKS[:8]}


def test_live_connections_of_launch_renamed_nodes_use_the_static_node_in_focus_paths():
    graph = deepcopy(_STATIC_GRAPH)
    graph["nodes"][0]["launch_names"] = ["/robot/camera_renamed"]
    live = deepcopy(_TWO_MACHINES)
    live["nodes"][0].update(name="camera_renamed", full="/robot/camera_renamed")
    for edge in live["edges"]:
        if edge["node"] == "/jetson_camera":
            edge["node"] = "/robot/camera_renamed"
    out = _run(live, graph=graph, actions=[{"kind": "focus", "id": _CAMERA_ID}])
    assert "live:/robot/camera_renamed" not in out["elements"]
    assert (_CAMERA_ID, "topic:/diagnostics") in _bold_paths(out)
    assert ("topic:/jetson/image", "live:/jetson_detector") in _bold_paths(out)


@pytest.mark.parametrize("feedback", [False, True])
def test_focus_panel_preserves_main_arrangement_including_feedback(feedback):
    graph = deepcopy(_PATH_GRAPH)
    levels = {_PATH_FOCUS: 0, "topic:/input": -1, "topic:/second": -1,
              "node:demo/relay": -2, "node:demo/root_b": -2,
              "topic:/output": 1, "node:demo/sink": 2}
    dual = []
    if feedback:
        graph["topics"].append({"name": "/feedback", "msg_type": "std_msgs/String"})
        graph["edges"].extend([
            {"source": "node:demo/sink", "target": "topic:/feedback", "kind": "pub"},
            {"source": "topic:/feedback", "target": _PATH_FOCUS, "kind": "sub"},
        ])
        levels["topic:/feedback"] = -1
        dual = ["node:demo/sink", "topic:/feedback", "topic:/output"]
    out = _run(_NO_LIVE, graph=graph, egos={
        _PATH_FOCUS: {"center": _PATH_FOCUS, "levels": levels, "dual": dual},
    }, actions=[{"kind": "focus", "id": _PATH_FOCUS}])
    assert out["focus_positions"] == {eid: out["rendered_positions"][eid] for eid in levels}
    # Unequal upstream path lengths put these inputs in different main columns;
    # signed-distance placement would collapse them into the same column.
    assert out["focus_positions"]["topic:/input"]["x"] != out["focus_positions"]["topic:/second"]["x"]


def test_focus_panel_uses_dragged_positions_when_opening_and_refocusing():
    target = "topic:/idle/image"
    out = _run(_NO_LIVE, graph=_STATIC_GRAPH, egos={
        _CAMERA_ID: {"center": _CAMERA_ID, "levels": {_CAMERA_ID: 0, target: 1}, "dual": []},
        target: {"center": target, "levels": {_CAMERA_ID: -1, target: 0}, "dual": []},
    }, actions=[
        {"kind": "drag", "id": _CAMERA_ID, "x": -120, "y": 310},
        {"kind": "focus", "id": _CAMERA_ID},
        {"kind": "drag", "id": target, "x": 540, "y": -90},
        {"kind": "focus-click", "id": target},
    ])
    assert out["focus_positions"] == {
        _CAMERA_ID: {"x": -120, "y": 310}, target: {"x": 540, "y": -90},
    }
    assert all(pos == out["rendered_positions"][eid]
               for eid, pos in out["focus_positions"].items())


def test_live_only_focus_panel_preserves_current_main_arrangement():
    target = "topic:/jetson/image"
    levels = {target: 0, "live:/jetson_camera": -1, "live:/jetson_detector": 1}
    out = _run(_TWO_MACHINES, egos={
        target: {"center": target, "levels": levels, "dual": [], "live_only": True},
    }, actions=[{"kind": "focus", "id": target}])
    assert out["focus_positions"] == {eid: out["rendered_positions"][eid] for eid in levels}
