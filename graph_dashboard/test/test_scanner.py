# Copyright 2026 JB
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Unit tests for the AST scanner against a synthetic mini-workspace.

Covers the three resolution tiers (literal, local/module assignment,
rospy.get_param default) and the must-not-crash dynamic-placeholder path.
"""
from pathlib import Path
import textwrap

from graph_dashboard.scanner import (
    _find_src_root,
    compute_ego,
    compute_reachability,
    scan_workspace,
)

_FAKE_NODE = """
import rospy
from sensor_msgs.msg import Image

TOPIC_CONST = "/from/module/const"


class FakeNode:
    def __init__(self):
        rospy.init_node("fake_node")
        out_topic = rospy.get_param("~out_topic", "/from/param/default")
        self._pub = rospy.Publisher(out_topic, Image, queue_size=5)
        self._pub2 = rospy.Publisher("/literal/topic", Image, queue_size=5)
        self._sub = rospy.Subscriber(TOPIC_CONST, Image, self.cb, queue_size=5)
        self._sub2 = rospy.Subscriber(f"/dyn/{out_topic}", Image, self.cb, queue_size=5)
"""


def _write_ws(tmp_path: Path) -> Path:
    src = tmp_path / "catkin_ws" / "src"
    pkg = src / "fake_pkg" / "src" / "fake_pkg"
    pkg.mkdir(parents=True)
    (pkg / "fake_node.py").write_text(textwrap.dedent(_FAKE_NODE), encoding="utf-8")
    return src


def test_resolution_tiers_and_dynamic(tmp_path):
    graph = scan_workspace(_write_ws(tmp_path))

    nodes = {n["node_name"]: n for n in graph["nodes"]}
    assert "fake_node" in nodes
    assert nodes["fake_node"]["package"] == "fake_pkg"
    assert nodes["fake_node"]["class_name"] == "FakeNode"

    topic_names = {t["name"] for t in graph["topics"]}
    assert "/from/param/default" in topic_names
    assert "/literal/topic" in topic_names
    assert "/from/module/const" in topic_names

    dynamic = [t for t in graph["topics"] if t["dynamic"]]
    assert len(dynamic) == 1
    assert dynamic[0]["name"].startswith("?")
    assert graph["summary"]["dynamic_topic_count"] == 1
    assert len(graph["summary"]["dynamic_call_sites"]) == 1

    kinds = {(e["source"], e["target"], e["kind"]) for e in graph["edges"]}
    node_id = nodes["fake_node"]["id"]
    assert (node_id, "topic:/from/param/default", "pub") in kinds
    assert ("topic:/from/module/const", node_id, "sub") in kinds

    msg_types = {t["name"]: t["msg_type"] for t in graph["topics"]}
    assert msg_types["/literal/topic"] == "sensor_msgs/Image"

    qos_by_topic = {}
    for e in graph["edges"]:
        name = (e["target"] if e["kind"] == "pub" else e["source"])[len("topic:"):]
        qos_by_topic[name] = e["qos"]
    assert qos_by_topic["/literal/topic"] == {
        "depth": 5, "reliability": None, "durability": "volatile"}


def _edges(*pairs):
    return [{"source": s, "target": t} for s, t in pairs]


def test_reachability_transitive_both_directions():
    # a -> t1 -> b -> t2 -> c : a linear chain.
    closures = compute_reachability(
        _edges(("a", "t1"), ("t1", "b"), ("b", "t2"), ("t2", "c"))
    )
    assert closures["b"]["up"] == ["a", "t1"]
    assert closures["b"]["down"] == ["c", "t2"]
    assert closures["a"]["up"] == []
    assert closures["a"]["down"] == ["b", "c", "t1", "t2"]
    assert closures["c"]["down"] == []


def test_reachability_cycle_terminates_and_is_complete():
    # a -> t -> b -> t2 -> a : a cycle (test harnesses can wire these up).
    # Everyone reaches everyone, nobody lists itself.
    closures = compute_reachability(
        _edges(("a", "t"), ("t", "b"), ("b", "t2"), ("t2", "a"))
    )
    for elem in ("a", "t", "b", "t2"):
        others = sorted(x for x in ("a", "t", "b", "t2") if x != elem)
        assert closures[elem]["up"] == others
        assert closures[elem]["down"] == others


def test_ego_levels_linear_chain():
    # a -> t1 -> b -> t2 -> c, centered on b: inputs negative, outputs
    # positive, distance = BFS hops.
    ego = compute_ego(
        _edges(("a", "t1"), ("t1", "b"), ("b", "t2"), ("t2", "c")), "b"
    )
    assert ego["levels"] == {"a": -2, "t1": -1, "b": 0, "t2": 1, "c": 2}
    assert ego["dual"] == []


def test_ego_cycle_single_deterministic_placement():
    # a -> t -> b -> t2 -> a : from a, every element is reachable both ways.
    # Rule: smaller |distance| wins, tie -> upstream. t: down 1 / up 3 -> +1.
    # t2: up 1 / down 3 -> -1. b: up 2 / down 2 -> tie -> upstream -2.
    ego = compute_ego(
        _edges(("a", "t"), ("t", "b"), ("b", "t2"), ("t2", "a")), "a"
    )
    assert ego["levels"] == {"a": 0, "t": 1, "b": -2, "t2": -1}
    assert ego["dual"] == ["b", "t", "t2"]


def test_ego_depth_clamp_direct_neighborhood_only():
    # Perception-pipeline shape: camera -> /camera/image_raw -> preprocess
    # -> /perception/image_proc -> tracker -> /tracker/landmarks.
    # Node focus (max_depth=2): before/after nodes with connecting topics —
    # preprocess present at -2, camera and its topic ABSENT.
    edges = _edges(
        ("camera", "t_raw"), ("t_raw", "preprocess"),
        ("preprocess", "t_proc"), ("t_proc", "tracker"),
        ("tracker", "t_landmarks"), ("t_landmarks", "viewer"),
    )
    ego = compute_ego(edges, "tracker", max_depth=2)
    assert ego["levels"] == {
        "tracker": 0, "t_proc": -1, "preprocess": -2,
        "t_landmarks": 1, "viewer": 2,
    }
    assert "camera" not in ego["levels"]
    assert "t_raw" not in ego["levels"]
    # Topic focus (max_depth=1): publishers and subscribers only.
    ego_t = compute_ego(edges, "t_proc", max_depth=1)
    assert ego_t["levels"] == {"t_proc": 0, "preprocess": -1, "tracker": 1}


def test_scan_output_includes_closures(tmp_path):
    graph = scan_workspace(_write_ws(tmp_path))
    node_id = graph["nodes"][0]["id"]
    assert "topic:/literal/topic" in graph["closures"][node_id]["down"]


_FAKE_CPP = """
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

/* multi-line
   comment with nh.advertise<Fake>("nope", 1); inside */
class CppTalker {
 public:
  explicit CppTalker(ros::NodeHandle& nh) {
    pub_ = nh.advertise<std_msgs::String>("/chatter", 10, true);
    // pub_ = nh.advertise<Fake>("commented_out", 1);
    img_sub_ = nh.subscribe<sensor_msgs::Image>(
        "/camera/image_raw", 5, &CppTalker::onImage, this);
    dyn_pub_ = nh.advertise<std_msgs::String>(topic_param_, 1);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cpp_talker");
  ros::NodeHandle nh;
  CppTalker talker(nh);
  ros::spin();
}
"""


def test_cpp_scanner_fixture(tmp_path):
    src = tmp_path / "catkin_ws" / "src"
    pkg = src / "cpp_pkg" / "src"
    pkg.mkdir(parents=True)
    (pkg / "cpp_talker.cpp").write_text(_FAKE_CPP, encoding="utf-8")
    graph = scan_workspace(src)

    nodes = {n["node_name"]: n for n in graph["nodes"]}
    assert "cpp_talker" in nodes
    assert nodes["cpp_talker"]["language"] == "cpp"
    assert nodes["cpp_talker"]["package"] == "cpp_pkg"
    assert nodes["cpp_talker"]["class_name"] == "CppTalker"

    topics = {t["name"]: t for t in graph["topics"]}
    assert topics["/chatter"]["msg_type"] == "std_msgs/String"
    assert topics["/camera/image_raw"]["msg_type"] == "sensor_msgs/Image"
    # Commented-out calls must not leak in; the dynamic topic is a placeholder.
    assert "nope" not in topics and "commented_out" not in topics
    assert topics["?topic_param_"]["dynamic"]
    assert graph["summary"]["cpp_files_scanned"] == 1

    qos_by_topic = {}
    for e in graph["edges"]:
        name = (e["target"] if e["kind"] == "pub" else e["source"])[len("topic:"):]
        qos_by_topic[name] = e["qos"]
    assert qos_by_topic["/chatter"] == {
        "depth": 10, "reliability": None, "durability": "transient_local"}
    assert qos_by_topic["/camera/image_raw"] == {
        "depth": 5, "reliability": None, "durability": "volatile"}
    assert qos_by_topic["?topic_param_"]["depth"] == 1


def test_python_only_fixture_unaffected_by_cpp_pass(tmp_path):
    # Invariance check independent of repo contents: a fixture tree with no
    # .cpp/.cc/.hpp/.hh anywhere must yield only "python" nodes — the C++
    # pass runs (it always does), finds nothing, and adds nothing.
    graph = scan_workspace(_write_ws(tmp_path))
    assert graph["nodes"]
    assert all(n["language"] == "python" for n in graph["nodes"])
    assert not any(f.endswith((".cpp", ".cc", ".hpp", ".hh"))
                   for f in (n["source_file"] for n in graph["nodes"]))


def test_find_src_root_name_agnostic(tmp_path):
    # Workspace named anything (only "catkin_ws" matched would be a bug).
    ws = tmp_path / "any_name_ws"
    pkg = ws / "src" / "some_pkg"
    pkg.mkdir(parents=True)
    (pkg / "package.xml").write_text("<package/>", encoding="utf-8")
    (pkg / "node.py").write_text("x = 1\n", encoding="utf-8")

    src = ws / "src"
    assert _find_src_root(ws) == src            # from the workspace root
    assert _find_src_root(src) == src           # from inside src/
    assert _find_src_root(pkg) == src           # from inside a package
    # Legacy spelling still works from one level above a catkin_ws/ workspace.
    legacy_root = tmp_path / "repo"
    legacy_pkg = legacy_root / "catkin_ws" / "src" / "p"
    legacy_pkg.mkdir(parents=True)
    (legacy_pkg / "package.xml").write_text("<package/>", encoding="utf-8")
    assert _find_src_root(legacy_root) == legacy_root / "catkin_ws" / "src"


def test_find_src_root_requires_packages(tmp_path):
    # A src/ with no package.xml anywhere must not be picked up. (The walk
    # continues above tmp_path, so assert nothing INSIDE the fixture won.)
    (tmp_path / "empty_ws" / "src").mkdir(parents=True)
    found = _find_src_root(tmp_path / "empty_ws")
    assert found is None or not str(found).startswith(str(tmp_path))


def test_parse_error_does_not_crash(tmp_path):
    src = _write_ws(tmp_path)
    (src / "fake_pkg" / "src" / "fake_pkg" / "broken.py").write_text(
        "def oops(:\n", encoding="utf-8")
    graph = scan_workspace(src)
    assert len(graph["summary"]["parse_errors"]) == 1
    assert graph["summary"]["node_count"] == 1
