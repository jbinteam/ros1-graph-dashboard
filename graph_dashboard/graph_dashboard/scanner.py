# Copyright 2026 JB
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""scan — static ROS-graph extractor over every source in the workspace.

Unlike rqt_graph (which introspects the LIVE graph), this reads source code
with the `ast` module — never importing or executing any scanned file — and
reports every `rospy.Publisher` / `rospy.Subscriber` call declared anywhere
under `catkin_ws/src/`, whether or not those nodes are running.

Topic-name resolution is best-effort static analysis, in this order:
  1. a string literal at the call site;
  2. a local/self variable assigned a string literal, followed through
     simple assignments (module constants included);
  3. the repo's dominant pattern:
         x_topic = rospy.get_param("~x_topic", "/default")
     which resolves to the declared DEFAULT (a launch file, roslaunch
     remap, or the parameter server can override it at runtime — the
     static view shows the declared wiring, not a live remap).
Anything else (f-strings, arithmetic, values passed in from outside the
class) is recorded as a dynamic placeholder "?<expr>" and counted in the
coverage summary instead of crashing or being silently dropped.

Node naming: the string literal in the nearest `rospy.init_node("name")`
call within the same class (or module body, for module-level calls); falls
back to the class name, or the file stem for calls made outside any class
(test harnesses, bench scripts).
"""
import argparse
import ast
import datetime
from pathlib import Path
import json
import re
import sys

_EXCLUDE_DIRS = {"build", "devel", "install", "log", "__pycache__", ".pytest_cache", ".git"}
_PUBSUB = {"Publisher", "Subscriber"}
_CPP_EXTS = {".cpp", ".cc", ".hpp", ".hh"}


def _src_has_packages(src_dir: Path):
    """Report whether a catkin package (package.xml) lives beneath src_dir.

    Bounded glob depth (packages may sit under a repo subdirectory), so
    walking up through huge trees stays cheap.
    """
    for pattern in ("package.xml", "*/package.xml", "*/*/package.xml",
                    "*/*/*/package.xml"):
        try:
            if next(iter(src_dir.glob(pattern)), None) is not None:
                return True
        except OSError:
            pass
    return False


def _find_src_root(start: Path):
    """Locate the workspace src/ directory, name-agnostic.

    Walking up from `start`: a dir literally named src containing at least
    one package.xml wins (covers running from inside src or a package);
    else a src/ child with packages (any workspace name — my_catkin_ws,
    ws, …); else the legacy catkin_ws/src spelling (so running from a
    repo root one level above the workspace keeps working).
    """
    for d in (start, *start.parents):
        if d.name == "src" and _src_has_packages(d):
            return d
        cand = d / "src"
        if cand.is_dir() and _src_has_packages(cand):
            return cand
        legacy = d / "catkin_ws" / "src"
        if legacy.is_dir() and _src_has_packages(legacy):
            return legacy
    return None


def _iter_py_files(src_root: Path):
    for path in sorted(src_root.rglob("*.py")):
        if any(part in _EXCLUDE_DIRS for part in path.parts):
            continue
        yield path


def _iter_cpp_files(src_root: Path):
    for path in sorted(src_root.rglob("*")):
        if path.suffix not in _CPP_EXTS:
            continue
        if any(part in _EXCLUDE_DIRS for part in path.parts):
            continue
        yield path


def _const_str(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _get_param_default(node):
    """Return the default-literal from a `rospy.get_param("key", default)` call.

    Matches any `<x>.get_param("key", "default")` spelling (the repo's
    dominant pattern is `rospy.get_param`, but `nh.get_param` and similar
    aliases are just as valid rospy usage) — the whole call, not an
    attribute chain, since ROS 1 parameter reads are single-step (unlike
    ROS 2's declare/get split).
    """
    if not (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "get_param"
        and len(node.args) >= 2
    ):
        return None
    return _const_str(node.args[1])


def _ros1_queue_info(call):
    """Best-effort static queue_size/latch of one Publisher/Subscriber call.

    Keyword arguments ONLY, deliberately: in rospy's real signatures the
    positional slots that follow (topic, msg_type) are
    `subscriber_listener` for Publisher and `callback` for Subscriber, not
    the queue size — reading a positional there would report another
    argument's value as the depth. `queue_size` and `latch` are keyword
    arguments in essentially all rospy code for exactly that reason.
    Unknown stays None/False — never a guess. Reuses the same {"depth",
    "reliability", "durability"} shape the ROS 2 scanner used for QoS so
    the web UI's qosText() needs no changes: "reliability" is always None
    (no ROS 1 concept), "durability" carries "transient_local" when
    latched, else "volatile".
    """
    depth = None
    latch = False
    for kw in call.keywords:
        if kw.arg == "queue_size" and isinstance(kw.value, ast.Constant) \
                and isinstance(kw.value.value, int):
            depth = kw.value.value
        elif kw.arg == "latch" and isinstance(kw.value, ast.Constant) \
                and isinstance(kw.value.value, bool):
            latch = kw.value.value
    return {
        "depth": depth,
        "reliability": None,
        "durability": "transient_local" if latch else "volatile",
    }


def _walk_skip_nested_classes(node):
    """Yield descendants of `node` without descending into nested ClassDefs."""
    stack = list(ast.iter_child_nodes(node))
    while stack:
        child = stack.pop()
        yield child
        if not isinstance(child, ast.ClassDef):
            stack.extend(ast.iter_child_nodes(child))


class _Scope:
    """Best-effort constant environment for one class (or the module level)."""

    def __init__(self, module_consts):
        self.module_consts = module_consts
        self.assigns = {}  # "x" or "self.x" -> str value

    def collect_assigns(self, body_nodes):
        for n in body_nodes:
            if not (isinstance(n, ast.Assign) and len(n.targets) == 1):
                continue
            target = n.targets[0]
            if isinstance(target, ast.Name):
                name = target.id
            elif (
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
            ):
                name = "self." + target.attr
            else:
                continue
            val = self.resolve(n.value)
            if val is not None:
                self.assigns[name] = val

    def resolve(self, expr):
        """Resolve `expr` to a string, or None if it is not statically known."""
        lit = _const_str(expr)
        if lit is not None:
            return lit
        default = _get_param_default(expr)
        if default is not None:
            return default
        if isinstance(expr, ast.Name):
            return self.assigns.get(expr.id, self.module_consts.get(expr.id))
        if (
            isinstance(expr, ast.Attribute)
            and isinstance(expr.value, ast.Name)
            and expr.value.id == "self"
        ):
            return self.assigns.get("self." + expr.attr)
        return None


def _msg_type_name(expr, msg_imports):
    """Render the message-type argument, expanding `from x.msg import Y` imports."""
    if isinstance(expr, ast.Name) and expr.id in msg_imports:
        return msg_imports[expr.id]
    try:
        return ast.unparse(expr)
    except Exception:  # unparse failure must never kill the scan
        return "?"


def _node_name_for_scope(body_nodes, fallback):
    """Extract the literal from `rospy.init_node("name", ...)` in this scope."""
    for n in body_nodes:
        if (
            isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr == "init_node"
            and n.args
        ):
            name = _const_str(n.args[0])
            if name is not None:
                return name
    return fallback


def _file_node_name(tree):
    """The `rospy.init_node("name")` literal anywhere in the file, else None.

    ROS 1's dominant idiom splits what ROS 2 keeps together: the pub/sub
    calls live in a class, but `init_node` is called in `main()` outside
    it. Falling back to the class name there would label the node with
    something the running graph never uses — and the live overlay matches
    static names against live master names, so such a node could never be
    marked running. One process is one ROS 1 node, so a file-level
    `init_node` literal is the right name for every pub/sub in that file
    that has no closer one.
    """
    return _node_name_for_scope(list(ast.walk(tree)), None)


class GraphBuilder:
    """Accumulates nodes / topics / edges across all scanned files."""

    def __init__(self):
        self.nodes = {}  # id -> record
        self.topics = {}  # name -> record
        self.edges = []
        self._edge_keys = set()
        self.dynamic_sites = []  # coverage: unresolved call sites
        self.parse_errors = []
        self.files_scanned = 0
        self.cpp_files_scanned = 0

    def add_call(self, call, scope, node_id, msg_imports, rel_file):
        kind = "pub" if call.func.attr == "Publisher" else "sub"
        if len(call.args) < 2:
            return
        topic_expr = call.args[0]
        msg_type = _msg_type_name(call.args[1], msg_imports)
        qos = _ros1_queue_info(call)
        topic = scope.resolve(topic_expr)
        if topic is None:
            try:
                expr_text = ast.unparse(topic_expr)
            except Exception:
                expr_text = "<unparseable>"
        else:
            expr_text = None
        self.record(kind, msg_type, topic, expr_text, qos, node_id, rel_file, call.lineno)

    def record(self, kind, msg_type, topic, expr_text, qos, node_id, rel_file, line):
        """Shared edge/topic recorder for the Python and C++ scanners.

        `topic` is the resolved name, or None with `expr_text` describing
        the unresolvable expression (becomes a "?<expr>" placeholder).
        """
        dynamic = topic is None
        if dynamic:
            topic = "?" + expr_text
            self.dynamic_sites.append(
                {"file": rel_file, "line": line, "expr": expr_text, "kind": kind}
            )
        trec = self.topics.setdefault(
            topic, {"name": topic, "msg_types": [], "dynamic": dynamic}
        )
        if msg_type not in trec["msg_types"]:
            trec["msg_types"].append(msg_type)
        if kind == "pub":
            source, target = node_id, "topic:" + topic
        else:
            source, target = "topic:" + topic, node_id
        key = (source, target, kind)
        if key not in self._edge_keys:
            self._edge_keys.add(key)
            self.edges.append(
                {"source": source, "target": target, "kind": kind,
                 "msg_type": msg_type, "qos": qos}
            )

    def ensure_node(self, node_id, node_name, package, rel_file, class_name, is_test,
                    language="python"):
        self.nodes.setdefault(
            node_id,
            {
                "id": node_id,
                "node_name": node_name,
                "package": package,
                "source_file": rel_file,
                "class_name": class_name,
                "is_test": is_test,
                "language": language,
            },
        )


# --------------------------------------------------------------------------
# C++ (roscpp) heuristic scanner — regex-based, not a real C++ parse.
# Covers the dominant idioms: nh.advertise<T>("topic", queue_size, latch) /
# nh.subscribe<T>("topic", queue_size, callback), ros::init(argc, argv,
# "name") for node naming, and literal queue/latch spellings. Composed
# nodelets, remappings, and macro-built names are NOT resolved (README
# states this); a variable topic becomes the same "?<expr>" placeholder
# the Python scanner uses.

_CPP_CALL_RE = re.compile(
    r"\b\w+\s*(?:\.|->)\s*(advertise|subscribe)\s*<\s*([A-Za-z0-9_:\s]+?)\s*>\s*\("
)
_CPP_INIT_NAME_RE = re.compile(r'\bros::init\s*\(\s*\w+\s*,\s*\w+\s*,\s*"([^"]+)"')
_CPP_CLASS_RE = re.compile(r"class\s+(\w+)\b")


def _cpp_strip_comments(text):
    """Remove comments, preserving line numbers for reported call sites."""
    text = re.sub(
        r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), text, flags=re.S
    )
    return re.sub(r"//[^\n]*", "", text)


def _cpp_call_args(text, start, max_args=4, max_span=2000):
    """Split the top-level arguments of a call whose '(' is at start-1."""
    args = []
    cur = []
    depth = 1
    in_str = False
    i = start
    end = min(len(text), start + max_span)
    while i < end and depth > 0 and len(args) < max_args:
        ch = text[i]
        if in_str:
            cur.append(ch)
            if ch == '"' and text[i - 1] != "\\":
                in_str = False
        elif ch == '"':
            in_str = True
            cur.append(ch)
        elif ch in "([{":
            depth += 1
            cur.append(ch)
        elif ch in ")]}":
            depth -= 1
            if depth > 0:
                cur.append(ch)
        elif ch == "," and depth == 1:
            args.append("".join(cur).strip())
            cur = []
        else:
            cur.append(ch)
        i += 1
    if cur and len(args) < max_args:
        args.append("".join(cur).strip())
    return args


def _ros1_cpp_queue(args):
    """Best-effort queue_size/latch from roscpp advertise/subscribe args.

    Signature shape: advertise<T>("topic", queue_size[, latch]);
    subscribe<T>("topic", queue_size, callback). `args[1]` is always the
    queue length when it's a bare integer literal; a trailing bare
    `true`/`false` (advertise's optional 3rd arg) sets latch.
    """
    info = {"depth": None, "reliability": None, "durability": "volatile"}
    if len(args) > 1 and re.fullmatch(r"\d+", args[1].strip()):
        info["depth"] = int(args[1].strip())
    for a in args[2:]:
        a = a.strip()
        if a == "true":
            info["durability"] = "transient_local"
        elif a == "false":
            info["durability"] = "volatile"
    return info


def _scan_cpp_file(path, src_root, builder):
    rel_file = str(path.relative_to(src_root))
    package = path.relative_to(src_root).parts[0]
    is_test = "test" in path.relative_to(src_root).parts[1:-1]
    try:
        text = _cpp_strip_comments(path.read_text(encoding="utf-8", errors="replace"))
    except OSError as exc:
        builder.parse_errors.append({"file": rel_file, "error": str(exc)})
        return
    builder.files_scanned += 1
    builder.cpp_files_scanned += 1

    calls = list(_CPP_CALL_RE.finditer(text))
    if not calls:
        return
    # Whole-file node attribution (heuristic): the literal in ros::init(...),
    # else the first class name found, else the file stem.
    init_m = _CPP_INIT_NAME_RE.search(text)
    class_m = _CPP_CLASS_RE.search(text)
    node_name = (
        init_m.group(1) if init_m else (class_m.group(1) if class_m else path.stem)
    )
    node_id = "node:{}/{}".format(package, node_name)
    builder.ensure_node(
        node_id, node_name, package, rel_file,
        class_m.group(1) if class_m else None, is_test, language="cpp",
    )
    for m in calls:
        kind = "pub" if m.group(1) == "advertise" else "sub"
        msg_type = re.sub(r"\s+", "", m.group(2)).replace("::", "/")
        args = _cpp_call_args(text, m.end())
        if not args:
            continue
        topic_arg = args[0]
        lit = re.fullmatch(r'"([^"]*)"', topic_arg)
        topic = lit.group(1) if lit else None
        expr_text = None if lit else re.sub(r"\s+", " ", topic_arg)
        qos = _ros1_cpp_queue(args)
        line = text.count("\n", 0, m.start()) + 1
        builder.record(kind, msg_type, topic, expr_text, qos, node_id, rel_file, line)


def _scan_file(path, src_root, builder):
    rel_file = str(path.relative_to(src_root))
    package = path.relative_to(src_root).parts[0]
    is_test = "test" in path.relative_to(src_root).parts[1:-1]
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (SyntaxError, UnicodeDecodeError) as exc:
        builder.parse_errors.append({"file": rel_file, "error": str(exc)})
        return
    builder.files_scanned += 1

    msg_imports = {}
    module_consts = {}
    for n in ast.walk(tree):
        if isinstance(n, ast.ImportFrom) and n.module and n.module.endswith(".msg"):
            for alias in n.names:
                msg_imports[alias.asname or alias.name] = (
                    n.module[: -len(".msg")] + "/" + alias.name
                )
    for stmt in tree.body:
        if isinstance(stmt, ast.Assign) and len(stmt.targets) == 1:
            target = stmt.targets[0]
            if isinstance(target, ast.Name):
                val = _const_str(stmt.value)
                if val is not None:
                    module_consts[target.id] = val

    # Name resolution order for a class: its own init_node literal, then
    # the file's (rospy's dominant idiom calls init_node in main(), not in
    # the class), then the class name.
    file_node_name = _file_node_name(tree)

    classes = [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]
    for cls in classes:
        body = list(_walk_skip_nested_classes(cls))
        calls = [
            n
            for n in body
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr in _PUBSUB
        ]
        if not calls:
            continue
        scope = _Scope(module_consts)
        scope.collect_assigns(body)
        node_name = _node_name_for_scope(body, file_node_name or cls.name)
        node_id = "node:{}/{}".format(package, node_name)
        builder.ensure_node(node_id, node_name, package, rel_file, cls.name, is_test)
        for call in calls:
            builder.add_call(call, scope, node_id, msg_imports, rel_file)

    # Calls made outside any class (bench scripts, test drivers, plain
    # rospy scripts — a very common ROS 1 style with no Node-like class at
    # all) are attributed to a file-stem pseudo-node so they still appear.
    module_body = list(_walk_skip_nested_classes(tree))
    module_calls = [
        n
        for n in module_body
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute)
        and n.func.attr in _PUBSUB
    ]
    if module_calls:
        scope = _Scope(module_consts)
        scope.collect_assigns(module_body)
        node_name = _node_name_for_scope(module_body, path.stem)
        node_id = "node:{}/{}".format(package, node_name)
        builder.ensure_node(node_id, node_name, package, rel_file, None, is_test)
        for call in module_calls:
            builder.add_call(call, scope, node_id, msg_imports, rel_file)


def compute_reachability(edges):
    """Transitive up/down closure for every element that touches an edge.

    Returns {id: {"up": [ancestor ids], "down": [descendant ids]}} following
    the REAL directed graph (cycles included, BFS with a visited set, so a
    test harness publishing upstream cannot loop it). The element itself is
    not listed in its own closures even when it sits on a cycle.
    """
    fwd = {}
    rev = {}
    for e in edges:
        fwd.setdefault(e["source"], []).append(e["target"])
        rev.setdefault(e["target"], []).append(e["source"])

    def _bfs(start, adj):
        seen = set()
        queue = [start]
        while queue:
            for nxt in adj.get(queue.pop(), []):
                if nxt != start and nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
        return sorted(seen)

    return {
        i: {"up": _bfs(i, rev), "down": _bfs(i, fwd)}
        for i in set(fwd) | set(rev)
    }


def compute_ego(edges, center, max_depth=None):
    """Signed BFS distance from `center` for its ego-graph.

    Negative = upstream (inputs), positive = downstream (outputs), 0 = the
    center itself. `max_depth` clamps the BFS radius per side (None = whole
    transitive closure): the focus panel uses 2 for a node center (its
    topics at ±1 plus the "before"/"after" nodes at ±2) and 1 for a topic
    center (its publishers/subscribers). An element reachable in BOTH
    directions (harness cycles) appears exactly once, deterministically:
    the side with the smaller absolute distance wins, ties go upstream.
    Returns {"center", "levels": {id: signed int}, "dual": [ids that were
    reachable both ways]}.
    """
    fwd = {}
    rev = {}
    for e in edges:
        fwd.setdefault(e["source"], []).append(e["target"])
        rev.setdefault(e["target"], []).append(e["source"])

    def _dists(adj):
        dist = {}
        frontier = [center]
        d = 0
        while frontier:
            if max_depth is not None and d >= max_depth:
                break
            d += 1
            nxt = []
            for u in frontier:
                for v in adj.get(u, []):
                    if v != center and v not in dist:
                        dist[v] = d
                        nxt.append(v)
            frontier = nxt
        return dist

    up = _dists(rev)
    down = _dists(fwd)
    levels = {center: 0}
    dual = []
    for elem in set(up) | set(down):
        if elem in up and elem in down:
            dual.append(elem)
            levels[elem] = -up[elem] if up[elem] <= down[elem] else down[elem]
        elif elem in up:
            levels[elem] = -up[elem]
        else:
            levels[elem] = down[elem]
    return {"center": center, "levels": levels, "dual": sorted(dual)}


def scan_workspace(src_root):
    """Scan every Python file under `src_root`; return the graph as a dict."""
    src_root = Path(src_root).resolve()
    builder = GraphBuilder()
    for path in _iter_py_files(src_root):
        _scan_file(path, src_root, builder)
    for path in _iter_cpp_files(src_root):
        _scan_cpp_file(path, src_root, builder)

    topics = []
    for name, trec in sorted(builder.topics.items()):
        topics.append(
            {
                "name": name,
                "msg_type": " | ".join(trec["msg_types"]),
                "dynamic": trec["dynamic"],
            }
        )
    summary = {
        "files_scanned": builder.files_scanned,
        "cpp_files_scanned": builder.cpp_files_scanned,
        "parse_errors": builder.parse_errors,
        "node_count": len(builder.nodes),
        "topic_count": len(topics),
        "edge_count": len(builder.edges),
        "dynamic_topic_count": sum(1 for t in topics if t["dynamic"]),
        "dynamic_call_sites": builder.dynamic_sites,
    }
    return {
        "generated": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "src_root": str(src_root),
        "nodes": sorted(builder.nodes.values(), key=lambda n: n["id"]),
        "topics": topics,
        "edges": builder.edges,
        # Per-element transitive up/down reachability, precomputed here so
        # the web view's hover-chain highlight is trivial (and testable).
        "closures": compute_reachability(builder.edges),
        "summary": summary,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Statically scan catkin_ws/src for the declared ROS graph."
    )
    parser.add_argument(
        "--src",
        type=Path,
        default=None,
        help="workspace src directory (default: auto-detect a src/ with packages upward from cwd)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("graph.json"),
        help="output JSON path (default: ./graph.json)",
    )
    args = parser.parse_args(argv)

    src_root = args.src or _find_src_root(Path.cwd())
    if src_root is None or not Path(src_root).is_dir():
        print(
            "error: could not find a catkin src/ directory with packages by "
            "walking up from the current directory; pass --src /path/to/ws/src",
            file=sys.stderr,
        )
        return 2

    graph = scan_workspace(src_root)
    args.output.write_text(json.dumps(graph, indent=2) + "\n", encoding="utf-8")

    s = graph["summary"]
    print("scanned {} files under {}".format(s["files_scanned"], graph["src_root"]))
    print(
        "graph: {} nodes, {} topics, {} edges "
        "({} dynamic/unresolved topic names)".format(
            s["node_count"], s["topic_count"], s["edge_count"], s["dynamic_topic_count"]
        )
    )
    for site in s["dynamic_call_sites"]:
        print(
            "  dynamic: {}:{} ({}) topic expr: {}".format(
                site["file"], site["line"], site["kind"], site["expr"]
            )
        )
    for err in s["parse_errors"]:
        print("  parse error: {}: {}".format(err["file"], err["error"]))
    print("wrote {}".format(args.output.resolve()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
