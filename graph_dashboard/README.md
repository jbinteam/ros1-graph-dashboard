# graph_dashboard

Local web dashboard showing the **whole project's declared ROS graph** —
every node, topic, and pub/sub edge found by statically scanning the Python
sources under `catkin_ws/src/` — with a live overlay painted on top.

`rqt_graph` shows only what is running *right now*. This dashboard shows the
architecture as the source code declares it, whether or not anything is
running, and then marks what actually *is* running. That is the view a team
needs when work is split across packages: the full picture, who publishes
what, and what your change might ripple into.

**Scanned, never executed:** the scanner parses source text with Python's
`ast` module. It never imports or runs any scanned package — frozen or
vendored sources are included in the picture without a single line of them
executing.

This is the **ROS 1 (Noetic)** port of the sibling
[`ros2-graph-dashboard`](../ros2-graph-dashboard) project — same product,
adapted to rospy/roscpp/catkin and the ROS 1 master-based graph model
(no DDS, no QoS).

## Quickstart

```bash
source /opt/ros/noetic/setup.bash && source catkin_ws/devel/setup.bash

rosrun graph_dashboard scan          # writes ./graph.json, prints a summary
rosrun graph_dashboard serve         # dashboard on http://127.0.0.1:8092/
rosrun graph_dashboard bench_test    # end-to-end self-test (ephemeral port)
```

All three auto-detect the workspace `src/` directory (any workspace name)
by walking up from the current directory until they find a `src/`
containing at least one `package.xml` — run them from the workspace root,
from inside `src/`, or from a package directory. Pass
`--src /path/to/ws/src` to point elsewhere. `serve`
also takes `--host` (default 127.0.0.1) and `--port` (default **8092** —
8091 belongs to the sibling ROS 2 dashboard, so both can run at once on the
same machine). `scan` takes `--output` (default `./graph.json`).

## Using the page

- **Hover** a node or topic: lights its full transitive chain (everything
  that can affect it or be affected by it, cycles included), dims the rest.
- **Click** a node or topic: splits the page; the bottom **focus panel**
  shows the direct neighborhood — for a node, its topics (±1) and the
  before/after nodes (±2); for a topic, its publishers and subscribers.
  Click inside the panel to walk the chain hop by hop; `Esc` or ✕ closes.
  Focusing a **topic** also opens a live tap strip (measured Hz, bandwidth,
  latest message as thumbnail or field tree) — the server subscribes to
  that one topic only while the panel is open and releases it ~5 s after
  the panel closes. The strip's Hz/bandwidth are the topic's TRUE measured
  rate; the display itself is polled at only 2 Hz and shows the newest
  message each poll, so a choppy preview does not mean a slow topic.
- **Legend chips** are filters: click a package chip to hide/show that
  package (topics hide only when *every* node touching them is hidden);
  "hide test harnesses" hides the dashed bench/test elements. Filters apply
  to the main graph only — the focus panel always shows the true
  neighborhood, even filtered-out elements, so the ego view never lies.
- **Tooltips** carry the details: nodes list their declared pubs/subs with
  message types and best-effort static queue info; topics list publishers
  and subscribers. The bracketed tag shows what the scanner can prove from
  the call site (a literal `queue_size`, `latch=True`); anything else reads
  "unknown" — never a guess.

### URL parameters (shareable per-team deep links)

| Param | Effect | Example |
|-------|--------|---------|
| `?highlight=<name>` | light that element's chain on load | `/?highlight=camera_driver` |
| `?focus=<name>` | open the focus panel on load | `/?focus=/estop` |
| `?hide=<list>` | comma-separated packages and/or `tests` to hide | `/?hide=sim,tests` |

Names accept a node name, a topic name (with or without the leading `/`),
or a full element id (`node:<pkg>/<name>`, `topic:/<name>`). Parameters
compose: `/?focus=camera_driver&hide=sim,tests`.

## API

| Endpoint | Returns |
|----------|---------|
| `GET /api/graph` | full static graph JSON (nodes, topics, edges with queue/latch info, per-element reachability closures, coverage summary) — re-scanned per request, so a reload always reflects the current source |
| `GET /api/ego?id=<element>` | signed-distance ego graph of one element (negative = inputs, positive = outputs), clamped to the direct neighborhood (±2 for nodes, ±1 for topics); 404 for unknown ids |
| `GET /api/live` | live-graph snapshot: running nodes and topic endpoint counts, sampled every ~2 s from the ROS master |
| `GET /api/tap?id=topic:/<name>` | on-demand live tap of ONE topic: Hz, bandwidth, latest message (JPEG thumbnail for images, truncated field tree otherwise). Poll-driven lifecycle — first poll subscribes (raw, best-effort), ~5 s without polls unsubscribes; max 3 concurrent taps; honest JSON statuses for dead topics; 404 for unknown ids |
| `GET /` | the dashboard page |
| `GET /vendor/vis-network.min.js` | vendored render library (offline lab use) |

## Design rules

- **Topic-name resolution** (static, in order): string literal at the call
  site → simple local/self/module-constant assignment →
  `rospy.get_param("key", default)`'s **default** literal. Unresolvable
  names appear as `?<expr>` placeholders and are counted in the coverage
  summary. The static view shows declared defaults; runtime roslaunch
  remaps or parameter-server overrides are not applied.
- **Node naming:** the literal in the nearest `rospy.init_node("name")`
  call in the same class, else the file's `init_node` literal wherever it
  sits (rospy's dominant idiom puts pub/sub in a class but calls
  `init_node` in `main()`, and one process is one ROS 1 node), else the
  class name, else the file stem — this matches idiomatic rospy, which has
  no `Node`-subclass convention. Getting this right matters beyond labels:
  the live overlay matches these names against live master names, so a
  class-name guess would leave a running node permanently marked idle.
- **Queue info** comes from the `queue_size=` / `latch=` **keywords**
  only. rospy's positional slots after the topic and type are
  `subscriber_listener` (Publisher) and `callback` (Subscriber), never the
  queue size, so a positional read there would report another argument's
  value as the depth.
- **C++ (roscpp) coverage is heuristic**, not a real C++ parse: it finds
  `nh.advertise<T>("topic", queue_size, latch)` / `nh.subscribe<T>(...)`
  call sites in `*.cpp/*.cc/*.hpp/*.hh` through either a value or a
  pointer NodeHandle (`nh.advertise<T>` and `nh_->advertise<T>` alike;
  the templated spelling only — callback-signature type inference is out
  of scope), takes the topic from
  the string literal (a variable becomes the same `?<expr>` placeholder),
  the message type from the template argument, the node name from a
  `ros::init(argc, argv, "name")` literal (else the first class found in
  the file, else the file stem), and the queue/latch info only from
  literal spellings (an integer queue length, a trailing `true`/`false`
  latch flag). **Known limits:** nodelets, launch-time remappings, and
  macro-built names are not resolved — they show as placeholders or are
  missed in the static view. Anything actually running still appears via
  the live overlay, so the picture degrades to "live-only", never to
  invisible. C++ nodes carry a dark ring and a "C++ (heuristic scan)"
  tooltip tag.
- **Layout:** left-to-right longest-path layering over the acyclic part of
  the graph. The graph can genuinely contain cycles (test harnesses
  subscribe downstream and publish upstream), so back edges are found by
  DFS first and excluded from layering — they simply draw right-to-left,
  as rqt_graph does. Column order minimizes crossings (barycenter pass).
- **Hover chain:** true transitive closure over the real directed graph,
  cycles included — harness feedback paths deliberately light up.
- **Ego placement:** BFS distance from the focused element, clamped as
  above. An element reachable both upstream and downstream appears exactly
  once: the side with the smaller |distance| wins, ties go upstream (the
  tooltip says so when it happens).

## Live overlay

A background thread in `serve` polls the ROS 1 master's system state
(`rosgraph.Master.getSystemState()` / `getTopicTypes()`) every ~2 s —
pure XML-RPC queries against `ROS_MASTER_URI`, so unlike the ROS 2 version
**no live node is ever created**; nothing shows up in `rosnode list` as a
side effect. A static node is marked **running** when its declared node
name (fully-qualified, e.g. `/camera_driver`) matches a live node's name.
Running nodes render saturated with a bold green border; declared-but-idle
nodes dim; live nodes that appear in no source file (`rviz`, an anonymous
`rostopic echo`, …) render as dotted ellipses beneath the graph so the
picture never hides a running process.

**Graceful degradation:** `rosgraph` is imported lazily inside the probe
thread. Where ROS isn't sourced, or no `roscore` is reachable, `/api/live`
reports `{"available": false, "reason": ...}`, the page shows a "live:
unavailable" chip, and everything static keeps working.

## Compatibility

- **ROS 1 (Noetic) only.** ROS 2 is unsupported here — see the sibling
  [`ros2-graph-dashboard`](../ros2-graph-dashboard) repo for that graph
  model and client API.
- **Python packages get full static coverage** (AST scan, parameter-default
  resolution, queue/latch extraction). **C++ packages get heuristic
  coverage** — see the design rules above for exactly what is and isn't
  resolved.
- **The live overlay and topic taps are language-agnostic**: they observe
  the ROS master graph itself, so C++, Python, and nodelet-hosted nodes all
  appear when running regardless of what the static scan could see.

## Installing into another workspace

Generate a self-contained installer and hand the single file to a teammate:

```bash
bash scripts/make_install.sh      # from the repo root — writes ./install.sh
```

`install.sh` carries the whole package as an embedded tarball (no git, no
network, no root). Dropped into ANY directory and run with
`bash install.sh`, it creates or reuses `./src`, checks the environment
(sourcing `/opt/ros/noetic/setup.bash` if nothing is sourced, with
actionable errors otherwise), initializes the catkin workspace if needed,
extracts, builds **only this package** (`catkin_make --only-pkg-with-deps
graph_dashboard`, so dropping it into a populated workspace can't be
derailed by an unrelated package, with the whitelist cleared from the
CMake cache afterwards), and runs the package self-test on an
ephemeral port. Re-running refuses to overwrite an existing
`src/graph_dashboard` unless given `--force`, which keeps a timestamped
backup at the workspace root. In a workspace without a pinned reference
graph, `bench_test` automatically skips the repo-contract checks and runs
its generic subset.

## Vendored third-party code

`graph_dashboard/web/vis-network.min.js` — vis-network 9.1.9
(<https://visjs.github.io/vis-network/>), vendored for offline lab use.
Dual-licensed Apache-2.0 / MIT; the license banner is retained at the top
of the vendored file.

## Tests

`python3 -m pytest test/` runs unit tests for the scanner's resolution
tiers, node-naming fallbacks, C++ value/pointer NodeHandle spellings,
reachability closures (cycle-safe), and ego depth clamping, plus the
live-only-element HTTP tests. Inside a built workspace,
`catkin_make run_tests --pkg graph_dashboard` runs the same suite —
`CMakeLists.txt` registers pytest through `catkin_run_tests_target` rather
than `catkin_add_nosetests`, because these tests use pytest fixtures
(`tmp_path`, `monkeypatch`) that nose cannot supply.
`rosrun graph_dashboard bench_test` is the
end-to-end check: a fresh scan, every endpoint of a real server boot on an
ephemeral port, the 404 path, the vendored asset, and clean shutdown.

The sibling ROS 2 repo also ships an `ament_copyright`/`ament_flake8`/
`ament_pep257`/`ament_xmllint` lint suite via `colcon test`; those are
ROS 2 (`ament`) tooling with no ROS 1 equivalent, so this port ships
functional tests only.
