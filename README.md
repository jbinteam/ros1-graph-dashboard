# ros1-graph-dashboard

[![ROS 2 version](https://img.shields.io/badge/ROS%202-ros2--graph--dashboard-22314E?logo=ros&logoColor=white)](https://github.com/jbinteam/ros2-graph-dashboard)

**rqt_graph shows you what's running right now. This shows you the whole project.**

A zero-dependency web dashboard that draws the *complete* ROS 1 node/topic
graph of a workspace by statically scanning its source code — every node
and topic declared anywhere in `src/`, whether or not it is currently
running — then overlays live reality on top:

- **Static big picture** — Python (`rospy`, full AST analysis) and C++
  (`roscpp`, heuristic scan, dark ring) nodes, topics, queue/latch info
  where statically visible, test harnesses dashed. Re-scanned on every
  page reload.
- **Live overlay** — running nodes light up green (ROS master system state
  sampled every 2 s via XML-RPC, no live node ever created); nodes and
  topics created only at runtime (dynamic names, nodelets) are laid out
  and connected as first-class dotted elements, so the picture never
  lies — and they can be focused and tapped like anything else.
- **Focus paths** — click a node or topic to open its neighborhood below;
  the upper graph keeps every upstream root and downstream path bold in
  blue, including live connections, while unrelated branches dim. Walking
  the focus panel updates the highlighted paths.
- **Topic tap** — focus a topic to see its true measured Hz / bandwidth and
  the newest message, via an on-demand best-effort raw (`AnyMsg`)
  subscription that self-releases and is verified not to disturb the
  pipeline it watches. Color images render as live thumbnails; **depth
  images** (16UC1/32FC1) get a Turbo colormap with the measured min–max
  range in meters; **pointclouds** (PointCloud2) render as an interactive
  3D orbit view (≤30k points, rgb or depth-gradient coloring, zero extra
  dependencies); everything else falls back to a field tree.
- **Noise controls** — legend chips hide test harnesses, image_transport
  variants (`/compressed`, `/theora`, …), or every idle node (active-only
  view) with one click. Topic type chips show just the message types you
  select, including topics discovered only at runtime.
- **Per-team deep links** — `/?focus=<node>&hide=pkg1,tests,activeonly`
  gives each sub-team a scoped view of their corner of the system.

## Quick start

Easiest — the self-contained installer (no clone needed, works offline):

```bash
cp install.sh ~/my_catkin_ws/     # any workspace folder, or an empty dir
cd ~/my_catkin_ws
bash install.sh                   # extracts, builds, self-tests
source devel/setup.bash
rosrun graph_dashboard serve      # → http://127.0.0.1:8092
```

Or clone into an existing workspace:

```bash
cd ~/my_catkin_ws/src
git clone https://github.com/jbinteam/ros1-graph-dashboard.git
ln -s ros1-graph-dashboard/graph_dashboard graph_dashboard  # or copy the folder
cd .. && catkin_make
source devel/setup.bash
rosrun graph_dashboard serve   # run from the workspace root
```

## Updating an existing installation

The message-type filter and bold focus paths are dashboard page changes;
they add no dependencies. Update the package that your dashboard actually
uses, then hard-refresh the page with **Ctrl+Shift+R**.

**Git checkout linked into a catkin workspace using `devel/setup.bash`:**

```bash
cd ~/my_catkin_ws/src/ros1-graph-dashboard  # use your existing clone's path
git pull --ff-only
```

For this update, a browser hard refresh is enough: the dashboard reads the
page from source on each request. No rebuild or dashboard restart is
needed. If you copied `graph_dashboard/` into `src/` instead of linking it,
copy the updated package there again before refreshing.

**Installed using the self-contained `install.sh`:** pulling the Git repo
does not update the separate package extracted into your workspace. Stop
the dashboard with **Ctrl+C** in its terminal, then run:

```bash
# From your existing Git clone:
git pull --ff-only
cp install.sh ~/my_catkin_ws/install.sh

cd ~/my_catkin_ws
source /opt/ros/noetic/setup.bash
bash install.sh --force
source devel/setup.bash
rosrun graph_dashboard serve
```

The installer keeps the previous package in a timestamped backup, replaces
the extracted copy, rebuilds this package, and runs its self-test. Reuse
any `--src`, `--host`, or `--port` options from your previous dashboard
command, then hard-refresh the browser.

If you run from a catkin **`install/` directory** rather than `devel/`,
rerun your usual `catkin_make install` step after updating the source so
the installed web assets are refreshed. Source `install/setup.bash`,
restart the dashboard, and hard-refresh its page.

## Compatibility

ROS 1 Noetic only. Python packages get full static coverage; C++ coverage
is heuristic — nodelets, remappings, and macro-built names show as
placeholders or fall back to the live overlay. ROS 2 is not supported here
— see the [ROS 2 version](https://github.com/jbinteam/ros2-graph-dashboard).
Full details, URL parameters, API, and design rules:
[`graph_dashboard/README.md`](graph_dashboard/README.md).

## Repo layout

| Path | What |
|------|------|
| `graph_dashboard/` | the catkin package (scanner, server, web UI, `bench_test`) |
| `install.sh` | self-extracting installer carrying the package (regenerate after changes) |
| `scripts/make_install.sh` | regenerates `install.sh` from `graph_dashboard/` |

## License

MIT. The web UI vendors [vis-network](https://visjs.org) (MIT/Apache-2.0).
