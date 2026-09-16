#!/usr/bin/env bash
# Copyright 2026 JB
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
#
# make_install.sh — regenerate the self-extracting installer ./install.sh
# from the repo's graph_dashboard/ sources. Run from anywhere:
#   bash scripts/make_install.sh
# Hand the resulting install.sh to a teammate; dropped into any directory
# and run with `bash install.sh`, it turns that directory into (or reuses)
# a ROS 1 catkin workspace with graph_dashboard built and self-tested. The
# installer CARRIES the package as a base64 tarball — no git, no network.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PKG="$ROOT/graph_dashboard"
OUT="$ROOT/install.sh"

[ -d "$PKG" ] || { echo "ERROR: $PKG not found" >&2; exit 1; }

VERSION="$(sed -n 's/.*<version>\(.*\)<\/version>.*/\1/p' "$PKG/package.xml" | head -n1)"
REV="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
STAMP="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

PAYLOAD="$(tar -czf - -C "$ROOT" \
  --exclude='__pycache__' --exclude='*.pyc' --exclude='.pytest_cache' \
  graph_dashboard | base64)"

{
  sed -e "s/@VERSION@/$VERSION/" -e "s/@REV@/$REV/" -e "s/@STAMP@/$STAMP/" <<'HEAD'
#!/usr/bin/env bash
# graph_dashboard self-extracting installer
#   package version: @VERSION@   source rev: @REV@   generated: @STAMP@
#
# Drop this file into any directory and run:  bash install.sh
# The directory becomes (or is reused as) a ROS 1 catkin workspace with the
# graph_dashboard package extracted into ./src, built, and self-tested.
# No network, no root. Re-run with --force to replace an existing install
# (a timestamped .bak of the old package directory is kept).
set -euo pipefail

FORCE=0
for arg in "$@"; do
  case "$arg" in
    --force) FORCE=1 ;;
    -h|--help)
      sed -n '2,9p' "$0"; exit 0 ;;
    *) echo "unknown argument: $arg (only --force is supported)" >&2; exit 2 ;;
  esac
done

say() { printf 'install.sh: %s\n' "$*"; }
die() { printf 'install.sh: ERROR: %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------- workspace
if [ -d src ]; then
  say "using existing ./src (this directory is the workspace root)"
else
  mkdir src
  say "created ./src — this directory is now a ROS 1 catkin workspace root"
fi

if [ -e src/graph_dashboard ]; then
  if [ "$FORCE" -eq 1 ]; then
    # Backup lives OUTSIDE src/ so catkin can never see it as a duplicate
    # package.
    BAK="graph_dashboard.bak.$(date +%Y%m%d-%H%M%S)"
    mv src/graph_dashboard "$BAK"
    say "existing src/graph_dashboard moved to ./$BAK"
  else
    die "src/graph_dashboard already exists — re-run with --force to replace it (the old directory is kept as a timestamped .bak)"
  fi
fi

# ------------------------------------------------------------- environment
command -v python3 >/dev/null 2>&1 \
  || die "python3 not found — install Python 3 first (e.g. sudo apt install python3)"

if [ -z "${ROS_DISTRO:-}" ]; then
  if [ -f /opt/ros/noetic/setup.bash ]; then
    say "ROS 1 not sourced — sourcing /opt/ros/noetic/setup.bash for you (add it to your ~/.bashrc to skip this)"
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
    set -u
  else
    die "no ROS 1 Noetic environment found: 'source /opt/ros/noetic/setup.bash' first, or install ROS 1 Noetic (http://wiki.ros.org/noetic/Installation)"
  fi
else
  say "ROS environment already sourced (ROS_DISTRO=$ROS_DISTRO)"
  if [ "$ROS_DISTRO" != "noetic" ]; then
    say "warning: ROS_DISTRO=$ROS_DISTRO, not noetic — this package targets ROS 1 Noetic; proceeding anyway"
  fi
fi

command -v catkin_make >/dev/null 2>&1 \
  || die "catkin_make not found even after sourcing ROS — your ROS 1 installation looks incomplete"
command -v rosrun >/dev/null 2>&1 \
  || die "rosrun not found even after sourcing ROS — your ROS 1 installation looks incomplete"

if [ ! -f src/CMakeLists.txt ]; then
  say "initializing catkin workspace (catkin_init_workspace) ..."
  catkin_init_workspace src
fi

# ----------------------------------------------------------------- extract
say "extracting graph_dashboard into ./src ..."
base64 -d <<'PAYLOAD_EOF' | tar -xzf - -C src
HEAD
  printf '%s\n' "$PAYLOAD"
  cat <<'TAIL'
PAYLOAD_EOF
[ -f src/graph_dashboard/package.xml ] || die "extraction failed (src/graph_dashboard/package.xml missing)"
say "extracted $(find src/graph_dashboard -type f | wc -l) files"

# ------------------------------------------------------------------- build
say "building (catkin_make) ..."
catkin_make

# ------------------------------------------------------------- self-verify
set +u
# shellcheck disable=SC1091
source devel/setup.bash
set -u
say "running the package self-test (ephemeral port, safe next to a running dashboard) ..."
if rosrun graph_dashboard bench_test --src src; then
  say "self-test passed"
else
  die "self-test FAILED — see the check list above"
fi

# ------------------------------------------------------------------- usage
cat <<'USAGE'

graph_dashboard installed. From this directory:

  source devel/setup.bash
  rosrun graph_dashboard scan          # write graph.json + summary
  rosrun graph_dashboard serve         # dashboard: http://127.0.0.1:8092/
  rosrun graph_dashboard bench_test    # re-run the self-test any time

Deep links for your team:  /?focus=<node-or-topic>  opens the focus panel,
/?hide=<pkg1,pkg2,tests>  hides packages/test harnesses, ?highlight=<name>
lights a full chain. Full docs: src/graph_dashboard/README.md
USAGE
TAIL
} > "$OUT"

chmod +x "$OUT"
SIZE="$(wc -c < "$OUT")"
echo "wrote $OUT ($SIZE bytes, package $VERSION, rev $REV)"

if command -v shellcheck >/dev/null 2>&1; then
  shellcheck "$OUT" && echo "shellcheck: install.sh clean"
  shellcheck "$0" && echo "shellcheck: make_install.sh clean"
else
  echo "shellcheck not available — skipped"
fi
