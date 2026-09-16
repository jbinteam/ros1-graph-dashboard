# Copyright 2026 JB
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""bench_test — one-command self-test of the scanner and dashboard server.

Follows the sibling ROS 2 package's bench pattern: run everything
end-to-end, print one line per check and an N/N summary, exit nonzero on
any failure. Boots the real HTTP server on an EPHEMERAL port (never 8092,
so it cannot collide with a dashboard the advisor has open) and tears it
down cleanly. The /api/live check accepts "unavailable" as a pass with a
note — the live overlay is optional by design.
"""
import argparse
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
import urllib.error
import urllib.parse
import urllib.request

from graph_dashboard import server as srv
from graph_dashboard.scanner import _find_src_root, scan_workspace

# Repo-contract mode (a pinned set of expected edges checked whenever this
# package is deployed inside a specific reference workspace) is left empty
# here: unlike the ROS 2 sibling package, there is no ROS 1 reference
# workspace on this machine to pin edges against yet. `repo_mode` below
# naturally stays False for every deployment until someone fills this in
# for their own workspace, exercising the same generic-workspace skip path
# a standalone install of the ROS 2 version already relies on.
CONTRACT_EDGES = []

_EGO_CENTER = None


def _get(url, timeout=10.0):
    """GET a URL, returning (status, body) without raising on HTTP errors."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return resp.status, resp.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read()


class _Checks:
    def __init__(self):
        self.results = []

    def check(self, name, ok, note=""):
        self.results.append(ok)
        line = "  [{}] {}".format("PASS" if ok else "FAIL", name)
        if note:
            line += " — " + note
        print(line)
        return ok


def main(argv=None):
    parser = argparse.ArgumentParser(description="graph_dashboard end-to-end self-test.")
    parser.add_argument("--src", type=Path, default=None,
                        help="workspace src directory (default: auto-detect)")
    args = parser.parse_args(argv)
    src_root = args.src or _find_src_root(Path.cwd())
    if src_root is None or not Path(src_root).is_dir():
        print("error: could not find a catkin src/ directory with packages; "
              "pass --src", file=sys.stderr)
        return 2
    src_root = Path(src_root).resolve()

    c = _Checks()
    print("bench_test: scanning {}".format(src_root))
    graph = scan_workspace(src_root)
    s = graph["summary"]
    print("  graph: {} nodes, {} topics, {} edges, {} dynamic".format(
        s["node_count"], s["topic_count"], s["edge_count"], s["dynamic_topic_count"]))
    repo_mode = bool(CONTRACT_EDGES) and _EGO_CENTER in {n["id"] for n in graph["nodes"]}
    if repo_mode:
        edge_set = {(e["source"], e["target"]) for e in graph["edges"]}
        for src_id, dst_id in CONTRACT_EDGES:
            c.check("contract edge {} -> {}".format(src_id, dst_id),
                    (src_id, dst_id) in edge_set)
    else:
        print("  note: no reference workspace pinned — contract and ego-clamp "
              "checks skipped (generic workspace mode)")
    c.check("no parse errors", not s["parse_errors"],
            str(s["parse_errors"]) if s["parse_errors"] else "")
    c.check("closures present", "closures" in graph)

    print("bench_test: booting server on an ephemeral port")
    srv._Handler.src_root = src_root
    srv._Handler.prober = srv.LiveProber()
    srv._Handler.tap_manager = srv.TapManager()
    srv._Handler.scan_cache = srv._ScanCache()
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), srv._Handler)
    port = httpd.server_address[1]
    thread = threading.Thread(target=httpd.serve_forever, daemon=True, name="bench-http")
    thread.start()
    base = "http://127.0.0.1:{}".format(port)
    try:
        status, body = _get(base + "/")
        c.check("GET / is 200 html", status == 200 and b"graph_dashboard" in body)

        status, body = _get(base + "/api/graph")
        ok = status == 200
        if ok:
            api = json.loads(body)
            ok = api["summary"]["edge_count"] == s["edge_count"]
        c.check("GET /api/graph matches direct scan", ok)

        status, body = _get(base + "/api/live")
        live = json.loads(body) if status == 200 else {}
        note = "" if live.get("available") else "unavailable: {}".format(
            live.get("reason", "?"))
        c.check("GET /api/live well-formed", status == 200 and "available" in live, note)

        if repo_mode:
            status, body = _get(base + "/api/ego?id=" + urllib.parse.quote(_EGO_CENTER))
            ok = status == 200
            c.check("GET /api/ego on contract center", ok)

        status, _ = _get(base + "/api/ego?id=nonsense")
        c.check("GET /api/ego unknown id is 404", status == 404)

        status, _ = _get(base + "/api/tap?id=nonsense")
        c.check("GET /api/tap unknown id is 404", status == 404)
        if graph["topics"]:
            tid = "topic:" + graph["topics"][0]["name"]
            status, body = _get(base + "/api/tap?id=" + urllib.parse.quote(tid))
            ok = status == 200 and "status" in json.loads(body)
            c.check("GET /api/tap dead topic well-formed JSON", ok,
                    json.loads(body).get("status", "") if status == 200 else "")

        status, body = _get(base + "/vendor/vis-network.min.js")
        c.check("vendored vis-network served", status == 200 and len(body) > 100_000,
                "{} bytes".format(len(body)))
    finally:
        httpd.shutdown()
        httpd.server_close()
        thread.join(timeout=5)
    c.check("server shut down cleanly", not thread.is_alive())

    passed = sum(c.results)
    total = len(c.results)
    print("bench_test: {}/{} checks passed".format(passed, total))
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
