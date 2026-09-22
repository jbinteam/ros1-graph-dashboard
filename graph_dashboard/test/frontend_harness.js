// Runs the REAL page script from web/index.html under Node, with just
// enough of the DOM and vis-network stubbed for applyLive() and the filter
// rules to execute.
//
// The point is to exercise the shipped frontend rather than a
// reimplementation of it: the bug this was written for (a machine's topics
// staying visible after its nodes were filtered away) lived in logic that
// looked correct when read, and only showed up when the real code ran
// against a real two-machine sample.
//
// Usage:  PAGE=<index.html> node frontend_harness.js  < payload.json
//
// stdin  — one /api/live payload, plus an optional "_host_filter" key
// stdout — JSON: {machines, elements, visible, hidden}
'use strict';
const fs = require('fs');

const pagePath = process.env.PAGE ||
  require('path').join(__dirname, '..', 'graph_dashboard', 'web', 'index.html');
const html = fs.readFileSync(pagePath, 'utf8');
let script = html.split('<script>').pop().split('</script>')[0];

// The page keeps its state in closure variables, which is right for the
// page and awkward for a test. Rather than export hooks from production
// code, the harness injects an accessor into its in-memory copy only.
const HOOK = `
globalThis.__harness = {
  reset() {
    base = {}; nodesDS = new vis.DataSet([]); edgesDS = new vis.DataSet([]);
    staticEdges = []; staticElementIds = []; staticLayoutEdges = [];
    liveOnlyIds = new Set(); liveEdges = []; liveOnlySetSig = "";
    topicTouchers = {}; liveTopicTouchers = {}; hostFilter = null;
    liveAvail = true; staticSummary = { node_count: 0, topic_count: 0, edge_count: 0 };
  },
  apply(live) { applyLive(live); },
  setHostFilter(v) { hostFilter = v; },
  ids() { return Object.keys(base); },
  machines() { return [...liveHosts.keys()]; },
  isHidden(id) { return isHiddenMain(id); },
};
`;
const anchor = 'function applyLive(live) {';
if (!script.includes(anchor)) throw new Error('applyLive anchor not found in page');
script = script.replace(anchor, HOOK + anchor);

// ---------------------------------------------------------------- stubs
class El {
  constructor() {
    this._html = ''; this.textContent = ''; this.children = [];
    this.classes = new Set(); this.dataset = {}; this.style = {};
    this.classList = {
      toggle: (c, on) => { on ? this.classes.add(c) : this.classes.delete(c); },
      add: c => this.classes.add(c),
      remove: c => this.classes.delete(c),
      contains: c => this.classes.has(c),
    };
  }
  set innerHTML(v) {
    this._html = v;
    this.children = [];
    const re = /<span class="([^"]*)"[^>]*data-(?:host|pkg)="([^"]*)"[^>]*>(.*?)<\/span>/g;
    let m;
    while ((m = re.exec(v)) !== null) {
      const c = new El();
      m[1].split(/\s+/).filter(Boolean).forEach(x => c.classes.add(x));
      c.dataset.host = m[2];
      c.textContent = m[3].replace(/<[^>]*>/g, '');
      this.children.push(c);
    }
  }
  get innerHTML() { return this._html; }
  querySelectorAll() { return this.children; }
  addEventListener() {}
}
const els = {};
global.document = {
  getElementById: id => (els[id] = els[id] || new El()),
  querySelectorAll: () => [],
  addEventListener: () => {},   // keeps the page's own init from running
  body: new El(),
};
global.location = { search: '', href: 'http://harness/' };
global.history = { replaceState: () => {} };
class DataSet {
  constructor(items) { this.items = new Map((items || []).map(i => [i.id, i])); }
  add(i) { this.items.set(i.id, i); }
  remove(id) { this.items.delete(id); }
  update(list) {
    for (const u of [].concat(list)) {
      this.items.set(u.id, Object.assign(this.items.get(u.id) || {}, u));
    }
  }
}
global.vis = {
  DataSet,
  Network: class {
    on() {} once() {} fit() {} moveTo() {} setOptions() {} destroy() {}
    getScale() { return 1; } getPositions() { return {}; }
  },
};

// ---------------------------------------------------------------- drive
new Function(script)();
const payload = JSON.parse(fs.readFileSync(0, 'utf8'));
const hostFilter = payload._host_filter === undefined ? null : payload._host_filter;
delete payload._host_filter;

const H = globalThis.__harness;
H.reset();
// Twice: live-only elements are inserted into `base` on first sight only,
// so a one-shot run would miss anything that goes stale on later polls.
H.apply(payload);
H.apply(payload);
H.setHostFilter(hostFilter);

const ids = H.ids();
process.stdout.write(JSON.stringify({
  machines: H.machines().sort(),
  elements: ids.sort(),
  visible: ids.filter(i => !H.isHidden(i)).sort(),
  hidden: ids.filter(i => H.isHidden(i)).sort(),
}, null, 2));
