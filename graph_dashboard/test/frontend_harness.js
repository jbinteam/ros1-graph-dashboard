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
// stdin — one /api/live payload, plus optional graph, URL, click and update fixtures
// stdout — graph visibility/styles, focus, filter buttons and the resulting URL
'use strict';
const fs = require('fs');

const pagePath = process.env.PAGE ||
  require('path').join(__dirname, '..', 'graph_dashboard', 'web', 'index.html');
const html = fs.readFileSync(pagePath, 'utf8');
let script = html.split('<script>').pop().split('</script>')[0];
// Drive the real load() ourselves after the fetch fixtures are installed.
script = script.replace('load().catch(err => {', 'Promise.resolve().catch(err => {');

// The page keeps its state in closure variables, which is right for the
// page and awkward for a test. Rather than export hooks from production
// code, the harness injects an accessor into its in-memory copy only.
const HOOK = `
globalThis.__harness = {
  load() { return load(); },
  apply(live) { applyLive(live); },
  setHostFilter(v) { hostFilter = v; restyle(); },
  ids() { return Object.keys(base); },
  machines() { return [...liveHosts.keys()]; },
  isHidden(id) { return isHiddenMain(id); },
  edges() { return [...edgesDS.items.values()].map(e => ({
    from: e.from, to: e.to, hidden: e.hidden, width: e.width, color: e.color.color, dashes: e.dashes,
  })); },
  nodeStyles() { return Object.fromEntries([...nodesDS.items.values()].map(n => [n.id, {
    border_width: n.borderWidth, border_color: n.color.border, font_color: n.font.color,
  }])); },
  focusCenter() { return focusId; },
  positions() { return network.getPositions(); },
  async act(action) {
    if (action.kind === "focus") {
      await focusOn(action.id);
    } else if (action.kind === "focus-click") {
      await Promise.all(focusNet.emit("click", {nodes: action.id ? [action.id] : []}));
    } else if (action.kind === "hover") {
      network.emit("hoverNode", {node: action.id});
    } else if (action.kind === "blur") {
      network.emit("blurNode", {node: action.id});
    } else if (action.kind === "close") {
      closeFocus();
    } else if (action.kind === "live") {
      applyLive(action.live);
    }
  },
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
    this.attributes = {}; this.listeners = {};
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
    const re = /<span class="([^"]*)"[^>]*data-(host|pkg)="([^"]*)"[^>]*>(.*?)<\/span>/g;
    let m;
    while ((m = re.exec(v)) !== null) {
      const c = new El();
      m[1].split(/\s+/).filter(Boolean).forEach(x => c.classes.add(x));
      c.dataset[m[2]] = m[3];
      c.textContent = m[4].replace(/<[^>]*>/g, '');
      this.children.push(c);
    }
  }
  get innerHTML() { return this._html; }
  replaceChildren() { this.children = []; }
  appendChild(el) { this.children.push(el); }
  contains(el) { return this.children.includes(el); }
  focus() { document.activeElement = this; }
  setAttribute(key, value) { this.attributes[key] = value; }
  getAttribute(key) { return this.attributes[key]; }
  querySelectorAll() { return this.children; }
  addEventListener(event, callback) { this.listeners[event] = callback; }
  click() { if (this.listeners.click) this.listeners.click(); }
}
const els = {};
global.document = {
  getElementById: id => (els[id] = els[id] || new El()),
  createElement: () => new El(),
  querySelectorAll: selector => selector === '#pkg-chips .toggle' ? (els['pkg-chips'] || {}).children || [] : [],
  addEventListener: () => {},   // keeps the page's own init from running
  body: new El(),
};
global.location = { search: '', href: 'http://harness/' };
global.history = { replaceState: (_state, _title, url) => {
  location.href = String(url); location.search = new URL(location.href).search;
} };
global.setInterval = () => 1;
global.clearInterval = () => {};
global.requestAnimationFrame = callback => callback();
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
    constructor(container, data) { this.handlers = {}; this.setData(data); }
    setData(data) { this.data = data; }
    on(event, callback) { (this.handlers[event] = this.handlers[event] || []).push(callback); }
    emit(event, data) { return [...(this.handlers[event] || [])].map(fn => fn(data)); }
    once() {} fit() {} moveTo() {} redraw() {} setOptions() {} destroy() {}
    getScale() { return 1; }
    getPositions() {
      return Object.fromEntries([...this.data.nodes.items.values()].map(n =>
        [n.id, {x: n.x || 0, y: n.y || 0}]));
    }
  },
};

// ---------------------------------------------------------------- drive
const payload = JSON.parse(fs.readFileSync(0, 'utf8'));
const hostFilter = payload._host_filter;
const graph = payload._graph || {
  nodes: [], topics: [], edges: [], closures: {},
  summary: {node_count: 0, topic_count: 0, edge_count: 0, dynamic_topic_count: 0, files_scanned: 0},
};
const clicks = payload._click_types || [];
const nextLive = payload._next_live;
const actions = payload._actions || [];
const egos = payload._egos || {};
location.href = 'http://harness/' + (payload._url || '');
location.search = new URL(location.href).search;
delete payload._host_filter;
delete payload._graph;
delete payload._click_types;
delete payload._next_live;
delete payload._url;
delete payload._actions;
delete payload._egos;
global.fetch = async path => ({ok: true, json: async () => {
  if (path === '/api/graph') return graph;
  if (path.startsWith('/api/ego')) {
    const center = new URL(path, location.href).searchParams.get('id');
    return egos[center] || {center, levels: {[center]: 0}, dual: []};
  }
  return payload;
} });
new Function(script)();

const H = globalThis.__harness;
async function run() {
  await H.load();
  // Twice: catch state that goes stale once elements already exist.
  H.apply(payload);
  if (hostFilter !== undefined) H.setHostFilter(hostFilter);
  for (const type of clicks) {
    const button = els['msg-type-chips'].children.find(c => c.dataset.msgType === type);
    if (!button) throw new Error('message type button missing: ' + type);
    button.click();
  }
  if (nextLive) H.apply(nextLive);
  for (const action of actions) await H.act(action);

  const ids = H.ids();
  process.stdout.write(JSON.stringify({
    machines: H.machines().sort(),
    elements: ids.sort(),
    visible: ids.filter(i => !H.isHidden(i)).sort(),
    hidden: ids.filter(i => H.isHidden(i)).sort(),
    edges: H.edges(),
    node_styles: H.nodeStyles(),
    focus_id: H.focusCenter(),
    rendered_positions: H.positions(),
    type_buttons: els['msg-type-chips'].children.map(c => ({
      type: c.dataset.msgType, label: c.textContent, pressed: c.getAttribute('aria-pressed'),
    })),
    url: location.href,
  }, null, 2));
}
run().catch(err => { console.error(err); process.exitCode = 1; });
