"use strict";
// Neato Rooms PWA. Plain JS, no build step. All URLs are relative (Home Assistant ingress).

const $ = (sel) => document.querySelector(sel);
const $$ = (sel) => Array.from(document.querySelectorAll(sel));

const S = {
  status: null,
  map: null, // {width,height,resolution,origin,version}
  mapCanvas: null,
  areas: [],
  live: { pose: { x: 0, y: 0, theta: 0 }, path: [], scan: [] },
  selected: new Set(),
  tab: "clean",
  draw: null, // {kind, pts: [[x,y]], rect: bool}
  poseMode: null, // null | {start:[x,y], theta}
  view: { cx: 0, cy: 0, scale: 60 }, // px per metre
  fitted: false,
  connected: false,
};

const KIND_LABEL = { room: "Room", zone: "Zone", nogo: "No-go" };
const MISSION_LABEL = { explore: "Exploring", manual_map: "Mapping", clean: "Cleaning", dock: "Docking", calibrate: "Calibrating" };

// ---------------------------------------------------------------------------
// API

async function api(path, opts = {}) {
  const init = { method: opts.method || (opts.body ? "POST" : "GET"), headers: {} };
  if (opts.body !== undefined) {
    init.headers["Content-Type"] = "application/json";
    init.body = JSON.stringify(opts.body);
  }
  const res = await fetch(path, init);
  let data = null;
  try { data = await res.json(); } catch (_) { /* empty */ }
  if (!res.ok) throw new Error((data && data.detail) || `HTTP ${res.status}`);
  return data;
}

async function act(fn) {
  try { return await fn(); } catch (e) { toast(e.message); }
}

let toastTimer;
function toast(msg) {
  const t = $("#toast");
  t.textContent = msg;
  t.classList.remove("hidden");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.add("hidden"), 4000);
}

// ---------------------------------------------------------------------------
// WebSocket

let wsRetry = 500;
function connect() {
  const url = new URL("api/ws", location.href);
  url.protocol = location.protocol === "https:" ? "wss:" : "ws:";
  const ws = new WebSocket(url);
  ws.onopen = () => { wsRetry = 500; S.connected = true; renderStatus(); };
  ws.onclose = () => {
    S.connected = false;
    renderStatus();
    setTimeout(connect, wsRetry);
    wsRetry = Math.min(wsRetry * 2, 10000);
  };
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === "status") { S.status = msg; renderStatus(); }
    else if (msg.type === "map") setMap(msg);
    else if (msg.type === "areas") { S.areas = msg.areas; pruneSelection(); renderAreas(); }
    else if (msg.type === "live") { S.live = msg; requestDraw(); }
  };
}

// ---------------------------------------------------------------------------
// Map image

function cssVar(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

function hexToRgb(hex) {
  const v = hex.replace("#", "");
  const n = parseInt(v.length === 3 ? v.split("").map((c) => c + c).join("") : v, 16);
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
}

function setMap(msg) {
  $("#map-empty").classList.toggle("hidden", !msg.empty);
  if (msg.empty) { S.map = null; S.mapCanvas = null; requestDraw(); return; }
  const raw = atob(msg.data);
  const { width: w, height: h } = msg;
  const c = document.createElement("canvas");
  c.width = w; c.height = h;
  const ctx = c.getContext("2d");
  const img = ctx.createImageData(w, h);
  const free = hexToRgb(cssVar("--map-free"));
  const occ = hexToRgb(cssVar("--map-occ"));
  const unk = hexToRgb(cssVar("--map-unknown"));
  for (let row = 0; row < h; row++) {
    const dst = (h - 1 - row) * w; // row 0 is the lowest y; canvas row 0 is the top
    for (let col = 0; col < w; col++) {
      const v = raw.charCodeAt(row * w + col);
      const o = (dst + col) * 4;
      const rgb = v === 1 ? free : v === 2 ? occ : unk;
      img.data[o] = rgb[0]; img.data[o + 1] = rgb[1]; img.data[o + 2] = rgb[2];
      img.data[o + 3] = v === 0 ? 0 : 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  S.map = msg;
  S.mapCanvas = c;
  if (!S.fitted) { fit(); S.fitted = true; }
  requestDraw();
}

// ---------------------------------------------------------------------------
// View transform

const canvas = $("#map");
const ctx = canvas.getContext("2d");

function resize() {
  const r = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(r.width * dpr);
  canvas.height = Math.round(r.height * dpr);
  requestDraw();
}

function toScreen(x, y) {
  const r = canvas.getBoundingClientRect();
  return [(x - S.view.cx) * S.view.scale + r.width / 2, r.height / 2 - (y - S.view.cy) * S.view.scale];
}

function toWorld(sx, sy) {
  const r = canvas.getBoundingClientRect();
  return [(sx - r.width / 2) / S.view.scale + S.view.cx, (r.height / 2 - sy) / S.view.scale + S.view.cy];
}

function fit() {
  const r = canvas.getBoundingClientRect();
  if (!S.map) { S.view = { cx: 0, cy: 0, scale: 60 }; requestDraw(); return; }
  const wm = S.map.width * S.map.resolution, hm = S.map.height * S.map.resolution;
  S.view.cx = S.map.origin[0] + wm / 2;
  S.view.cy = S.map.origin[1] + hm / 2;
  S.view.scale = Math.max(5, Math.min(r.width / (wm + 0.5), r.height / (hm + 0.5)));
  requestDraw();
}

function zoomAt(sx, sy, factor) {
  const [wx, wy] = toWorld(sx, sy);
  S.view.scale = Math.max(5, Math.min(400, S.view.scale * factor));
  const [nx, ny] = toWorld(sx, sy);
  S.view.cx += wx - nx;
  S.view.cy += wy - ny;
  requestDraw();
}

// ---------------------------------------------------------------------------
// Drawing

let drawQueued = false;
function requestDraw() {
  if (drawQueued) return;
  drawQueued = true;
  requestAnimationFrame(() => { drawQueued = false; draw(); });
}

function polyPath(pts) {
  ctx.beginPath();
  pts.forEach(([x, y], i) => {
    const [sx, sy] = toScreen(x, y);
    i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy);
  });
  ctx.closePath();
}

function centroid(pts) {
  let x = 0, y = 0;
  pts.forEach((p) => { x += p[0]; y += p[1]; });
  return [x / pts.length, y / pts.length];
}

function draw() {
  const dpr = window.devicePixelRatio || 1;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const r = canvas.getBoundingClientRect();
  ctx.clearRect(0, 0, r.width, r.height);

  if (S.mapCanvas) {
    const m = S.map;
    const [sx, sy] = toScreen(m.origin[0], m.origin[1] + m.height * m.resolution);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(S.mapCanvas, sx, sy, m.width * m.resolution * S.view.scale, m.height * m.resolution * S.view.scale);
  }

  // Areas
  ctx.font = "600 13px system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  for (const a of S.areas) {
    const sel = S.selected.has(a.id);
    const color = a.kind === "nogo" ? cssVar("--danger") : a.color;
    polyPath(a.polygon);
    ctx.globalAlpha = a.kind === "nogo" ? 0.25 : sel ? 0.4 : 0.18;
    ctx.fillStyle = color;
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.lineWidth = sel ? 3 : 1.5;
    ctx.strokeStyle = color;
    ctx.setLineDash(a.kind === "zone" ? [6, 4] : a.kind === "nogo" ? [3, 3] : []);
    ctx.stroke();
    ctx.setLineDash([]);
    const [cx, cy] = toScreen(...centroid(a.polygon));
    ctx.lineWidth = 4;
    ctx.lineJoin = "round";
    ctx.strokeStyle = cssVar("--panel");
    ctx.strokeText(a.name, cx, cy);
    ctx.fillStyle = cssVar("--text");
    ctx.fillText(a.name, cx, cy);
  }

  // Draft polygon
  if (S.draw && S.draw.pts.length) {
    const pts = S.draw.pts;
    ctx.strokeStyle = cssVar("--accent");
    ctx.fillStyle = cssVar("--accent");
    ctx.lineWidth = 2;
    ctx.beginPath();
    pts.forEach(([x, y], i) => { const [sx, sy] = toScreen(x, y); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); });
    if (pts.length > 2 || S.draw.rect) ctx.closePath();
    ctx.stroke();
    ctx.globalAlpha = 0.15; ctx.fill(); ctx.globalAlpha = 1;
    for (const [x, y] of pts) {
      const [sx, sy] = toScreen(x, y);
      ctx.beginPath(); ctx.arc(sx, sy, 5, 0, Math.PI * 2); ctx.fill();
    }
  }

  // Dock (map origin)
  {
    const [dx, dy] = toScreen(0, 0);
    ctx.fillStyle = cssVar("--ok");
    ctx.fillRect(dx - 6, dy - 6, 12, 12);
  }

  // Planned path
  const live = S.live;
  if (live.path && live.path.length) {
    ctx.strokeStyle = cssVar("--accent");
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    let [px, py] = toScreen(live.pose.x, live.pose.y);
    ctx.moveTo(px, py);
    for (const [x, y] of live.path) { [px, py] = toScreen(x, y); ctx.lineTo(px, py); }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Scan
  if (live.scan) {
    ctx.fillStyle = "#e0625b";
    for (const [x, y] of live.scan) {
      const [sx, sy] = toScreen(x, y);
      ctx.fillRect(sx - 1.5, sy - 1.5, 3, 3);
    }
  }

  // Robot
  const pose = S.poseMode && S.poseMode.start
    ? { x: S.poseMode.start[0], y: S.poseMode.start[1], theta: S.poseMode.theta }
    : live.pose;
  const [rx, ry] = toScreen(pose.x, pose.y);
  const rr = 0.17 * S.view.scale;
  ctx.beginPath();
  ctx.arc(rx, ry, Math.max(rr, 6), 0, Math.PI * 2);
  ctx.fillStyle = cssVar("--panel");
  ctx.fill();
  ctx.lineWidth = 2.5;
  ctx.strokeStyle = cssVar("--accent");
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(rx, ry);
  ctx.lineTo(rx + Math.cos(pose.theta) * Math.max(rr, 8), ry - Math.sin(pose.theta) * Math.max(rr, 8));
  ctx.stroke();
}

// ---------------------------------------------------------------------------
// Pointer input: pan / pinch / tap / draw

const pointers = new Map();
let gesture = null;

canvas.addEventListener("pointerdown", (e) => {
  canvas.setPointerCapture(e.pointerId);
  pointers.set(e.pointerId, { x: e.offsetX, y: e.offsetY });
  if (pointers.size === 1) {
    gesture = { x0: e.offsetX, y0: e.offsetY, t0: performance.now(), moved: false, lastX: e.offsetX, lastY: e.offsetY };
    const w = toWorld(e.offsetX, e.offsetY);
    if (S.draw && S.draw.rect) { gesture.rect = w; S.draw.pts = [w, w, w, w]; requestDraw(); }
    if (S.poseMode) { S.poseMode.start = w; S.poseMode.theta = 0; gesture.pose = true; requestDraw(); }
  } else if (pointers.size === 2) {
    const [a, b] = [...pointers.values()];
    gesture = { pinch: Math.hypot(a.x - b.x, a.y - b.y), mid: [(a.x + b.x) / 2, (a.y + b.y) / 2], moved: true };
  }
});

canvas.addEventListener("pointermove", (e) => {
  if (!pointers.has(e.pointerId) || !gesture) return;
  pointers.set(e.pointerId, { x: e.offsetX, y: e.offsetY });
  if (pointers.size === 2 && gesture.pinch) {
    const [a, b] = [...pointers.values()];
    const d = Math.hypot(a.x - b.x, a.y - b.y);
    const mid = [(a.x + b.x) / 2, (a.y + b.y) / 2];
    S.view.cx -= (mid[0] - gesture.mid[0]) / S.view.scale;
    S.view.cy += (mid[1] - gesture.mid[1]) / S.view.scale;
    zoomAt(mid[0], mid[1], d / gesture.pinch);
    gesture.pinch = d;
    gesture.mid = mid;
    return;
  }
  if (Math.hypot(e.offsetX - gesture.x0, e.offsetY - gesture.y0) > 6) gesture.moved = true;
  const w = toWorld(e.offsetX, e.offsetY);
  if (gesture.rect) {
    const [x0, y0] = gesture.rect;
    S.draw.pts = [[x0, y0], [w[0], y0], [w[0], w[1]], [x0, w[1]]];
    requestDraw();
  } else if (gesture.pose) {
    S.poseMode.theta = Math.atan2(w[1] - S.poseMode.start[1], w[0] - S.poseMode.start[0]);
    requestDraw();
  } else if (gesture.moved) {
    S.view.cx -= (e.offsetX - gesture.lastX) / S.view.scale;
    S.view.cy += (e.offsetY - gesture.lastY) / S.view.scale;
    requestDraw();
  }
  gesture.lastX = e.offsetX;
  gesture.lastY = e.offsetY;
});

function endPointer(e) {
  if (!pointers.has(e.pointerId)) return;
  pointers.delete(e.pointerId);
  if (pointers.size > 0) { gesture = null; return; }
  const g = gesture;
  gesture = null;
  if (!g) return;
  if (g.pose) { finishPose(); return; }
  if (g.rect) {
    if (!g.moved) S.draw.pts = [];
    requestDraw();
    return;
  }
  if (!g.moved && performance.now() - g.t0 < 500) onTap(toWorld(e.offsetX, e.offsetY));
}
canvas.addEventListener("pointerup", endPointer);
canvas.addEventListener("pointercancel", endPointer);

canvas.addEventListener("wheel", (e) => {
  e.preventDefault();
  zoomAt(e.offsetX, e.offsetY, Math.exp(-e.deltaY * 0.0015));
}, { passive: false });

function pointInPoly([x, y], poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i], [xj, yj] = poly[j];
    if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

function onTap(w) {
  if (S.draw && !S.draw.rect) { S.draw.pts.push(w); requestDraw(); return; }
  if (S.tab === "clean") {
    const hit = S.areas.filter((a) => a.kind !== "nogo" && pointInPoly(w, a.polygon)).pop();
    if (hit) toggleSelect(hit.id);
  }
}

// ---------------------------------------------------------------------------
// Status & mission UI

function renderStatus() {
  const conn = $("#chip-conn");
  const st = S.status;
  const robot = st && st.robot;
  const online = S.connected && robot && robot.connected;
  conn.textContent = !S.connected ? "server offline" : online ? "robot online" : "robot offline";
  conn.className = "chip " + (online ? "on" : "off");
  $("#chip-batt").textContent = robot && robot.battery != null ? `${robot.battery}%${robot.charging ? " ⚡" : ""}` : "–%";
  $("#chip-state").textContent = robot ? prettyState(robot.state, robot) : "–";

  const m = st && st.mission;
  const banner = $("#mission");
  const running = m && m.status === "running";
  const recent = m && !running && Date.now() / 1000 - m.started < 3600;
  banner.classList.toggle("hidden", !(running || (recent && (m.status !== "done" || m.kind === "calibrate"))));
  if (m) {
    banner.classList.toggle("failed", m.status === "failed");
    $("#mission-kind").textContent = running ? MISSION_LABEL[m.kind] || m.kind
      : m.status === "failed" ? "Failed:" : m.status === "done" ? "Done:" : "Stopped:";
    $("#mission-msg").textContent = m.message || "";
    $("#mission-bar").style.width = `${Math.round((m.progress || 0) * 100)}%`;
    $("#btn-cancel").textContent = running ? "Stop" : "Dismiss";
  }
  const manualMapping = running && m.kind === "manual_map";
  $("#map-idle").classList.toggle("hidden", !!manualMapping);
  $("#map-drive").classList.toggle("hidden", !manualMapping);
  $$("#map-idle button, #btn-clean, #btn-dock, [data-native]").forEach((b) => {
    if (b.id === "btn-clean") b.disabled = running || S.selected.size === 0;
    else b.disabled = !!running;
  });
  $("#map-empty").classList.toggle("hidden", !!(S.map || (st && st.has_map)));
  const cal = st && st.calibration;
  $("#calib-result").classList.toggle("hidden", !cal);
  if (cal) $("#calib-result").textContent = JSON.stringify(cal, null, 2);
}

function prettyState(s, robot) {
  if (robot.error) return robot.error;
  if (robot.on_dock) return robot.charging ? "charging" : "docked";
  if (!s) return "–";
  return s.replace(/^ST_C_|^ST_|^UIMGR_STATE_/, "").replace(/([a-z])([A-Z])/g, "$1 $2").toLowerCase();
}

$("#btn-cancel").addEventListener("click", async () => {
  const m = S.status && S.status.mission;
  if (m && m.status === "running") await act(() => api("api/mission/cancel", { body: {} }));
  else $("#mission").classList.add("hidden");
});

// ---------------------------------------------------------------------------
// Areas

function pruneSelection() {
  const ids = new Set(S.areas.map((a) => a.id));
  for (const id of [...S.selected]) if (!ids.has(id)) S.selected.delete(id);
}

function toggleSelect(id) {
  S.selected.has(id) ? S.selected.delete(id) : S.selected.add(id);
  renderAreas();
  renderStatus();
}

function areaRow(a, editable) {
  const div = document.createElement("div");
  div.className = "area" + (S.selected.has(a.id) && !editable ? " selected" : "");
  const sw = document.createElement("span");
  sw.className = "swatch";
  sw.style.background = a.kind === "nogo" ? cssVar("--danger") : a.color;
  const name = document.createElement("span");
  name.className = "name";
  name.textContent = a.name;
  const kind = document.createElement("span");
  kind.className = "kind";
  kind.textContent = KIND_LABEL[a.kind];
  div.append(sw, name, kind);
  if (editable) {
    const ren = document.createElement("button");
    ren.textContent = "Rename";
    ren.onclick = () => {
      const n = prompt("Name", a.name);
      if (n) act(() => api(`api/areas/${a.id}`, { method: "PATCH", body: { name: n } }));
    };
    const del = document.createElement("button");
    del.textContent = "Delete";
    del.className = "danger";
    del.onclick = () => {
      if (confirm(`Delete "${a.name}"?`)) act(() => api(`api/areas/${a.id}`, { method: "DELETE" }));
    };
    div.append(ren, del);
  } else {
    div.onclick = () => toggleSelect(a.id);
    div.style.cursor = "pointer";
  }
  return div;
}

function renderAreas() {
  const cleanable = S.areas.filter((a) => a.kind !== "nogo");
  $("#clean-list").replaceChildren(...cleanable.map((a) => areaRow(a, false)));
  $("#clean-empty").classList.toggle("hidden", cleanable.length > 0);
  $("#area-edit-list").replaceChildren(...S.areas.map((a) => areaRow(a, true)));
  $("#btn-clean").textContent = S.selected.size ? `Clean ${S.selected.size} selected` : "Clean selected";
  renderStatus();
  requestDraw();
}

// Drawing new areas
$$("[data-draw]").forEach((b) => b.addEventListener("click", () => {
  const kind = b.dataset.draw;
  S.draw = { kind, pts: [], rect: kind !== "room" };
  $("#draw-name").value = "";
  $("#draw-name").placeholder = kind === "room" ? "Name (e.g. Kitchen)" : kind === "zone" ? "Name (e.g. Under the table)" : "Name (e.g. Rug)";
  $("#draw-help").textContent = kind === "room"
    ? "Tap each corner of the room on the map. Drag with two fingers to pan or zoom."
    : "Drag on the map to draw a rectangle.";
  $("#btn-draw-undo").classList.toggle("hidden", kind !== "room");
  $("#draw-idle").classList.add("hidden");
  $("#draw-active").classList.remove("hidden");
  showHint(`Drawing ${KIND_LABEL[kind].toLowerCase()}`);
}));

function endDraw() {
  S.draw = null;
  $("#draw-idle").classList.remove("hidden");
  $("#draw-active").classList.add("hidden");
  showHint(null);
  requestDraw();
}

$("#btn-draw-undo").addEventListener("click", () => { if (S.draw) { S.draw.pts.pop(); requestDraw(); } });
$("#btn-draw-cancel").addEventListener("click", endDraw);
$("#btn-draw-save").addEventListener("click", async () => {
  if (!S.draw || S.draw.pts.length < 3) { toast("Add at least 3 corners first"); return; }
  const body = { kind: S.draw.kind, polygon: S.draw.pts, name: $("#draw-name").value.trim() || undefined };
  const ok = await act(() => api("api/areas", { body }));
  if (ok) endDraw();
});

function showHint(text) {
  const h = $("#hint");
  h.textContent = text || "";
  h.classList.toggle("hidden", !text);
}

// ---------------------------------------------------------------------------
// Buttons

$$(".tabs button").forEach((b) => b.addEventListener("click", () => {
  S.tab = b.dataset.tab;
  $$(".tabs button").forEach((x) => x.classList.toggle("active", x === b));
  $$(".panel").forEach((p) => p.classList.toggle("hidden", p.id !== `tab-${S.tab}`));
  if (S.tab !== "areas" && S.draw) endDraw();
}));

$("#btn-clean").addEventListener("click", () => act(() => api("api/mission", {
  body: { kind: "clean", area_ids: [...S.selected], passes: Number($("#passes").value) },
})));
$("#btn-dock").addEventListener("click", () => act(() => api("api/mission", { body: { kind: "dock" } })));
$$("[data-native]").forEach((b) => b.addEventListener("click", () => act(() => api(`api/native/${b.dataset.native}`, { body: {} }))));

function startMapping(kind) {
  const extend = $("#extend-map").checked;
  if (!extend && S.status && S.status.has_map && !confirm("Start a new map? Your rooms will stay, but may no longer line up.")) return;
  act(() => api("api/mission", { body: { kind, new_map: !extend } }));
}
$("#btn-explore").addEventListener("click", () => startMapping("explore"));
$("#btn-manual-map").addEventListener("click", () => startMapping("manual_map"));
$("#btn-finish-map").addEventListener("click", () => act(() => api("api/drive", { body: { cmd: "finish" } })));
$$("[data-drive]").forEach((b) => b.addEventListener("click", () => act(() => api("api/drive", {
  body: { cmd: b.dataset.drive, amount: Number(b.dataset.amount) },
}))));
$("#btn-delete-map").addEventListener("click", () => {
  if (confirm("Delete the map? Rooms are kept.")) act(() => api("api/map", { method: "DELETE" }));
});
$("#btn-calibrate").addEventListener("click", () => act(() => api("api/mission", { body: { kind: "calibrate" } })));
$("#btn-setpose").addEventListener("click", () => {
  S.poseMode = { start: null, theta: 0 };
  showHint("Press where the robot is, drag towards where it faces");
});

async function finishPose() {
  const p = S.poseMode;
  S.poseMode = null;
  showHint(null);
  if (p && p.start) await act(() => api("api/pose", { body: { x: p.start[0], y: p.start[1], theta: p.theta } }));
  requestDraw();
}

$("#btn-center").addEventListener("click", () => { S.view.cx = S.live.pose.x; S.view.cy = S.live.pose.y; requestDraw(); });
$("#btn-fit").addEventListener("click", fit);

// ---------------------------------------------------------------------------

window.addEventListener("resize", resize);
window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => api("api/map").then(setMap).catch(() => {}));
resize();
connect();
if ("serviceWorker" in navigator) navigator.serviceWorker.register("sw.js").catch(() => {});
