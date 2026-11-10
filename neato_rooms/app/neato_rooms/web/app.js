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
  view: { cx: 0, cy: 0, scale: 60, rot: 0 }, // px per metre; rot = plan alignment (map shown rotated by -rot)
  plan: null, // /api/floorplan
  planTap: "align",
  review: null, // /api/review: changes found by the last scan, waiting to be applied
  reviewKey: "",
  reviewView: "after", // which map to show while reviewing
  reviewFocus: null,
  preview: null, // {msg, canvas}: the map with the pending scan applied
  fitted: false,
  connected: false,
};

const KIND_LABEL = { room: "Room", zone: "Zone", nogo: "No-go" };
const MISSION_LABEL = { explore: "Exploring", manual_map: "Mapping", clean: "Cleaning", dock: "Docking", calibrate: "Calibrating", refine: "Refining map", rescan: "Re-scanning" };

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
    else if (msg.type === "floorplan") loadPlan();
    else if (msg.type === "review") loadReview();
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
  // Mapping streams the map every couple of seconds; the plan is rebuilt once it settles.
  const m = S.status && S.status.mission;
  if (!(m && m.status === "running")) { loadPlan(); if (S.review) loadPreviewMap(); }
  if (msg.empty) { S.map = null; S.mapCanvas = null; requestDraw(); return; }
  S.map = msg;
  S.mapCanvas = mapCanvas(msg);
  if (!S.fitted) { fit(); S.fitted = true; }
  requestDraw();
}

function mapCanvas(msg) {
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
  return c;
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
  const c = Math.cos(S.view.rot), s = Math.sin(S.view.rot);
  const dx = x - S.view.cx, dy = y - S.view.cy;
  const qx = c * dx + s * dy, qy = -s * dx + c * dy;
  return [qx * S.view.scale + r.width / 2, r.height / 2 - qy * S.view.scale];
}

function toWorld(sx, sy) {
  const r = canvas.getBoundingClientRect();
  const qx = (sx - r.width / 2) / S.view.scale, qy = (r.height / 2 - sy) / S.view.scale;
  const c = Math.cos(S.view.rot), s = Math.sin(S.view.rot);
  return [c * qx - s * qy + S.view.cx, s * qx + c * qy + S.view.cy];
}

function fit() {
  const r = canvas.getBoundingClientRect();
  if (!S.map) { S.view = { cx: 0, cy: 0, scale: 60 }; requestDraw(); return; }
  const wm = S.map.width * S.map.resolution, hm = S.map.height * S.map.resolution;
  S.view.cx = S.map.origin[0] + wm / 2;
  S.view.cy = S.map.origin[1] + hm / 2;
  const c = Math.abs(Math.cos(S.view.rot)), s = Math.abs(Math.sin(S.view.rot));
  const w = c * wm + s * hm, h = s * wm + c * hm; // extent of the rotated map
  S.view.scale = Math.max(5, Math.min(r.width / (w + 0.5), r.height / (h + 0.5)));
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

  const showPreview = S.review && S.reviewView === "after" && S.preview;
  const [m, image] = showPreview ? [S.preview.msg, S.preview.canvas] : [S.map, S.mapCanvas];
  if (image) {
    const [sx, sy] = toScreen(m.origin[0], m.origin[1] + m.height * m.resolution);
    ctx.imageSmoothingEnabled = false;
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(S.view.rot);
    ctx.drawImage(image, 0, 0, m.width * m.resolution * S.view.scale, m.height * m.resolution * S.view.scale);
    ctx.restore();
  }
  if (S.review && S.tab === "mapping") drawChanges();

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
  const th = pose.theta - S.view.rot;
  ctx.lineTo(rx + Math.cos(th) * Math.max(rr, 8), ry - Math.sin(th) * Math.max(rr, 8));
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
  const rv = st && st.review;
  const key = rv ? `${rv.session}:${rv.changes}:${rv.accepted}` : "";
  if (key !== S.reviewKey) { S.reviewKey = key; loadReview(); }
  const manualMapping = running && m.kind === "manual_map";
  $("#map-idle").classList.toggle("hidden", !!manualMapping);
  $("#map-drive").classList.toggle("hidden", !manualMapping);
  if (S.wasRunning && !running) loadPlan();
  S.wasRunning = !!running;
  $$("#map-idle button, #btn-clean, #btn-dock, [data-native], #btn-refine, #review button").forEach((b) => {
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
  $("#plan").classList.toggle("hidden", S.tab !== "plan");
  $$(".map-tools").forEach((t) => t.classList.toggle("hidden", S.tab === "plan"));
  if (S.tab === "plan") loadPlan();
  requestDraw();
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
$("#btn-rescan").addEventListener("click", () => act(() => api("api/mission", {
  body: { kind: "rescan", area_ids: [...S.selected].filter((id) => S.areas.some((a) => a.id === id && a.kind === "room")) },
})));
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
// Floor plan

const SVGNS = "http://www.w3.org/2000/svg";
let planLoading = null;

async function loadPlan() {
  if (planLoading) return planLoading;
  planLoading = (async () => {
    try {
      const preview = S.review && $("#plan-preview").checked;
      S.plan = await api(preview ? "api/floorplan?preview=1" : "api/floorplan");
    } catch (e) {
      S.plan = null;
    }
    planLoading = null;
    const rot = S.plan ? S.plan.rotation : 0;
    if (Math.abs(rot - S.view.rot) > 1e-6) { S.view.rot = rot; fit(); }
    renderPlan();
  })();
  return planLoading;
}

function svgEl(parent, tag, attrs, text) {
  const el = document.createElementNS(SVGNS, tag);
  for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
  if (text !== undefined) el.textContent = text;
  parent.appendChild(el);
  return el;
}

function renderPlan() {
  const svg = $("#plan");
  svg.replaceChildren();
  const p = S.plan;
  const settings = p ? p.settings : null;
  $("#plan-summary").textContent = p
    ? `Approx. interior area ${p.area_text} · ${p.faces.filter((f) => f.visible && f.kind !== "fixture").length} walls, ${p.doors.length} doors`
    : "No map yet — map the house first.";
  $$("#plan-svg, #plan-pdf").forEach((a) => a.classList.toggle("hidden", !p));
  if (!p) return;
  if (document.activeElement !== $("#plan-title")) $("#plan-title").value = settings.title;
  $("#plan-units").value = settings.units;
  $("#plan-paper").value = settings.paper;

  // Plan coordinates are metres, y up; the SVG flips y so text is placed at (x, -y).
  const [x0, y0, x1, y1] = p.bounds;
  const pad = 0.6;
  svg.setAttribute("viewBox", `${x0 - pad} ${-y1 - pad} ${x1 - x0 + 2 * pad} ${y1 - y0 + 2 * pad}`);
  const pts = (list) => list.map(([x, y]) => `${x},${-y}`).join(" ");
  const g = svgEl(svg, "g", {});
  for (const f of p.faces) {
    if (f.visible && f.kind === "fixture") svgEl(g, "polygon", { points: pts(f.body), class: "fixture" });
  }
  for (const d of p.doors) {
    const dx = Math.cos(d.angle), dy = Math.sin(d.angle);
    const nx = -dy * d.side, ny = dx * d.side;
    const hx = d.x - dx * d.width / 2, hy = d.y - dy * d.width / 2;
    const leaf = [hx + nx * d.width, hy + ny * d.width], end = [hx + dx * d.width, hy + dy * d.width];
    svgEl(g, "path", {
      class: "door",
      d: `M${hx},${-hy} L${leaf[0]},${-leaf[1]} A${d.width},${d.width} 0 0 ${d.side > 0 ? 0 : 1} ${end[0]},${-end[1]}`,
    });
  }
  for (const f of p.faces) {
    if (f.kind === "fixture" && f.visible) continue;
    if (f.visible && f.kind !== "furniture") {
      svgEl(g, "polygon", { points: pts(f.body), class: "wall", "data-id": f.id });
    } else if (!f.visible) {
      svgEl(g, "line", { x1: f.a[0], y1: -f.a[1], x2: f.b[0], y2: -f.b[1], class: "ghost", "data-id": f.id });
    }
    svgEl(g, "line", { x1: f.a[0], y1: -f.a[1], x2: f.b[0], y2: -f.b[1], class: "hit", "data-id": f.id });
  }
  for (const r of p.rooms) {
    svgEl(g, "text", { x: r.x, y: -r.y - 0.04, "font-size": 0.22, "font-weight": 600 }, r.name.toUpperCase());
    if (r.dims) svgEl(g, "text", { x: r.x, y: -r.y + 0.22, "font-size": 0.18, class: "dim" }, r.dims);
  }
}

$("#plan").addEventListener("click", (e) => {
  const id = e.target.getAttribute && e.target.getAttribute("data-id");
  if (id === null || id === undefined) return;
  const face_id = Number(id);
  if (S.planTap === "align") {
    act(() => api("api/floorplan/align", { body: { mode: "face", face_id } })).then(loadPlan);
  } else {
    act(() => api("api/floorplan/toggle", { body: { face_id } })).then(loadPlan);
  }
});
$$("[data-plan-tap]").forEach((b) => b.addEventListener("click", () => {
  S.planTap = b.dataset.planTap;
  $$("[data-plan-tap]").forEach((x) => x.classList.toggle("active", x === b));
}));
$("#plan-rot-auto").addEventListener("click", () => act(() => api("api/floorplan/align", { body: { mode: "auto" } })).then(loadPlan));
$$("[data-plan-rot]").forEach((b) => b.addEventListener("click", () => act(() => api("api/floorplan/align", {
  body: { mode: "rotate", delta_deg: Number(b.dataset.planRot) },
})).then(loadPlan)));
async function savePlanSettings() {
  await act(() => api("api/floorplan/settings", {
    method: "PUT",
    body: { title: $("#plan-title").value, units: $("#plan-units").value, paper: $("#plan-paper").value },
  }));
  loadPlan();
}
$("#plan-title").addEventListener("change", savePlanSettings);
$("#plan-units").addEventListener("change", savePlanSettings);
$("#plan-paper").addEventListener("change", savePlanSettings);
$("#btn-refine").addEventListener("click", () => act(() => api("api/mission", { body: { kind: "refine" } })));
$("#plan-preview").addEventListener("change", loadPlan);

// ---------------------------------------------------------------------------
// Reviewing what a new scan changed

async function loadReview() {
  try {
    S.review = await api("api/review");
  } catch (e) {
    S.review = null;
  }
  if (!S.review) { S.preview = null; S.reviewFocus = null; }
  renderReview();
  if (S.review) loadPreviewMap();
  if (S.tab === "plan") loadPlan();
  requestDraw();
}

async function loadPreviewMap() {
  try {
    const msg = await api("api/review/map");
    S.preview = msg.empty ? null : { msg, canvas: mapCanvas(msg) };
  } catch (e) {
    S.preview = null;
  }
  requestDraw();
}

function fmtLen(m) {
  const units = S.plan ? S.plan.settings.units : "imperial";
  if (units === "metric") return `${m.toFixed(1)} m`;
  const inches = Math.round(m * 39.3701);
  return `${Math.floor(inches / 12)}′${inches % 12}″`;
}

function fmtArea(m2) {
  const units = S.plan ? S.plan.settings.units : "imperial";
  return units === "metric" ? `${m2.toFixed(1)} m²` : `${Math.round(m2 * 10.7639)} sq ft`;
}

function changeLabel(c) {
  const size = `${fmtLen(c.dims_m[0])} × ${fmtLen(c.dims_m[1])}`;
  if (c.kind === "moved") return `Moved ${fmtLen(c.distance_m)} (${size})`;
  if (c.kind === "added") return `Something new (${size})`;
  if (c.kind === "removed") return `Gone (${size})`;
  return `Newly mapped area, ${fmtArea(c.size_m2)}`;
}

const CHANGE_COLOR = { added: "--chg-added", removed: "--chg-removed", moved: "--chg-removed", new_area: "--chg-area" };

function renderReview() {
  const r = S.review;
  $("#review").classList.toggle("hidden", !r);
  $("#plan-preview-row").classList.toggle("hidden", !r);
  if (!r) return;
  const n = r.changes.length;
  $("#review-title").textContent = n ? `Review ${n} change${n > 1 ? "s" : ""}` : "Check the new scans";
  $("#review-help").textContent = r.session.stitched && !n
    ? "These scans were stitched on where they overlap the map. Switch between Before and After to check they line up, then apply them."
    : "Red is what's gone, green is what's new, blue is newly mapped. Tap a change to find it on the map. Untick any to leave out, then apply. "
      + "Every scan also sharpens the walls it saw again.";
  $("#review-list").replaceChildren(...r.changes.map((c) => {
    const div = document.createElement("div");
    div.className = "area" + (c.accept ? "" : " off") + (S.reviewFocus === c.id ? " focus" : "");
    const sw = document.createElement("span");
    sw.className = "swatch";
    sw.style.background = cssVar(c.kind === "moved" ? "--chg-added" : CHANGE_COLOR[c.kind]);
    const name = document.createElement("span");
    name.className = "name";
    name.textContent = changeLabel(c);
    const keep = document.createElement("input");
    keep.type = "checkbox";
    keep.checked = c.accept;
    keep.title = "Apply this change to the map";
    keep.setAttribute("aria-label", `Apply: ${changeLabel(c)}`);
    keep.onclick = (e) => e.stopPropagation();
    keep.onchange = () => act(() => api(`api/review/changes/${c.id}`, { body: { accept: keep.checked } }));
    div.onclick = () => {
      S.reviewFocus = c.id;
      const [bx0, by0, bx1, by1] = c.to ? [Math.min(c.bbox[0], c.to.bbox[0]), Math.min(c.bbox[1], c.to.bbox[1]),
        Math.max(c.bbox[2], c.to.bbox[2]), Math.max(c.bbox[3], c.to.bbox[3])] : c.bbox;
      S.view.cx = (bx0 + bx1) / 2;
      S.view.cy = (by0 + by1) / 2;
      renderReview();
      requestDraw();
    };
    div.append(sw, name, keep);
    return div;
  }));
  const running = S.status && S.status.mission && S.status.mission.status === "running";
  $$("#review button, #review input").forEach((b) => { b.disabled = !!running; });
}

function drawChanges() {
  const res = S.map ? S.map.resolution : 0.05;
  const px = Math.max(2, res * S.view.scale);
  const dot = (pts) => {
    for (const [x, y] of pts) {
      const [sx, sy] = toScreen(x, y);
      ctx.fillRect(sx - px / 2, sy - px / 2, px, px);
    }
  };
  const box = ([x0, y0, x1, y1], pad) => polyPath([[x0 - pad, y0 - pad], [x1 + pad, y0 - pad], [x1 + pad, y1 + pad], [x0 - pad, y1 + pad]]);
  for (const c of S.review.changes) {
    ctx.globalAlpha = c.accept ? 0.75 : 0.25;
    ctx.fillStyle = cssVar(CHANGE_COLOR[c.kind]);
    dot(c.points);
    if (c.to_points) { ctx.fillStyle = cssVar("--chg-added"); dot(c.to_points); }
    ctx.globalAlpha = 1;
    const focus = S.reviewFocus === c.id;
    ctx.lineWidth = focus ? 3 : 1.5;
    ctx.setLineDash(c.accept ? [] : [5, 4]);
    ctx.strokeStyle = cssVar(CHANGE_COLOR[c.kind]);
    box(c.bbox, 0.08);
    ctx.stroke();
    if (c.to) {
      ctx.strokeStyle = cssVar("--chg-added");
      box(c.to.bbox, 0.08);
      ctx.stroke();
      // Arrow from where it was to where it went.
      const [ax, ay] = toScreen(c.x, c.y), [bx, by] = toScreen(c.to.x, c.to.y);
      const ang = Math.atan2(by - ay, bx - ax);
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(bx, by);
      ctx.lineTo(bx - 10 * Math.cos(ang - 0.4), by - 10 * Math.sin(ang - 0.4));
      ctx.moveTo(bx, by);
      ctx.lineTo(bx - 10 * Math.cos(ang + 0.4), by - 10 * Math.sin(ang + 0.4));
      ctx.stroke();
    }
    ctx.setLineDash([]);
  }
}

$$("[data-review-view]").forEach((b) => b.addEventListener("click", () => {
  S.reviewView = b.dataset.reviewView;
  $$("[data-review-view]").forEach((x) => x.classList.toggle("active", x === b));
  requestDraw();
}));
$("#btn-review-apply").addEventListener("click", () => act(() => api("api/review/apply", { body: {} })));
$("#btn-review-discard").addEventListener("click", () => {
  if (confirm("Discard the new scans? The map stays as it was.")) act(() => api("api/review/discard", { body: {} }));
});

// ---------------------------------------------------------------------------

window.addEventListener("resize", resize);
window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => api("api/map").then(setMap).catch(() => {}));
resize();
connect();
if ("serviceWorker" in navigator) navigator.serviceWorker.register("sw.js").catch(() => {});
