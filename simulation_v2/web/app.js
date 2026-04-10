/* eslint-disable no-console */

const COLORS = [
  "#00CED1",
  "#FF1493",
  "#32CD32",
  "#FF8C00",
  "#9370DB",
  "#FF6347",
  "#4169E1",
  "#FFD700",
  "#00FA9A",
  "#FF69B4",
  "#8A2BE2",
  "#DC143C",
  "#00BFFF",
  "#FF4500",
  "#9ACD32",
  "#FF1493",
  "#1E90FF",
  "#FF8C00",
  "#BA55D3",
  "#20B2AA",
];

const state = {
  map: null,
  agvs: [],
  vMaxMmS: 1000,
  simTimeS: 0,
  connected: false,
  lastOkTs: 0,
  metrics: null,
  reservedNodes: [],

  scale: 1.0,
  offsetX: 0,
  offsetY: 0,
  isDragging: false,
  lastX: 0,
  lastY: 0,

  showPath: true,
  showTrail: true,
  showLabels: true,
  showTasks: true,
  showNodes: false,
  showNodeIds: false,
  showRegions: true,
  showBridges: true,
  showReserved: true,
  displayCount: 50,
  pollMs: 120,

  pollTimer: null,
};

const canvas = document.getElementById("map-canvas");
const ctx = canvas.getContext("2d");
const ZOOM_MIN = 0.1;
const ZOOM_MAX = 8.0;

function clampScale(v) {
  return Math.max(ZOOM_MIN, Math.min(ZOOM_MAX, v));
}

function canvasPointFromEvent(e) {
  const rect = canvas.getBoundingClientRect();
  return {
    x: e.clientX - rect.left,
    y: e.clientY - rect.top,
  };
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * ratio));
  canvas.height = Math.max(1, Math.floor(rect.height * ratio));
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
}

function mapBounds() {
  const nodes = state.map?.nodes || [];
  if (!nodes.length) {
    return { minX: 0, minY: 0, maxX: 1, maxY: 1 };
  }
  let minX = nodes[0].x;
  let maxX = nodes[0].x;
  let minY = nodes[0].y;
  let maxY = nodes[0].y;
  for (const n of nodes) {
    if (n.x < minX) minX = n.x;
    if (n.x > maxX) maxX = n.x;
    if (n.y < minY) minY = n.y;
    if (n.y > maxY) maxY = n.y;
  }
  return { minX, minY, maxX, maxY };
}

function canvasTransform() {
  const { minX, minY, maxX, maxY } = mapBounds();
  const w = Math.max(1, maxX - minX);
  const h = Math.max(1, maxY - minY);
  const baseScale = Math.min((canvas.clientWidth * 0.9) / w, (canvas.clientHeight * 0.9) / h);
  const mapCenterX = (minX + maxX) / 2;
  const mapCenterY = (minY + maxY) / 2;
  const centerX = canvas.clientWidth / 2;
  const centerY = canvas.clientHeight / 2;

  const toCanvas = (x, y) => {
    const cx = (x - mapCenterX) * baseScale * state.scale + centerX + state.offsetX;
    const cy = (y - mapCenterY) * baseScale * state.scale + centerY + state.offsetY;
    return { cx, cy };
  };
  return { toCanvas, baseScale, mapCenterX, mapCenterY, centerX, centerY };
}

function zoomAtPoint(cx, cy, nextScale) {
  const { baseScale, mapCenterX, mapCenterY, centerX, centerY } = canvasTransform();
  if (!(baseScale > 0)) return;
  const mapX = (cx - centerX - state.offsetX) / (baseScale * state.scale) + mapCenterX;
  const mapY = (cy - centerY - state.offsetY) / (baseScale * state.scale) + mapCenterY;
  state.scale = clampScale(nextScale);
  state.offsetX = cx - centerX - (mapX - mapCenterX) * baseScale * state.scale;
  state.offsetY = cy - centerY - (mapY - mapCenterY) * baseScale * state.scale;
}

function speedStateMmS(v) {
  if (!(v > 0)) return "stopped";
  const ratio = v / Math.max(1, state.vMaxMmS);
  if (ratio >= 0.8) return "high";
  if (ratio >= 0.4) return "medium";
  return "low";
}

function speedColor(kind) {
  if (kind === "high") return "#22c55e";
  if (kind === "medium") return "#f59e0b";
  if (kind === "low") return "#ef4444";
  return "#9ca3af";
}

function regionColor(regionId, alpha) {
  const id = Number(regionId);
  if (!Number.isFinite(id) || id < 0) return `rgba(160, 160, 180, ${alpha ?? 0.25})`;
  const hue = (id * 47) % 360;
  const a = Number.isFinite(alpha) ? alpha : 0.55;
  return `hsla(${hue}, 70%, 55%, ${a})`;
}

function regionIdForNode(nodeId) {
  const key = nodeId !== null && nodeId !== undefined ? String(nodeId) : "";
  if (!key || !state.map || !state.map.nodePositions) return null;
  const pos = state.map.nodePositions[key];
  const rid = Number(pos?.region);
  if (!Number.isFinite(rid)) return null;
  return rid;
}

function getRegionStats() {
  const regionInfo = state.map?.regions || {};
  let stats = [];
  if (Array.isArray(regionInfo.stats) && regionInfo.stats.length) {
    stats = regionInfo.stats
      .map((s) => ({
        id: Number(s.id),
        nodeCount: Number(s.nodeCount || 0),
        softLimit: Number(s.softLimit || 0),
        hardLimit: Number(s.hardLimit || 0),
      }))
      .filter((s) => Number.isFinite(s.id) && s.id >= 0);
  } else if (Array.isArray(state.map?.nodes)) {
    const counts = new Map();
    for (const n of state.map.nodes) {
      const rid = Number(n.region);
      if (!Number.isFinite(rid) || rid < 0) continue;
      counts.set(rid, (counts.get(rid) || 0) + 1);
    }
    stats = Array.from(counts.entries()).map(([id, nodeCount]) => ({
      id: Number(id),
      nodeCount: Number(nodeCount || 0),
      softLimit: 0,
      hardLimit: 0,
    }));
  }
  stats.sort((a, b) => a.id - b.id);
  return stats;
}

function updateRegionPanel() {
  const panel = document.getElementById("region-panel");
  const summaryEl = document.getElementById("region-summary");
  const listEl = document.getElementById("region-list");
  if (!panel || !summaryEl || !listEl) return;
  const regionInfo = state.map?.regions || {};
  const regionCount = Number(regionInfo.count || 0);
  const stats = getRegionStats();
  if (!regionCount || !stats.length) {
    summaryEl.innerHTML = `<span class="pill">无区域数据</span>`;
    listEl.innerHTML = "";
    return;
  }

  const agvCounts = new Map();
  for (const agv of state.agvs) {
    const nid = parseNodeId(agv.nodeId);
    const rid = nid !== null ? regionIdForNode(nid) : null;
    if (rid === null) continue;
    agvCounts.set(rid, (agvCounts.get(rid) || 0) + 1);
  }

  const reservedCounts = new Map();
  for (const item of state.reservedNodes || []) {
    const nid = parseNodeId(item.nodeId);
    const rid = nid !== null ? regionIdForNode(nid) : null;
    if (rid === null) continue;
    const inc = Number(item.count || 1);
    reservedCounts.set(rid, (reservedCounts.get(rid) || 0) + (Number.isFinite(inc) ? inc : 1));
  }

  const mode = String(regionInfo.mode || "-");
  const grid = Number(regionInfo.grid || 0);
  const softBase = Number(regionInfo.softLimit || 0);
  const hardBase = Number(regionInfo.hardLimit || 0);
  const pills = [
    `模式 ${mode}`,
    `区域 ${regionCount}`,
    grid > 0 ? `网格 ${grid}` : "",
    softBase > 0 || hardBase > 0 ? `基准 ${softBase}/${hardBase}` : "",
    `AGV ${state.agvs.length}`,
  ]
    .filter((x) => x)
    .map((x) => `<span class="pill">${x}</span>`)
    .join("");
  summaryEl.innerHTML = pills;

  listEl.innerHTML = "";
  for (const st of stats) {
    const agvCount = agvCounts.get(st.id) || 0;
    const resCount = reservedCounts.get(st.id) || 0;
    const limitText = st.softLimit > 0 || st.hardLimit > 0 ? `${st.softLimit}/${st.hardLimit}` : "-";
    const item = document.createElement("div");
    item.className = "region-item";
    item.innerHTML = `
      <div class="region-head">
        <div class="region-title">
          <span class="region-dot" style="background:${regionColor(st.id, 0.9)}"></span>
          <span>R${st.id}</span>
        </div>
        <div>${st.nodeCount} 节点</div>
      </div>
      <div class="region-meta">
        <div>AGV ${agvCount}</div>
        <div>预约 ${resCount}</div>
      </div>
      <div class="region-meta">
        <div>软/硬 ${limitText}</div>
        <div></div>
      </div>
    `;
    listEl.appendChild(item);
  }
}

function parseNodeId(value) {
  const n = Number(value);
  return Number.isFinite(n) ? Math.trunc(n) : null;
}

function findActiveMarker(agv) {
  if (!agv || !agv.assigned) return null;
  const cur = parseNodeId(agv.nodeId);
  if (cur === null) return null;
  const markers = Array.isArray(agv.subtasks) ? agv.subtasks : [];
  for (const m of markers) {
    const nid = parseNodeId(m.nodeId);
    if (nid !== null && nid === cur) return m;
  }
  return null;
}

function findNextMarker(agv) {
  if (!agv || !agv.assigned) return null;
  const cur = parseNodeId(agv.nodeId);
  const markers = Array.isArray(agv.subtasks) ? agv.subtasks : [];
  for (const m of markers) {
    const nid = parseNodeId(m.nodeId);
    if (nid === null) continue;
    if (cur === null || nid !== cur) return m;
  }
  return markers.length ? markers[markers.length - 1] : null;
}

function formatTask(marker) {
  if (!marker) return "-";
  const taskId = marker.taskId || "-";
  const subTaskId = marker.subTaskId || "-";
  const seq = Number.isFinite(Number(marker.sequence)) ? `#${marker.sequence}` : "";
  return `${taskId}/${subTaskId} ${seq}`.trim();
}

function formatDestination(marker) {
  if (!marker) return "-";
  const label = marker.label ? String(marker.label) : "";
  const location = marker.location ? String(marker.location) : "";
  const nodeId = parseNodeId(marker.nodeId);
  if (label) return nodeId !== null ? `${label} (N${nodeId})` : label;
  if (location) return nodeId !== null ? `${location} (N${nodeId})` : location;
  return nodeId !== null ? `N${nodeId}` : "-";
}

function fitToMap() {
  state.scale = 1.0;
  state.offsetX = 0;
  state.offsetY = 0;
  draw();
}

function drawMap() {
  if (!state.map) return;
  const { toCanvas } = canvasTransform();
  const nodePos = state.map.nodePositions;
  const showRegions = state.showRegions && Number(state.map?.regions?.count || 0) > 0;
  const bridgeNodeIds = Array.isArray(state.map?.bridgeNodeIds) ? state.map.bridgeNodeIds : [];
  const bridgeNodeSet = new Set(bridgeNodeIds.map((id) => String(id)));
  const bridgeEdges = Array.isArray(state.map?.bridgeEdges) ? state.map.bridgeEdges : [];

  // edges
  ctx.lineWidth = 1;
  for (const e of state.map.edges) {
    const a = nodePos[String(e.startNode)];
    const b = nodePos[String(e.endNode)];
    if (!a || !b) continue;
    const p0 = toCanvas(a.x, a.y);
    const p1 = toCanvas(b.x, b.y);
    let stroke = "rgba(160, 160, 180, 0.25)";
    if (showRegions) {
      const ra = Number(a.region);
      const rb = Number(b.region);
      if (Number.isFinite(ra) && ra >= 0 && Number.isFinite(rb) && rb >= 0) {
        if (ra === rb) {
          stroke = regionColor(ra, 0.35);
        } else {
          const grad = ctx.createLinearGradient(p0.cx, p0.cy, p1.cx, p1.cy);
          grad.addColorStop(0, regionColor(ra, 0.35));
          grad.addColorStop(1, regionColor(rb, 0.35));
          stroke = grad;
        }
      }
    }
    ctx.strokeStyle = stroke;
    ctx.beginPath();
    ctx.moveTo(p0.cx, p0.cy);
    ctx.lineTo(p1.cx, p1.cy);
    ctx.stroke();
  }

  if (state.showBridges && bridgeEdges.length) {
    ctx.save();
    ctx.strokeStyle = "rgba(249, 115, 22, 0.95)";
    ctx.lineWidth = 4;
    ctx.globalAlpha = 0.85;
    for (const e of bridgeEdges) {
      const a = nodePos[String(e.startNode)];
      const b = nodePos[String(e.endNode)];
      if (!a || !b) continue;
      const p0 = toCanvas(a.x, a.y);
      const p1 = toCanvas(b.x, b.y);
      ctx.beginPath();
      ctx.moveTo(p0.cx, p0.cy);
      ctx.lineTo(p1.cx, p1.cy);
      ctx.stroke();
    }
    ctx.restore();
  }

  if (state.showBridges && bridgeNodeIds.length) {
    ctx.save();
    for (const rawId of bridgeNodeIds) {
      const pos = nodePos[String(rawId)];
      if (!pos) continue;
      const p = toCanvas(pos.x, pos.y);
      ctx.beginPath();
      ctx.arc(p.cx, p.cy, 7, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(251, 146, 60, 0.20)";
      ctx.fill();
      ctx.beginPath();
      ctx.arc(p.cx, p.cy, 4.5, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(249, 115, 22, 0.95)";
      ctx.fill();
      ctx.strokeStyle = "rgba(255, 237, 213, 0.95)";
      ctx.lineWidth = 1.2;
      ctx.stroke();
    }
    ctx.restore();
  }

  if (!state.showNodes) return;

  // nodes (optional)
  const showIds = state.showNodeIds;
  if (showIds) {
    ctx.save();
    ctx.fillStyle = "rgba(148, 163, 184, 0.9)";
    ctx.font = "10px monospace";
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
  }
  for (const n of state.map.nodes) {
    const p = toCanvas(n.x, n.y);
    ctx.beginPath();
    const r = n.type === 7 || n.type === 9 ? 4 : n.type === 1 || n.type === 2 ? 3 : 1;
    const typeColor = n.type === 7 || n.type === 9 ? "#ff4500" : n.type === 1 || n.type === 2 ? "#ffd700" : "#6b7280";
    const regionId = Number(n.region);
    const hasRegion = showRegions && Number.isFinite(regionId) && regionId >= 0;
    const isBridge = bridgeNodeSet.has(String(n.id));
    const c = isBridge ? "#fb923c" : (hasRegion ? regionColor(regionId, 0.85) : typeColor);
    ctx.arc(p.cx, p.cy, r, 0, Math.PI * 2);
    ctx.fillStyle = c;
    ctx.globalAlpha = 0.85;
    ctx.fill();
    ctx.globalAlpha = 1.0;
    if (hasRegion) {
      ctx.strokeStyle = typeColor;
      ctx.lineWidth = 1;
      ctx.stroke();
    }
    if (showIds) {
      const id = Number.isFinite(n.id) ? n.id : n.nodeId;
      if (id !== undefined) {
        ctx.fillText(String(id), p.cx + 6, p.cy - 6);
      }
    }
  }
  if (showIds) ctx.restore();
}

function buildReservedNodes(agvs, map) {
  const nodePos = map?.nodePositions || {};
  const entries = new Map();
  const addNode = (rawId, fallbackX, fallbackY, seen) => {
    const id = Number(rawId);
    if (!Number.isFinite(id) || id < 0) return;
    const nodeId = Math.trunc(id);
    const key = String(nodeId);
    let entry = entries.get(key);
    if (!entry) {
      const pos = nodePos[key];
      const posX = Number(pos?.x);
      const posY = Number(pos?.y);
      const fx = Number(fallbackX);
      const fy = Number(fallbackY);
      const x = Number.isFinite(posX) ? posX : fx;
      const y = Number.isFinite(posY) ? posY : fy;
      if (!Number.isFinite(x) || !Number.isFinite(y)) return;
      entry = { nodeId, x, y, count: 0 };
      entries.set(key, entry);
    }
    if (seen && seen.has(key)) return;
    if (seen) seen.add(key);
    entry.count += 1;
  };

  for (const agv of agvs || []) {
    const seen = new Set();
    const points = Array.isArray(agv.trail) ? agv.trail : [];
    if (points.length) {
      for (const pt of points) {
        addNode(pt.nodeId, pt.x, pt.y, seen);
      }
    }
    addNode(agv.nodeId, agv.x, agv.y, seen);
  }

  return Array.from(entries.values());
}

function buildReservedNodesFromSnapshot(reserved, map) {
  if (!Array.isArray(reserved)) return [];
  const nodePos = map?.nodePositions || {};
  const entries = new Map();
  for (const item of reserved) {
    const rawId = item && typeof item === "object" ? item.nodeId : item;
    const id = Number(rawId);
    if (!Number.isFinite(id) || id < 0) continue;
    const nodeId = Math.trunc(id);
    const key = String(nodeId);
    let entry = entries.get(key);
    if (!entry) {
      const pos = nodePos[key];
      const x = Number(pos?.x);
      const y = Number(pos?.y);
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      entry = { nodeId, x, y, count: 0 };
      entries.set(key, entry);
    }
    entry.count += 1;
  }
  return Array.from(entries.values());
}

function drawReservedNodes() {
  if (!state.showReserved || !state.reservedNodes.length) return;
  const { toCanvas } = canvasTransform();
  ctx.save();
  for (const node of state.reservedNodes) {
    const p = toCanvas(node.x, node.y);
    const count = Math.max(1, Number(node.count || 1));
    const r = Math.min(10, 3 + (count - 1) * 1.5);
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, r + 3, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(249, 115, 22, 0.18)";
    ctx.fill();
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, r, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(249, 115, 22, 0.65)";
    ctx.fill();
    ctx.strokeStyle = "rgba(251, 146, 60, 0.9)";
    ctx.lineWidth = 1;
    ctx.stroke();
  }
  ctx.restore();
}

function drawPolyline(points, style) {
  if (!points || points.length < 2) return;
  const { toCanvas } = canvasTransform();
  ctx.save();
  ctx.strokeStyle = style.color;
  ctx.lineWidth = style.width || 2;
  ctx.globalAlpha = style.alpha ?? 0.55;
  if (style.dash) ctx.setLineDash(style.dash);
  ctx.beginPath();
  const p0 = toCanvas(points[0].x, points[0].y);
  ctx.moveTo(p0.cx, p0.cy);
  for (let i = 1; i < points.length; i++) {
    const pi = toCanvas(points[i].x, points[i].y);
    ctx.lineTo(pi.cx, pi.cy);
  }
  ctx.stroke();
  ctx.restore();
}

function drawAGVs() {
  const { toCanvas } = canvasTransform();
  const list = state.agvs.slice(0, state.displayCount);
  for (let i = 0; i < list.length; i++) {
    const agv = list[i];
    const color = COLORS[i % COLORS.length];
    if (state.showPath) drawPolyline(agv.path, { color, width: 2.2, alpha: 0.35, dash: null });
    if (state.showTrail) drawPolyline(agv.trail, { color, width: 2.8, alpha: 0.75, dash: [8, 6] });

    // next subtask marker (only for assigned)
    const firstSubtask = Array.isArray(agv.subtasks) && agv.subtasks.length ? agv.subtasks[0] : agv.nextSubtask;
    if (state.showTasks && agv.assigned && firstSubtask) {
      const p = toCanvas(firstSubtask.x, firstSubtask.y);
      ctx.save();
      ctx.globalAlpha = 0.95;
      ctx.fillStyle = "rgba(255,255,255,0.92)";
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      const s = 10;
      ctx.beginPath();
      ctx.rect(p.cx - s / 2, p.cy - s / 2, s, s);
      ctx.fill();
      ctx.stroke();
      if (state.showLabels) {
        const label = firstSubtask.label ? String(firstSubtask.label) : `S:${firstSubtask.nodeId}`;
        ctx.fillStyle = "#e5e7eb";
        ctx.font = "bold 11px Arial";
        ctx.textAlign = "left";
        ctx.textBaseline = "middle";
        ctx.fillText(`${label} ${agv.id}`, p.cx + 10, p.cy - 12);
      }
      ctx.restore();
    }

    const p = toCanvas(agv.x, agv.y);
    const sp = speedStateMmS(agv.speed);
    const fill = speedColor(sp);

    // halo
    ctx.save();
    ctx.globalAlpha = 0.25;
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, 18, 0, Math.PI * 2);
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.restore();

    // body
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, 9, 0, Math.PI * 2);
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    ctx.stroke();

    // heading
    const ang = ((agv.angle || 0) * Math.PI) / 180.0;
    const hx = Math.cos(ang) * 14;
    const hy = Math.sin(ang) * 14;
    ctx.beginPath();
    ctx.moveTo(p.cx, p.cy);
    ctx.lineTo(p.cx + hx, p.cy + hy);
    ctx.strokeStyle = "rgba(17,17,17,0.9)";
    ctx.lineWidth = 2;
    ctx.stroke();

    if (state.showLabels) {
      ctx.fillStyle = color;
      ctx.font = "bold 11px Arial";
      ctx.textAlign = "center";
      ctx.textBaseline = "bottom";
      ctx.fillText(agv.id, p.cx, p.cy - 14);
      ctx.textBaseline = "top";
      ctx.fillStyle = fill;
      const vms = (agv.speed || 0) / 1000.0;
      const phase = String(agv.phase || "");
      const phaseText = String(agv.phaseText || "");
      const leftS = Number(agv.phaseLeftS || 0);
      let label = "";
      if (phase && phase !== "move") {
        label = phaseText || phase;
        if (leftS > 0.05) label += ` ${leftS.toFixed(1)}s`;
      } else if (agv.assigned) {
        label = `${vms.toFixed(2)} m/s`;
      } else {
        label = `空闲 ${vms.toFixed(2)} m/s`;
      }
      ctx.fillText(label, p.cx, p.cy + 14);
    }
  }
}

function draw() {
  ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  drawMap();
  drawReservedNodes();
  drawAGVs();
}

function updateSidebar() {
  document.getElementById("stat-sim").textContent = `${state.simTimeS.toFixed(1)}s`;
  document.getElementById("stat-agv").textContent = String(state.agvs.length);
  const assigned = state.agvs.filter((a) => a.assigned).length;
  document.getElementById("stat-assigned").textContent = String(assigned);
  const mapName = state.map?.info?.name || "-";
  document.getElementById("stat-map").textContent = mapName;
  const metrics = state.metrics || {};
  const avgPublishStart = Number(metrics.avg_publish_to_start_s);
  document.getElementById("stat-publish-start").textContent = Number.isFinite(avgPublishStart)
    ? `${avgPublishStart.toFixed(1)}s`
    : "-";
  const distMm = Number(metrics.completed_task_distance_mm);
  if (Number.isFinite(distMm)) {
    const distM = distMm / 1000.0;
    const distText = distM >= 1000 ? `${(distM / 1000.0).toFixed(2)} km` : `${distM.toFixed(1)} m`;
    document.getElementById("stat-distance").textContent = distText;
  } else {
    document.getElementById("stat-distance").textContent = "-";
  }
  const throughput = Number(metrics.throughput_tasks_per_min);
  document.getElementById("stat-throughput").textContent = Number.isFinite(throughput)
    ? `${throughput.toFixed(2)} /min`
    : "-";

  const listEl = document.getElementById("agv-list");
  listEl.innerHTML = "";
  const list = state.agvs.slice(0, state.displayCount);
  for (let i = 0; i < list.length; i++) {
    const agv = list[i];
    const color = COLORS[i % COLORS.length];
    const sp = speedStateMmS(agv.speed);
    const fill = speedColor(sp);
    const vms = (agv.speed || 0) / 1000.0;
    const activeMarker = agv.phase === "dwell" ? findActiveMarker(agv) : null;
    const nextMarker = findNextMarker(agv);
    const currentMarker = activeMarker || nextMarker;
    const taskText = formatTask(currentMarker);
    const destText = formatDestination(nextMarker);
    let subtaskLabel = "";
    if (currentMarker) {
      const label = currentMarker.label ? String(currentMarker.label) : "";
      const fallback = `S:${currentMarker.nodeId}`;
      const showLabel = label && label !== destText && label !== taskText;
      subtaskLabel = showLabel ? label : fallback;
    }
    const phaseText = String(agv.phaseText || "");
    const leftS = Number(agv.phaseLeftS || 0);
    const phaseLabel = phaseText ? `${phaseText}${leftS > 0.05 ? ` ${leftS.toFixed(1)}s` : ""}` : "";
    const item = document.createElement("div");
    item.className = "agv-item";
    item.style.borderLeftColor = color;
    item.innerHTML = `
      <div class="agv-header">
        <div class="agv-id">${agv.id}${agv.assigned ? "" : " (空闲)"}</div>
        <div class="agv-speed" style="color:${fill}">${vms.toFixed(2)} m/s</div>
      </div>
      <div class="agv-meta">
        <div>${agv.nodeId || "?"} → ${agv.nextNodeId || "-"}</div>
        <div>${phaseLabel}</div>
      </div>
      <div class="agv-meta">
        <div>任务 ${taskText}</div>
        <div>目的地 ${destText}</div>
      </div>
      ${subtaskLabel ? `<div class="agv-meta"><div>子任务 ${subtaskLabel}</div><div></div></div>` : ""}
    `;
    listEl.appendChild(item);
  }

  updateRegionPanel();
}

function setConn(ok) {
  const dot = document.getElementById("conn-dot");
  const text = document.getElementById("conn-text");
  if (ok) {
    dot.classList.add("ok");
    dot.classList.remove("warn");
    dot.classList.remove("bad");
    text.textContent = "在线";
  } else {
    dot.classList.remove("ok");
    dot.classList.add("warn");
    text.textContent = "等待数据...";
  }
}

async function fetchMap() {
  const r = await fetch("/map.json", { cache: "no-store" });
  if (!r.ok) throw new Error(`map fetch failed: ${r.status}`);
  const data = await r.json();
  state.map = data;
  state.vMaxMmS = Math.max(1, Number(data.info?.maxSpeed || 1000));
  const regionCount = Number(data.regions?.count || 0);
  const regionToggle = document.getElementById("toggle-regions");
  const bridgeToggle = document.getElementById("toggle-bridges");
  if (regionToggle) {
    const hasRegions = Number.isFinite(regionCount) && regionCount > 0;
    regionToggle.disabled = !hasRegions;
    if (!hasRegions) {
      state.showRegions = false;
      regionToggle.checked = false;
    }
  }
  if (bridgeToggle) {
    const hasBridges = Array.isArray(data.bridgeNodeIds) && data.bridgeNodeIds.length > 0;
    bridgeToggle.disabled = !hasBridges;
    if (!hasBridges) {
      state.showBridges = false;
      bridgeToggle.checked = false;
    }
  }
  document.title = `CRCS-V1.0 在线仿真 - ${data.info?.name || "map"}`;
  fitToMap();
}

async function pollOnce() {
  const r = await fetch("/state.json", { cache: "no-store" });
  if (!r.ok) throw new Error(`state fetch failed: ${r.status}`);
  const data = await r.json();
  state.simTimeS = Number(data.sim_time_s || 0);
  state.vMaxMmS = Math.max(1, Number(data.v_max_mm_s || state.vMaxMmS));
  state.agvs = (data.agvs || []).slice().sort((a, b) => String(a.id).localeCompare(String(b.id)));
  state.metrics = data.metrics || null;
  if (Array.isArray(data.reserved_nodes)) {
    state.reservedNodes = buildReservedNodesFromSnapshot(data.reserved_nodes, state.map);
  } else {
    state.reservedNodes = [];
  }
  state.lastOkTs = Date.now();
  setConn(true);
  updateSidebar();
  draw();
}

function schedulePoll() {
  if (state.pollTimer) clearInterval(state.pollTimer);
  state.pollTimer = setInterval(() => {
    pollOnce().catch((err) => {
      console.warn("[poll] failed:", err?.message || err);
      const age = Date.now() - state.lastOkTs;
      setConn(age < 2000);
    });
  }, state.pollMs);
}

function initUi() {
  document.getElementById("btn-fit").addEventListener("click", fitToMap);
  document.getElementById("toggle-path").addEventListener("change", (e) => {
    state.showPath = e.target.checked;
    draw();
  });
  document.getElementById("toggle-trail").addEventListener("change", (e) => {
    state.showTrail = e.target.checked;
    draw();
  });
  document.getElementById("toggle-label").addEventListener("change", (e) => {
    state.showLabels = e.target.checked;
    draw();
  });
  document.getElementById("toggle-task").addEventListener("change", (e) => {
    state.showTasks = e.target.checked;
    draw();
  });
  document.getElementById("toggle-nodes").addEventListener("change", (e) => {
    state.showNodes = e.target.checked;
    draw();
  });
  document.getElementById("toggle-node-ids").addEventListener("change", (e) => {
    state.showNodeIds = e.target.checked;
    draw();
  });
  document.getElementById("toggle-regions").addEventListener("change", (e) => {
    state.showRegions = e.target.checked;
    draw();
  });
  document.getElementById("toggle-bridges").addEventListener("change", (e) => {
    state.showBridges = e.target.checked;
    draw();
  });
  document.getElementById("toggle-reserved").addEventListener("change", (e) => {
    state.showReserved = e.target.checked;
    draw();
  });

  const countSlider = document.getElementById("count-slider");
  const countValue = document.getElementById("count-value");
  countSlider.addEventListener("input", (e) => {
    state.displayCount = Math.max(1, Number(e.target.value || 1));
    countValue.textContent = String(state.displayCount);
    updateSidebar();
    draw();
  });

  const pollSlider = document.getElementById("poll-slider");
  const pollValue = document.getElementById("poll-value");
  pollSlider.addEventListener("input", (e) => {
    state.pollMs = Math.max(50, Number(e.target.value || 120));
    pollValue.textContent = `${state.pollMs}ms`;
    schedulePoll();
  });

  // drag/pan
  canvas.addEventListener("mousedown", (e) => {
    if (e.button !== 0) return;
    state.isDragging = true;
    state.lastX = e.clientX;
    state.lastY = e.clientY;
    canvas.classList.add("dragging");
  });
  canvas.addEventListener("mousemove", (e) => {
    if (!state.isDragging) return;
    const dx = e.clientX - state.lastX;
    const dy = e.clientY - state.lastY;
    state.lastX = e.clientX;
    state.lastY = e.clientY;
    state.offsetX += dx;
    state.offsetY += dy;
    draw();
  });
  const endDrag = () => {
    state.isDragging = false;
    canvas.classList.remove("dragging");
  };
  canvas.addEventListener("mouseup", endDrag);
  canvas.addEventListener("mouseleave", endDrag);

  // wheel zoom
  canvas.addEventListener(
    "wheel",
    (e) => {
      e.preventDefault();
      const point = canvasPointFromEvent(e);
      const zoom = Math.exp(-e.deltaY * 0.001);
      const nextScale = clampScale(state.scale * zoom);
      zoomAtPoint(point.x, point.y, nextScale);
      draw();
    },
    { passive: false }
  );
}

async function main() {
  resizeCanvas();
  window.addEventListener("resize", () => {
    resizeCanvas();
    draw();
  });

  initUi();
  setConn(false);

  try {
    await fetchMap();
  } catch (err) {
    console.error("Failed to load map:", err);
  }

  schedulePoll();
}

main().catch((err) => console.error(err));
