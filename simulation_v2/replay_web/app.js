/* eslint-disable no-console */

const COLORS = [
  "#2ec4b6",
  "#ff9f43",
  "#4dabf7",
  "#ff6b6b",
  "#95d5b2",
  "#ffd166",
  "#c77dff",
  "#72efdd",
  "#f4a261",
  "#90caf9",
];

const ZOOM_MIN = 0.55;
const ZOOM_MAX = 8.0;
const MAIN_AUTO_ZOOM_MAX = 1.7;
const FOCUS_AUTO_ZOOM_MAX = 6.5;
const DIRECTOR_RAMP_IN_S = 0.45;
const DIRECTOR_RAMP_OUT_S = 0.65;

function createCameraState() {
  return {
    centerX: 0,
    centerY: 0,
    zoom: 1,
    targetCenterX: 0,
    targetCenterY: 0,
    targetZoom: 1,
    dragging: false,
    pointerId: null,
    dragStartX: 0,
    dragStartY: 0,
    dragCenterX: 0,
    dragCenterY: 0,
  };
}

const replay = {
  bundle: null,
  map: null,
  frames: [],
  frameTimes: [],
  timeline: [],
  scenes: [],
  firstPaths: {},
  summary: null,
  totalTimeS: 0,
  directorBaseRate: 1.5,
  currentTimeS: 0,
  playing: true,
  globalSpeed: 1.0,
  autoDirector: true,
  autoFocus: true,
  autoMainCamera: false,
  highlightFocus: true,
  showReserved: false,
  showBridges: true,
  displayMode: "trail",
  followAgvId: "",
  lastTickTs: 0,
  mainCamera: createCameraState(),
  focusCamera: createCameraState(),
  ui: {
    sceneListBuilt: false,
    activeSceneId: "",
    lastScrolledSceneId: "",
    lastDetailKey: "",
  },
};

const mainCanvas = document.getElementById("main-canvas");
const focusCanvas = document.getElementById("focus-canvas");
const mainCtx = mainCanvas.getContext("2d");
const focusCtx = focusCanvas.getContext("2d");

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function lerp(a, b, t) {
  return a + (b - a) * clamp(t, 0, 1);
}

function smoothstep(t) {
  const x = clamp(t, 0, 1);
  return x * x * (3 - 2 * x);
}

function resizeCanvas(canvas, ctx) {
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * ratio));
  canvas.height = Math.max(1, Math.floor(rect.height * ratio));
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
}

function parseNodeId(value) {
  const num = Number(value);
  return Number.isFinite(num) ? Math.trunc(num) : null;
}

function mapBounds() {
  const nodes = replay.map?.nodes || [];
  if (!nodes.length) return { minX: 0, minY: 0, maxX: 1, maxY: 1 };
  let minX = nodes[0].x;
  let maxX = nodes[0].x;
  let minY = nodes[0].y;
  let maxY = nodes[0].y;
  for (const node of nodes) {
    minX = Math.min(minX, node.x);
    maxX = Math.max(maxX, node.x);
    minY = Math.min(minY, node.y);
    maxY = Math.max(maxY, node.y);
  }
  return { minX, minY, maxX, maxY };
}

function boundsCenter(bounds) {
  return {
    x: (bounds.minX + bounds.maxX) / 2,
    y: (bounds.minY + bounds.maxY) / 2,
  };
}

function fitScale(canvas, bounds, padding = 0.12) {
  const width = Math.max(1, bounds.maxX - bounds.minX);
  const height = Math.max(1, bounds.maxY - bounds.minY);
  const usableW = Math.max(1, canvas.clientWidth * (1 - padding * 2));
  const usableH = Math.max(1, canvas.clientHeight * (1 - padding * 2));
  return Math.max(0.0001, Math.min(usableW / width, usableH / height));
}

function mapBaseScale(canvas) {
  return fitScale(canvas, mapBounds(), 0.12);
}

function zoomForBounds(canvas, bounds, padding = 0.16) {
  return fitScale(canvas, bounds, padding) / Math.max(0.0001, mapBaseScale(canvas));
}

function currentFrameIndex() {
  if (!replay.frameTimes.length) return 0;
  let lo = 0;
  let hi = replay.frameTimes.length - 1;
  while (lo <= hi) {
    const mid = Math.floor((lo + hi) / 2);
    if (replay.frameTimes[mid] <= replay.currentTimeS) {
      lo = mid + 1;
    } else {
      hi = mid - 1;
    }
  }
  return Math.max(0, Math.min(replay.frames.length - 1, hi));
}

function currentFrame() {
  return replay.frames[currentFrameIndex()] || { agvs: [], reserved_nodes: [] };
}

function currentSceneAt(timeS) {
  if (!Array.isArray(replay.scenes) || !replay.scenes.length) return null;
  for (const scene of replay.scenes) {
    const startS = Number(scene.start_s || 0);
    const endS = Number(scene.end_s ?? startS);
    if (timeS >= startS && timeS <= endS) {
      return scene;
    }
  }
  return null;
}

function currentScene() {
  return currentSceneAt(replay.currentTimeS);
}

function currentEventAt(timeS) {
  for (const item of replay.timeline) {
    const startS = Number(item.start_s || 0);
    const endS = Number(item.end_s || startS);
    if (timeS >= startS && timeS <= endS) {
      return item;
    }
  }
  return null;
}

function currentEvent() {
  return currentEventAt(replay.currentTimeS);
}

function currentCueAt(timeS) {
  let selected = null;
  let bestDistance = Number.POSITIVE_INFINITY;
  for (const item of replay.timeline) {
    const startS = Number(item.start_s || 0);
    const endS = Number(item.end_s || startS);
    if (timeS < startS - DIRECTOR_RAMP_IN_S || timeS > endS + DIRECTOR_RAMP_OUT_S) {
      continue;
    }
    let distance = 0;
    if (timeS < startS) distance = startS - timeS;
    else if (timeS > endS) distance = timeS - endS;
    if (distance < bestDistance) {
      selected = item;
      bestDistance = distance;
    }
  }
  return selected;
}

function currentCue() {
  return currentCueAt(replay.currentTimeS);
}

function cueBlendFactor(event, timeS) {
  if (!event) return 0;
  const startS = Number(event.start_s || 0);
  const endS = Number(event.end_s || startS);
  if (timeS >= startS && timeS <= endS) return 1;
  if (timeS < startS) {
    return smoothstep((timeS - (startS - DIRECTOR_RAMP_IN_S)) / Math.max(0.001, DIRECTOR_RAMP_IN_S));
  }
  return 1 - smoothstep((timeS - endS) / Math.max(0.001, DIRECTOR_RAMP_OUT_S));
}

function effectiveSpeedAt(timeS) {
  const scene = currentSceneAt(timeS);
  if (!replay.autoDirector) {
    return replay.globalSpeed;
  }
  const baseRate = replay.directorBaseRate;
  const cue = currentCueAt(timeS);
  if (!cue) {
    const sceneRate = Number(scene?.playback_rate || 0);
    const rate = Number.isFinite(sceneRate) && sceneRate > 0 ? sceneRate : baseRate;
    return rate * replay.globalSpeed;
  }
  const cueRate = Number(cue.playback_rate);
  const eventRate = Number.isFinite(cueRate) && cueRate > 0 ? cueRate : 1;
  const mix = cueBlendFactor(cue, timeS);
  return lerp(baseRate, eventRate, mix) * replay.globalSpeed;
}

function effectiveSpeed() {
  return effectiveSpeedAt(replay.currentTimeS);
}

function routeColor(index) {
  return COLORS[index % COLORS.length];
}

function speedColor(speed) {
  if (!(speed > 0)) return "#9bb1d0";
  if (speed >= 800) return "#2ec4b6";
  if (speed >= 400) return "#ffd166";
  return "#ff7b7b";
}

function pointForNode(nodeId) {
  const key = nodeId !== null && nodeId !== undefined ? String(nodeId) : "";
  return replay.map?.nodePositions?.[key] || null;
}

function allAgvIds() {
  const ids = [];
  const seen = new Set();
  const source = Array.isArray(replay.summary?.agv_ids) && replay.summary.agv_ids.length ? replay.summary.agv_ids : null;
  if (source) {
    for (const value of source) {
      const id = String(value || "").trim();
      if (!id || seen.has(id)) continue;
      seen.add(id);
      ids.push(id);
    }
    return ids;
  }
  for (const frame of replay.frames) {
    for (const agv of frame.agvs || []) {
      const id = String(agv.id || "").trim();
      if (!id || seen.has(id)) continue;
      seen.add(id);
      ids.push(id);
    }
  }
  return ids.sort();
}

function focusDescriptor(frame, event) {
  const scene = currentScene();
  const agvIds = new Set();
  const nodeIds = new Set();
  const rawFocus = event?.focus || scene?.focus || {};

  if (replay.followAgvId) {
    agvIds.add(replay.followAgvId);
    const firstPath = replay.firstPaths?.[replay.followAgvId];
    const subtaskNodeId = firstPath ? parseNodeId(firstPath.first_subtask?.nodeId) : null;
    if (subtaskNodeId !== null) nodeIds.add(subtaskNodeId);
  } else if (Array.isArray(rawFocus.agv_ids)) {
    // No explicitly followed AGV — use scene/event focus AGVs
    for (const id of rawFocus.agv_ids) {
      const s = String(id || "").trim();
      if (s) agvIds.add(s);
    }
  }

  if (Array.isArray(rawFocus.node_ids)) {
    for (const nid of rawFocus.node_ids) {
      const parsed = parseNodeId(nid);
      if (parsed !== null) nodeIds.add(parsed);
    }
  }

  return {
    agvIds: Array.from(agvIds),
    nodeIds: Array.from(nodeIds),
    zoom: Math.max(1.0, Number(rawFocus.zoom || (replay.followAgvId ? 1.8 : 1.3)) || 1.3),
    mode: String(rawFocus.mode || (replay.followAgvId ? "path" : "auto")),
  };
}

function collectFocusPoints(frame, focus) {
  const points = [];
  const pushPoint = (x, y) => {
    if (Number.isFinite(Number(x)) && Number.isFinite(Number(y))) {
      points.push({ x: Number(x), y: Number(y) });
    }
  };

  for (const agvId of focus.agvIds) {
    const agv = frame.agvs.find((item) => String(item.id) === agvId);
    if (!agv) continue;
    pushPoint(agv.x, agv.y);
    const nextNode = pointForNode(parseNodeId(agv.nextNodeId));
    if (nextNode) pushPoint(nextNode.x, nextNode.y);
    const nextSubtask = agv.nextSubtask;
    if (nextSubtask) pushPoint(nextSubtask.x, nextSubtask.y);
    const routePoints =
      focus.mode === "path"
        ? agv.path || []
        : focus.mode === "trail"
          ? agv.trail || []
          : [...(agv.trail || []).slice(0, 6), ...(agv.path || []).slice(0, 6)];
    for (const pt of routePoints.slice(0, 6)) {
      pushPoint(pt.x, pt.y);
    }
    const firstPath = replay.firstPaths?.[agvId];
    if (firstPath && Array.isArray(firstPath.points)) {
      for (const pt of firstPath.points.slice(0, 16)) {
        pushPoint(pt.x, pt.y);
      }
    }
  }

  for (const nodeId of focus.nodeIds) {
    const pos = pointForNode(nodeId);
    if (pos) pushPoint(pos.x, pos.y);
  }

  return points;
}

function focusBounds(frame, focus) {
  const points = collectFocusPoints(frame, focus);
  if (!points.length) return mapBounds();
  let minX = points[0].x;
  let maxX = points[0].x;
  let minY = points[0].y;
  let maxY = points[0].y;
  for (const pt of points) {
    minX = Math.min(minX, pt.x);
    maxX = Math.max(maxX, pt.x);
    minY = Math.min(minY, pt.y);
    maxY = Math.max(maxY, pt.y);
  }
  const span = Math.max(120, maxX - minX, maxY - minY);
  const zoomBias = clamp(2.2 / Math.max(1.0, focus.zoom), 0.34, 1.3);
  const pad = Math.max(320, span * 0.72 * zoomBias);
  return {
    minX: minX - pad,
    maxX: maxX + pad,
    minY: minY - pad,
    maxY: maxY + pad,
  };
}

function createTransform(canvas, camera) {
  const scale = mapBaseScale(canvas) * camera.zoom;
  const centerX = canvas.clientWidth / 2;
  const centerY = canvas.clientHeight / 2;
  return {
    scale,
    toCanvas(x, y) {
      return {
        cx: (x - camera.centerX) * scale + centerX,
        cy: (y - camera.centerY) * scale + centerY,
      };
    },
    toWorld(cx, cy) {
      return {
        x: (cx - centerX) / Math.max(0.0001, scale) + camera.centerX,
        y: (cy - centerY) / Math.max(0.0001, scale) + camera.centerY,
      };
    },
  };
}

function cameraTarget(frame, event, variant) {
  const wholeMap = mapBounds();
  const wholeCenter = boundsCenter(wholeMap);
  const focus = focusDescriptor(frame, event);
  const bounds = focusBounds(frame, focus);
  const center = boundsCenter(bounds);
  const fitZoom = zoomForBounds(variant === "main" ? mainCanvas : focusCanvas, bounds, variant === "main" ? 0.22 : 0.16);

  if (variant === "main") {
    if (!replay.autoMainCamera) {
      return {
        centerX: replay.mainCamera.targetCenterX || replay.mainCamera.centerX || wholeCenter.x,
        centerY: replay.mainCamera.targetCenterY || replay.mainCamera.centerY || wholeCenter.y,
        zoom: clamp(replay.mainCamera.targetZoom || replay.mainCamera.zoom || 1, ZOOM_MIN, ZOOM_MAX),
      };
    }
    if (!event && !replay.followAgvId && focus.agvIds.length === 0 && focus.nodeIds.length === 0) {
      return { centerX: wholeCenter.x, centerY: wholeCenter.y, zoom: 1 };
    }
    const desired = event ? focus.zoom * 0.58 : 1.28;
    return {
      centerX: center.x,
      centerY: center.y,
      zoom: clamp(Math.min(MAIN_AUTO_ZOOM_MAX, Math.min(fitZoom * 0.72, Math.max(1.0, desired))), 1.0, MAIN_AUTO_ZOOM_MAX),
    };
  }

  if (!replay.autoFocus && !replay.followAgvId) {
    return { centerX: wholeCenter.x, centerY: wholeCenter.y, zoom: 1.05 };
  }
  if (!event && !replay.followAgvId && focus.agvIds.length === 0 && focus.nodeIds.length === 0) {
    return { centerX: wholeCenter.x, centerY: wholeCenter.y, zoom: 1.05 };
  }

  const desired = event ? focus.zoom : replay.followAgvId ? 1.8 : 1.2;
  return {
    centerX: center.x,
    centerY: center.y,
    zoom: clamp(Math.min(fitZoom * 1.08, Math.max(1.05, desired)), 1.05, FOCUS_AUTO_ZOOM_MAX),
  };
}

function applyCameraState(camera, target, options = {}) {
  const snap = Boolean(options.snap);
  const easing = Number(options.easing || 0.18);
  if (
    snap ||
    !Number.isFinite(camera.centerX) ||
    !Number.isFinite(camera.centerY) ||
    !Number.isFinite(camera.zoom) ||
    camera.zoom <= 0
  ) {
    camera.centerX = target.centerX;
    camera.centerY = target.centerY;
    camera.zoom = target.zoom;
  } else {
    camera.centerX = lerp(camera.centerX, target.centerX, easing);
    camera.centerY = lerp(camera.centerY, target.centerY, easing);
    camera.zoom = lerp(camera.zoom, target.zoom, easing);
  }
  camera.targetCenterX = target.centerX;
  camera.targetCenterY = target.centerY;
  camera.targetZoom = target.zoom;
}

function syncCameras(frame, event, options = {}) {
  applyCameraState(replay.mainCamera, cameraTarget(frame, event, "main"), {
    snap: options.snapMain,
    easing: 0.15,
  });
  applyCameraState(replay.focusCamera, cameraTarget(frame, event, "focus"), {
    snap: options.snapFocus,
    easing: 0.2,
  });
}

function drawMap(ctx, canvas, transform) {
  const nodePos = replay.map?.nodePositions || {};
  ctx.save();
  ctx.lineWidth = 1;
  for (const edge of replay.map?.edges || []) {
    const a = nodePos[String(edge.startNode)];
    const b = nodePos[String(edge.endNode)];
    if (!a || !b) continue;
    const p0 = transform.toCanvas(a.x, a.y);
    const p1 = transform.toCanvas(b.x, b.y);
    ctx.strokeStyle = "rgba(152, 176, 209, 0.22)";
    ctx.beginPath();
    ctx.moveTo(p0.cx, p0.cy);
    ctx.lineTo(p1.cx, p1.cy);
    ctx.stroke();
  }
  for (const node of replay.map?.nodes || []) {
    const p = transform.toCanvas(node.x, node.y);
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, 1.6, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(189, 207, 236, 0.55)";
    ctx.fill();
  }
  ctx.restore();
}

function drawBridges(ctx, transform) {
  if (!replay.showBridges) return;
  const bridgeEdges = Array.isArray(replay.map?.bridgeEdges) ? replay.map.bridgeEdges : [];
  const bridgeNodeIds = Array.isArray(replay.map?.bridgeNodeIds) ? replay.map.bridgeNodeIds : [];
  const nodePos = replay.map?.nodePositions || {};
  if (bridgeEdges.length) {
    ctx.save();
    ctx.strokeStyle = "rgba(234, 88, 12, 0.95)";
    ctx.lineWidth = 4;
    ctx.globalAlpha = 0.78;
    for (const edge of bridgeEdges) {
      const a = nodePos[String(edge.startNode)];
      const b = nodePos[String(edge.endNode)];
      if (!a || !b) continue;
      const p0 = transform.toCanvas(a.x, a.y);
      const p1 = transform.toCanvas(b.x, b.y);
      ctx.beginPath();
      ctx.moveTo(p0.cx, p0.cy);
      ctx.lineTo(p1.cx, p1.cy);
      ctx.stroke();
    }
    ctx.restore();
  }
  if (bridgeNodeIds.length) {
    ctx.save();
    for (const rawId of bridgeNodeIds) {
      const pos = nodePos[String(rawId)];
      if (!pos) continue;
      const p = transform.toCanvas(pos.x, pos.y);
      ctx.beginPath();
      ctx.arc(p.cx, p.cy, 8, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(251, 146, 60, 0.18)";
      ctx.fill();
      ctx.beginPath();
      ctx.arc(p.cx, p.cy, 4.5, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(234, 88, 12, 0.96)";
      ctx.fill();
    }
    ctx.restore();
  }
}

function drawBackdrop(ctx, canvas, focus, event) {
  const active = replay.highlightFocus && Boolean(replay.followAgvId);
  if (!active) return;
  ctx.save();
  ctx.fillStyle = "rgba(3, 8, 16, 0.1)";
  ctx.fillRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  ctx.restore();
}

function drawPolyline(ctx, transform, points, style) {
  if (!points || points.length < 2) return;
  ctx.save();
  ctx.strokeStyle = style.color;
  ctx.lineWidth = style.width || 2;
  ctx.globalAlpha = style.alpha ?? 0.8;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  if (style.dash) ctx.setLineDash(style.dash);
  ctx.beginPath();
  const first = transform.toCanvas(points[0].x, points[0].y);
  ctx.moveTo(first.cx, first.cy);
  for (let i = 1; i < points.length; i += 1) {
    const pt = transform.toCanvas(points[i].x, points[i].y);
    ctx.lineTo(pt.cx, pt.cy);
  }
  ctx.stroke();
  ctx.restore();
}

function drawReserved(ctx, transform, frame, focus) {
  if (!replay.showReserved) return;
  const hasEmphasis = replay.highlightFocus && (focus.agvIds.length || focus.nodeIds.length);
  ctx.save();
  for (const item of frame.reserved_nodes || []) {
    const nid = parseNodeId(item.nodeId);
    const pos = pointForNode(nid);
    if (!pos) continue;
    const p = transform.toCanvas(pos.x, pos.y);
    const isFocus = focus.nodeIds.includes(nid);
    const alpha = hasEmphasis && !isFocus ? 0.38 : 1;
    ctx.globalAlpha = alpha;
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, isFocus ? 13 : 8, 0, Math.PI * 2);
    ctx.fillStyle = isFocus ? "rgba(255, 99, 71, 0.28)" : "rgba(255, 159, 67, 0.18)";
    ctx.fill();
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, isFocus ? 8 : 5, 0, Math.PI * 2);
    ctx.fillStyle = item.reasonLabel === "temp_goal" ? "rgba(255, 107, 107, 0.82)" : "rgba(255, 159, 67, 0.82)";
    ctx.fill();
  }
  ctx.restore();
}

function drawFocusNodes(ctx, transform, focus) {
  const pulse = 0.55 + 0.45 * Math.sin(replay.currentTimeS * 6.4);
  ctx.save();
  for (const nid of focus.nodeIds) {
    const pos = pointForNode(nid);
    if (!pos) continue;
    const p = transform.toCanvas(pos.x, pos.y);
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, 15 + pulse * 5, 0, Math.PI * 2);
    ctx.strokeStyle = `rgba(255, 214, 102, ${0.35 + pulse * 0.35})`;
    ctx.lineWidth = 2.4;
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, 10, 0, Math.PI * 2);
    ctx.strokeStyle = "rgba(255, 214, 102, 0.95)";
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = "rgba(255, 214, 102, 0.95)";
    ctx.font = "bold 11px Arial";
    ctx.textAlign = "center";
    ctx.textBaseline = "bottom";
    ctx.fillText(`N${nid}`, p.cx, p.cy - 16);
  }
  ctx.restore();
}

function drawSubtaskMarker(ctx, transform, pt, label) {
  const x = Number(pt.x), y = Number(pt.y);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return;
  const sp = transform.toCanvas(x, y);
  ctx.beginPath();
  ctx.arc(sp.cx, sp.cy, 12, 0, Math.PI * 2);
  ctx.fillStyle = "rgba(255, 214, 102, 0.22)";
  ctx.fill();
  ctx.beginPath();
  ctx.arc(sp.cx, sp.cy, 6, 0, Math.PI * 2);
  ctx.fillStyle = "#ffd166";
  ctx.fill();
  ctx.font = "bold 12px Arial";
  ctx.fillStyle = "#ffd166";
  ctx.textAlign = "left";
  ctx.textBaseline = "bottom";
  ctx.fillText(label, sp.cx + 14, sp.cy - 6);
}

function drawSelectedFirstPath(ctx, transform) {
  const scene = currentScene();
  const focus = focusDescriptor(currentFrame(), currentEvent());
  
  // Only draw paths when user explicitly selected an AGV
  // or for conflict/bridge scenes (≤2 AGVs where paths are informative)
  const agvIds = new Set();
  if (replay.followAgvId) {
    agvIds.add(String(replay.followAgvId));
  } else {
    const kind = String(scene?.kind || "");
    const isMultiAgvScene = kind.includes("conflict") || kind === "bridge";
    if (isMultiAgvScene) {
      for (const id of focus.agvIds) agvIds.add(String(id));
    }
  }
  
  if (agvIds.size === 0) return;

  const isAssignment = scene && String(scene.kind || "").includes("assignment");
  const pathColors = ["#0891b2", "#e85d04", "#7b2cbf", "#2d6a4f"];
  let colorIdx = 0;

  for (const agvId of agvIds) {
    const pathInfo = replay.firstPaths?.[agvId];
    if (!pathInfo || !Array.isArray(pathInfo.points) || pathInfo.points.length < 2) continue;

    const subtask = pathInfo.first_subtask;
    const subtaskNodeId = subtask ? parseNodeId(subtask.nodeId) : null;
    const subtaskNodes = Array.isArray(pathInfo.subtask_nodes) ? pathInfo.subtask_nodes : [];
    const lineColor = pathColors[colorIdx % pathColors.length];
    colorIdx += 1;

    // Assignment scene: truncate at first subtask; others: show full path
    let displayPoints = pathInfo.points;
    if (isAssignment && subtaskNodeId !== null) {
      for (let i = 0; i < pathInfo.points.length; i += 1) {
        if (parseNodeId(pathInfo.points[i].nodeId) === subtaskNodeId) {
          displayPoints = pathInfo.points.slice(0, i + 1);
          break;
        }
      }
    }
    if (displayPoints.length < 2) continue;

    ctx.save();
    ctx.shadowColor = `${lineColor}73`;
    ctx.shadowBlur = 18;
    drawPolyline(ctx, transform, displayPoints, {
      color: lineColor,
      width: 6.2,
      alpha: 0.98,
    });
    const first = displayPoints[0];
    const second = displayPoints[Math.min(displayPoints.length - 1, 1)];
    if (first) {
      const p = transform.toCanvas(first.x, first.y);
      ctx.beginPath();
      ctx.arc(p.cx, p.cy, 7, 0, Math.PI * 2);
      ctx.fillStyle = "#67e8f9";
      ctx.fill();
    }

    // Highlight subtask node(s) along the displayed path
    if (isAssignment) {
      if (subtask && Number.isFinite(Number(subtask.x)) && Number.isFinite(Number(subtask.y))) {
        drawSubtaskMarker(ctx, transform, subtask, `${agvId} 首个子任务点`);
      }
    } else {
      const subtaskSet = new Set(subtaskNodes.map(n => Number(n)));
      if (subtaskNodeId !== null) subtaskSet.add(subtaskNodeId);
      let labelIdx = 0;
      for (const pt of displayPoints) {
        const nid = parseNodeId(pt.nodeId);
        if (nid !== null && subtaskSet.has(nid)) {
          labelIdx += 1;
          drawSubtaskMarker(ctx, transform, pt, `${agvId} 子任务点 ${labelIdx}`);
        }
      }
      if (labelIdx === 0 && subtask && Number.isFinite(Number(subtask.x)) && Number.isFinite(Number(subtask.y))) {
        drawSubtaskMarker(ctx, transform, subtask, `${agvId} 子任务点`);
      }
    }
    if (first && second) {
      const p0 = transform.toCanvas(first.x, first.y);
      const p1 = transform.toCanvas(second.x, second.y);
      const angle = Math.atan2(p1.cy - p0.cy, p1.cx - p0.cx);
      ctx.translate(p1.cx, p1.cy);
      ctx.rotate(angle);
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(-11, -5);
      ctx.lineTo(-11, 5);
      ctx.closePath();
      ctx.fillStyle = lineColor;
      ctx.fill();
    }
    ctx.restore();
  }
}

function drawAgvs(ctx, transform, frame, focus, event) {
  const mode = replay.displayMode;
  const hasEmphasis = replay.highlightFocus && Boolean(replay.followAgvId);
  const pulse = 0.5 + 0.5 * Math.sin(replay.currentTimeS * 7.2);
  for (let i = 0; i < frame.agvs.length; i += 1) {
    const agv = frame.agvs[i];
    const color = routeColor(i);
    const isFocus = focus.agvIds.includes(String(agv.id));
    const dimFactor = hasEmphasis && !isFocus ? 0.28 : 1;

    const isFollowed = String(agv.id) === String(replay.followAgvId);
    if (mode !== "path") {
      drawPolyline(ctx, transform, agv.trail || [], {
        color,
        width: isFocus ? 4.4 : 2.8,
        alpha: (isFocus ? 0.96 : 0.72) * dimFactor,
        dash: [8, 5],
      });
    }

    const p = transform.toCanvas(agv.x, agv.y);
    const bodyColor = speedColor(Number(agv.speed || 0));

    ctx.save();
    ctx.globalAlpha = (isFocus ? 0.4 : 0.18) * dimFactor;
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, isFocus ? 22 : 16, 0, Math.PI * 2);
    ctx.fillStyle = isFocus ? "rgba(255, 214, 102, 0.7)" : bodyColor;
    ctx.fill();
    ctx.restore();

    if (isFocus && replay.highlightFocus) {
      ctx.save();
      ctx.beginPath();
      ctx.arc(p.cx, p.cy, 17 + pulse * 8, 0, Math.PI * 2);
      ctx.strokeStyle = `rgba(255, 224, 130, ${0.4 + pulse * 0.35})`;
      ctx.lineWidth = 2.6;
      ctx.stroke();
      ctx.restore();
    }

    ctx.save();
    ctx.globalAlpha = dimFactor;
    ctx.beginPath();
    ctx.arc(p.cx, p.cy, isFocus ? 10 : 8, 0, Math.PI * 2);
    ctx.fillStyle = bodyColor;
    ctx.fill();
    ctx.strokeStyle = isFocus ? "#ffe082" : color;
    ctx.lineWidth = isFocus ? 3.5 : 2.6;
    ctx.stroke();

    const ang = (Number(agv.angle || 0) * Math.PI) / 180;
    ctx.beginPath();
    ctx.moveTo(p.cx, p.cy);
    ctx.lineTo(p.cx + Math.cos(ang) * 16, p.cy + Math.sin(ang) * 16);
    ctx.strokeStyle = "rgba(6, 14, 20, 0.9)";
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.fillStyle = isFocus ? "#ffe082" : "#dbe7fa";
    ctx.font = isFocus ? "bold 13px Arial" : "bold 11px Arial";
    ctx.textAlign = "center";
    ctx.textBaseline = "bottom";
    ctx.fillText(String(agv.id), p.cx, p.cy - 14);

    if (isFocus) {
      const nextNode = parseNodeId(agv.nextNodeId);
      const phase = String(agv.phaseText || agv.phase || "");
      const extra = nextNode !== null ? ` -> N${nextNode}` : "";
      ctx.textBaseline = "top";
      ctx.fillText(`${phase}${extra}`, p.cx, p.cy + 14);
    }
    ctx.restore();
  }
}

function drawScene(ctx, canvas, frame, event, variant) {
  ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  const focus = focusDescriptor(frame, event);
  const transform = createTransform(canvas, variant === "main" ? replay.mainCamera : replay.focusCamera);
  drawMap(ctx, canvas, transform);
  drawBridges(ctx, transform);
  drawBackdrop(ctx, canvas, focus, event);
  drawReserved(ctx, transform, frame, focus);
  drawAgvs(ctx, transform, frame, focus, event);
  drawSelectedFirstPath(ctx, transform);
  drawFocusNodes(ctx, transform, focus);
}

function formatTime(seconds) {
  return `${Number(seconds || 0).toFixed(1)}s`;
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}

function bindScrollablePanes(root = document) {
  for (const pane of root.querySelectorAll(".scroll-pane")) {
    if (pane.dataset.scrollBound === "1") continue;
    pane.dataset.scrollBound = "1";
    let dragging = false;
    let pointerId = null;
    let startY = 0;
    let startScrollTop = 0;
    let moved = false;
    pane.addEventListener("pointerdown", (event) => {
      if (event.target.closest("[data-agv-id]") || event.target.closest("[data-scene-id]")) return;
      dragging = true;
      moved = false;
      pointerId = event.pointerId;
      startY = event.clientY;
      startScrollTop = pane.scrollTop;
      pane.classList.add("dragging");
      pane.setPointerCapture(event.pointerId);
    });
    pane.addEventListener("pointermove", (event) => {
      if (!dragging || pointerId !== event.pointerId) return;
      const deltaY = event.clientY - startY;
      if (Math.abs(deltaY) > 4) moved = true;
      pane.scrollTop = startScrollTop - deltaY;
    });
    const stopDrag = (event) => {
      if (!dragging) return;
      if (event?.pointerId !== undefined && pointerId !== event.pointerId) return;
      dragging = false;
      pointerId = null;
      pane.classList.remove("dragging");
    };
    pane.addEventListener("pointerup", stopDrag);
    pane.addEventListener("pointercancel", stopDrag);
    pane.addEventListener(
      "click",
      (event) => {
        if (moved) {
          event.preventDefault();
          event.stopPropagation();
        }
      },
      true
    );
  }
}

function updateStageBadges(event) {
  const speedBadge = document.getElementById("speed-badge");
  const cameraBadge = document.getElementById("camera-badge");
  const cue = currentCue();
  const rate = effectiveSpeed().toFixed(2);
  if (event) {
    speedBadge.textContent = `慢放 ${rate}x`;
    speedBadge.style.background = "rgba(255, 159, 67, 0.18)";
    speedBadge.style.borderColor = "rgba(255, 159, 67, 0.42)";
    speedBadge.style.color = "#ffd6a8";
  } else if (cue && replay.autoDirector) {
    speedBadge.textContent = `切镜 ${rate}x`;
    speedBadge.style.background = "rgba(77, 171, 247, 0.16)";
    speedBadge.style.borderColor = "rgba(77, 171, 247, 0.36)";
    speedBadge.style.color = "#d7ecff";
  } else if (replay.autoDirector) {
    speedBadge.textContent = `巡航 ${rate}x`;
    speedBadge.style.background = "rgba(46, 196, 182, 0.16)";
    speedBadge.style.borderColor = "rgba(46, 196, 182, 0.35)";
    speedBadge.style.color = "#aef7ef";
  } else {
    speedBadge.textContent = `手动 ${rate}x`;
    speedBadge.style.background = "rgba(123, 146, 175, 0.18)";
    speedBadge.style.borderColor = "rgba(155, 177, 208, 0.34)";
    speedBadge.style.color = "#d8e6fb";
  }

  if (replay.autoMainCamera) {
    cameraBadge.textContent = replay.followAgvId ? `主镜头跟随 ${replay.followAgvId}` : "主镜头自动";
  } else {
    cameraBadge.textContent = "主镜头手动";
  }
}

function renderDetailCards(scene) {
  const root = document.getElementById("detail-content");
  if (!root) return;
  if (!scene || !Array.isArray(scene.cards)) {
    root.innerHTML = "";
    return;
  }
  const blocks = [];
  for (const card of scene.cards) {
    const type = String(card.type || "");
    if (type === "task_input") {
      const items = Array.isArray(card.items) ? card.items : [];
      blocks.push(`
        <div class="detail-card">
          <div class="detail-card-title">${escapeHtml(card.title || "输入任务")}</div>
          <div class="task-list scroll-pane">
            ${items
              .map(
                (item) => `
                  <div class="task-row">
                    <div class="task-tag">${escapeHtml(item.task_id || "TASK")}</div>
                    <div class="task-body">
                      <div class="task-main">N${escapeHtml(item.pickup_node ?? "-")} → N${escapeHtml(item.delivery_node ?? "-")}</div>
                      <div class="task-meta">优先级 ${escapeHtml(item.priority ?? "-")}</div>
                    </div>
                  </div>
                `
              )
              .join("")}
          </div>
        </div>
      `);
      continue;
    }
    if (type === "assignment_result" || type === "path_result") {
      const items = Array.isArray(card.items) ? card.items : [];
      blocks.push(`
        <div class="detail-card">
          <div class="detail-card-title">${escapeHtml(card.title || "AGV 结果")}</div>
          <div class="assignment-list scroll-pane">
            ${items
              .map((item) => {
                const agvId = String(item.agv_id || "").trim();
                const active = agvId && agvId === replay.followAgvId;
                const mainText = type === "assignment_result" ? escapeHtml(item.task_chain_text || "-") : escapeHtml(item.route_text || "-");
                const subText =
                  type === "assignment_result"
                    ? `${item.has_first_path ? "点击可高亮" : "当前无首个Path"} · 等效时间 ${escapeHtml(item.eta_text || "-")}`
                    : `点击可高亮 · Path 点数 ${escapeHtml(item.point_count ?? "-")}`;
                const statusText =
                  type === "assignment_result"
                    ? (item.assigned === false ? "未接单" : "已接单")
                    : "Path";
                return `
                  <button class="assignment-button ${active ? "active" : ""}" data-agv-id="${escapeHtml(agvId)}">
                    <div class="assignment-top">
                      <div class="agv-tag">${escapeHtml(agvId)}</div>
                      <div class="status-tag ${item.assigned === false ? "" : "active"}">${escapeHtml(statusText)}</div>
                      <div class="assignment-eta">${type === "assignment_result" ? (item.assigned === false ? "-" : `ETA ${escapeHtml(item.eta_text || "-")}`) : `Path ${escapeHtml(item.point_count ?? "-")} 点`}</div>
                    </div>
                    <div class="assignment-chain">${mainText}</div>
                    <div class="detail-meta-text">${subText}</div>
                  </button>
                `;
              })
              .join("")}
          </div>
          <div class="focus-hint">点击 AGV 后，会在主画面中高亮该车第一次计算出的全局 Path。</div>
        </div>
      `);
      continue;
    }
    if (type === "bridge_summary") {
      const bridgeNodes = Array.isArray(card.bridge_nodes) ? card.bridge_nodes : [];
      blocks.push(`
        <div class="detail-card">
          <div class="detail-card-title">${escapeHtml(card.title || "桥区路径")}</div>
          <div class="detail-meta-text">AGV：${escapeHtml(card.agv_id || "-")}</div>
          <div class="detail-meta-text">路径：${escapeHtml(card.route_text || "-")}</div>
          <div class="detail-meta-text">桥区节点：${bridgeNodes.map((nid) => `N${escapeHtml(nid)}`).join(" / ") || "-"}</div>
        </div>
      `);
      continue;
    }
    if (type === "conflict_summary") {
      blocks.push(`
        <div class="detail-card">
          <div class="detail-card-title">${escapeHtml(card.title || "对向冲突")}</div>
          <div class="detail-meta-text">涉及车辆：${escapeHtml(card.owner_agv || "-")} / ${escapeHtml(card.contender_agv || "-")}</div>
          <div class="detail-meta-text">关键节点：${card.node_id !== null && card.node_id !== undefined ? `N${escapeHtml(card.node_id)}` : "-"}</div>
          ${card.bridge_related ? '<div class="bridge-badge">桥区相关</div>' : ""}
          <div class="detail-meta-text">${escapeHtml(card.summary || "")}</div>
        </div>
      `);
    }
  }
  root.innerHTML = blocks.join("");
  bindScrollablePanes(root);
}

function updateBanner(event) {
  const title = document.getElementById("banner-title");
  const desc = document.getElementById("banner-desc");
  const eyebrow = document.getElementById("banner-eyebrow");
  const focusTitle = document.getElementById("focus-title");
  const focusSubtitle = document.getElementById("focus-subtitle");
  const meta = document.getElementById("focus-meta");
  const scene = currentScene();
  const focus = focusDescriptor(currentFrame(), event);
  updateStageBadges(event);

  if (!scene) {
    if (replay.followAgvId) {
      eyebrow.textContent = "AGV Focus";
      title.textContent = `跟随 ${replay.followAgvId}`;
      desc.textContent = "当前没有激活高层场景，主画面保持巡航，并持续高亮你选中的 AGV 首次全局 Path。";
      focusTitle.textContent = `AGV ${replay.followAgvId}`;
      focusSubtitle.textContent = "点击右侧 AGV 卡片后，这里会保持聚焦显示。";
    } else {
      eyebrow.textContent = "Replay";
      title.textContent = "等待场景";
      desc.textContent = "正在根据回放时间定位高层场景。";
      focusTitle.textContent = "场景说明";
      focusSubtitle.textContent = "当前没有激活的场景。";
    }
    const idleItems = [];
    if (replay.followAgvId) idleItems.push(`<span class="pill">已选 AGV ${escapeHtml(replay.followAgvId)}</span>`);
    idleItems.push(`<span class="pill">${replay.autoMainCamera ? "自动镜头" : "手动镜头"}</span>`);
    idleItems.push(`<span class="pill">${replay.showBridges ? "桥区显示中" : "桥区隐藏"}</span>`);
    meta.innerHTML = idleItems.join("");
    const detailKey = `none:${replay.followAgvId}`;
    if (replay.ui.lastDetailKey !== detailKey) {
      renderDetailCards(null);
      replay.ui.lastDetailKey = detailKey;
    }
    return;
  }

  eyebrow.textContent = scene.scene_label || "Scene";
  title.textContent = scene.title || "场景";
  desc.textContent = scene.subtitle || "";
  focusTitle.textContent = scene.title || "场景说明";
  focusSubtitle.textContent = scene.subtitle || "";
  const items = [];
  items.push(`<span class="pill">${escapeHtml(scene.scene_label || "")}</span>`);
  for (const agvId of focus.agvIds) {
    items.push(`<span class="pill">AGV ${escapeHtml(agvId)}</span>`);
  }
  for (const nid of focus.nodeIds) {
    items.push(`<span class="pill">节点 N${escapeHtml(nid)}</span>`);
  }
  if (replay.followAgvId) {
    items.push(`<span class="pill">高亮 Path: ${escapeHtml(replay.followAgvId)}</span>`);
  }
  items.push(`<span class="pill">场景时段 ${formatTime(scene.start_s)} - ${formatTime(scene.end_s)}</span>`);
  meta.innerHTML = items.join("");
  const detailKey = `${scene.scene_id}:${replay.followAgvId}`;
  if (replay.ui.lastDetailKey !== detailKey) {
    renderDetailCards(scene);
    replay.ui.lastDetailKey = detailKey;
  }
}

function updateStats() {
  return;
}

function updateBadges() {
  const top = document.getElementById("top-badges");
  if (!replay.summary) return;
  const items = [
    `AGV ${replay.summary.agv_count || 0}`,
    `总时长 ${formatTime(replay.summary.total_sim_time_s)}`,
    `Scenes ${replay.summary.scene_count || replay.scenes.length || 0}`,
  ];
  top.innerHTML = items.map((item) => `<span class="pill">${item}</span>`).join("");
}

function renderEventList(force = false) {
  const root = document.getElementById("event-list");
  if (force || !replay.ui.sceneListBuilt) {
    root.innerHTML = replay.scenes
      .map(
        (scene) => `
          <div class="scene-item" data-scene-id="${scene.scene_id}">
            <div class="scene-top">
              <div class="scene-index">${scene.order}</div>
              <div class="eyebrow">${escapeHtml(scene.scene_label || `Scene ${scene.order}`)}</div>
            </div>
            <div class="scene-time">${formatTime(scene.start_s)} - ${formatTime(scene.end_s)}</div>
            <div class="scene-title">${escapeHtml(scene.title || "")}</div>
            <div class="scene-subtitle">${escapeHtml(scene.subtitle || "")}</div>
          </div>
        `
      )
      .join("");
    replay.ui.sceneListBuilt = true;
  }

  const activeId = currentScene()?.scene_id || "";
  if (!force && activeId === replay.ui.activeSceneId) return;
  replay.ui.activeSceneId = activeId;
  for (const el of root.querySelectorAll(".scene-item")) {
    const isActive = el.getAttribute("data-scene-id") === activeId;
    el.classList.toggle("active", isActive);
    if (isActive && replay.ui.lastScrolledSceneId !== activeId) {
      el.scrollIntoView({ block: "nearest" });
      replay.ui.lastScrolledSceneId = activeId;
    }
  }
}

function refreshButtons() {
  document.getElementById("btn-play").textContent = replay.playing ? "暂停" : "播放";
  const stepButton = document.getElementById("btn-step");
  if (stepButton) {
    stepButton.textContent = "禁止跳播";
  }
}

function bindDragScroll(pane) {
  if (!pane || pane.dataset.scrollBound === "1") return;
  pane.dataset.scrollBound = "1";
  let dragging = false;
  let pointerId = null;
  let startY = 0;
  let startScrollTop = 0;
  let moved = false;
  pane.addEventListener("pointerdown", (event) => {
    if (event.target.closest("[data-scene-id]") || event.target.closest("[data-agv-id]")) {
      return;
    }
    dragging = true;
    moved = false;
    pointerId = event.pointerId;
    startY = event.clientY;
    startScrollTop = pane.scrollTop;
    pane.classList.add("dragging");
    pane.setPointerCapture(event.pointerId);
  });
  pane.addEventListener("pointermove", (event) => {
    if (!dragging || pointerId !== event.pointerId) return;
    const deltaY = event.clientY - startY;
    if (Math.abs(deltaY) > 4) moved = true;
    pane.scrollTop = startScrollTop - deltaY;
  });
  const stopDrag = (event) => {
    if (!dragging) return;
    if (event?.pointerId !== undefined && pointerId !== event.pointerId) return;
    dragging = false;
    pointerId = null;
    pane.classList.remove("dragging");
  };
  pane.addEventListener("pointerup", stopDrag);
  pane.addEventListener("pointercancel", stopDrag);
  pane.addEventListener(
    "click",
    (event) => {
      if (moved) {
        event.preventDefault();
        event.stopPropagation();
      }
    },
    true
  );
}

function populateFollowSelect() {
  const select = document.getElementById("follow-select");
  if (!select) return;
  const options = ['<option value="">自动镜头</option>'];
  for (const agvId of allAgvIds()) {
    options.push(`<option value="${agvId}">跟随 AGV ${agvId}</option>`);
  }
  select.innerHTML = options.join("");
  select.value = replay.followAgvId;
}

function setAutoMainCamera(enabled) {
  replay.autoMainCamera = enabled;
  const toggle = document.getElementById("toggle-main-camera");
  if (toggle) {
    toggle.checked = enabled;
  }
}

function ensureManualMainCamera() {
  if (replay.autoMainCamera) {
    setAutoMainCamera(false);
  }
}

function resetMainView() {
  setAutoMainCamera(true);
  const center = boundsCenter(mapBounds());
  replay.mainCamera.centerX = center.x;
  replay.mainCamera.centerY = center.y;
  replay.mainCamera.zoom = 1;
  replay.mainCamera.targetCenterX = center.x;
  replay.mainCamera.targetCenterY = center.y;
  replay.mainCamera.targetZoom = 1;
  render({ snapCamera: true });
}

function zoomMainView(multiplier, anchorPoint = null) {
  if (!mainCanvas.clientWidth || !mainCanvas.clientHeight) return;
  ensureManualMainCamera();
  const camera = replay.mainCamera;
  const point = anchorPoint || { x: mainCanvas.clientWidth / 2, y: mainCanvas.clientHeight / 2 };
  const before = createTransform(mainCanvas, camera);
  const world = before.toWorld(point.x, point.y);
  camera.zoom = clamp(camera.zoom * multiplier, ZOOM_MIN, ZOOM_MAX);
  const after = createTransform(mainCanvas, camera);
  const worldAfter = after.toWorld(point.x, point.y);
  camera.centerX += world.x - worldAfter.x;
  camera.centerY += world.y - worldAfter.y;
  camera.targetCenterX = camera.centerX;
  camera.targetCenterY = camera.centerY;
  camera.targetZoom = camera.zoom;
  render({ snapCamera: true });
}

function seekToTime(timeS, options = {}) {
  replay.currentTimeS = clamp(Number(timeS || 0), 0, replay.totalTimeS);
  if (options.pause !== false) {
    replay.playing = false;
  }
  replay.lastTickTs = 0;
  refreshButtons();
  render({
    forceEventList: Boolean(options.forceEventList),
    snapCamera: Boolean(options.snapCamera),
    autoSnap: Boolean(options.autoSnap),
  });
}

function jumpBy(deltaS) {
  seekToTime(replay.currentTimeS + Number(deltaS || 0), { pause: true, snapCamera: true });
}

function jumpEvent(direction) {
  if (!replay.scenes.length) return;
  const currentTime = replay.currentTimeS;
  if (direction > 0) {
    const next = replay.scenes.find((item) => Number(item.start_s || 0) > currentTime + 0.001);
    if (next) {
      seekToTime(Number(next.start_s || 0), { pause: true, snapCamera: true, autoSnap: true, forceEventList: true });
      return;
    }
    seekToTime(replay.totalTimeS, { pause: true, snapCamera: true, autoSnap: true, forceEventList: true });
    return;
  }

  let prev = replay.scenes[0];
  for (const item of replay.scenes) {
    if (Number(item.start_s || 0) >= currentTime - 0.001) break;
    prev = item;
  }
  seekToTime(Number(prev.start_s || 0), { pause: true, snapCamera: true, autoSnap: true, forceEventList: true });
}

function render(options = {}) {
  const frame = currentFrame();
  const event = currentEvent();
  const snap = Boolean(options.snapCamera || !replay.playing);
  const savedAutoMain = replay.autoMainCamera;
  if (options.autoSnap) {
    replay.autoMainCamera = true;
  }
  syncCameras(frame, event, { snapMain: snap, snapFocus: snap });
  if (options.autoSnap) {
    replay.autoMainCamera = savedAutoMain;
  }
  drawScene(mainCtx, mainCanvas, frame, event, "main");
  drawScene(focusCtx, focusCanvas, frame, event, "focus");
  updateBanner(event);
  updateStats();
  renderEventList(Boolean(options.forceEventList));
  document.getElementById("timeline").value = replay.totalTimeS > 0 ? String(replay.currentTimeS / replay.totalTimeS) : "0";
  document.getElementById("time-label").textContent = `${formatTime(replay.currentTimeS)} / ${formatTime(replay.totalTimeS)}`;
}

function nextSceneAfter(timeS) {
  if (!Array.isArray(replay.scenes)) return null;
  for (const scene of replay.scenes) {
    if (Number(scene.start_s || 0) > timeS + 0.5) return scene;
  }
  return null;
}

function tick(ts) {
  if (!replay.lastTickTs) replay.lastTickTs = ts;
  const dt = Math.min(0.1, Math.max(0, (ts - replay.lastTickTs) / 1000));
  replay.lastTickTs = ts;
  if (replay.playing && replay.frames.length) {
    const speed = effectiveSpeed();
    const prevSceneId = currentScene()?.scene_id || "";
    replay.currentTimeS += dt * speed;

    // Auto-director: skip gaps between scenes (jump to next scene start)
    let jumped = false;
    if (replay.autoDirector) {
      const scene = currentSceneAt(replay.currentTimeS);
      if (!scene) {
        const next = nextSceneAfter(replay.currentTimeS);
        if (next) {
          const nextStart = Number(next.start_s || 0);
          const gap = nextStart - replay.currentTimeS;
          if (gap > 2.0) {
            replay.currentTimeS = nextStart - 1.0;
            jumped = true;
          }
        }
      }
    }

    if (replay.currentTimeS >= replay.totalTimeS) {
      replay.currentTimeS = replay.totalTimeS;
      replay.playing = false;
      refreshButtons();
    }

    // Snap camera when entering a new scene
    const newSceneId = currentScene()?.scene_id || "";
    const sceneChanged = newSceneId && newSceneId !== prevSceneId;
    if (sceneChanged || jumped) {
      render({ autoSnap: true, forceEventList: true });
    } else {
      render();
    }
  }
  requestAnimationFrame(tick);
}

function bindCanvasInteractions() {
  mainCanvas.addEventListener(
    "wheel",
    (event) => {
      event.preventDefault();
      const rect = mainCanvas.getBoundingClientRect();
      const anchor = {
        x: event.clientX - rect.left,
        y: event.clientY - rect.top,
      };
      zoomMainView(event.deltaY < 0 ? 1.12 : 0.89, anchor);
    },
    { passive: false }
  );

  mainCanvas.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) return;
    ensureManualMainCamera();
    mainCanvas.classList.add("dragging");
    mainCanvas.setPointerCapture(event.pointerId);
    replay.mainCamera.dragging = true;
    replay.mainCamera.pointerId = event.pointerId;
    replay.mainCamera.dragStartX = event.clientX;
    replay.mainCamera.dragStartY = event.clientY;
    replay.mainCamera.dragCenterX = replay.mainCamera.centerX;
    replay.mainCamera.dragCenterY = replay.mainCamera.centerY;
  });

  mainCanvas.addEventListener("pointermove", (event) => {
    if (!replay.mainCamera.dragging || replay.mainCamera.pointerId !== event.pointerId) return;
    const transform = createTransform(mainCanvas, replay.mainCamera);
    const dx = event.clientX - replay.mainCamera.dragStartX;
    const dy = event.clientY - replay.mainCamera.dragStartY;
    replay.mainCamera.centerX = replay.mainCamera.dragCenterX - dx / Math.max(0.0001, transform.scale);
    replay.mainCamera.centerY = replay.mainCamera.dragCenterY - dy / Math.max(0.0001, transform.scale);
    replay.mainCamera.targetCenterX = replay.mainCamera.centerX;
    replay.mainCamera.targetCenterY = replay.mainCamera.centerY;
    replay.mainCamera.targetZoom = replay.mainCamera.zoom;
    render({ snapCamera: true });
  });

  const stopDrag = (event) => {
    if (replay.mainCamera.pointerId !== null && event?.pointerId !== undefined && replay.mainCamera.pointerId !== event.pointerId) {
      return;
    }
    replay.mainCamera.dragging = false;
    replay.mainCamera.pointerId = null;
    mainCanvas.classList.remove("dragging");
  };

  mainCanvas.addEventListener("pointerup", stopDrag);
  mainCanvas.addEventListener("pointercancel", stopDrag);
}

function bindKeyboard() {
  window.addEventListener("keydown", (event) => {
    const tag = String(document.activeElement?.tagName || "").toUpperCase();
    if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") return;
    if (event.code === "Space") {
      event.preventDefault();
      replay.playing = !replay.playing;
      replay.lastTickTs = 0;
      refreshButtons();
      render();
    }
  });
}

function bindUi() {
  document.getElementById("btn-play").addEventListener("click", () => {
    replay.playing = !replay.playing;
    replay.lastTickTs = 0;
    refreshButtons();
    render();
  });
  document.getElementById("btn-reset").addEventListener("click", () => {
    seekToTime(0, { pause: true, snapCamera: true, forceEventList: true });
  });
  document.getElementById("btn-view-reset").addEventListener("click", () => {
    resetMainView();
  });
  document.getElementById("btn-prev-event").addEventListener("click", () => jumpEvent(-1));
  document.getElementById("btn-next-event").addEventListener("click", () => jumpEvent(1));
  document.getElementById("speed-select").addEventListener("change", (event) => {
    replay.globalSpeed = Number(event.target.value || 1);
    render();
  });
  document.getElementById("timeline").addEventListener("input", (event) => {
    const ratio = Number(event.target.value || 0);
    replay.currentTimeS = clamp(ratio * replay.totalTimeS, 0, replay.totalTimeS);
    replay.lastTickTs = 0;
    render({ snapCamera: true, forceEventList: true });
  });
  document.getElementById("toggle-bridges").addEventListener("change", (event) => {
    replay.showBridges = event.target.checked;
    render();
  });
  document.getElementById("event-list").addEventListener("click", (event) => {
    if (!(event.target instanceof Element)) return;
    const item = event.target.closest("[data-scene-id]");
    if (!item) return;
    const sceneId = String(item.getAttribute("data-scene-id") || "");
    const scene = replay.scenes.find((entry) => String(entry.scene_id) === sceneId);
    if (!scene) return;
    seekToTime(Number(scene.start_s || 0), { pause: true, snapCamera: true, autoSnap: true, forceEventList: true });
  });
  document.getElementById("detail-content").addEventListener("click", (event) => {
    if (!(event.target instanceof Element)) return;
    const button = event.target.closest("[data-agv-id]");
    if (!button) return;
    const agvId = String(button.getAttribute("data-agv-id") || "");
    if (!agvId) return;
    replay.followAgvId = replay.followAgvId === agvId ? "" : agvId;
    if (replay.followAgvId) {
      const frame = currentFrame();
      const agv = frame.agvs.find((a) => String(a.id) === replay.followAgvId);
      if (agv && Number.isFinite(Number(agv.x)) && Number.isFinite(Number(agv.y))) {
        replay.mainCamera.centerX = Number(agv.x);
        replay.mainCamera.centerY = Number(agv.y);
        replay.mainCamera.targetCenterX = Number(agv.x);
        replay.mainCamera.targetCenterY = Number(agv.y);
        replay.mainCamera.zoom = clamp(2.2, ZOOM_MIN, ZOOM_MAX);
        replay.mainCamera.targetZoom = replay.mainCamera.zoom;
      }
    }
    render({ snapCamera: true, forceEventList: true });
  });

  bindCanvasInteractions();
  bindKeyboard();
  bindDragScroll(document.getElementById("detail-content"));
}

function initializeCameras() {
  const center = boundsCenter(mapBounds());
  replay.mainCamera = createCameraState();
  replay.focusCamera = createCameraState();
  replay.mainCamera.centerX = center.x;
  replay.mainCamera.centerY = center.y;
  replay.mainCamera.zoom = 1;
  replay.mainCamera.targetCenterX = center.x;
  replay.mainCamera.targetCenterY = center.y;
  replay.mainCamera.targetZoom = 1;
  replay.focusCamera.centerX = center.x;
  replay.focusCamera.centerY = center.y;
  replay.focusCamera.zoom = 1.05;
  replay.focusCamera.targetCenterX = center.x;
  replay.focusCamera.targetCenterY = center.y;
  replay.focusCamera.targetZoom = 1.05;
}

async function loadBundle() {
  const response = await fetch("/bundle.json", { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`bundle fetch failed: ${response.status}`);
  }
  const data = await response.json();
  replay.bundle = data;
  replay.map = data.map;
  replay.frames = data.frames || [];
  replay.frameTimes = replay.frames.map((frame) => Number(frame.sim_time_s || 0));
  replay.timeline = data.timeline || [];
  replay.scenes = data.scenes || [];
  replay.firstPaths = data.first_paths || {};
  replay.summary = data.summary || {};
  replay.totalTimeS = replay.frames.length ? Number(replay.frames[replay.frames.length - 1].sim_time_s || 0) : 0;
  replay.directorBaseRate = Number(data.metadata?.director_base_rate || 2.8);
  replay.ui.sceneListBuilt = false;
  replay.ui.activeSceneId = "";
  replay.ui.lastScrolledSceneId = "";
  replay.ui.lastDetailKey = "";
  replay.followAgvId = "";
  document.title = `AGV 导演版回放 - ${data.metadata?.session_name || "session"}`;
  initializeCameras();
  populateFollowSelect();
  const bridgeToggle = document.getElementById("toggle-bridges");
  if (bridgeToggle) {
    const hasBridges = Array.isArray(replay.map?.bridgeNodeIds) && replay.map.bridgeNodeIds.length > 0;
    bridgeToggle.disabled = !hasBridges;
    if (!hasBridges) {
      replay.showBridges = false;
      bridgeToggle.checked = false;
    } else {
      replay.showBridges = true;
      bridgeToggle.checked = true;
    }
  }
  bindDragScroll(document.getElementById("event-list"));
}

async function main() {
  resizeCanvas(mainCanvas, mainCtx);
  resizeCanvas(focusCanvas, focusCtx);
  window.addEventListener("resize", () => {
    resizeCanvas(mainCanvas, mainCtx);
    resizeCanvas(focusCanvas, focusCtx);
    render({ snapCamera: true });
  });
  bindUi();
  refreshButtons();
  await loadBundle();
  updateBadges();
  render({ forceEventList: true, snapCamera: true });
  requestAnimationFrame(tick);
}

main().catch((err) => {
  console.error(err);
  document.getElementById("banner-title").textContent = "回放加载失败";
  document.getElementById("banner-desc").textContent = err?.message || String(err);
});
