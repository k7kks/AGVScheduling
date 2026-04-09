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
  summary: null,
  totalTimeS: 0,
  directorBaseRate: 2.8,
  currentTimeS: 0,
  playing: true,
  globalSpeed: 1.0,
  autoDirector: true,
  autoFocus: true,
  autoMainCamera: true,
  highlightFocus: true,
  showReserved: true,
  displayMode: "both",
  followAgvId: "",
  lastTickTs: 0,
  mainCamera: createCameraState(),
  focusCamera: createCameraState(),
  ui: {
    eventListBuilt: false,
    activeEventId: "",
    lastScrolledEventId: "",
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
  if (!replay.autoDirector) {
    return replay.globalSpeed;
  }
  const baseRate = replay.directorBaseRate;
  const cue = currentCueAt(timeS);
  if (!cue) {
    return baseRate * replay.globalSpeed;
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
  const agvIds = new Set();
  const nodeIds = new Set();
  const rawFocus = event?.focus || {};
  for (const agvId of rawFocus.agv_ids || []) {
    const value = String(agvId || "").trim();
    if (value) agvIds.add(value);
  }
  for (const nodeId of rawFocus.node_ids || []) {
    const parsed = parseNodeId(nodeId);
    if (parsed !== null) nodeIds.add(parsed);
  }

  if (replay.followAgvId) {
    agvIds.add(replay.followAgvId);
    const followAgv = frame.agvs.find((item) => String(item.id) === replay.followAgvId);
    const followNodes = [
      parseNodeId(followAgv?.nodeId),
      parseNodeId(followAgv?.nextNodeId),
      parseNodeId(followAgv?.nextSubtask?.nodeId),
    ];
    for (const nodeId of followNodes) {
      if (nodeId !== null) nodeIds.add(nodeId);
    }
  }

  if (!agvIds.size && !nodeIds.size) {
    const assigned = frame.agvs
      .filter((agv) => agv.assigned || Number(agv.speed || 0) > 1e-6)
      .slice(0, 2);
    for (const agv of assigned) agvIds.add(String(agv.id));
  }

  return {
    agvIds: Array.from(agvIds),
    nodeIds: Array.from(nodeIds),
    zoom: Math.max(1.0, Number(rawFocus.zoom || (replay.followAgvId ? 1.8 : 1.3)) || 1.3),
    mode: String(rawFocus.mode || (replay.followAgvId ? "follow" : "auto")),
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
    if (!event && !replay.followAgvId) {
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

function drawBackdrop(ctx, canvas, focus, event) {
  const active = replay.highlightFocus && (Boolean(event) || Boolean(replay.followAgvId));
  if (!active) return;
  const alpha = event ? 0.16 : 0.1;
  ctx.save();
  ctx.fillStyle = `rgba(3, 8, 16, ${alpha})`;
  ctx.fillRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  ctx.restore();
}

function drawPolyline(ctx, transform, points, style) {
  if (!points || points.length < 2) return;
  ctx.save();
  ctx.strokeStyle = style.color;
  ctx.lineWidth = style.width || 2;
  ctx.globalAlpha = style.alpha ?? 0.8;
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

function drawAgvs(ctx, transform, frame, focus, event) {
  const mode = replay.displayMode;
  const hasEmphasis = replay.highlightFocus && (Boolean(event) || Boolean(replay.followAgvId));
  const pulse = 0.5 + 0.5 * Math.sin(replay.currentTimeS * 7.2);
  for (let i = 0; i < frame.agvs.length; i += 1) {
    const agv = frame.agvs[i];
    const color = routeColor(i);
    const isFocus = focus.agvIds.includes(String(agv.id));
    const dimFactor = hasEmphasis && !isFocus ? 0.28 : 1;

    if (mode !== "trail") {
      drawPolyline(ctx, transform, agv.path || [], {
        color,
        width: isFocus ? 3.4 : 2.1,
        alpha: (isFocus ? 0.6 : 0.3) * dimFactor,
      });
    }
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
  drawBackdrop(ctx, canvas, focus, event);
  drawReserved(ctx, transform, frame, focus);
  drawAgvs(ctx, transform, frame, focus, event);
  drawFocusNodes(ctx, transform, focus);
}

function formatTime(seconds) {
  return `${Number(seconds || 0).toFixed(1)}s`;
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
    cameraBadge.textContent = replay.followAgvId ? `主镜头 跟随 ${replay.followAgvId}` : "主镜头 自动";
  } else {
    cameraBadge.textContent = "主镜头 手动";
  }
}

function updateBanner(event) {
  const title = document.getElementById("banner-title");
  const desc = document.getElementById("banner-desc");
  const focusTitle = document.getElementById("focus-title");
  const focusSubtitle = document.getElementById("focus-subtitle");
  const meta = document.getElementById("focus-meta");
  const focus = focusDescriptor(currentFrame(), event);
  updateStageBadges(event);

  if (!event) {
    if (replay.followAgvId) {
      title.textContent = `跟随 AGV ${replay.followAgvId}`;
      desc.textContent = "当前没有激活章节，主视角保持自动巡航，重点镜头持续跟随你选中的车辆。";
      focusTitle.textContent = `AGV ${replay.followAgvId} 局部镜头`;
      focusSubtitle.textContent = "你可以滚轮缩放主画面、拖拽平移，或者回到自动镜头。";
    } else {
      title.textContent = "自动巡航段";
      desc.textContent = "导演脚本正在快放常规执行过程，重点事件会提前减速、切镜并做局部高亮。";
      focusTitle.textContent = "重点镜头";
      focusSubtitle.textContent = "当前没有激活的重点事件，右侧镜头保持跟随主要执行区域。";
    }
    const idleItems = [];
    if (replay.followAgvId) idleItems.push(`<span class="pill">跟随 AGV ${replay.followAgvId}</span>`);
    idleItems.push(`<span class="pill">主镜头 ${replay.autoMainCamera ? "自动" : "手动"}</span>`);
    idleItems.push(`<span class="pill">局部镜头 ${replay.autoFocus ? "自动" : "全图"}</span>`);
    meta.innerHTML = idleItems.join("");
    return;
  }

  title.textContent = event.title || "重点事件";
  desc.textContent = event.subtitle || "";
  focusTitle.textContent = event.title || "重点镜头";
  focusSubtitle.textContent = event.subtitle || "";
  const items = [];
  for (const agvId of focus.agvIds) {
    items.push(`<span class="pill">AGV ${agvId}</span>`);
  }
  for (const nid of focus.nodeIds) {
    items.push(`<span class="pill">节点 N${nid}</span>`);
  }
  items.push(`<span class="pill">镜头 ${focus.zoom.toFixed(2)}x</span>`);
  items.push(`<span class="pill">慢放 ${Number(event.playback_rate || 1).toFixed(2)}x</span>`);
  meta.innerHTML = items.join("");
}

function updateStats() {
  const stats = document.getElementById("stats");
  if (!replay.summary) return;
  const frame = currentFrame();
  const event = currentEvent();
  const html = [
    ["仿真时长", formatTime(replay.summary.total_sim_time_s)],
    ["帧数", String(replay.summary.total_frames)],
    ["导演事件", String(replay.summary.director_event_count)],
    ["当前 AGV", String((frame.agvs || []).length)],
    ["章节状态", event ? "聚焦中" : "巡航中"],
    ["镜头模式", replay.autoMainCamera ? "自动" : "手动"],
  ]
    .map(
      ([label, value]) => `
        <div class="stat">
          <span class="label">${label}</span>
          <span class="value">${value}</span>
        </div>
      `
    )
    .join("");
  stats.innerHTML = html;
}

function updateBadges() {
  const top = document.getElementById("top-badges");
  if (!replay.summary) return;
  const items = [
    `AGV ${replay.summary.agv_count || 0}`,
    `总时长 ${formatTime(replay.summary.total_sim_time_s)}`,
    `章节 ${replay.summary.director_event_count || 0}`,
  ];
  top.innerHTML = items.map((item) => `<span class="pill">${item}</span>`).join("");
}

function renderEventList(force = false) {
  const root = document.getElementById("event-list");
  if (force || !replay.ui.eventListBuilt) {
    root.innerHTML = replay.timeline
      .map(
        (event) => `
          <div class="event-item" data-event-id="${event.event_id}">
            <div class="event-time">${formatTime(event.start_s)} - ${formatTime(event.end_s)}</div>
            <div class="event-title">${event.title}</div>
            <div class="event-subtitle">${event.subtitle || ""}</div>
          </div>
        `
      )
      .join("");
    replay.ui.eventListBuilt = true;
  }

  const activeId = currentEvent()?.event_id || "";
  if (!force && activeId === replay.ui.activeEventId) return;
  replay.ui.activeEventId = activeId;
  for (const el of root.querySelectorAll(".event-item")) {
    const isActive = el.getAttribute("data-event-id") === activeId;
    el.classList.toggle("active", isActive);
    if (isActive && replay.ui.lastScrolledEventId !== activeId) {
      el.scrollIntoView({ block: "nearest" });
      replay.ui.lastScrolledEventId = activeId;
    }
  }
}

function refreshButtons() {
  document.getElementById("btn-play").textContent = replay.playing ? "暂停" : "继续播放";
  const stepButton = document.getElementById("btn-step");
  if (stepButton) {
    stepButton.textContent = "禁止跳播";
  }
}

function populateFollowSelect() {
  const select = document.getElementById("follow-select");
  const options = ['<option value="">自动镜头</option>'];
  for (const agvId of allAgvIds()) {
    options.push(`<option value="${agvId}">跟随 AGV ${agvId}</option>`);
  }
  select.innerHTML = options.join("");
  select.value = replay.followAgvId;
}

function setAutoMainCamera(enabled) {
  replay.autoMainCamera = enabled;
  document.getElementById("toggle-main-camera").checked = enabled;
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
  });
}

function jumpBy(deltaS) {
  seekToTime(replay.currentTimeS + Number(deltaS || 0), { pause: true, snapCamera: true });
}

function jumpEvent(direction) {
  if (!replay.timeline.length) return;
  const currentTime = replay.currentTimeS;
  if (direction > 0) {
    const next = replay.timeline.find((item) => Number(item.start_s || 0) > currentTime + 0.001);
    if (next) {
      seekToTime(Number(next.start_s || 0), { pause: true, snapCamera: true, forceEventList: true });
      return;
    }
    seekToTime(replay.totalTimeS, { pause: true, snapCamera: true, forceEventList: true });
    return;
  }

  let prev = replay.timeline[0];
  for (const item of replay.timeline) {
    if (Number(item.start_s || 0) >= currentTime - 0.001) break;
    prev = item;
  }
  seekToTime(Number(prev.start_s || 0), { pause: true, snapCamera: true, forceEventList: true });
}

function render(options = {}) {
  const frame = currentFrame();
  const event = currentEvent();
  const snap = Boolean(options.snapCamera || !replay.playing);
  syncCameras(frame, event, { snapMain: snap, snapFocus: snap });
  drawScene(mainCtx, mainCanvas, frame, event, "main");
  drawScene(focusCtx, focusCanvas, frame, event, "focus");
  updateBanner(event);
  updateStats();
  renderEventList(Boolean(options.forceEventList));
  document.getElementById("timeline").value = replay.totalTimeS > 0 ? String(replay.currentTimeS / replay.totalTimeS) : "0";
  document.getElementById("time-label").textContent = `${formatTime(replay.currentTimeS)} / ${formatTime(replay.totalTimeS)}`;
}

function tick(ts) {
  if (!replay.lastTickTs) replay.lastTickTs = ts;
  const dt = Math.max(0, (ts - replay.lastTickTs) / 1000);
  replay.lastTickTs = ts;
  if (replay.playing && replay.frames.length) {
    replay.currentTimeS += dt * effectiveSpeed();
    if (replay.currentTimeS >= replay.totalTimeS) {
      replay.currentTimeS = replay.totalTimeS;
      replay.playing = false;
      refreshButtons();
    }
    render();
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
  document.getElementById("btn-zoom-in").addEventListener("click", () => {
    zoomMainView(1.18);
  });
  document.getElementById("btn-zoom-out").addEventListener("click", () => {
    zoomMainView(0.84);
  });
  document.getElementById("btn-view-reset").addEventListener("click", () => {
    resetMainView();
  });
  document.getElementById("speed-select").addEventListener("change", (event) => {
    replay.globalSpeed = Number(event.target.value || 1);
    render();
  });
  document.getElementById("display-select").addEventListener("change", (event) => {
    replay.displayMode = String(event.target.value || "both");
    render();
  });
  document.getElementById("follow-select").addEventListener("change", (event) => {
    replay.followAgvId = String(event.target.value || "");
    render({ snapCamera: true });
  });
  document.getElementById("toggle-director").addEventListener("change", (event) => {
    replay.autoDirector = event.target.checked;
    render();
  });
  document.getElementById("toggle-main-camera").addEventListener("change", (event) => {
    setAutoMainCamera(event.target.checked);
    render({ snapCamera: true });
  });
  document.getElementById("toggle-focus").addEventListener("change", (event) => {
    replay.autoFocus = event.target.checked;
    render({ snapCamera: true });
  });
  document.getElementById("toggle-reserved").addEventListener("change", (event) => {
    replay.showReserved = event.target.checked;
    render();
  });
  document.getElementById("toggle-highlight").addEventListener("change", (event) => {
    replay.highlightFocus = event.target.checked;
    render();
  });

  bindCanvasInteractions();
  bindKeyboard();
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
  replay.summary = data.summary || {};
  replay.totalTimeS = replay.frames.length ? Number(replay.frames[replay.frames.length - 1].sim_time_s || 0) : 0;
  replay.directorBaseRate = Number(data.metadata?.director_base_rate || 2.8);
  replay.ui.eventListBuilt = false;
  replay.ui.activeEventId = "";
  replay.ui.lastScrolledEventId = "";
  document.title = `AGV 导演版回放 - ${data.metadata?.session_name || "session"}`;
  initializeCameras();
  populateFollowSelect();
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
