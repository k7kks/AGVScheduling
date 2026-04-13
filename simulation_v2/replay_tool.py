#!/usr/bin/env python3
import argparse
import json
import os
import sys
import urllib.parse
from bisect import bisect_right
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def load_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8") as fp:
        for line in fp:
            text = line.strip()
            if not text:
                continue
            try:
                row = json.loads(text)
            except json.JSONDecodeError:
                continue
            if isinstance(row, dict):
                rows.append(row)
    return rows


def parse_iso_datetime(text: Any) -> Optional[datetime]:
    value = str(text or "").strip()
    if not value:
        return None
    try:
        if value.endswith("Z"):
            value = value[:-1] + "+00:00"
        return datetime.fromisoformat(value)
    except Exception:
        return None


def duration_text_from_iso(start_text: Any, end_text: Any) -> str:
    start_dt = parse_iso_datetime(start_text)
    end_dt = parse_iso_datetime(end_text)
    if start_dt is None or end_dt is None:
        return "-"
    duration_s = max(0, int(round((end_dt - start_dt).total_seconds())))
    if duration_s <= 0:
        return "-"
    minutes, seconds = divmod(duration_s, 60)
    hours, minutes = divmod(minutes, 60)
    if hours > 0:
        return f"{hours}h {minutes}m"
    if minutes > 0:
        return f"{minutes}m {seconds}s"
    return f"{seconds}s"


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


def node_id(value: Any) -> Optional[int]:
    try:
        if value is None or value == "":
            return None
        return int(value)
    except Exception:
        return None


def frame_index_at(frames: List[Dict[str, Any]], sim_time_s: float) -> int:
    if not frames:
        return 0
    times = [safe_float(frame.get("sim_time_s"), 0.0) for frame in frames]
    idx = bisect_right(times, sim_time_s) - 1
    return max(0, min(len(frames) - 1, idx))


def frame_at(frames: List[Dict[str, Any]], sim_time_s: float) -> Dict[str, Any]:
    if not frames:
        return {}
    return frames[frame_index_at(frames, sim_time_s)]


def unique_ints(values: List[Optional[int]], limit: int = 8) -> List[int]:
    out: List[int] = []
    for value in values:
        if value is None:
            continue
        ivalue = int(value)
        if ivalue in out:
            continue
        out.append(ivalue)
        if len(out) >= limit:
            break
    return out


def short_route_text(node_ids: List[Any], limit: int = 4) -> str:
    ints = []
    for value in node_ids[: max(1, limit)]:
        parsed = node_id(value)
        if parsed is not None:
            ints.append(parsed)
    if not ints:
        return "-"
    text = " -> ".join(f"N{x}" for x in ints)
    if len(node_ids) > len(ints):
        text += " -> ..."
    return text


def route_intersects_bridge(node_ids: List[Any], bridge_node_set: set[int]) -> bool:
    if not bridge_node_set:
        return False
    for value in node_ids or []:
        parsed = node_id(value)
        if parsed is not None and parsed in bridge_node_set:
            return True
    return False


def extract_first_paths(frames: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    first_paths: Dict[str, Dict[str, Any]] = {}
    for frame in frames:
        sim_time_s = safe_float(frame.get("sim_time_s"), 0.0)
        for agv in frame.get("agvs", []) or []:
            if not isinstance(agv, dict):
                continue
            agv_id = str(agv.get("id", "")).strip()
            if not agv_id or agv_id in first_paths:
                continue
            raw_points = agv.get("path")
            if not isinstance(raw_points, list) or len(raw_points) < 2:
                continue
            points: List[Dict[str, Any]] = []
            node_ids: List[int] = []
            for point in raw_points:
                if not isinstance(point, dict):
                    continue
                nid = node_id(point.get("nodeId"))
                if nid is not None:
                    node_ids.append(nid)
                points.append(
                    {
                        "x": safe_float(point.get("x"), 0.0),
                        "y": safe_float(point.get("y"), 0.0),
                        "nodeId": nid,
                    }
                )
            if len(points) < 2:
                continue
            first_paths[agv_id] = {
                "agv_id": agv_id,
                "start_s": sim_time_s,
                "point_count": len(points),
                "node_ids": node_ids,
                "route_text": short_route_text(node_ids, limit=6),
                "points": points,
                "first_subtask": dict(agv.get("nextSubtask") or {}) if isinstance(agv.get("nextSubtask"), dict) else None,
            }
    return first_paths


def build_task_aliases(raw_events: List[Dict[str, Any]]) -> Dict[str, str]:
    aliases: Dict[str, str] = {}

    def note(task_id: Any) -> None:
        value = str(task_id or "").strip()
        if not value or value in aliases:
            return
        aliases[value] = f"T{len(aliases) + 1:02d}"

    for raw in raw_events:
        kind = str(raw.get("kind", "")).strip()
        if kind == "tasks_published":
            tasks = raw.get("tasks")
            if isinstance(tasks, list):
                for task in tasks:
                    if isinstance(task, dict):
                        note(task.get("task_id") or task.get("taskId"))
        elif kind == "assignment":
            assignments = raw.get("assignments")
            if isinstance(assignments, list):
                for entry in assignments:
                    if not isinstance(entry, dict):
                        continue
                    tasks = entry.get("tasks")
                    if not isinstance(tasks, list):
                        continue
                    for task in tasks:
                        if isinstance(task, dict):
                            note(task.get("task_id") or task.get("taskId"))
    return aliases


def task_alias(task_id: Any, aliases: Dict[str, str]) -> str:
    value = str(task_id or "").strip()
    if not value:
        return "TASK"
    return aliases.get(value, value[:8])


def summarize_task_items(tasks: List[Dict[str, Any]], aliases: Dict[str, str], limit: int = 12) -> List[Dict[str, Any]]:
    items: List[Dict[str, Any]] = []
    for task in tasks[:limit]:
        if not isinstance(task, dict):
            continue
        raw_task_id = str(task.get("task_id") or task.get("taskId") or "TASK")
        items.append(
            {
                "task_id": task_alias(raw_task_id, aliases),
                "raw_task_id": raw_task_id,
                "pickup_node": node_id(task.get("pickup_node")),
                "delivery_node": node_id(task.get("delivery_node")),
                "priority": safe_int(task.get("priority"), 0),
            }
        )
    return items


def summarize_assignment_items(
    assignments: List[Dict[str, Any]],
    first_paths: Dict[str, Dict[str, Any]],
    aliases: Dict[str, str],
    *,
    allowed_agv_ids: Optional[set[str]] = None,
    limit: int = 128,
) -> List[Dict[str, Any]]:
    items: List[Dict[str, Any]] = []
    for entry in assignments:
        if not isinstance(entry, dict):
            continue
        agv_id = str(entry.get("agv_id", "")).strip()
        if not agv_id:
            continue
        if allowed_agv_ids is not None and agv_id not in allowed_agv_ids:
            continue
        tasks = entry.get("tasks")
        if not isinstance(tasks, list):
            tasks = []
        task_ids: List[str] = []
        raw_task_ids: List[str] = []
        first_task_id = ""
        first_pickup = None
        for task in tasks:
            if not isinstance(task, dict):
                continue
            raw_task_id = str(task.get("task_id", "")).strip()
            if raw_task_id:
                raw_task_ids.append(raw_task_id)
                task_ids.append(task_alias(raw_task_id, aliases))
                if not first_task_id:
                    first_task_id = raw_task_id
            if first_pickup is None:
                first_pickup = node_id(task.get("pickup_node"))
        items.append(
            {
                "agv_id": agv_id,
                "task_ids": task_ids,
                "raw_task_ids": raw_task_ids,
                "task_chain_text": (" -> ".join(task_ids[:4]) + (" -> ..." if len(task_ids) > 4 else "")) if task_ids else "未接单",
                "eta_text": duration_text_from_iso(entry.get("estimated_start_time"), entry.get("estimated_completion_time")) if task_ids else "-",
                "first_task_id": first_task_id,
                "first_pickup_node": first_pickup,
                "has_first_path": agv_id in first_paths,
                "assigned": bool(task_ids),
                "task_count": len(task_ids),
            }
        )
        if len(items) >= limit:
            break
    items.sort(key=lambda item: (0 if item.get("assigned") else 1, item.get("agv_id", "")))
    return items


def latest_tasks_before(
    task_events: List[Dict[str, Any]],
    when_s: float,
    *,
    after_s: float = -1.0,
) -> Optional[Dict[str, Any]]:
    chosen: Optional[Dict[str, Any]] = None
    chosen_time = -1.0
    for raw in task_events:
        sim_time_s = safe_float(raw.get("sim_time_s"), 0.0)
        if sim_time_s > when_s or sim_time_s < after_s:
            continue
        if sim_time_s >= chosen_time:
            chosen = raw
            chosen_time = sim_time_s
    return chosen


def describe_task_batch(tasks: List[Dict[str, Any]]) -> str:
    if not tasks:
        return "无任务详情"
    labels: List[str] = []
    for task in tasks[:3]:
        task_id = str(task.get("task_id", "")).strip() or "TASK"
        pickup = node_id(task.get("pickup_node"))
        delivery = node_id(task.get("delivery_node"))
        if pickup is not None and delivery is not None:
            labels.append(f"{task_id} N{pickup}->N{delivery}")
        else:
            labels.append(task_id)
    if len(tasks) > len(labels):
        labels.append(f"+{len(tasks) - len(labels)}")
    return " / ".join(labels)


def find_frame_agv(frame: Dict[str, Any], agv_id: str) -> Optional[Dict[str, Any]]:
    for agv in frame.get("agvs", []) or []:
        if str(agv.get("id", "")) == agv_id:
            return agv
    return None


def build_conflict_event(
    raw: Dict[str, Any],
    frames: List[Dict[str, Any]],
    bridge_node_set: set[int],
) -> Optional[Dict[str, Any]]:
    reserved_nodes = raw.get("reserved_nodes")
    if not isinstance(reserved_nodes, list) or not reserved_nodes:
        return None
    frame = frame_at(frames, safe_float(raw.get("sim_time_s"), 0.0))
    for item in reserved_nodes:
        if not isinstance(item, dict):
            continue
        reason = str(item.get("reasonLabel", "")).strip()
        owner = str(item.get("owner", "")).strip()
        nid = node_id(item.get("nodeId"))
        if nid is None:
            continue
        if reason not in ("waiting_point", "temp_goal"):
            continue
        related = [owner] if owner else []
        contender = ""
        for agv in frame.get("agvs", []) or []:
            agv_id = str(agv.get("id", "")).strip()
            if not agv_id or agv_id == owner:
                continue
            next_node = node_id(agv.get("nextNodeId"))
            path_nodes = [node_id(pt.get("nodeId")) for pt in agv.get("path", [])[:6]]
            trail_nodes = [node_id(pt.get("nodeId")) for pt in agv.get("trail", [])[:6]]
            if nid == next_node or nid in path_nodes or nid in trail_nodes:
                contender = agv_id
                related.append(agv_id)
                break
        title = "冲突消解" if contender else "关键预约"
        if reason == "temp_goal":
            title = "绕行脱困"
        if contender:
            subtitle = f"{owner or 'AGV'} 占用 N{nid}，{contender} 触发等待或重规划"
        elif owner:
            subtitle = f"{owner} 在 N{nid} 建立重点预约，系统保持安全窗口"
        else:
            subtitle = f"N{nid} 出现关键预约变化"
        return {
            "kind": "conflict",
            "start_s": max(0.0, safe_float(raw.get("sim_time_s"), 0.0) - 0.1),
            "duration_s": 3.2,
            "playback_rate": 0.35 if contender else 0.5,
            "title": title,
            "subtitle": subtitle,
            "severity": "attention",
            "focus": {
                "agv_ids": [x for x in related if x],
                "node_ids": [nid],
                "zoom": 2.8,
                "mode": "conflict",
            },
            "owner_agv": owner,
            "contender_agv": contender,
            "node_id": nid,
            "bridge_related": nid in bridge_node_set,
            "raw_event_index": safe_int(raw.get("event_index"), 0),
        }
    return None


def build_timeline(
    raw_events: List[Dict[str, Any]],
    frames: List[Dict[str, Any]],
    bridge_node_set: set[int],
) -> List[Dict[str, Any]]:
    timeline: List[Dict[str, Any]] = []
    recent_by_key: Dict[str, float] = {}

    def should_keep(key: str, when_s: float, min_gap_s: float) -> bool:
        last = recent_by_key.get(key)
        if last is not None and (when_s - last) < min_gap_s:
            return False
        recent_by_key[key] = when_s
        return True

    for raw in raw_events:
        kind = str(raw.get("kind", "")).strip()
        sim_time_s = safe_float(raw.get("sim_time_s"), 0.0)
        event: Optional[Dict[str, Any]] = None
        if kind == "tasks_published":
            tasks = raw.get("tasks")
            if not isinstance(tasks, list) or not tasks:
                continue
            key = f"tasks:{describe_task_batch(tasks)}"
            if not should_keep(key, sim_time_s, 0.8):
                continue
            node_ids = []
            for task in tasks[:4]:
                if not isinstance(task, dict):
                    continue
                node_ids.extend([node_id(task.get("pickup_node")), node_id(task.get("delivery_node"))])
            event = {
                "kind": "tasks",
                "start_s": max(0.0, sim_time_s - 0.1),
                "duration_s": 2.5,
                "playback_rate": 0.7,
                "title": f"新任务进入系统 ({safe_int(raw.get('task_count'), len(tasks))})",
                "subtitle": describe_task_batch(tasks),
                "severity": "feature",
                "focus": {
                    "agv_ids": [],
                    "node_ids": unique_ints(node_ids, limit=8),
                    "zoom": 1.9,
                    "mode": "tasks",
                },
                "raw_event_index": safe_int(raw.get("event_index"), 0),
            }
        elif kind == "assignment":
            assignments = raw.get("assignments")
            if not isinstance(assignments, list) or not assignments:
                continue
            agv_ids: List[str] = []
            node_ids: List[Optional[int]] = []
            snippets: List[str] = []
            for entry in assignments:
                if not isinstance(entry, dict):
                    continue
                agv_id = str(entry.get("agv_id", "")).strip()
                if agv_id:
                    agv_ids.append(agv_id)
                tasks = entry.get("tasks")
                if not isinstance(tasks, list):
                    tasks = []
                if tasks:
                    snippets.append(f"{agv_id}: {describe_task_batch(tasks)}")
                for task in tasks:
                    if not isinstance(task, dict):
                        continue
                    node_ids.extend([node_id(task.get("pickup_node")), node_id(task.get("delivery_node"))])
            key = f"assignment:{','.join(agv_ids)}"
            if not should_keep(key, sim_time_s, 0.6):
                continue
            event = {
                "kind": "assignment",
                "start_s": max(0.0, sim_time_s - 0.15),
                "duration_s": 3.0,
                "playback_rate": 0.45,
                "title": "任务分配",
                "subtitle": " | ".join(snippets[:3]) if snippets else "系统完成一轮调度决策",
                "severity": "feature",
                "focus": {
                    "agv_ids": agv_ids[:4],
                    "node_ids": unique_ints(node_ids, limit=10),
                    "zoom": 2.2,
                    "mode": "assignment",
                },
                "raw_event_index": safe_int(raw.get("event_index"), 0),
            }
        elif kind == "path_update":
            device_id = str(raw.get("device_id", "")).strip()
            if not device_id or safe_int(raw.get("point_count"), 0) <= 1:
                continue
            key = f"path:{device_id}:{short_route_text(raw.get('node_ids') or [], limit=5)}"
            if not should_keep(key, sim_time_s, 0.8):
                continue
            route = list(raw.get("node_ids") or [])
            event = {
                "kind": "path",
                "start_s": max(0.0, sim_time_s - 0.1),
                "duration_s": 2.4,
                "playback_rate": 0.55,
                "title": f"路径规划 {device_id}",
                "subtitle": short_route_text(route, limit=5),
                "severity": "info",
                "focus": {
                    "agv_ids": [device_id],
                    "node_ids": unique_ints([node_id(x) for x in route], limit=8),
                    "zoom": 2.4,
                    "mode": "path",
                },
                "raw_event_index": safe_int(raw.get("event_index"), 0),
            }
        elif kind == "trail_update":
            device_id = str(raw.get("device_id", "")).strip()
            if not device_id or safe_int(raw.get("point_count"), 0) <= 1:
                continue
            key = f"trail:{device_id}:{short_route_text(raw.get('node_ids') or [], limit=4)}"
            if not should_keep(key, sim_time_s, 1.2):
                continue
            route = list(raw.get("node_ids") or [])
            event = {
                "kind": "trail",
                "start_s": max(0.0, sim_time_s - 0.05),
                "duration_s": 1.8,
                "playback_rate": 0.75,
                "title": f"预约窗口 {device_id}",
                "subtitle": short_route_text(route, limit=4),
                "severity": "info",
                "focus": {
                    "agv_ids": [device_id],
                    "node_ids": unique_ints([node_id(x) for x in route], limit=6),
                    "zoom": 2.6,
                    "mode": "trail",
                },
                "raw_event_index": safe_int(raw.get("event_index"), 0),
            }
        elif kind == "reservation_snapshot":
            event = build_conflict_event(raw, frames, bridge_node_set)
            if event is not None:
                key = f"conflict:{event['subtitle']}"
                if not should_keep(key, safe_float(event.get("start_s"), sim_time_s), 1.5):
                    event = None
        if event is None:
            continue
        event["event_id"] = f"evt_{len(timeline):04d}"
        event["end_s"] = safe_float(event.get("start_s"), 0.0) + safe_float(event.get("duration_s"), 0.0)
        timeline.append(event)
    return timeline


def build_assignment_scene(
    *,
    scene_id: str,
    order: int,
    title: str,
    subtitle: str,
    assignment_raw: Dict[str, Any],
    task_raw: Optional[Dict[str, Any]],
    first_paths: Dict[str, Dict[str, Any]],
    task_aliases: Dict[str, str],
    focus_agv_ids: Optional[List[str]] = None,
    allowed_agv_ids: Optional[set[str]] = None,
) -> Optional[Dict[str, Any]]:
    assignments = assignment_raw.get("assignments")
    if not isinstance(assignments, list):
        return None
    assignment_items = summarize_assignment_items(assignments, first_paths, task_aliases, allowed_agv_ids=allowed_agv_ids)
    if not assignment_items:
        return None
    tasks = task_raw.get("tasks") if isinstance(task_raw, dict) else []
    if not isinstance(tasks, list):
        tasks = []
    task_items = summarize_task_items(tasks, task_aliases)
    start_s = min(
        safe_float(task_raw.get("sim_time_s"), safe_float(assignment_raw.get("sim_time_s"), 0.0)) if isinstance(task_raw, dict) else safe_float(assignment_raw.get("sim_time_s"), 0.0),
        safe_float(assignment_raw.get("sim_time_s"), 0.0),
    )
    node_ids: List[Optional[int]] = []
    for item in task_items:
        node_ids.extend([item.get("pickup_node"), item.get("delivery_node")])
    for item in assignment_items:
        node_ids.append(item.get("first_pickup_node"))
    agv_ids = focus_agv_ids or [item.get("agv_id", "") for item in assignment_items[:4]]
    return {
        "scene_id": scene_id,
        "order": order,
        "scene_label": f"Scene {order}",
        "kind": "assignment" if order == 1 else "idle_assignment",
        "title": title,
        "subtitle": subtitle,
        "start_s": max(0.0, start_s - 0.15),
        "focus": {
            "agv_ids": [x for x in agv_ids if x],
            "node_ids": unique_ints(node_ids, limit=10),
            "zoom": 2.15,
            "mode": "assignment",
        },
        "cards": [
            {
                "type": "task_input",
                "title": "输入任务",
                "task_count": len(task_items),
                "items": task_items,
            },
            {
                "type": "assignment_result",
                "title": "AGV 分配结果",
                "items": assignment_items,
            },
        ],
    }


def build_path_scene(
    *,
    order: int,
    first_paths: Dict[str, Dict[str, Any]],
    after_s: float,
    preferred_agv_ids: Optional[List[str]] = None,
) -> Optional[Dict[str, Any]]:
    candidates = sorted(first_paths.values(), key=lambda item: safe_float(item.get("start_s"), 0.0))
    if preferred_agv_ids:
      preferred = [first_paths[agv_id] for agv_id in preferred_agv_ids if agv_id in first_paths]
      if preferred:
          candidates = sorted(preferred, key=lambda item: safe_float(item.get("start_s"), 0.0))
    filtered = [item for item in candidates if safe_float(item.get("start_s"), 0.0) >= after_s]
    chosen = filtered[0] if filtered else (candidates[0] if candidates else None)
    if chosen is None:
        return None
    agv_id = str(chosen.get("agv_id", "")).strip()
    node_ids = [node_id(x) for x in chosen.get("node_ids", [])]
    items = []
    source_items = filtered[:6] if filtered else candidates[:6]
    for path_info in source_items:
        items.append(
            {
                "agv_id": str(path_info.get("agv_id", "")).strip(),
                "route_text": str(path_info.get("route_text", "")).strip(),
                "point_count": safe_int(path_info.get("point_count"), 0),
                "has_first_path": True,
            }
        )
    return {
        "scene_id": "scene_02_path",
        "order": order,
        "scene_label": f"Scene {order}",
        "kind": "path_planning",
        "title": "路径规划",
        "subtitle": "点击 AGV 可高亮首次全局 Path，查看到第一个子任务点的规划结果。",
        "start_s": max(0.0, safe_float(chosen.get("start_s"), 0.0) - 0.1),
        "focus": {
            "agv_ids": [agv_id] if agv_id else [],
            "node_ids": unique_ints(node_ids, limit=10),
            "zoom": 2.5,
            "mode": "path",
        },
        "cards": [
            {
                "type": "path_result",
                "title": "首次全局 Path",
                "items": items,
            }
        ],
    }


def build_bridge_scene(
    *,
    order: int,
    first_paths: Dict[str, Dict[str, Any]],
    bridge_node_set: set[int],
    after_s: float,
) -> Optional[Dict[str, Any]]:
    if not bridge_node_set:
        return None
    candidates = [
        item
        for item in sorted(first_paths.values(), key=lambda x: safe_float(x.get("start_s"), 0.0))
        if route_intersects_bridge(item.get("node_ids", []), bridge_node_set)
        and safe_float(item.get("start_s"), 0.0) >= after_s
    ]
    if not candidates:
        candidates = [
            item
            for item in sorted(first_paths.values(), key=lambda x: safe_float(x.get("start_s"), 0.0))
            if route_intersects_bridge(item.get("node_ids", []), bridge_node_set)
        ]
    chosen = candidates[0] if candidates else None
    if chosen is None:
        return None
    agv_id = str(chosen.get("agv_id", "")).strip()
    bridge_nodes = [nid for nid in chosen.get("node_ids", []) if node_id(nid) in bridge_node_set][:8]
    start_s = max(0.0, safe_float(chosen.get("start_s"), 0.0) - 0.1)
    return {
        "scene_id": "scene_bridge",
        "order": order,
        "scene_label": f"Scene {order}",
        "kind": "bridge",
        "title": "桥场景",
        "subtitle": f"{agv_id} 路径经过独木桥(N575)区域，展示桥区路径组织。",
        "start_s": start_s,
        "end_s": start_s + 25.0,
        "focus": {
            "agv_ids": [agv_id] if agv_id else [],
            "node_ids": unique_ints([node_id(x) for x in bridge_nodes], limit=10),
            "zoom": 2.6,
            "mode": "path",
        },
        "cards": [
            {
                "type": "bridge_summary",
                "title": "桥区路径",
                "agv_id": agv_id,
                "route_text": str(chosen.get("route_text", "")).strip(),
                "bridge_nodes": [node_id(x) for x in bridge_nodes if node_id(x) is not None],
                "has_first_path": bool(agv_id),
            }
        ],
    }


def build_bridge_wait_scene(
    *,
    order: int,
    frames: List[Dict[str, Any]],
    bridge_node_set: set[int],
    preferred_nodes: set[int],
    map_edges: List[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    """Detect a real bridge wait→clearance→entry cycle from frame data.

    Finds occupancy periods on preferred bridge nodes, then checks whether
    another AGV was stopped in the bridge approach zone during that period.
    Picks the transition with the clearest waiting evidence.
    """
    preferred_node_values = {int(x) for x in preferred_nodes}
    if not bridge_node_set or not preferred_node_values:
        return None

    # Compute approach zone: nodes within 3 hops of preferred nodes, excluding bridge nodes
    adj: Dict[int, set] = {}
    for e in map_edges:
        a = int(e.get("startNode", e.get("startNodeId", 0)))
        b = int(e.get("endNode", e.get("endNodeId", 0)))
        adj.setdefault(a, set()).add(b)
        adj.setdefault(b, set()).add(a)
    approach_zone: set[int] = set()
    frontier = set(preferred_node_values)
    for _ in range(3):
        next_frontier: set[int] = set()
        for n in frontier:
            for nb in adj.get(n, set()):
                if nb not in preferred_node_values and nb not in bridge_node_set and nb not in approach_zone:
                    approach_zone.add(nb)
                    next_frontier.add(nb)
        frontier = next_frontier

    # --- Phase 1: find bridge occupancy periods ---
    # Each period: (start_s, end_s, agv_ids_on_bridge)
    periods: List[Dict[str, Any]] = []
    current_occ: set[str] = set()
    period_start: Optional[float] = None
    for frame in frames:
        sim_time_s = safe_float(frame.get("sim_time_s"), 0.0)
        on_pref: set[str] = set()
        for agv in frame.get("agvs", []) or []:
            nid = node_id(agv.get("nodeId"))
            if nid is not None and nid in preferred_node_values:
                on_pref.add(str(agv.get("id", "")).strip())
        if on_pref and not current_occ:
            period_start = sim_time_s
        if not on_pref and current_occ and period_start is not None:
            periods.append({
                "start_s": period_start,
                "end_s": sim_time_s,
                "agv_ids": set(current_occ),
            })
        current_occ = on_pref

    # --- Phase 2: for each period, find a waiting AGV that enters after ---
    candidates: List[Dict[str, Any]] = []
    for period in periods:
        if period["start_s"] < 30.0:
            continue
        occ_agvs = period["agv_ids"]
        # Scan frames during this occupancy period for stopped AGVs in approach zone
        waiting_agv_id: Optional[str] = None
        waiting_node: Optional[int] = None
        for frame in frames:
            fst = safe_float(frame.get("sim_time_s"), 0.0)
            if fst < period["start_s"] or fst > period["end_s"]:
                continue
            for agv in frame.get("agvs", []) or []:
                aid = str(agv.get("id", "")).strip()
                if not aid or aid in occ_agvs:
                    continue
                speed = safe_float(agv.get("speed"), 0.0)
                if speed > 1e-6:
                    continue
                nid_now = node_id(agv.get("nodeId"))
                if nid_now is None or nid_now not in approach_zone:
                    continue
                path_nodes = {node_id(pt.get("nodeId")) for pt in agv.get("path", []) or []}
                if path_nodes.intersection(preferred_node_values):
                    waiting_agv_id = aid
                    waiting_node = nid_now
                    break
            if waiting_agv_id:
                break
        if not waiting_agv_id:
            continue
        # Check if the waiting AGV actually enters the bridge after the period
        entry_time_s: Optional[float] = None
        for frame in frames:
            fst = safe_float(frame.get("sim_time_s"), 0.0)
            if fst <= period["end_s"]:
                continue
            for agv in frame.get("agvs", []) or []:
                if str(agv.get("id", "")).strip() != waiting_agv_id:
                    continue
                nid_now = node_id(agv.get("nodeId"))
                if nid_now is not None and nid_now in preferred_node_values:
                    entry_time_s = fst
                    break
            if entry_time_s is not None:
                break
        bridge_agv = sorted(occ_agvs)[0] if occ_agvs else ""
        gap_s = (entry_time_s - period["end_s"]) if entry_time_s is not None else 999.0
        candidates.append({
            "period_start": period["start_s"],
            "period_end": period["end_s"],
            "bridge_agv": bridge_agv,
            "waiting_agv": waiting_agv_id,
            "waiting_node": waiting_node,
            "entry_time_s": entry_time_s,
            "gap_s": gap_s,
            "complete": entry_time_s is not None,
        })

    if not candidates:
        return None

    # Prefer complete cycles; among those, pick the tightest gap
    complete = [c for c in candidates if c["complete"]]
    chosen = min(complete, key=lambda c: c["gap_s"]) if complete else candidates[0]

    bridge_agv = chosen["bridge_agv"]
    waiting_agv = chosen["waiting_agv"]
    entry_time_s = chosen.get("entry_time_s")
    start_s = max(0.0, chosen["period_start"] - 5.0)
    end_s = (entry_time_s + 5.0) if entry_time_s else (chosen["period_end"] + 10.0)

    subtitle = (
        f"{waiting_agv} 在桥外等待，{bridge_agv} 离开桥区后放行进入。"
        if entry_time_s is not None
        else f"{waiting_agv} 在桥外等待，前方桥区仍被 {bridge_agv} 占用。"
    )
    return {
        "scene_id": "scene_bridge",
        "order": order,
        "scene_label": f"Scene {order}",
        "kind": "bridge",
        "title": "桥场景",
        "subtitle": subtitle,
        "start_s": start_s,
        "end_s": end_s,
        "playback_rate": 1.0,
        "focus": {
            "agv_ids": [bridge_agv, waiting_agv],
            "node_ids": sorted(preferred_node_values),
            "zoom": 3.0,
            "mode": "path",
        },
        "cards": [
            {
                "type": "bridge_summary",
                "title": "桥区通行管理",
                "agv_id": bridge_agv,
                "route_text": f"{bridge_agv} 在桥内，{waiting_agv} 等待进入",
                "bridge_nodes": sorted(preferred_node_values),
                "has_first_path": True,
            }
        ],
    }


def build_conflict_scene(
    *,
    order: int,
    conflict_events: List[Dict[str, Any]],
    after_s: float,
) -> Optional[Dict[str, Any]]:
    candidates = [item for item in conflict_events if safe_float(item.get("start_s"), 0.0) >= after_s]
    if not candidates and conflict_events:
        candidates = list(conflict_events)
    chosen = candidates[0] if candidates else (conflict_events[0] if conflict_events else None)
    if chosen is None:
        return None
    owner = str(chosen.get("owner_agv", "")).strip()
    contender = str(chosen.get("contender_agv", "")).strip()
    node_value = node_id(chosen.get("node_id"))
    agv_ids = [x for x in [owner, contender] if x]
    return {
        "scene_id": "scene_04_conflict",
        "order": order,
        "scene_label": f"Scene {order}",
        "kind": "headon_conflict",
        "title": "对向冲突",
        "subtitle": chosen.get("subtitle") or "系统检测到对向会车风险，并通过等待或让行消解冲突。",
        "start_s": max(0.0, safe_float(chosen.get("start_s"), 0.0) - 0.1),
        "focus": {
            "agv_ids": agv_ids,
            "node_ids": [node_value] if node_value is not None else [],
            "zoom": 2.8,
            "mode": "conflict",
        },
        "cards": [
            {
                "type": "conflict_summary",
                "title": "冲突双方",
                "owner_agv": owner or "-",
                "contender_agv": contender or "-",
                "node_id": node_value,
                "bridge_related": bool(chosen.get("bridge_related")),
                "summary": chosen.get("subtitle") or "",
            }
        ],
    }


def build_representative_conflict_scene(
    *,
    order: int,
    conflict_events: List[Dict[str, Any]],
    min_start_s: float,
) -> Optional[Dict[str, Any]]:
    grouped: Dict[Tuple[str, str], List[Dict[str, Any]]] = {}
    for item in conflict_events:
        owner = str(item.get("owner_agv", "")).strip()
        contender = str(item.get("contender_agv", "")).strip()
        if not owner or not contender:
            continue
        grouped.setdefault((owner, contender), []).append(item)

    best: Optional[Tuple[Tuple[str, str], List[Dict[str, Any]]]] = None
    for key, events in grouped.items():
        later = [event for event in events if safe_float(event.get("start_s"), 0.0) >= min_start_s]
        source = later
        if not source:
            continue
        if best is None or len(source) > len(best[1]):
            best = (key, source)
    if best is None:
        return build_conflict_scene(order=order, conflict_events=conflict_events, after_s=min_start_s)

    (owner, contender), source = best
    source.sort(key=lambda event: safe_float(event.get("start_s"), 0.0))
    start_s = safe_float(source[0].get("start_s"), 0.0)
    end_s = safe_float(source[min(len(source) - 1, 3)].get("start_s"), start_s) + 4.0
    node_ids = [node_id(event.get("node_id")) for event in source[:4]]
    subtitle = source[0].get("subtitle") or f"{owner} 与 {contender} 在窄路段发生对向冲突。"
    return {
        "scene_id": "scene_04_conflict",
        "order": order,
        "scene_label": f"Scene {order}",
        "kind": "headon_conflict",
        "title": "对向冲突",
        "subtitle": subtitle,
        "start_s": start_s,
        "end_s": end_s,
        "playback_rate": 0.9,
        "focus": {
            "agv_ids": [owner, contender],
            "node_ids": unique_ints(node_ids, limit=6),
            "zoom": 2.9,
            "mode": "conflict",
        },
        "cards": [
            {
                "type": "conflict_summary",
                "title": "冲突双方",
                "owner_agv": owner,
                "contender_agv": contender,
                "node_id": node_id(source[0].get("node_id")),
                "bridge_related": False,
                "summary": subtitle,
            }
        ],
    }


def build_blocking_scene(
    *,
    order: int,
    frames: List[Dict[str, Any]],
    min_start_s: float = 20.0,
    max_start_s: float = 999.0,
) -> Optional[Dict[str, Any]]:
    """Find the most sustained real blocking interaction from frame data."""
    blocking: Dict[Tuple[str, str], Dict[str, Any]] = {}
    for frame_idx, frame in enumerate(frames):
        sim_time_s = safe_float(frame.get("sim_time_s"), 0.0)
        if sim_time_s < min_start_s or sim_time_s > max_start_s:
            continue
        if frame_idx % 3 != 0:
            continue
        agvs = frame.get("agvs", []) or []
        node_to_agv: Dict[int, str] = {}
        for agv in agvs:
            nid = node_id(agv.get("nodeId"))
            if nid is not None:
                node_to_agv[nid] = str(agv.get("id", "")).strip()
        for agv in agvs:
            agv_id = str(agv.get("id", "")).strip()
            speed = safe_float(agv.get("speed"), 0.0)
            if speed > 10.0 or not agv_id:
                continue
            next_val = agv.get("nextNodeId")
            if next_val is None or next_val == "":
                continue
            path_nodes: List[int] = []
            for pt in agv.get("path", [])[:5]:
                pn = node_id(pt.get("nodeId"))
                if pn is not None:
                    path_nodes.append(pn)
            for pn in path_nodes[:3]:
                if pn in node_to_agv and node_to_agv[pn] != agv_id:
                    blocker_id = node_to_agv[pn]
                    key = (blocker_id, agv_id)
                    if key not in blocking:
                        blocking[key] = {
                            "first_t": sim_time_s,
                            "last_t": sim_time_s,
                            "node": pn,
                            "count": 0,
                            "blocker": blocker_id,
                            "blocked": agv_id,
                        }
                    blocking[key]["last_t"] = sim_time_s
                    blocking[key]["count"] += 1
                    break
    if not blocking:
        return None
    best = max(blocking.values(), key=lambda b: (b["count"], b["last_t"] - b["first_t"]))
    if best["count"] < 2:
        return None
    start_s = max(0.0, best["first_t"] - 2.0)
    end_s = best["last_t"] + 5.0
    blocker = best["blocker"]
    blocked = best["blocked"]
    blocking_node = best["node"]
    return {
        "scene_id": "scene_conflict",
        "order": order,
        "scene_label": f"Scene {order}",
        "kind": "headon_conflict",
        "title": "冲突消解",
        "subtitle": f"{blocked} 前方路径被 {blocker} 占用（N{blocking_node}），系统安排等待后放行。",
        "start_s": start_s,
        "end_s": end_s,
        "playback_rate": 1.0,
        "focus": {
            "agv_ids": [blocker, blocked],
            "node_ids": [blocking_node],
            "zoom": 2.8,
            "mode": "conflict",
        },
        "cards": [
            {
                "type": "conflict_summary",
                "title": "冲突双方",
                "owner_agv": blocker,
                "contender_agv": blocked,
                "node_id": blocking_node,
                "bridge_related": False,
                "summary": f"{blocked} 前方被 {blocker} 占用 N{blocking_node}，等待路径释放后继续行驶。",
            }
        ],
    }


def build_headon_conflict_scene(
    *,
    order: int,
    frames: List[Dict[str, Any]],
    map_edges: List[Dict[str, Any]],
    min_start_s: float = 20.0,
    max_start_s: float = 999.0,
) -> Optional[Dict[str, Any]]:
    """Find two AGVs approaching each other on the same edge (head-on conflict)."""
    # Build adjacency with edge info
    edge_set: set[Tuple[int, int]] = set()
    for e in map_edges:
        a = int(e.get("startNode", e.get("startNodeId", 0)))
        b = int(e.get("endNode", e.get("endNodeId", 0)))
        edge_set.add((a, b))
        edge_set.add((b, a))

    # Look for frames where two AGVs are on the same edge moving toward each other
    headon_events: Dict[Tuple[str, str], Dict[str, Any]] = {}
    for frame_idx, frame in enumerate(frames):
        sim_time_s = safe_float(frame.get("sim_time_s"), 0.0)
        if sim_time_s < min_start_s or sim_time_s > max_start_s:
            continue
        if frame_idx % 3 != 0:
            continue
        agvs = frame.get("agvs", []) or []
        # Build a map of (current_node, next_node) -> agv_id for moving AGVs
        moving: List[Tuple[str, int, int, float]] = []
        for agv in agvs:
            agv_id = str(agv.get("id", "")).strip()
            if not agv_id:
                continue
            cur_node = node_id(agv.get("nodeId"))
            next_node = node_id(agv.get("nextNodeId"))
            speed = safe_float(agv.get("speed"), 0.0)
            if cur_node is not None and next_node is not None and cur_node != next_node:
                moving.append((agv_id, cur_node, next_node, speed))

        # Check pairs for head-on (A→B while B→A, or close approach on same corridor)
        for i in range(len(moving)):
            for j in range(i + 1, len(moving)):
                a_id, a_cur, a_next, a_spd = moving[i]
                b_id, b_cur, b_next, b_spd = moving[j]
                # Direct head-on: A going to B's node and B going to A's node
                is_headon = (a_next == b_cur and b_next == a_cur)
                # Near head-on: A and B on same corridor segment approaching each other
                if not is_headon:
                    # A heading toward B's position, B heading toward A's position
                    is_headon = (a_next == b_cur or b_next == a_cur) and (a_cur, b_cur) in edge_set
                if not is_headon:
                    # Check if they share a next node (converging conflict)
                    is_headon = (a_next == b_next) and (a_cur, a_next) in edge_set and (b_cur, b_next) in edge_set
                if not is_headon:
                    continue
                key = tuple(sorted([a_id, b_id]))  # type: ignore
                conflict_node = a_next if a_next == b_cur else (b_next if b_next == a_cur else a_next)
                if key not in headon_events:
                    headon_events[key] = {
                        "first_t": sim_time_s,
                        "last_t": sim_time_s,
                        "count": 0,
                        "agv_a": a_id,
                        "agv_b": b_id,
                        "node": conflict_node,
                    }
                headon_events[key]["last_t"] = sim_time_s
                headon_events[key]["count"] += 1

    if not headon_events:
        return None

    # Pick the most sustained head-on encounter
    best = max(headon_events.values(), key=lambda h: (h["count"], h["last_t"] - h["first_t"]))
    if best["count"] < 2:
        return None

    start_s = max(0.0, best["first_t"] - 3.0)
    end_s = best["last_t"] + 8.0
    agv_a = best["agv_a"]
    agv_b = best["agv_b"]
    conflict_node = best["node"]

    return {
        "scene_id": "scene_conflict",
        "order": order,
        "scene_label": f"Scene {order}",
        "kind": "headon_conflict",
        "title": "冲突消解",
        "subtitle": f"{agv_a} 与 {agv_b} 在 N{conflict_node} 附近对向行驶，系统检测冲突并安排让行。",
        "start_s": start_s,
        "end_s": end_s,
        "playback_rate": 0.8,
        "focus": {
            "agv_ids": [agv_a, agv_b],
            "node_ids": [conflict_node] if conflict_node is not None else [],
            "zoom": 2.8,
            "mode": "conflict",
        },
        "cards": [
            {
                "type": "conflict_summary",
                "title": "对向冲突",
                "owner_agv": agv_a,
                "contender_agv": agv_b,
                "node_id": conflict_node,
                "bridge_related": False,
                "summary": f"{agv_a} 与 {agv_b} 对向行驶，在 N{conflict_node} 处冲突消解。",
            }
        ],
    }


def build_idle_assignment_scene(
    *,
    order: int,
    assignment_events: List[Dict[str, Any]],
    task_events: List[Dict[str, Any]],
    first_paths: Dict[str, Dict[str, Any]],
    task_aliases: Dict[str, str],
    after_s: float,
    scene_start_override: Optional[float] = None,
) -> Optional[Dict[str, Any]]:
    seen_assigned: set[str] = set()
    first_non_empty_seen = False
    for raw in assignment_events:
        sim_time_s = safe_float(raw.get("sim_time_s"), 0.0)
        assignments = raw.get("assignments")
        if not isinstance(assignments, list):
            continue
        current_non_empty: set[str] = set()
        for entry in assignments:
            if not isinstance(entry, dict):
                continue
            agv_id = str(entry.get("agv_id", "")).strip()
            tasks = entry.get("tasks")
            if agv_id and isinstance(tasks, list) and tasks:
                current_non_empty.add(agv_id)
        if current_non_empty:
            if not first_non_empty_seen:
                seen_assigned.update(current_non_empty)
                first_non_empty_seen = True
                continue
            new_agvs = current_non_empty - seen_assigned
            seen_assigned.update(current_non_empty)
            if sim_time_s < after_s:
                continue
            if not new_agvs and sim_time_s < after_s:
                continue
            task_raw = latest_tasks_before(task_events, sim_time_s, after_s=after_s - 0.01)
            allowed = new_agvs if new_agvs else current_non_empty
            scene = build_assignment_scene(
                scene_id="scene_05_idle_assign",
                order=order,
                title="空闲 AGV 接单",
                subtitle="当部分车辆空闲后，系统会继续把剩余任务投递给新的可用 AGV。",
                assignment_raw=raw,
                task_raw=task_raw,
                first_paths=first_paths,
                task_aliases=task_aliases,
                allowed_agv_ids=allowed,
            )
            if scene is not None:
                if scene_start_override is not None:
                    scene["start_s"] = max(0.0, float(scene_start_override))
                    scene["focus"] = {
                        **(scene.get("focus") or {}),
                        "agv_ids": sorted(list(allowed))[:2],
                    }
                    scene["playback_rate"] = 1.0
                    scene["subtitle"] = "部分 AGV 完成任务后重新空闲，系统继续分配剩余任务。"
                return scene
    return None


def assign_scene_ranges(scenes: List[Dict[str, Any]], total_sim_s: float) -> List[Dict[str, Any]]:
    ordered = [dict(scene) for scene in scenes if scene]
    ordered.sort(key=lambda item: safe_int(item.get("order"), 0))
    min_span_by_order = {
        1: 6.0,
        2: 6.0,
        3: 8.0,
        4: 10.0,
        5: 8.0,
    }
    cursor_s = 0.0
    for idx, scene in enumerate(ordered):
        start_s = max(safe_float(scene.get("start_s"), 0.0), cursor_s)
        scene["start_s"] = start_s
        min_span = min_span_by_order.get(safe_int(scene.get("order"), 0), 6.0)
        if idx + 1 < len(ordered):
            next_raw_start = safe_float(ordered[idx + 1].get("start_s"), total_sim_s)
            cursor_s = min(total_sim_s, max(next_raw_start, start_s + min_span))
            scene["end_s"] = max(start_s, cursor_s - 0.05)
        else:
            scene["end_s"] = max(start_s, total_sim_s)
    return ordered


def build_scenes(
    raw_events: List[Dict[str, Any]],
    frames: List[Dict[str, Any]],
    timeline: List[Dict[str, Any]],
    map_json: Dict[str, Any],
    first_paths: Dict[str, Dict[str, Any]],
) -> List[Dict[str, Any]]:
    total_sim_s = safe_float(frames[-1].get("sim_time_s"), 0.0) if frames else 0.0
    bridge_node_set = {int(x) for x in map_json.get("bridgeNodeIds", []) or [] if node_id(x) is not None}
    task_aliases = build_task_aliases(raw_events)
    task_events = [raw for raw in raw_events if str(raw.get("kind", "")).strip() == "tasks_published"]
    assignment_events = [
        raw
        for raw in raw_events
        if str(raw.get("kind", "")).strip() == "assignment"
        and isinstance(raw.get("assignments"), list)
    ]
    conflict_events = [item for item in timeline if str(item.get("kind", "")).strip() == "conflict"]

    first_assignment_raw = None
    for raw in assignment_events:
        assignments = raw.get("assignments")
        if not isinstance(assignments, list):
            continue
        if any(isinstance(entry, dict) and isinstance(entry.get("tasks"), list) and entry.get("tasks") for entry in assignments):
            first_assignment_raw = raw
            break
    if first_assignment_raw is None:
        return []

    first_assignment_time = safe_float(first_assignment_raw.get("sim_time_s"), 0.0)
    first_task_raw = latest_tasks_before(task_events, first_assignment_time)
    first_assigned_agv_ids = [
        str(entry.get("agv_id", "")).strip()
        for entry in first_assignment_raw.get("assignments", [])
        if isinstance(entry, dict) and isinstance(entry.get("tasks"), list) and entry.get("tasks")
    ]
    scene_1 = build_assignment_scene(
        scene_id="scene_01_assignment",
        order=1,
        title="任务分配",
        subtitle="系统接收任务并完成首轮 AGV 分配。",
        assignment_raw=first_assignment_raw,
        task_raw=first_task_raw,
        first_paths=first_paths,
        task_aliases=task_aliases,
    )
    if scene_1 is not None:
        scene_1["start_s"] = 0.0
        scene_1["end_s"] = 8.0
        scene_1["playback_rate"] = 1.0

    scene_2 = build_path_scene(order=2, first_paths=first_paths, after_s=0.0, preferred_agv_ids=first_assigned_agv_ids)
    if scene_2 is not None:
        scene_2["start_s"] = 8.0
        scene_2["end_s"] = 18.0
        scene_2["playback_rate"] = 1.0

    # Scene 3: bridge scene — prefer wait cycle, fall back to simple bridge traversal
    scene_bridge = build_bridge_wait_scene(order=3, frames=frames, bridge_node_set=bridge_node_set, preferred_nodes={574, 575, 576, 577, 578, 579, 580}, map_edges=map_json.get("edges", []))
    if scene_bridge is None:
        scene_bridge = build_bridge_scene(order=3, first_paths=first_paths, bridge_node_set={574, 575, 576, 577, 578, 579, 580}, after_s=30.0)
    if scene_bridge is not None:
        scene_bridge["playback_rate"] = 1.0

    # Scene 4: conflict (starts after bridge to avoid overlap)
    conflict_min_s = 20.0
    if scene_bridge is not None:
        conflict_min_s = max(conflict_min_s, safe_float(scene_bridge.get("end_s"), 20.0) + 1.0)
    scene_conflict = build_headon_conflict_scene(
        order=4, frames=frames, map_edges=map_json.get("edges", []),
        min_start_s=conflict_min_s, max_start_s=250.0,
    )
    if scene_conflict is None:
        scene_conflict = build_blocking_scene(order=4, frames=frames, min_start_s=conflict_min_s, max_start_s=250.0)
    if scene_conflict is not None:
        scene_conflict["playback_rate"] = 1.0

    # Scene 5: idle AGV picks up new tasks
    idle_start = max(160.0, total_sim_s * 0.7)
    last_scene_end = 0.0
    if scene_bridge is not None:
        last_scene_end = max(last_scene_end, safe_float(scene_bridge.get("end_s"), 0.0))
    if scene_conflict is not None:
        last_scene_end = max(last_scene_end, safe_float(scene_conflict.get("end_s"), 0.0))
    idle_start = max(idle_start, last_scene_end + 2.0)
    scene_5 = build_idle_assignment_scene(
        order=5,
        assignment_events=assignment_events,
        task_events=task_events,
        first_paths=first_paths,
        task_aliases=task_aliases,
        after_s=70.0,
        scene_start_override=idle_start,
    )
    if scene_5 is not None:
        scene_5["end_s"] = total_sim_s
        scene_5["playback_rate"] = 1.0

    # Collect scenes, sort chronologically, re-assign order numbers
    raw_scenes = [s for s in [scene_1, scene_2, scene_conflict, scene_bridge, scene_5] if s]
    raw_scenes.sort(key=lambda s: safe_float(s.get("start_s"), 0.0))
    for idx, scene in enumerate(raw_scenes):
        scene["order"] = idx + 1
        scene["scene_label"] = f"Scene {idx + 1}"
    return raw_scenes


def build_summary(
    session_meta: Dict[str, Any],
    frames: List[Dict[str, Any]],
    raw_events: List[Dict[str, Any]],
    timeline: List[Dict[str, Any]],
    scenes: List[Dict[str, Any]],
) -> Dict[str, Any]:
    total_sim_s = safe_float(frames[-1].get("sim_time_s"), 0.0) if frames else 0.0
    kinds = {}
    agv_ids: List[str] = []
    seen_agv_ids = set()
    for frame in frames:
        agvs = frame.get("agvs")
        if not isinstance(agvs, list):
            continue
        for agv in agvs:
            if not isinstance(agv, dict):
                continue
            agv_id = str(agv.get("id", "")).strip()
            if not agv_id or agv_id in seen_agv_ids:
                continue
            seen_agv_ids.add(agv_id)
            agv_ids.append(agv_id)
    for event in timeline:
        key = str(event.get("kind", "")).strip() or "unknown"
        kinds[key] = safe_int(kinds.get(key), 0) + 1
    return {
        "session_name": str(session_meta.get("session_name", "")).strip(),
        "agv_count": safe_int(session_meta.get("agv_count"), 0),
        "agv_ids": agv_ids,
        "total_frames": len(frames),
        "raw_event_count": len(raw_events),
        "director_event_count": len(timeline),
        "scene_count": len(scenes),
        "total_sim_time_s": total_sim_s,
        "chapters": kinds,
    }


def build_bundle(session_dir: Path, *, bundle_name: str = "leader_demo_bundle.json") -> Path:
    session_json = load_json(session_dir / "session.json")
    map_json = load_json(session_dir / "map.json")
    frames = load_jsonl(session_dir / "frames.jsonl")
    raw_events = load_jsonl(session_dir / "events.jsonl")
    frames.sort(key=lambda x: safe_float(x.get("sim_time_s"), 0.0))
    raw_events.sort(key=lambda x: safe_float(x.get("sim_time_s"), 0.0))
    bridge_node_set = {int(x) for x in map_json.get("bridgeNodeIds", []) or [] if node_id(x) is not None}
    timeline = build_timeline(raw_events, frames, bridge_node_set)
    first_paths = extract_first_paths(frames)

    # Enrich first_paths with all subtask (pickup/delivery) node IDs from assignments
    assignment_events = [
        raw for raw in raw_events
        if str(raw.get("kind", "")).strip() == "assignment"
        and isinstance(raw.get("assignments"), list)
    ]
    for raw in assignment_events:
        for entry in raw.get("assignments", []):
            if not isinstance(entry, dict):
                continue
            agv_id = str(entry.get("agv_id", "")).strip()
            if agv_id not in first_paths:
                continue
            if "subtask_nodes" in first_paths[agv_id]:
                continue
            tasks = entry.get("tasks")
            if not isinstance(tasks, list):
                continue
            st_nodes: list[int] = []
            path_node_set = set(first_paths[agv_id].get("node_ids", []))
            for task in tasks:
                if not isinstance(task, dict):
                    continue
                for key in ("pickup_node", "delivery_node"):
                    nid = node_id(task.get(key))
                    if nid is not None and nid not in st_nodes:
                        st_nodes.append(nid)
            # Keep only those on the actual path, but if none match, keep all
            on_path = [n for n in st_nodes if n in path_node_set]
            first_paths[agv_id]["subtask_nodes"] = on_path if on_path else st_nodes

    scenes = build_scenes(raw_events, frames, timeline, map_json, first_paths)
    summary = build_summary(session_json, frames, raw_events, timeline, scenes)
    bundle = {
        "metadata": {
            **session_json,
            "bundle_name": bundle_name,
            "profile": "leader_demo",
            "director_base_rate": 1.0,
        },
        "summary": summary,
        "map": map_json,
        "frames": frames,
        "timeline": timeline,
        "scenes": scenes,
        "first_paths": first_paths,
    }
    output_path = session_dir / bundle_name
    output_path.write_text(json.dumps(bundle, ensure_ascii=False), encoding="utf-8")
    return output_path


class ReplayServer:
    def __init__(self, bundle_path: Path, web_root: Path, host: str, port: int) -> None:
        self._bundle_path = bundle_path
        self._web_root = web_root
        self._host = host
        self._port = port
        self._server: Optional[ThreadingHTTPServer] = None

    def start(self) -> None:
        bundle_path = self._bundle_path
        web_root = self._web_root

        class Handler(BaseHTTPRequestHandler):
            def _send_bytes(self, body: bytes, content_type: str) -> None:
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _send_file(self, path: Path, content_type: str) -> None:
                if not path.exists() or not path.is_file():
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                self._send_bytes(path.read_bytes(), content_type)

            def log_message(self, format: str, *args: Any) -> None:  # noqa: A003
                return

            def do_GET(self) -> None:  # noqa: N802
                parsed = urllib.parse.urlparse(self.path)
                if parsed.path in ("/", "/index.html"):
                    return self._send_file(web_root / "index.html", "text/html; charset=utf-8")
                if parsed.path == "/app.js":
                    return self._send_file(web_root / "app.js", "application/javascript; charset=utf-8")
                if parsed.path == "/bundle.json":
                    return self._send_file(bundle_path, "application/json; charset=utf-8")
                self.send_error(HTTPStatus.NOT_FOUND)

        self._server = ThreadingHTTPServer((self._host, self._port), Handler)

    def serve_forever(self) -> None:
        if self._server is None:
            self.start()
        assert self._server is not None
        print(f"[replay] http://{self._host}:{self._port}/")
        try:
            self._server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            self._server.server_close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline leader-demo replay builder/server")
    sub = parser.add_subparsers(dest="command", required=True)

    build = sub.add_parser("build", help="Build a replay bundle from a recorded session")
    build.add_argument("--session", required=True, help="Replay session directory")
    build.add_argument("--bundle-name", default="leader_demo_bundle.json", help="Output bundle filename")

    serve = sub.add_parser("serve", help="Serve the leader-demo replay UI")
    serve.add_argument("--session", required=True, help="Replay session directory")
    serve.add_argument("--bundle-name", default="leader_demo_bundle.json", help="Bundle filename to serve/build")
    serve.add_argument("--host", default=os.getenv("SIM_REPLAY_HOST", "127.0.0.1"))
    serve.add_argument("--port", type=int, default=int(os.getenv("SIM_REPLAY_PORT", "18181")))
    serve.add_argument("--rebuild", action="store_true", help="Rebuild bundle before serving")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = Path(__file__).resolve().parent
    session_dir = Path(args.session).resolve()
    if not session_dir.exists() or not session_dir.is_dir():
        print(f"[replay] session not found: {session_dir}", file=sys.stderr)
        return 2
    bundle_path = session_dir / args.bundle_name
    if args.command == "build":
        output = build_bundle(session_dir, bundle_name=args.bundle_name)
        print(f"[replay] bundle built: {output}")
        return 0
    if args.rebuild or not bundle_path.exists():
        bundle_path = build_bundle(session_dir, bundle_name=args.bundle_name)
    web_root = root / "replay_web"
    if not web_root.exists():
        print(f"[replay] web root not found: {web_root}", file=sys.stderr)
        return 2
    server = ReplayServer(bundle_path, web_root, str(args.host), int(args.port))
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
