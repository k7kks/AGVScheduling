#!/usr/bin/env python3
import argparse
import json
import math
import os
import random
import sys
import threading
import time
import urllib.parse
from collections import deque
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple


def _load_sim_v1():
    root = Path(__file__).resolve().parents[1]
    path = root / "simulation" / "sim_runner.py"
    spec = spec_from_file_location("sim_v1", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load sim_v1 module from {path}")
    mod = module_from_spec(spec)
    spec.loader.exec_module(mod)  # type: ignore[assignment]
    return mod


sim = _load_sim_v1()


def env(key: str, default: str = "") -> str:
    return os.getenv(key, default)


def env_bool(key: str, default: bool = False) -> bool:
    raw = env(key, "")
    if raw is None:
        return bool(default)
    value = str(raw).strip().lower()
    if not value:
        return bool(default)
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off"):
        return False
    try:
        return int(value) != 0
    except Exception:
        return bool(default)


def _dump_assign_result(payload: Dict[str, Any]) -> None:
    dump_dir = env("SIM_ASSIGN_RESULT_DIR", "")
    if not dump_dir:
        return
    keep = _safe_int(env("SIM_ASSIGN_RESULT_KEEP", "30"), 30)
    ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    ms = int((time.time() * 1000) % 1000)
    path = Path(dump_dir)
    try:
        path.mkdir(parents=True, exist_ok=True)
    except Exception:
        return
    filename = f"assign_result_{ts}_{ms:03d}.json"
    file_path = path / filename
    try:
        file_path.write_text(json.dumps(payload, ensure_ascii=True), encoding="utf-8")
    except Exception:
        return
    if keep <= 0:
        return
    try:
        files = sorted(
            path.glob("assign_result_*.json"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
    except Exception:
        return
    for p in files[keep:]:
        try:
            p.unlink()
        except Exception:
            pass


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        return float(value)
    except Exception:
        return default


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return default
        return int(value)
    except Exception:
        return default


def _resolve_trail_min_drive_points(default: int = 2) -> int:
    raw = str(env("SIM_TRAIL_MIN_DRIVE_POINTS", "")).strip()
    if raw:
        return max(2, _safe_int(raw, default))
    return max(2, int(default))


def _resolve_status_interval_s(default: float = 0.0) -> float:
    raw = str(env("SIM_STATUS_INTERVAL_MS", "")).strip()
    if raw:
        return max(0.0, _safe_float(raw, default) / 1000.0)
    raw = str(env("SIM_STATUS_INTERVAL", "")).strip()
    if raw:
        return max(0.0, _safe_float(raw, default))
    return max(0.0, float(default))


def _resolve_bootstrap_map_wait_s(default: float = 1.5) -> float:
    raw = str(env("SIM_BOOTSTRAP_MAP_WAIT_SEC", "")).strip()
    if raw:
        return max(0.0, _safe_float(raw, default))
    return max(0.0, float(default))


def _iso_now_local() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def _reason_label(value: Any) -> str:
    code = _safe_int(value, 99)
    if code == 0:
        return "reserved_path"
    if code == 1:
        return "waiting_point"
    if code == 2:
        return "temp_goal"
    return "other"


def _parse_task_node_id(task: Dict[str, Any], index: int) -> Optional[int]:
    if not isinstance(task, dict):
        return None
    point = task.get("point")
    if isinstance(point, dict):
        node_id = sim.parse_node_id(point.get("nodeId"))
        if node_id is not None:
            return int(node_id)
    sub_tasks = task.get("subTasks")
    if isinstance(sub_tasks, list) and 0 <= index < len(sub_tasks):
        sub_task = sub_tasks[index]
        if isinstance(sub_task, dict):
            point = sub_task.get("point")
            if isinstance(point, dict):
                node_id = sim.parse_node_id(point.get("nodeId"))
                if node_id is not None:
                    return int(node_id)
            node_id = sim.parse_node_id(sub_task.get("nodeId"))
            if node_id is not None:
                return int(node_id)
    node_id = sim.parse_node_id(task.get("nodeId"))
    if node_id is not None:
        return int(node_id)
    return None


def _summarize_candidate_task(task: Dict[str, Any]) -> Dict[str, Any]:
    task_id = str(task.get("taskId", "")).strip()
    sub_tasks = task.get("subTasks")
    node_ids: List[int] = []
    if isinstance(sub_tasks, list):
        for sub_task in sub_tasks:
            if not isinstance(sub_task, dict):
                continue
            node_id = _parse_task_node_id(sub_task, 0)
            if node_id is None:
                node_id = _parse_task_node_id({"subTasks": [sub_task]}, 0)
            if node_id is None:
                continue
            if not node_ids or node_ids[-1] != int(node_id):
                node_ids.append(int(node_id))
    bind_robot_id = str(task.get("bindRobotId", "")).strip()
    agv_requirements = task.get("agvRequirements")
    if not isinstance(agv_requirements, list):
        agv_requirements = []
    return {
        "task_id": task_id,
        "priority": _safe_int(task.get("priority"), 0),
        "pickup_node": node_ids[0] if node_ids else None,
        "delivery_node": node_ids[-1] if node_ids else None,
        "node_ids": node_ids[:8],
        "bind_robot_id": bind_robot_id,
        "agv_requirements": [str(x).strip() for x in agv_requirements if str(x).strip()],
        "expected_start_time": str(task.get("expectedStartTime", "") or ""),
    }


def _summarize_assigned_task(task: Dict[str, Any]) -> Dict[str, Any]:
    pickup_node = None
    delivery_node = None
    pickup = task.get("pickup_point")
    if isinstance(pickup, dict):
        node_id = sim.parse_node_id(pickup.get("nodeId"))
        if node_id is not None:
            pickup_node = int(node_id)
    delivery = task.get("delivery_point")
    if isinstance(delivery, dict):
        node_id = sim.parse_node_id(delivery.get("nodeId"))
        if node_id is not None:
            delivery_node = int(node_id)
    return {
        "task_id": str(task.get("task_id", "")).strip(),
        "priority": _safe_int(task.get("priority"), 0),
        "task_sequence": _safe_int(task.get("task_sequence"), 0),
        "task_type": str(task.get("task_type", "")).strip(),
        "pickup_node": pickup_node,
        "delivery_node": delivery_node,
        "expected_start_time": str(task.get("expected_start_time", "") or ""),
        "expected_completion_time": str(task.get("expected_completion_time", "") or ""),
    }


def _route_points_preview(points: List[Any], limit: int = 12) -> List[int]:
    node_ids: List[int] = []
    for pt in points[: max(1, int(limit))]:
        node_id = getattr(pt, "node_id", None)
        parsed = sim.parse_node_id(node_id)
        if parsed is None:
            continue
        value = int(parsed)
        if not node_ids or node_ids[-1] != value:
            node_ids.append(value)
    return node_ids


def _normalize_reserved_nodes(nodes: List[Any]) -> List[Dict[str, Any]]:
    normalized: List[Dict[str, Any]] = []
    for item in nodes or []:
        raw = item if isinstance(item, dict) else {"nodeId": item}
        node_id = sim.parse_node_id(raw.get("nodeId"))
        if node_id is None:
            continue
        normalized.append(
            {
                "nodeId": int(node_id),
                "owner": str(raw.get("owner", "") or ""),
                "reason": _safe_int(raw.get("reason"), 99),
                "reasonLabel": _reason_label(raw.get("reason")),
                "detail": str(raw.get("detail", "") or ""),
            }
        )
    normalized.sort(key=lambda x: (x["nodeId"], x["owner"], x["reason"], x["detail"]))
    return normalized


class ReplayRecorder:
    def __init__(
        self,
        root_dir: Path,
        session_name: str,
        map_payload: Dict[str, Any],
        *,
        record_interval_s: float,
        record_max_points: int,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        root_dir.mkdir(parents=True, exist_ok=True)
        safe_name = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in session_name.strip())
        if not safe_name:
            safe_name = time.strftime("replay_%Y%m%d_%H%M%S", time.localtime())
        session_dir = root_dir / safe_name
        suffix = 1
        while session_dir.exists():
            session_dir = root_dir / f"{safe_name}_{suffix:02d}"
            suffix += 1
        session_dir.mkdir(parents=True, exist_ok=True)

        self.session_dir = session_dir
        self.record_interval_s = max(0.02, float(record_interval_s))
        self.record_max_points = max(10, int(record_max_points))
        self._frames_fp = (self.session_dir / "frames.jsonl").open("w", encoding="utf-8")
        self._events_fp = (self.session_dir / "events.jsonl").open("w", encoding="utf-8")
        self._last_frame_sim_s: Optional[float] = None
        self._frame_count = 0
        self._event_count = 0
        self._last_event_sim_s = 0.0
        self._meta_flush_interval_s = max(0.5, _safe_float(env("SIM_REPLAY_META_FLUSH_SEC", "3.0"), 3.0))
        self._last_meta_flush_monotonic = time.monotonic()
        self._last_path_signature: Dict[str, Tuple[int, ...]] = {}
        self._last_trail_signature: Dict[str, Tuple[int, ...]] = {}
        self._last_reserved_signature = ""
        self._session_meta: Dict[str, Any] = {
            "session_name": session_dir.name,
            "created_at": _iso_now_local(),
            "record_interval_ms": int(round(self.record_interval_s * 1000.0)),
            "record_max_points": self.record_max_points,
            "files": {
                "map": "map.json",
                "frames": "frames.jsonl",
                "events": "events.jsonl",
            },
        }
        if metadata:
            self._session_meta.update(metadata)
        (self.session_dir / "map.json").write_text(
            json.dumps(map_payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self._write_session_meta()

    def _write_session_meta(self) -> None:
        session_json = dict(self._session_meta)
        session_json["frame_count"] = int(self._frame_count)
        session_json["event_count"] = int(self._event_count)
        session_json["last_sim_time_s"] = float(self._last_event_sim_s)
        (self.session_dir / "session.json").write_text(
            json.dumps(session_json, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def _maybe_flush_session_meta(self, *, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self._last_meta_flush_monotonic) < self._meta_flush_interval_s:
            return
        self._write_session_meta()
        self._last_meta_flush_monotonic = now

    def _write_jsonl(self, fp: Any, payload: Dict[str, Any]) -> None:
        fp.write(json.dumps(payload, ensure_ascii=False) + "\n")
        fp.flush()

    def record_tasks_published(self, payload: Dict[str, Any], *, sim_time_s: float, source: str) -> None:
        tasks = payload.get("candidateTasks")
        if not isinstance(tasks, list):
            return
        summary = [_summarize_candidate_task(task) for task in tasks if isinstance(task, dict)]
        if not summary:
            return
        self.record_event(
            "tasks_published",
            sim_time_s=sim_time_s,
            source=source,
            scheduling_request_id=str(payload.get("schedulingRequestId", "") or ""),
            task_count=len(summary),
            tasks=summary[:24],
        )

    def record_assignment_response(self, payload: Dict[str, Any], *, sim_time_s: float) -> None:
        assignments = payload.get("agv_assignments")
        if not isinstance(assignments, list):
            return
        normalized: List[Dict[str, Any]] = []
        for agv_entry in assignments:
            if not isinstance(agv_entry, dict):
                continue
            tasks = agv_entry.get("assigned_tasks")
            if not isinstance(tasks, list):
                continue
            task_summaries = [_summarize_assigned_task(task) for task in tasks if isinstance(task, dict)]
            normalized.append(
                {
                    "agv_id": str(agv_entry.get("agv_id", "")).strip(),
                    "estimated_start_time": str(agv_entry.get("estimated_start_time", "") or ""),
                    "estimated_completion_time": str(agv_entry.get("estimated_completion_time", "") or ""),
                    "tasks": task_summaries,
                }
            )
        self.record_event(
            "assignment",
            sim_time_s=sim_time_s,
            message_id=str(payload.get("messageId", "") or ""),
            allocator=str((payload.get("allocation_metadata") or {}).get("allocator", "") or ""),
            assignments=normalized,
            unassigned_tasks=list(payload.get("unassigned_tasks") or []),
        )

    def record_path_response(self, payload: Dict[str, Any], points: List[Any], *, sim_time_s: float) -> None:
        device_id = str(payload.get("deviceId", "") or "").strip()
        if not device_id:
            return
        signature = tuple(_route_points_preview(points, limit=16))
        if signature == self._last_path_signature.get(device_id):
            return
        self._last_path_signature[device_id] = signature
        self.record_event(
            "path_update",
            sim_time_s=sim_time_s,
            device_id=device_id,
            node_ids=list(signature),
            point_count=len(points),
            sub_task_id=str(payload.get("subTaskId", "") or ""),
            task_priority=_safe_int(payload.get("taskPriority"), 0),
        )

    def record_trail_response(self, payload: Dict[str, Any], points: List[Any], *, sim_time_s: float) -> None:
        device_id = str(payload.get("deviceId", "") or "").strip()
        if not device_id:
            return
        signature = tuple(_route_points_preview(points, limit=16))
        if signature == self._last_trail_signature.get(device_id):
            return
        self._last_trail_signature[device_id] = signature
        self.record_event(
            "trail_update",
            sim_time_s=sim_time_s,
            device_id=device_id,
            node_ids=list(signature),
            point_count=len(points),
            sub_task_id=str(payload.get("subTaskId", "") or ""),
            task_priority=_safe_int(payload.get("taskPriority"), 0),
        )

    def record_reserved_snapshot(self, payload: Dict[str, Any], *, sim_time_s: float) -> None:
        reserved = payload.get("reservedNodes")
        if not isinstance(reserved, list):
            return
        normalized = _normalize_reserved_nodes(reserved)
        signature = json.dumps(normalized, ensure_ascii=True, sort_keys=True)
        if signature == self._last_reserved_signature:
            return
        self._last_reserved_signature = signature
        self.record_event(
            "reservation_snapshot",
            sim_time_s=sim_time_s,
            reservation_count=len(normalized),
            reserved_nodes=normalized[:80],
        )

    def record_event(self, kind: str, *, sim_time_s: float, **payload: Any) -> None:
        self._last_event_sim_s = max(self._last_event_sim_s, float(sim_time_s))
        event = {
            "event_index": int(self._event_count),
            "kind": str(kind),
            "sim_time_s": float(sim_time_s),
        }
        event.update(payload)
        self._write_jsonl(self._events_fp, event)
        self._event_count += 1
        self._maybe_flush_session_meta()

    def maybe_record_frame(self, payload: Dict[str, Any]) -> None:
        sim_time_s = _safe_float(payload.get("sim_time_s"), 0.0)
        if self._last_frame_sim_s is not None and (sim_time_s - self._last_frame_sim_s) < (self.record_interval_s - 1e-9):
            return
        frame = dict(payload)
        frame["frame_index"] = int(self._frame_count)
        self._write_jsonl(self._frames_fp, frame)
        self._last_frame_sim_s = sim_time_s
        self._last_event_sim_s = max(self._last_event_sim_s, sim_time_s)
        self._frame_count += 1
        self._maybe_flush_session_meta()

    def close(self) -> None:
        try:
            self._frames_fp.close()
        finally:
            try:
                self._events_fp.close()
            finally:
                self._maybe_flush_session_meta(force=True)


class TaskPool:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._tasks_by_id: Dict[str, Dict[str, Any]] = {}
        self._created_ts_by_id: Dict[str, str] = {}

    def size(self) -> int:
        with self._lock:
            return len(self._tasks_by_id)

    def add_payload(self, payload: Dict[str, Any]) -> int:
        if not isinstance(payload, dict):
            return 0
        tasks = payload.get("candidateTasks", [])
        if not isinstance(tasks, list):
            return 0
        added = 0
        with self._lock:
            for task in tasks:
                if not isinstance(task, dict):
                    continue
                task_id = str(task.get("taskId", "")).strip()
                if not task_id:
                    continue
                if task_id in self._tasks_by_id:
                    existing = self._tasks_by_id[task_id]
                    ts = str(existing.get("timestamp", "")).strip()
                    if not ts:
                        ts = str(self._created_ts_by_id.get(task_id, "")).strip()
                    if not ts:
                        ts = sim.iso_timestamp()
                    existing["timestamp"] = ts
                    self._created_ts_by_id[task_id] = ts
                    continue
                new_task = dict(task)
                ts = str(new_task.get("timestamp", "")).strip()
                if not ts:
                    ts = sim.iso_timestamp()
                    new_task["timestamp"] = ts
                self._tasks_by_id[task_id] = new_task
                self._created_ts_by_id[task_id] = ts
                added += 1
        return added

    def remove_assigned(self, payload: Dict[str, Any]) -> int:
        if not isinstance(payload, dict):
            return 0
        assignments = payload.get("agv_assignments")
        if not isinstance(assignments, list):
            return 0
        assigned_task_ids: Set[str] = set()
        for agv_entry in assignments:
            if not isinstance(agv_entry, dict):
                continue
            tasks = agv_entry.get("assigned_tasks")
            if not isinstance(tasks, list):
                continue
            for task in tasks:
                if not isinstance(task, dict):
                    continue
                task_id = str(task.get("task_id", "")).strip()
                if not task_id:
                    task_id = str(task.get("taskId", "")).strip()
                if task_id:
                    assigned_task_ids.add(task_id)
        if not assigned_task_ids:
            return 0
        removed = 0
        with self._lock:
            for task_id in assigned_task_ids:
                if task_id in self._tasks_by_id:
                    self._tasks_by_id.pop(task_id, None)
                    self._created_ts_by_id.pop(task_id, None)
                    removed += 1
        return removed

    def build_payload(
        self,
        base_payload: Optional[Dict[str, Any]],
        *,
        forbidden_end_nodes: Optional[Set[int]] = None,
        occupied_by_node: Optional[Dict[int, str]] = None,
        allocation_algo: Optional[str] = None,
    ) -> Dict[str, Any]:
        payload: Dict[str, Any] = dict(base_payload or {})
        with self._lock:
            tasks = list(self._tasks_by_id.values())
        def task_bound_device(task: Dict[str, Any]) -> str:
            bind_robot_id = str(task.get("bindRobotId", "")).strip()
            if bind_robot_id:
                return bind_robot_id
            agv_reqs = task.get("agvRequirements")
            if isinstance(agv_reqs, list) and len(agv_reqs) == 1:
                return str(agv_reqs[0]).strip()
            return ""

        def task_end_node(task: Dict[str, Any]) -> Optional[int]:
            sub_tasks = task.get("subTasks")
            if not isinstance(sub_tasks, list) or not sub_tasks:
                return None
            last = sub_tasks[-1]
            if not isinstance(last, dict):
                return None
            point = last.get("point")
            if isinstance(point, dict):
                node_id = sim.parse_node_id(point.get("nodeId"))
                if node_id is not None:
                    return int(node_id)
            node_id = sim.parse_node_id(last.get("nodeId"))
            if node_id is not None:
                return int(node_id)
            return None

        if env_bool("SIM_FILTER_OCCUPIED_TASKS", True) and forbidden_end_nodes:
            filtered: List[Dict[str, Any]] = []
            for task in tasks:
                if not isinstance(task, dict):
                    continue
                end_node = task_end_node(task)
                if end_node is None or end_node not in forbidden_end_nodes:
                    filtered.append(task)
                    continue
                bound_device = task_bound_device(task)
                if bound_device and occupied_by_node and occupied_by_node.get(end_node) == bound_device:
                    filtered.append(task)
            tasks = filtered

        if env_bool("SIM_FILTER_TASK_OVERLAP", True):
            filtered = []
            used_global: Set[int] = set()
            used_by_owner: Dict[str, Set[int]] = {}
            for task in tasks:
                if not isinstance(task, dict):
                    continue
                end_node = task_end_node(task)
                if end_node is None:
                    filtered.append(task)
                    continue
                owner = task_bound_device(task)
                if owner:
                    owner_nodes = used_by_owner.setdefault(owner, set())
                    if end_node in owner_nodes:
                        filtered.append(task)
                        continue
                    if end_node in used_global:
                        continue
                    owner_nodes.add(end_node)
                    used_global.add(end_node)
                    filtered.append(task)
                else:
                    if end_node in used_global:
                        continue
                    used_global.add(end_node)
                    filtered.append(task)
            tasks = filtered
        payload["candidateTasks"] = tasks
        payload["taskNumber"] = len(tasks)
        batch = payload.get("batchConfig")
        if isinstance(batch, dict):
            batch = dict(batch)
            batch["maxBatchSize"] = len(tasks)
            payload["batchConfig"] = batch
        else:
            payload["batchConfig"] = {"maxBatchSize": len(tasks)}
        payload["requestTimestamp"] = sim.iso_timestamp()
        payload["schedulingRequestId"] = f"SIM_POOL_REQ_{int(time.time() * 1000)}"
        if allocation_algo:
            payload["allocationAlgorithm"] = str(allocation_algo)
        return payload


def apply_bind_tasks_ratio(payload: Dict[str, Any], rng: random.Random, ratio: float) -> int:
    ratio = max(0.0, min(1.0, float(ratio)))
    if ratio >= 1.0:
        return 0
    tasks = payload.get("candidateTasks", [])
    if not isinstance(tasks, list):
        return 0
    cleared = 0
    for task in tasks:
        if not isinstance(task, dict):
            continue
        bind_robot_id = str(task.get("bindRobotId", "")).strip()
        agv_reqs = task.get("agvRequirements")
        has_req = isinstance(agv_reqs, list) and bool(agv_reqs)
        if not bind_robot_id and not has_req:
            continue
        if rng.random() >= ratio:
            task["agvRequirements"] = []
            if "bindRobotId" in task:
                task["bindRobotId"] = ""
            cleared += 1
    return cleared


def load_map_raw(map_path: Path) -> Dict[str, Any]:
    data = json.loads(map_path.read_text(encoding="utf-8"))
    if isinstance(data, dict) and "mapData" in data and isinstance(data["mapData"], dict):
        data = data["mapData"]
    if not isinstance(data, dict):
        return {}
    return data


def load_json_path(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _auto_region_grid(agv_count: int, passable_nodes: int) -> int:
    min_grid = 3
    max_grid = 6
    env_min = _safe_int(env("CONGESTION_REGION_GRID_MIN", "0"), 0)
    env_max = _safe_int(env("CONGESTION_REGION_GRID_MAX", "0"), 0)
    if env_min > 0:
        min_grid = env_min
    if env_max > 0:
        max_grid = env_max
    if max_grid < min_grid:
        max_grid = min_grid
    if passable_nodes > 0:
        min_grid = min(min_grid, max(1, int(math.floor(math.sqrt(passable_nodes)))))
    grid = int(round(math.sqrt(max(1, int(agv_count)))))
    target_nodes = _safe_int(env("CONGESTION_REGION_TARGET_NODES", "25"), 0)
    if target_nodes > 0 and passable_nodes > 0:
        max_regions = max(1, int(passable_nodes / max(1, target_nodes)))
        grid_by_nodes = int(math.floor(math.sqrt(max_regions)))
        if grid_by_nodes > 0:
            grid = min(grid, grid_by_nodes)
    return max(min_grid, min(max_grid, grid))


def _node_passable(node: Dict[str, Any]) -> bool:
    if int(node.get("id", -1)) < 0:
        return False
    if int(node.get("type", 0)) == -1:
        return False
    allow_pass = node.get("allowPass", None)
    if allow_pass is None:
        return True
    try:
        return int(allow_pass) != 0
    except Exception:
        return True


def _build_graph_regions(nodes: List[Dict[str, Any]],
                         edges: List[Dict[str, Any]],
                         target_regions: int) -> Tuple[List[int], int]:
    count = len(nodes)
    region_by_index = [-1 for _ in range(count)]
    if count == 0 or target_regions <= 0:
        return region_by_index, 0

    candidates = [i for i, n in enumerate(nodes) if _node_passable(n)]
    if not candidates:
        return region_by_index, 0
    if target_regions > len(candidates):
        target_regions = len(candidates)

    first = min(candidates, key=lambda i: nodes[i]["x"] + nodes[i]["y"])
    seeds = [first]

    while len(seeds) < target_regions:
        best_idx = None
        best_dist = -1.0
        for idx in candidates:
            nx = nodes[idx]["x"]
            ny = nodes[idx]["y"]
            min_dist = float("inf")
            for seed_idx in seeds:
                sx = nodes[seed_idx]["x"]
                sy = nodes[seed_idx]["y"]
                d2 = (nx - sx) * (nx - sx) + (ny - sy) * (ny - sy)
                if d2 < min_dist:
                    min_dist = d2
            if min_dist > best_dist:
                best_dist = min_dist
                best_idx = idx
        if best_idx is None:
            break
        seeds.append(best_idx)

    id_to_index = {int(n["id"]): i for i, n in enumerate(nodes)}
    adj: List[Set[int]] = [set() for _ in range(count)]
    for e in edges:
        a = e.get("startNode")
        b = e.get("endNode")
        if a is None or b is None:
            continue
        ia = id_to_index.get(int(a))
        ib = id_to_index.get(int(b))
        if ia is None or ib is None or ia == ib:
            continue
        adj[ia].add(ib)
        adj[ib].add(ia)

    q = deque()
    for ridx, idx in enumerate(seeds):
        region_by_index[idx] = ridx
        q.append(idx)
    while q:
        cur = q.popleft()
        reg = region_by_index[cur]
        for nb in adj[cur]:
            if region_by_index[nb] >= 0:
                continue
            if not _node_passable(nodes[nb]):
                continue
            region_by_index[nb] = reg
            q.append(nb)

    for idx in candidates:
        if region_by_index[idx] >= 0:
            continue
        nx = nodes[idx]["x"]
        ny = nodes[idx]["y"]
        best_dist = float("inf")
        best_reg = 0
        for ridx, seed_idx in enumerate(seeds):
            sx = nodes[seed_idx]["x"]
            sy = nodes[seed_idx]["y"]
            d2 = (nx - sx) * (nx - sx) + (ny - sy) * (ny - sy)
            if d2 < best_dist:
                best_dist = d2
                best_reg = ridx
        region_by_index[idx] = best_reg

    return region_by_index, len(seeds)


def _build_passable_adjacency(nodes: List[Dict[str, Any]],
                              edges: List[Dict[str, Any]]) -> Tuple[Dict[int, int], List[Set[int]]]:
    id_to_index = {int(n["id"]): i for i, n in enumerate(nodes)}
    adj: List[Set[int]] = [set() for _ in range(len(nodes))]
    for e in edges:
        a = e.get("startNode")
        b = e.get("endNode")
        if a is None or b is None:
            continue
        ia = id_to_index.get(int(a))
        ib = id_to_index.get(int(b))
        if ia is None or ib is None or ia == ib:
            continue
        adj[ia].add(ib)
        adj[ib].add(ia)
    return id_to_index, adj


def _collect_bridge_region_groups(nodes: List[Dict[str, Any]],
                                  adj: List[Set[int]]) -> List[List[int]]:
    count = len(nodes)
    if count <= 0:
        return []

    bridge_core = [False for _ in range(count)]
    for idx, node in enumerate(nodes):
        if not _node_passable(node):
            continue
        degree = sum(1 for nb in adj[idx] if 0 <= nb < count and _node_passable(nodes[nb]))
        if degree == 2:
            bridge_core[idx] = True

    comp_by_index = [-1 for _ in range(count)]
    core_nodes_by_comp: List[List[int]] = []
    for idx in range(count):
        if not bridge_core[idx]:
            continue
        if comp_by_index[idx] >= 0:
            continue
        comp_id = len(core_nodes_by_comp)
        core_nodes_by_comp.append([])
        q = deque([idx])
        comp_by_index[idx] = comp_id
        while q:
            cur = q.popleft()
            core_nodes_by_comp[comp_id].append(cur)
            for nb in adj[cur]:
                if nb < 0 or nb >= count or not bridge_core[nb]:
                    continue
                if comp_by_index[nb] >= 0:
                    continue
                comp_by_index[nb] = comp_id
                q.append(nb)

    if not core_nodes_by_comp:
        return []

    min_nodes = max(1, _safe_int(env("CONGESTION_REGION_BRIDGE_MIN_NODES", "4"), 4))
    ordered: List[List[int]] = []
    for core_nodes in core_nodes_by_comp:
        if len(core_nodes) < min_nodes:
            continue
        core_set = set(core_nodes)
        endpoint_core_nodes: List[int] = []
        for idx in core_nodes:
            core_degree = sum(1 for nb in adj[idx] if nb in core_set)
            if core_degree <= 1:
                endpoint_core_nodes.append(idx)
        merged = set(core_nodes)
        for idx in endpoint_core_nodes:
            for nb in adj[idx]:
                if nb < 0 or nb >= count:
                    continue
                if not _node_passable(nodes[nb]):
                    continue
                if nb in core_set:
                    continue
                merged.add(nb)
        ordered.append(sorted(merged))
    ordered.sort(key=lambda items: items[0] if items else -1)
    return ordered


def _build_bridge_overlay(nodes: List[Dict[str, Any]],
                          edges: List[Dict[str, Any]]) -> Tuple[List[int], List[Dict[str, int]]]:
    if not env_bool("CONGESTION_REGION_BRIDGE_ENABLE", True):
        return [], []
    id_to_index, adj = _build_passable_adjacency(nodes, edges)
    groups = _collect_bridge_region_groups(nodes, adj)
    if not groups:
        return [], []

    bridge_indices: Set[int] = set()
    for group in groups:
        bridge_indices.update(group)

    bridge_node_ids = sorted(int(nodes[idx]["id"]) for idx in bridge_indices if 0 <= idx < len(nodes))
    bridge_node_set = set(bridge_node_ids)
    bridge_edges: List[Dict[str, int]] = []
    seen_edges: Set[Tuple[int, int]] = set()
    for e in edges:
        a = e.get("startNode")
        b = e.get("endNode")
        if a is None or b is None:
            continue
        a = int(a)
        b = int(b)
        if a == b:
            continue
        if a not in bridge_node_set or b not in bridge_node_set:
            continue
        key = (a, b) if a < b else (b, a)
        if key in seen_edges:
            continue
        seen_edges.add(key)
        bridge_edges.append({"startNode": key[0], "endNode": key[1]})
    return bridge_node_ids, bridge_edges


def _compute_region_stats(nodes: List[Dict[str, Any]],
                          region_by_index: List[int],
                          region_count: int,
                          agv_count: int) -> Tuple[List[Dict[str, Any]], int, int]:
    if region_count <= 0:
        return [], 0, 0
    region_node_counts = [0 for _ in range(region_count)]
    total_nodes = 0
    limit_len = min(len(nodes), len(region_by_index))
    for i in range(limit_len):
        reg = region_by_index[i]
        if reg < 0 or reg >= region_count:
            continue
        if not _node_passable(nodes[i]):
            continue
        region_node_counts[reg] += 1
        total_nodes += 1
    effective = sum(1 for c in region_node_counts if c > 0)
    if effective <= 0:
        effective = max(1, region_count)
    avg = float(agv_count) / float(effective)
    soft_limit = _safe_int(env("CONGESTION_REGION_SOFT", "0"), 0)
    hard_limit = _safe_int(env("CONGESTION_REGION_HARD", "0"), 0)
    if soft_limit <= 0:
        soft_limit = max(2, int(math.ceil(avg)) + 1)
    if hard_limit <= 0:
        bump = max(1, int(math.ceil(avg / 2.0)))
        hard_limit = soft_limit + bump
    if hard_limit <= soft_limit:
        hard_limit = soft_limit + 1
    avg_nodes = float(total_nodes) / float(max(1, effective))
    stats: List[Dict[str, Any]] = []
    for r in range(region_count):
        node_count = region_node_counts[r]
        if node_count <= 0:
            s = 0
            h = 0
        else:
            factor = (float(node_count) / avg_nodes) if avg_nodes > 0 else 1.0
            s = max(1, int(round(soft_limit * factor)))
            h = max(1, int(round(hard_limit * factor)))
            if h <= s:
                h = s + 1
        stats.append({"id": r, "nodeCount": node_count, "softLimit": s, "hardLimit": h})
    return stats, int(soft_limit), int(hard_limit)


def build_web_map_payload(map_raw: Dict[str, Any], agv_count: int) -> Dict[str, Any]:
    nodes = map_raw.get("node") or map_raw.get("nodes") or []
    edges = map_raw.get("edge") or map_raw.get("edges") or []
    node_meta: List[Dict[str, Any]] = []
    for n in nodes:
        if not isinstance(n, dict):
            continue
        nid = sim.parse_node_id(n.get("id", n.get("nodeId")))
        if nid is None:
            continue
        node_meta.append(
            {
                "id": int(nid),
                "x": _safe_float(n.get("x")),
                "y": _safe_float(n.get("y")),
                "type": _safe_int(n.get("type"), 0),
                "allowPass": n.get("allowPass"),
            }
        )

    out_edges: List[Dict[str, Any]] = []
    for e in edges:
        if not isinstance(e, dict):
            continue
        a = sim.parse_node_id(e.get("startId", e.get("startNodeId", e.get("startNode"))))
        b = sim.parse_node_id(e.get("endId", e.get("endNodeId", e.get("endNode"))))
        if a is None or b is None or a == b:
            continue
        out_edges.append({"startNode": int(a), "endNode": int(b)})

    bridge_node_ids, bridge_edges = _build_bridge_overlay(node_meta, out_edges)

    passable_nodes = sum(1 for n in node_meta if _node_passable(n))

    mode = env("CONGESTION_REGION_MODE", "graph").strip().lower()
    if mode not in ("grid", "graph", "cluster", "partition"):
        mode = "grid"
    grid = _safe_int(env("CONGESTION_REGION_GRID", "0"), 0)
    if grid <= 0:
        grid = _auto_region_grid(int(agv_count), passable_nodes)
    region_count = 0
    region_by_index: List[int] = []
    if grid > 0 and node_meta:
        if mode in ("graph", "cluster", "partition"):
            target_regions = _safe_int(env("CONGESTION_REGION_COUNT", "0"), 0)
            if target_regions <= 0:
                target_regions = grid * grid
            region_by_index, region_count = _build_graph_regions(node_meta, out_edges, target_regions)
        else:
            xs = [n["x"] for n in node_meta]
            ys = [n["y"] for n in node_meta]
            min_x = min(xs) if xs else 0.0
            max_x = max(xs) if xs else 1.0
            min_y = min(ys) if ys else 0.0
            max_y = max(ys) if ys else 1.0
            span_x = max(1.0, max_x - min_x)
            span_y = max(1.0, max_y - min_y)
            cell_w = span_x / float(grid)
            cell_h = span_y / float(grid)
            region_by_index = [-1 for _ in range(len(node_meta))]
            for i, n in enumerate(node_meta):
                if not _node_passable(n):
                    continue
                col = int((n["x"] - min_x) / cell_w) if cell_w > 0 else 0
                row = int((n["y"] - min_y) / cell_h) if cell_h > 0 else 0
                col = max(0, min(grid - 1, col))
                row = max(0, min(grid - 1, row))
                region_by_index[i] = row * grid + col
            region_count = grid * grid

    region_stats: List[Dict[str, Any]] = []
    region_soft = 0
    region_hard = 0
    if region_count > 0 and region_by_index:
        region_stats, region_soft, region_hard = _compute_region_stats(
            node_meta, region_by_index, region_count, int(agv_count)
        )

    out_nodes: List[Dict[str, Any]] = []
    node_positions: Dict[str, Dict[str, Any]] = {}
    for idx, n in enumerate(node_meta):
        region = region_by_index[idx] if idx < len(region_by_index) else -1
        out_nodes.append({"id": n["id"], "x": n["x"], "y": n["y"], "type": n["type"], "region": region})
        node_positions[str(n["id"])] = {"x": n["x"], "y": n["y"], "type": n["type"], "region": region}

    info = map_raw.get("info") if isinstance(map_raw.get("info"), dict) else {}
    max_speed = _safe_int(info.get("maxSpeed"), 1000)
    return {
        "info": {
            "name": str(info.get("name", "")),
            "mapId": _safe_int(info.get("mapId"), 0),
            "version": _safe_int(info.get("version"), 0),
            "maxSpeed": max(1, int(max_speed)),
        },
        "nodes": out_nodes,
        "edges": out_edges,
        "nodePositions": node_positions,
        "regions": {
            "mode": mode,
            "grid": grid,
            "count": int(region_count),
            "softLimit": int(region_soft),
            "hardLimit": int(region_hard),
            "stats": region_stats,
        },
        "bridgeNodeIds": bridge_node_ids,
        "bridgeEdges": bridge_edges,
        "bridgeNodeSource": "rule" if bridge_node_ids else "",
    }


@dataclass
class SegmentProfile:
    length_m: float
    v0: float
    v1: float
    v_peak: float
    t_acc: float
    t_cruise: float
    t_dec: float
    t_total: float
    accel: float
    decel: float

    def dist_at(self, t: float) -> float:
        t = max(0.0, min(self.t_total, float(t)))
        if self.length_m <= 1e-12:
            return 0.0
        if t <= self.t_acc + 1e-9:
            return self.v0 * t + 0.5 * self.accel * t * t
        t2 = t - self.t_acc
        d_acc = self.v0 * self.t_acc + 0.5 * self.accel * self.t_acc * self.t_acc
        if t2 <= self.t_cruise + 1e-9:
            return d_acc + self.v_peak * t2
        t3 = t2 - self.t_cruise
        d_cruise = self.v_peak * self.t_cruise
        return d_acc + d_cruise + self.v_peak * t3 - 0.5 * self.decel * t3 * t3

    def speed_at(self, t: float) -> float:
        t = max(0.0, min(self.t_total, float(t)))
        if t <= self.t_acc + 1e-9:
            return max(0.0, self.v0 + self.accel * t)
        t2 = t - self.t_acc
        if t2 <= self.t_cruise + 1e-9:
            return max(0.0, self.v_peak)
        t3 = t2 - self.t_cruise
        return max(0.0, self.v_peak - self.decel * t3)


def _build_segment_profile(
    length_m: float,
    v0: float,
    v1: float,
    v_max: float,
    accel: float,
    decel: float,
) -> SegmentProfile:
    d = max(0.0, float(length_m))
    v0 = max(0.0, float(v0))
    v1 = max(0.0, float(v1))
    v_max = max(0.0, float(v_max))
    accel = max(1e-6, float(accel))
    decel = max(1e-6, float(decel))

    if d <= 1e-9:
        return SegmentProfile(d, v0, v1, max(v0, v1), 0.0, 0.0, 0.0, 0.0, accel, decel)

    v_floor = max(v0, v1)

    v_peak_sq = (2.0 * d + (v0 * v0) / accel + (v1 * v1) / decel) / (1.0 / accel + 1.0 / decel)
    v_peak_tri = math.sqrt(max(0.0, v_peak_sq))
    if v_peak_tri < v_floor:
        v_peak_tri = v_floor

    if v_peak_tri <= v_max + 1e-9:
        v_peak = max(v_floor, v_peak_tri)
        t_acc = max(0.0, (v_peak - v0) / accel)
        t_dec = max(0.0, (v_peak - v1) / decel)
        t_total = t_acc + t_dec
        return SegmentProfile(d, v0, v1, v_peak, t_acc, 0.0, t_dec, t_total, accel, decel)

    v_peak = max(v_floor, v_max)
    t_acc = max(0.0, (v_peak - v0) / accel)
    t_dec = max(0.0, (v_peak - v1) / decel)
    d_acc = (v_peak * v_peak - v0 * v0) / (2.0 * accel) if v_peak > v0 else 0.0
    d_dec = (v_peak * v_peak - v1 * v1) / (2.0 * decel) if v_peak > v1 else 0.0
    d_cruise = max(0.0, d - d_acc - d_dec)
    t_cruise = d_cruise / max(1e-9, v_peak)
    t_total = t_acc + t_cruise + t_dec
    return SegmentProfile(d, v0, v1, v_peak, t_acc, t_cruise, t_dec, t_total, accel, decel)


def _turn_angle_deg(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> float:
    ax, ay = a
    bx, by = b
    cx, cy = c
    v1x = bx - ax
    v1y = by - ay
    v2x = cx - bx
    v2y = cy - by
    l1 = math.hypot(v1x, v1y)
    l2 = math.hypot(v2x, v2y)
    if l1 <= 1e-9 or l2 <= 1e-9:
        return 0.0
    dot = (v1x * v2x + v1y * v2y) / (l1 * l2)
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))


def _heading_deg(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    ax, ay = a
    bx, by = b
    if abs(ax - bx) <= 1e-9 and abs(ay - by) <= 1e-9:
        return 0.0
    return (math.degrees(math.atan2(by - ay, bx - ax)) + 360.0) % 360.0


def _angle_diff_deg(a_deg: float, b_deg: float) -> float:
    delta = (float(b_deg) - float(a_deg) + 540.0) % 360.0 - 180.0
    return abs(delta)


def _interp_angle_deg(a_deg: float, b_deg: float, t: float) -> float:
    t = max(0.0, min(1.0, float(t)))
    delta = (float(b_deg) - float(a_deg) + 540.0) % 360.0 - 180.0
    return (float(a_deg) + delta * t + 360.0) % 360.0


def _node_speed_limits(
    route: List[Any],
    v_max: float,
    turn_slowdown: float,
    min_factor: float,
    *,
    turn_stop_angle_deg: float,
) -> List[float]:
    n = len(route)
    if n <= 0:
        return []
    out = [v_max for _ in range(n)]
    out[0] = 0.0
    if n > 1:
        out[-1] = 0.0
    for i in range(1, n - 1):
        a = (float(route[i - 1].x), float(route[i - 1].y))
        b = (float(route[i].x), float(route[i].y))
        c = (float(route[i + 1].x), float(route[i + 1].y))
        ang = _turn_angle_deg(a, b, c)
        if ang >= max(0.0, float(turn_stop_angle_deg)):
            out[i] = 0.0
            continue
        factor = 1.0 - (max(0.0, min(180.0, ang)) / 180.0) * max(0.0, float(turn_slowdown))
        factor = max(float(min_factor), min(1.0, factor))
        out[i] = min(out[i], v_max * factor)
    return out


def _forward_backward_speeds(
    dist_m: List[float],
    node_vmax: List[float],
    accel: float,
    decel: float,
    *,
    start_speed: Optional[float] = None,
) -> List[float]:
    n = len(node_vmax)
    if n == 0:
        return []
    v = [max(0.0, float(x)) for x in node_vmax]
    if start_speed is not None:
        v[0] = max(0.0, float(start_speed))
    accel = max(1e-6, float(accel))
    decel = max(1e-6, float(decel))
    for i in range(n - 1):
        d = max(0.0, float(dist_m[i]))
        v[i + 1] = min(v[i + 1], math.sqrt(max(0.0, v[i] * v[i] + 2.0 * accel * d)))
    for i in range(n - 2, -1, -1):
        d = max(0.0, float(dist_m[i]))
        v[i] = min(v[i], math.sqrt(max(0.0, v[i + 1] * v[i + 1] + 2.0 * decel * d)))
    return v


@dataclass
class KinematicDriveState:
    route: List[Any]
    source: str
    idx: int = 0
    phase: str = "move"  # "move" | "turn"
    seg_t: float = 0.0
    seg_profile: Optional[SegmentProfile] = None
    v_nodes: Optional[List[float]] = None  # m/s per node
    turn_stop_angle_deg: float = 0.0
    turn_t: float = 0.0
    turn_total: float = 0.0
    turn_from_yaw: float = 0.0
    turn_to_yaw: float = 0.0

    def ensure_profile(self, v0: float, v1: float, v_max: float, accel: float, decel: float) -> SegmentProfile:
        if self.idx >= len(self.route) - 1:
            self.seg_profile = SegmentProfile(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, accel, decel)
            return self.seg_profile
        a = self.route[self.idx]
        b = self.route[self.idx + 1]
        length_m = math.hypot(float(b.x) - float(a.x), float(b.y) - float(a.y)) / 1000.0
        self.seg_profile = _build_segment_profile(length_m, v0, v1, v_max, accel, decel)
        self.seg_t = max(0.0, min(self.seg_t, self.seg_profile.t_total))
        return self.seg_profile


class SimStateV2(sim.SimState):
    def __init__(
        self,
        status_list: List[Dict[str, Any]],
        graph: Any,
        trail_preview: int,
        *,
        drive_mode: str,
        v_max_m_s: float,
        accel_m_s2: float,
        decel_m_s2: float,
        turn_slowdown: float,
        turn_min_factor: float,
        turn_stop_angle_deg: float,
        turn_rate_deg_s: float,
        turn_min_time_s: float,
        trail_end_stop: bool,
        trail_min_drive_points: int,
    ) -> None:
        super().__init__(status_list, graph, trail_preview, drive_mode=drive_mode)
        self._drive_v2: Dict[str, KinematicDriveState] = {}
        self._sim_time_s = 0.0
        self._v_max_m_s = max(0.1, float(v_max_m_s))
        self._accel_m_s2 = max(0.05, float(accel_m_s2))
        self._decel_m_s2 = max(0.05, float(decel_m_s2))
        self._turn_slowdown = max(0.0, float(turn_slowdown))
        self._turn_min_factor = max(0.05, min(1.0, float(turn_min_factor)))
        self._turn_stop_angle_deg = max(0.0, float(turn_stop_angle_deg))
        self._turn_rate_deg_s = max(0.0, float(turn_rate_deg_s))
        self._turn_min_time_s = max(0.0, float(turn_min_time_s))
        self._trail_end_stop = bool(trail_end_stop)
        self._trail_min_drive_points = max(2, int(trail_min_drive_points))
        self._prev_speed_by_id: Dict[str, float] = {}
        self._last_stop_log_ts: Dict[str, float] = {}
        self._stop_log_speed_eps = max(0.0, float(env("SIM_STOP_LOG_SPEED_EPS", "1.0")))
        self._stop_log_interval_s = max(0.0, float(env("SIM_STOP_LOG_INTERVAL_S", "0.5")))

    def sim_time_s(self) -> float:
        with self._lock:
            return float(self._sim_time_s)

    def _metrics_now_s(self) -> float:
        return float(self._sim_time_s)

    def _clear_drive_locked(self, device_id: str) -> None:
        super()._clear_drive_locked(device_id)
        self._drive_v2.pop(device_id, None)

    def _note_speed_drop(self, device_id: str, speed: float, ds: Optional[KinematicDriveState], route: List[Any], reason: str) -> None:
        prev = self._prev_speed_by_id.get(device_id)
        self._prev_speed_by_id[device_id] = speed
        if prev is None:
            return
        if prev <= self._stop_log_speed_eps or speed > self._stop_log_speed_eps:
            return
        now = time.time()
        last = self._last_stop_log_ts.get(device_id, 0.0)
        if now - last < self._stop_log_interval_s:
            return
        self._last_stop_log_ts[device_id] = now
        idx = ds.idx if ds is not None else -1
        source = ds.source if ds is not None else "n/a"
        phase = ds.phase if ds is not None else "n/a"
        print(
            f"[SimStop] deviceId={device_id} source={source} phase={phase} reason={reason} "
            f"route_len={len(route)} idx={idx} speed_prev={prev:.1f} speed_now={speed:.1f}"
        )

    def _limit_speed_change(self, device_id: str, target_speed: float, dt: float) -> float:
        prev = self._prev_speed_by_id.get(device_id)
        if prev is None or dt <= 1e-9:
            return max(0.0, float(target_speed))
        max_up = self._accel_m_s2 * dt * 1000.0
        max_down = self._decel_m_s2 * dt * 1000.0
        if target_speed >= prev:
            return max(0.0, min(float(target_speed), prev + max_up))
        return max(0.0, max(float(target_speed), prev - max_down))

    def _set_drive_locked(self, device_id: str, points: List[Any], *, source: str) -> None:
        if not device_id or not points:
            self._clear_drive_locked(device_id)
            return
        points = sim.dedup_consecutive(points)
        status = self.status_by_id.get(device_id, {})
        current_node = sim.parse_node_id(status.get("nodeId"))
        next_node = None
        next_dest = status.get("nextDestinationPoint")
        if isinstance(next_dest, dict):
            next_node = sim.parse_node_id(next_dest.get("nodeId"))

        idx = 0
        preserve = False
        if current_node is not None:
            if next_node is not None:
                for i in range(len(points) - 1):
                    if points[i].node_id == current_node and points[i + 1].node_id == next_node:
                        idx = i
                        preserve = True
                        break
            if not preserve:
                for i, pt in enumerate(points):
                    if pt.node_id == current_node:
                        idx = i
                        break

        idx = max(0, min(idx, len(points) - 1))
        self.drive_by_id[device_id] = points
        self.drive_source[device_id] = source
        self.drive_index[device_id] = idx

        base_v_max = self._v_max_m_s
        v_max = base_v_max
        accel = self._accel_m_s2
        decel = self._decel_m_s2

        dist_m: List[float] = []
        for i in range(len(points) - 1):
            a = points[i]
            b = points[i + 1]
            dist_m.append(math.hypot(float(b.x) - float(a.x), float(b.y) - float(a.y)) / 1000.0)
        trail_len_m = sum(dist_m)
        trail_min_points = self._trail_min_drive_points
        is_trail = source == "trail"
        trail_has_enough = is_trail and len(points) >= trail_min_points
        if is_trail and trail_len_m > 1e-6:
            brake_window_m = _safe_float(env("SIM_TRAIL_BRAKE_WINDOW_M", ""), 0.0)
            if brake_window_m <= 1e-6:
                stop_dist = (base_v_max * base_v_max) / (2.0 * max(1e-6, decel))
                brake_window_m = max(stop_dist * 2.0, stop_dist)
            if brake_window_m > 1e-6 and trail_len_m < brake_window_m:
                scale = max(0.1, min(1.0, trail_len_m / brake_window_m))
                v_max = max(0.05, base_v_max * scale)
        turn_stop_deg = self._turn_stop_angle_deg
        node_limits = _node_speed_limits(
            points,
            v_max,
            self._turn_slowdown,
            self._turn_min_factor,
            turn_stop_angle_deg=turn_stop_deg,
        )
        if is_trail and len(node_limits) > 1:
            # Short trail windows: brake to stop. Long enough: treat end as lookahead unless forced to stop.
            if not trail_has_enough:
                node_limits[-1] = 0.0
            elif not self._trail_end_stop:
                node_limits[-1] = max(0.0, float(node_limits[-2]))
        speed_mm_s = _safe_float(status.get("speed"), 0.0)
        cur_v = max(0.0, min(base_v_max, speed_mm_s / 1000.0))
        if env_bool("SIM_FORCE_START_ZERO", True) and device_id not in self._prev_speed_by_id:
            cur_v = 0.0
        v_nodes = _forward_backward_speeds(dist_m, node_limits, accel, decel, start_speed=cur_v)
        if v_nodes:
            cur_v = min(cur_v, v_nodes[min(idx, len(v_nodes) - 1)])

        existing = self._drive_v2.get(device_id)
        if (
            existing is not None
            and preserve
            and existing.idx < len(existing.route) - 1
            and idx < len(points) - 1
            and existing.route[existing.idx].node_id == points[idx].node_id
            and existing.route[existing.idx + 1].node_id == points[idx + 1].node_id
        ):
            # Preserve progress on the same segment while updating speed constraints.
            old_prof = existing.seg_profile
            old_t = float(existing.seg_t)
            existing.route = points
            existing.source = source
            existing.idx = idx
            existing.v_nodes = v_nodes
            existing.seg_profile = None
            existing.ensure_profile(cur_v, v_nodes[min(idx + 1, len(v_nodes) - 1)], v_max, accel, decel)
            ratio = 0.0
            if old_prof is not None and old_prof.length_m > 1e-9:
                ratio = max(0.0, min(1.0, old_prof.dist_at(old_t) / old_prof.length_m))
            if existing.seg_profile and existing.seg_profile.t_total > 1e-9:
                existing.seg_t = ratio * existing.seg_profile.t_total
            self._drive_v2[device_id] = existing
            return

        ds = KinematicDriveState(
            route=points,
            source=source,
            idx=idx,
            seg_t=0.0,
            seg_profile=None,
            v_nodes=v_nodes,
            turn_stop_angle_deg=turn_stop_deg,
        )
        ds.ensure_profile(cur_v, v_nodes[min(idx + 1, len(v_nodes) - 1)], v_max, accel, decel)

        # If we matched (current,next), try to preserve progress using position projection.
        if preserve and idx < len(points) - 1:
            a = points[idx]
            b = points[idx + 1]
            dx = float(b.x) - float(a.x)
            dy = float(b.y) - float(a.y)
            denom = dx * dx + dy * dy
            ratio = 0.0
            if denom > 1e-9:
                px = _safe_float(status.get("x"), float(a.x))
                py = _safe_float(status.get("y"), float(a.y))
                ratio = ((px - float(a.x)) * dx + (py - float(a.y)) * dy) / denom
                ratio = max(0.0, min(1.0, ratio))
            self.drive_progress[device_id] = ratio
            if ds.seg_profile and ds.seg_profile.t_total > 1e-9:
                # Best-effort: approximate t by scaling with ratio (keeps continuity without heavy inversion).
                ds.seg_t = ratio * ds.seg_profile.t_total
        else:
            self.drive_progress[device_id] = 0.0

        # If we are starting at a node and the heading differs, perform an in-place turn first.
        if (
            ds.idx < len(points) - 1
            and ds.seg_t <= 1e-9
            and self._turn_rate_deg_s > 1e-9
            and ds.turn_stop_angle_deg > 1e-9
        ):
            cur_yaw = _safe_float(status.get("angle"), 0.0)
            desired = _heading_deg((float(points[ds.idx].x), float(points[ds.idx].y)), (float(points[ds.idx + 1].x), float(points[ds.idx + 1].y)))
            diff = _angle_diff_deg(cur_yaw, desired)
            if diff >= ds.turn_stop_angle_deg:
                ds.phase = "turn"
                ds.turn_t = 0.0
                ds.turn_total = max(self._turn_min_time_s, diff / max(1e-9, self._turn_rate_deg_s))
                ds.turn_from_yaw = cur_yaw
                ds.turn_to_yaw = desired
                ds.seg_t = 0.0
                ds.seg_profile = None

        self._drive_v2[device_id] = ds

    def advance(self, dt: float, edge_duration: float) -> None:
        now_ts = int(time.time())
        dt = max(0.0, float(dt))
        with self._lock:
            self._sim_time_s += dt
            for device_id, status in self.status_by_id.items():
                ds = self._drive_v2.get(device_id)
                if ds is None or not ds.route:
                    status["speed"] = 0.0
                    status["updateTime"] = now_ts
                    status["nextDestinationPoint"] = {}
                    status["curTrailPoints"] = []
                    marker = self._marker_at_current_node_locked(device_id, status)
                    if marker is not None and self._start_subtask_dwell_locked(device_id, marker):
                        continue
                    self._consume_reached_subtasks_locked(device_id, status)
                    continue

                route = ds.route
                # Subtask dwell: when reaching a subtask node, stop in-place with speed=0 for a
                # configured/service duration before consuming (popping) that subtask.
                dwell_left = float(self._subtask_dwell_remaining_s_by_device.get(device_id, 0.0))
                if dwell_left > 1e-9:
                    hold_idx = max(0, min(int(ds.idx), len(route) - 1))
                    pt = route[hold_idx]
                    status["nodeId"] = str(pt.node_id)
                    status["x"] = float(pt.x)
                    status["y"] = float(pt.y)
                    status["speed"] = 0.0
                    status["updateTime"] = now_ts
                    dwell_left = max(0.0, dwell_left - dt)
                    if dwell_left <= 1e-9:
                        self._subtask_dwell_remaining_s_by_device.pop(device_id, None)
                        self._subtask_dwell_total_s_by_device.pop(device_id, None)
                        self._consume_reached_subtasks_locked(device_id, status)
                    else:
                        self._subtask_dwell_remaining_s_by_device[device_id] = dwell_left
                    if hold_idx < len(route) - 1:
                        nxt = route[hold_idx + 1]
                        status["nextDestinationPoint"] = {
                            "x": float(nxt.x),
                            "y": float(nxt.y),
                            "angle": _safe_float(status.get("angle"), float(pt.yaw)),
                            "nodeId": str(nxt.node_id),
                        }
                    else:
                        status["nextDestinationPoint"] = {}
                    self.drive_index[device_id] = hold_idx
                    self.drive_progress[device_id] = 0.0
                    if self.trail_preview > 0:
                        preview = []
                        for p in route[hold_idx + 1 : hold_idx + 1 + self.trail_preview]:
                            preview.append({"x": float(p.x), "y": float(p.y), "angle": float(p.yaw), "nodeId": str(p.node_id)})
                        status["curTrailPoints"] = preview
                    else:
                        status["curTrailPoints"] = []
                    self._note_speed_drop(device_id, float(status["speed"]), ds, route, "subtask_dwell")
                    continue

                marker = self._marker_at_current_node_locked(device_id, status)
                if ds.phase != "turn" and marker is not None and self._start_subtask_dwell_locked(device_id, marker):
                    ds.seg_t = 0.0
                    ds.seg_profile = None
                    hold_idx = max(0, min(int(ds.idx), len(route) - 1))
                    pt = route[hold_idx]
                    status["nodeId"] = str(pt.node_id)
                    status["x"] = float(pt.x)
                    status["y"] = float(pt.y)
                    status["speed"] = 0.0
                    status["updateTime"] = now_ts
                    self._note_speed_drop(device_id, float(status["speed"]), ds, route, "subtask_dwell_start")
                    continue

                if len(route) <= 1:
                    pt = route[0]
                    status["nodeId"] = str(pt.node_id)
                    status["x"] = float(pt.x)
                    status["y"] = float(pt.y)
                    # Keep orientation stable when stationary; trail/path may provide yaw=0 for 1-point routes.
                    status["angle"] = _safe_float(status.get("angle"), float(pt.yaw))
                    status["speed"] = 0.0
                    status["updateTime"] = now_ts
                    status["nextDestinationPoint"] = {}
                    status["curTrailPoints"] = []
                    self._consume_reached_subtasks_locked(device_id, status)
                    self._note_speed_drop(device_id, float(status["speed"]), ds, route, "route_len<=1")
                    continue

                v_nodes = ds.v_nodes or [0.0 for _ in range(len(route))]
                v_max = self._v_max_m_s
                accel = self._accel_m_s2
                decel = self._decel_m_s2

                # Integrate across segments.
                remaining = dt
                while remaining > 1e-9:
                    if ds.idx >= len(route) - 1:
                        break

                    # Safety: if we are exactly at a node but heading differs from the next edge,
                    # always perform an in-place turn (prevents instant/random yaw jumps on replans).
                    if (
                        ds.phase == "move"
                        and ds.idx < len(route) - 1
                        and self._turn_rate_deg_s > 1e-9
                        and ds.turn_stop_angle_deg > 1e-9
                    ):
                        cur_pt = route[ds.idx]
                        px = _safe_float(status.get("x"), float(cur_pt.x))
                        py = _safe_float(status.get("y"), float(cur_pt.y))
                        if math.hypot(px - float(cur_pt.x), py - float(cur_pt.y)) <= 1.0:
                            cur_yaw = _safe_float(status.get("angle"), 0.0)
                            desired = _heading_deg(
                                (float(cur_pt.x), float(cur_pt.y)),
                                (float(route[ds.idx + 1].x), float(route[ds.idx + 1].y)),
                            )
                            diff = _angle_diff_deg(cur_yaw, desired)
                            if diff >= ds.turn_stop_angle_deg:
                                ds.phase = "turn"
                                ds.turn_t = 0.0
                                ds.turn_total = max(self._turn_min_time_s, diff / max(1e-9, self._turn_rate_deg_s))
                                ds.turn_from_yaw = cur_yaw
                                ds.turn_to_yaw = desired
                                ds.seg_t = 0.0
                                ds.seg_profile = None
                                continue

                    if ds.phase == "turn":
                        time_left = float(ds.turn_total) - float(ds.turn_t)
                        if time_left <= 1e-9:
                            ds.phase = "move"
                            ds.turn_t = 0.0
                            ds.turn_total = 0.0
                            ds.turn_from_yaw = ds.turn_to_yaw
                            status["angle"] = float(ds.turn_to_yaw)
                            ds.seg_t = 0.0
                            ds.seg_profile = None
                            continue
                        step = min(remaining, time_left)
                        ds.turn_t += step
                        remaining -= step
                        if ds.turn_t >= ds.turn_total - 1e-9:
                            ds.phase = "move"
                            ds.turn_t = 0.0
                            ds.turn_total = 0.0
                            ds.turn_from_yaw = ds.turn_to_yaw
                            status["angle"] = float(ds.turn_to_yaw)
                            ds.seg_t = 0.0
                            ds.seg_profile = None
                        continue

                    v0 = v_nodes[min(ds.idx, len(v_nodes) - 1)]
                    v1 = v_nodes[min(ds.idx + 1, len(v_nodes) - 1)]
                    prof = ds.seg_profile
                    if prof is None:
                        prof = ds.ensure_profile(v0, v1, v_max, accel, decel)
                    time_left = prof.t_total - ds.seg_t
                    if time_left <= 1e-9:
                        ds.idx += 1
                        ds.seg_t = 0.0
                        ds.seg_profile = None
                        if ds.idx < len(route):
                            markers = self._sticky_subtask_markers_by_device.get(device_id) or []
                            if markers:
                                nid = markers[0].get("node_id")
                                if nid is not None and int(nid) == int(route[ds.idx].node_id):
                                    if self._start_subtask_dwell_locked(device_id, markers[0]):
                                        remaining = 0.0
                                        break
                        # If there is a significant direction change, insert an in-place turn.
                        if ds.idx < len(route) - 1 and self._turn_rate_deg_s > 1e-9 and ds.turn_stop_angle_deg > 1e-9 and ds.idx > 0:
                            a = (float(route[ds.idx - 1].x), float(route[ds.idx - 1].y))
                            b = (float(route[ds.idx].x), float(route[ds.idx].y))
                            c = (float(route[ds.idx + 1].x), float(route[ds.idx + 1].y))
                            from_yaw = _heading_deg(a, b)
                            to_yaw = _heading_deg(b, c)
                            diff = _angle_diff_deg(from_yaw, to_yaw)
                            if diff >= ds.turn_stop_angle_deg:
                                ds.phase = "turn"
                                ds.turn_t = 0.0
                                ds.turn_total = max(self._turn_min_time_s, diff / max(1e-9, self._turn_rate_deg_s))
                                ds.turn_from_yaw = from_yaw
                                ds.turn_to_yaw = to_yaw
                        continue
                    if remaining < time_left:
                        ds.seg_t += remaining
                        remaining = 0.0
                    else:
                        ds.seg_t = prof.t_total
                        remaining -= time_left
                        ds.idx += 1
                        ds.seg_t = 0.0
                        ds.seg_profile = None
                        if ds.idx < len(route):
                            markers = self._sticky_subtask_markers_by_device.get(device_id) or []
                            if markers:
                                nid = markers[0].get("node_id")
                                if nid is not None and int(nid) == int(route[ds.idx].node_id):
                                    if self._start_subtask_dwell_locked(device_id, markers[0]):
                                        remaining = 0.0
                                        break
                        # If there is a significant direction change, insert an in-place turn.
                        if ds.idx < len(route) - 1 and self._turn_rate_deg_s > 1e-9 and ds.turn_stop_angle_deg > 1e-9 and ds.idx > 0:
                            a = (float(route[ds.idx - 1].x), float(route[ds.idx - 1].y))
                            b = (float(route[ds.idx].x), float(route[ds.idx].y))
                            c = (float(route[ds.idx + 1].x), float(route[ds.idx + 1].y))
                            from_yaw = _heading_deg(a, b)
                            to_yaw = _heading_deg(b, c)
                            diff = _angle_diff_deg(from_yaw, to_yaw)
                            if diff >= ds.turn_stop_angle_deg:
                                ds.phase = "turn"
                                ds.turn_t = 0.0
                                ds.turn_total = max(self._turn_min_time_s, diff / max(1e-9, self._turn_rate_deg_s))
                                ds.turn_from_yaw = from_yaw
                                ds.turn_to_yaw = to_yaw

                if float(self._subtask_dwell_remaining_s_by_device.get(device_id, 0.0)) > 1e-9:
                    hold_idx = max(0, min(int(ds.idx), len(route) - 1))
                    pt = route[hold_idx]
                    status["nodeId"] = str(pt.node_id)
                    status["x"] = float(pt.x)
                    status["y"] = float(pt.y)
                    status["angle"] = _safe_float(status.get("angle"), float(pt.yaw))
                    status["speed"] = 0.0
                    status["updateTime"] = now_ts
                    if hold_idx < len(route) - 1:
                        nxt = route[hold_idx + 1]
                        status["nextDestinationPoint"] = {
                            "x": float(nxt.x),
                            "y": float(nxt.y),
                            "angle": float(status.get("angle", 0.0)),
                            "nodeId": str(nxt.node_id),
                        }
                    else:
                        status["nextDestinationPoint"] = {}
                    self.drive_index[device_id] = hold_idx
                    self.drive_progress[device_id] = 0.0
                    if self.trail_preview > 0:
                        preview = []
                        for p in route[hold_idx + 1 : hold_idx + 1 + self.trail_preview]:
                            preview.append({"x": float(p.x), "y": float(p.y), "angle": float(p.yaw), "nodeId": str(p.node_id)})
                        status["curTrailPoints"] = preview
                    else:
                        status["curTrailPoints"] = []
                    self._note_speed_drop(device_id, float(status["speed"]), ds, route, "subtask_dwell_hold")
                    continue

                if ds.idx >= len(route) - 1:
                    pt = route[-1]
                    status["nodeId"] = str(pt.node_id)
                    status["x"] = float(pt.x)
                    status["y"] = float(pt.y)
                    # Keep orientation stable when arriving; avoid jumping to point yaw (often 0 at terminal nodes).
                    status["angle"] = _safe_float(status.get("angle"), float(pt.yaw))
                    status["speed"] = 0.0
                    status["updateTime"] = now_ts
                    status["nextDestinationPoint"] = {}
                    status["curTrailPoints"] = []
                    marker = self._marker_at_current_node_locked(device_id, status)
                    if marker is not None and self._start_subtask_dwell_locked(device_id, marker):
                        self._note_speed_drop(device_id, float(status["speed"]), ds, route, "arrived_subtask_dwell_start")
                        continue
                    self._consume_reached_subtasks_locked(device_id, status)
                    self._note_speed_drop(device_id, float(status["speed"]), ds, route, "arrived")
                    continue

                if ds.phase == "turn":
                    cur_pt = route[ds.idx]
                    next_pt = route[ds.idx + 1] if ds.idx + 1 < len(route) else None
                    ratio = 0.0
                    if ds.turn_total > 1e-9:
                        ratio = max(0.0, min(1.0, ds.turn_t / ds.turn_total))
                    yaw = _interp_angle_deg(ds.turn_from_yaw, ds.turn_to_yaw, ratio)
                    status["nodeId"] = str(cur_pt.node_id)
                    status["x"] = float(cur_pt.x)
                    status["y"] = float(cur_pt.y)
                    status["angle"] = yaw
                    status["speed"] = 0.0
                    status["updateTime"] = now_ts
                    if next_pt is not None:
                        status["nextDestinationPoint"] = {
                            "x": float(next_pt.x),
                            "y": float(next_pt.y),
                            "angle": float(ds.turn_to_yaw),
                            "nodeId": str(next_pt.node_id),
                        }
                    else:
                        status["nextDestinationPoint"] = {}
                    self.drive_index[device_id] = ds.idx
                    self.drive_progress[device_id] = 0.0
                    if self.trail_preview > 0:
                        preview = []
                        for pt in route[ds.idx + 1 : ds.idx + 1 + self.trail_preview]:
                            preview.append({"x": float(pt.x), "y": float(pt.y), "angle": float(pt.yaw), "nodeId": str(pt.node_id)})
                        status["curTrailPoints"] = preview
                    else:
                        status["curTrailPoints"] = []
                    self._consume_reached_subtasks_locked(device_id, status)
                    self._note_speed_drop(device_id, float(status["speed"]), ds, route, "turn")
                else:
                    start_pt = route[ds.idx]
                    end_pt = route[ds.idx + 1]
                    v0 = v_nodes[min(ds.idx, len(v_nodes) - 1)]
                    v1 = v_nodes[min(ds.idx + 1, len(v_nodes) - 1)]
                    prof = ds.seg_profile or ds.ensure_profile(v0, v1, v_max, accel, decel)

                    dist_m = prof.dist_at(ds.seg_t)
                    speed_m_s = prof.speed_at(ds.seg_t)
                    ratio = 0.0
                    if prof.length_m > 1e-9:
                        ratio = max(0.0, min(1.0, dist_m / prof.length_m))

                    x = float(start_pt.x) + (float(end_pt.x) - float(start_pt.x)) * ratio
                    y = float(start_pt.y) + (float(end_pt.y) - float(start_pt.y)) * ratio
                    yaw = float(start_pt.yaw)
                    if float(start_pt.x) != float(end_pt.x) or float(start_pt.y) != float(end_pt.y):
                        yaw = (math.degrees(math.atan2(float(end_pt.y) - float(start_pt.y), float(end_pt.x) - float(start_pt.x))) + 360.0) % 360.0

                    status["nodeId"] = str(start_pt.node_id)
                    status["x"] = x
                    status["y"] = y
                    status["angle"] = yaw
                    status["speed"] = float(speed_m_s * 1000.0)  # mm/s
                    status["updateTime"] = now_ts

                    status["nextDestinationPoint"] = {
                        "x": float(end_pt.x),
                        "y": float(end_pt.y),
                        "angle": yaw,
                        "nodeId": str(end_pt.node_id),
                    }

                    self.drive_index[device_id] = ds.idx
                    self.drive_progress[device_id] = ratio
                    if self.trail_preview > 0:
                        preview = []
                        for pt in route[ds.idx + 1 : ds.idx + 1 + self.trail_preview]:
                            preview.append({"x": float(pt.x), "y": float(pt.y), "angle": float(pt.yaw), "nodeId": str(pt.node_id)})
                        status["curTrailPoints"] = preview
                    else:
                        status["curTrailPoints"] = []
                    self._consume_reached_subtasks_locked(device_id, status)
                    reason = "move"
                    if ds.source == "trail" and len(route) < self._trail_min_drive_points:
                        reason = "trail_short_stop"
                    self._note_speed_drop(device_id, float(status["speed"]), ds, route, reason)

            self._metrics_update_distances_locked()

        # unused but keeps signature compatible
        _ = edge_duration


def _point_type_text(pt: int) -> str:
    if pt == 0:
        return "取货中"
    if pt == 1:
        return "放货中"
    if pt == 3:
        return "充电中"
    if pt == 4:
        return "等待中"
    if pt == 5:
        return "维护中"
    if pt == 2:
        return "停留中"
    return "作业中"


def build_live_state_payload(
    state_obj: SimStateV2,
    map_payload: Dict[str, Any],
    *,
    include_routes: bool = True,
    max_points: int = 120,
) -> Dict[str, Any]:
    max_points = max(1, min(300, _safe_int(max_points, 120)))
    statuses = state_obj.snapshot_status_by_id()
    subtasks = state_obj.snapshot_subtask_markers()
    assigned_ids = set(state_obj.assigned_device_ids())
    paths = state_obj.snapshot_paths() if include_routes else {}
    trails = state_obj.snapshot_trails() if include_routes else {}

    agvs: List[Dict[str, Any]] = []
    for did, st in statuses.items():
        next_dest = st.get("nextDestinationPoint")
        next_node = None
        if isinstance(next_dest, dict):
            next_node = sim.parse_node_id(next_dest.get("nodeId"))
        cur_node = st.get("nodeId", "")
        speed = _safe_float(st.get("speed"), 0.0)
        dwell_left_s = float(state_obj._subtask_dwell_remaining_s_by_device.get(did, 0.0))
        dwell_total_s = float(state_obj._subtask_dwell_total_s_by_device.get(did, 0.0))
        ds = state_obj._drive_v2.get(did)

        phase = "move"
        phase_text = "行驶中"
        phase_left_s = 0.0
        phase_total_s = 0.0
        if dwell_left_s > 1e-9:
            phase = "dwell"
            phase_left_s = dwell_left_s
            phase_total_s = dwell_total_s if dwell_total_s > 1e-9 else dwell_left_s
            active_pt = -1
            cur_node_id = sim.parse_node_id(cur_node)
            markers_now = subtasks.get(did) or []
            if markers_now and cur_node_id is not None:
                try:
                    nid0 = int(markers_now[0].get("node_id", -1))
                    if nid0 == int(cur_node_id):
                        active_pt = int(markers_now[0].get("point_type", -1) or -1)
                except Exception:
                    active_pt = -1
            phase_text = _point_type_text(active_pt)
        elif ds is not None and str(getattr(ds, "phase", "")) == "turn":
            phase = "turn"
            phase_text = "转弯中"
            try:
                phase_total_s = float(getattr(ds, "turn_total", 0.0))
                phase_left_s = max(0.0, phase_total_s - float(getattr(ds, "turn_t", 0.0)))
            except Exception:
                phase_left_s = 0.0
                phase_total_s = 0.0
        elif did in assigned_ids:
            if speed <= 1e-9:
                phase = "wait"
                phase_text = "等待中"
            else:
                phase = "move"
                phase_text = "行驶中"
        else:
            if speed <= 1e-9:
                phase = "idle"
                phase_text = "空闲"
            else:
                phase = "move"
                phase_text = "行驶中"

        item = {
            "id": did,
            "x": _safe_float(st.get("x"), 0.0),
            "y": _safe_float(st.get("y"), 0.0),
            "angle": _safe_float(st.get("angle"), 0.0),
            "speed": speed,
            "nodeId": str(cur_node),
            "nextNodeId": str(next_node) if next_node is not None else "",
            "assigned": did in assigned_ids,
            "phase": phase,
            "phaseText": phase_text,
            "phaseLeftS": float(phase_left_s),
            "phaseTotalS": float(phase_total_s),
        }

        if include_routes:
            path_pts = paths.get(did) or []
            trail_pts = trails.get(did) or []
            item["path"] = [
                {"x": float(p.x), "y": float(p.y), "nodeId": int(p.node_id)}
                for p in path_pts[:max_points]
            ]
            item["trail"] = [
                {"x": float(p.x), "y": float(p.y), "nodeId": int(p.node_id)}
                for p in trail_pts[:max_points]
            ]
        else:
            item["path"] = []
            item["trail"] = []

        markers = subtasks.get(did) or []
        next_marker = None
        cur_node_id = sim.parse_node_id(cur_node)
        for marker in markers:
            try:
                nid = int(marker.get("node_id", -1))
            except Exception:
                continue
            if cur_node_id is None or nid != int(cur_node_id):
                next_marker = marker
                break
        if next_marker is None and markers:
            next_marker = markers[-1]

        item["subtasks"] = [
            {
                "nodeId": int(marker.get("node_id", -1)),
                "x": _safe_float(marker.get("x"), 0.0),
                "y": _safe_float(marker.get("y"), 0.0),
                "label": str(marker.get("label", "")),
                "taskId": str(marker.get("task_id", "")),
                "subTaskId": str(marker.get("sub_task_id", "")),
                "sequence": int(marker.get("sequence", 0) or 0),
                "pointType": int(marker.get("point_type", -1) or -1),
                "location": str(marker.get("location", "") or ""),
                "dwellS": float(marker.get("dwell_s", 0.0) or 0.0),
            }
            for marker in markers[:max_points]
        ]
        if next_marker is not None:
            item["nextSubtask"] = {
                "nodeId": int(next_marker.get("node_id", -1)),
                "x": _safe_float(next_marker.get("x"), 0.0),
                "y": _safe_float(next_marker.get("y"), 0.0),
                "pointType": int(next_marker.get("point_type", -1) or -1),
                "dwellS": float(next_marker.get("dwell_s", 0.0) or 0.0),
            }
        else:
            item["nextSubtask"] = None
        agvs.append(item)

    return {
        "ts_ms": int(time.time() * 1000),
        "sim_time_s": state_obj.sim_time_s(),
        "v_max_mm_s": int(map_payload.get("info", {}).get("maxSpeed", 1000) or 1000),
        "agvs": agvs,
        "metrics": state_obj.snapshot_metrics(),
        "reserved_nodes": state_obj.snapshot_reserved_nodes(),
    }


class LiveWebServer:
    def __init__(self, state: SimStateV2, map_payload: Dict[str, Any], web_root: Path, host: str, port: int) -> None:
        self._state = state
        self._map = map_payload
        self._web_root = web_root
        self._host = host
        self._port = port
        self._server: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        web_root = self._web_root
        state_obj = self._state
        map_payload = self._map

        class Handler(BaseHTTPRequestHandler):
            def _send_json(self, payload: Any) -> None:
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _send_file(self, path: Path, content_type: str) -> None:
                if not path.exists() or not path.is_file():
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                data = path.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, format: str, *args) -> None:  # noqa: A003
                # keep stdout clean
                return

            def do_GET(self) -> None:  # noqa: N802
                parsed = urllib.parse.urlparse(self.path)
                if parsed.path in ("/", "/index.html"):
                    return self._send_file(web_root / "index.html", "text/html; charset=utf-8")
                if parsed.path == "/app.js":
                    return self._send_file(web_root / "app.js", "application/javascript; charset=utf-8")
                if parsed.path == "/map.json":
                    return self._send_json(map_payload)
                if parsed.path == "/state.json":
                    qs = urllib.parse.parse_qs(parsed.query or "")
                    include_routes = qs.get("routes", ["1"])[0] != "0"
                    max_points = max(1, min(300, _safe_int(qs.get("max_points", ["120"])[0], 120)))
                    payload = build_live_state_payload(
                        state_obj,
                        map_payload,
                        include_routes=include_routes,
                        max_points=max_points,
                    )
                    return self._send_json(payload)

                self.send_error(HTTPStatus.NOT_FOUND)

        self._server = ThreadingHTTPServer((self._host, self._port), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def close(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="simulation_v2: online web visualizer + continuous-speed motion")
    p.add_argument("--map", dest="map_path", default=None, help="Map JSON path")
    p.add_argument("--map-version", default=env("MAP_VERSION", "sim-map"), help="Map version string")
    p.add_argument("--map-id", type=int, default=0, help="Map ID for status payloads")
    p.add_argument("--status", dest="status_path", default=None, help="Status JSON path")
    p.add_argument("--tasks", dest="tasks_path", default=None, help="Task JSON path")
    p.add_argument("--generate", action="store_true", help="Generate status/tasks from map")
    p.add_argument("--bootstrap", action="store_true", help="Send map/status/tasks before running")
    p.add_argument("--run", action="store_true", help="Start simulation loop")

    p.add_argument("--agv-count", type=int, default=50, help="Number of AGVs to generate")
    p.add_argument("--task-count", type=int, default=50, help="Number of tasks to generate")
    p.add_argument("--seed", type=int, default=7, help="Random seed for generation")
    p.add_argument("--device-prefix", default="AGV", help="Device ID prefix for generated AGVs")
    p.add_argument("--bind-tasks", action="store_true", help="Bind tasks round-robin to device IDs via agvRequirements")
    p.add_argument(
        "--bind-tasks-ratio",
        type=float,
        default=float(env("SIM_BIND_TASKS_RATIO", "1.0")),
        help="If --bind-tasks is on, keep binding for this fraction of tasks (0..1).",
    )
    p.add_argument("--include-status-in-tasks", action="store_true", help="Embed agvStatusList into tasks payload")
    p.add_argument(
        "--alloc-algo",
        default=env("SIM_ALLOC_ALGO", ""),
        help="Allocation algorithm name (e.g. posta/greedy/mlp) to include in task payloads.",
    )
    p.add_argument("--no-attach-status", action="store_true", help="Do not attach status to task payload")

    p.add_argument("--trail-preview", type=int, default=5, help="Preview trail points in status payloads")
    p.add_argument(
        "--drive-mode",
        choices=("path", "trail"),
        default=env("SIM_DRIVE_MODE", "trail"),
        help="How to move AGVs: trail=follow RobotTrailResponse only; path=ignore trail and follow RobotPathResponse.",
    )
    p.add_argument("--step-interval", type=float, default=0.1, help="Seconds per simulation tick")
    p.add_argument("--edge-duration", type=float, default=1.0, help="(unused in v2) kept for CLI compatibility")
    p.add_argument("--path-interval", type=float, default=0.0, help="Seconds between path requests (0 disables)")
    p.add_argument("--trail-interval", type=float, default=1.0, help="Seconds between trail requests")
    p.add_argument("--assigned-path-interval", type=float, default=0.0, help="Seconds between path requests for assigned AGVs")
    p.add_argument("--assigned-trail-interval", type=float, default=0.0, help="Seconds between trail requests for assigned AGVs")
    p.add_argument(
        "--status-interval",
        type=float,
        default=_resolve_status_interval_s(),
        help="Seconds between status publishes; 0=every tick.",
    )
    p.add_argument("--no-snap-to-map", action="store_true", help="Disable map-edge validation/truncation for route drawing")

    p.add_argument("--loop-tasks", action="store_true", help="Continuously send new random tasks")
    p.add_argument("--loop-mode", choices=("any-idle", "all-idle", "timer"), default=env("SIM_LOOP_MODE", "any-idle"))
    p.add_argument("--loop-task-min-count", type=int, default=1)
    p.add_argument("--loop-task-max-count", type=int, default=1)
    p.add_argument("--loop-task-min-delay", type=float, default=0.3)
    p.add_argument("--loop-task-max-delay", type=float, default=0.8)
    p.add_argument(
        "--auto-stop-idle-sec",
        type=float,
        default=_safe_float(env("SIM_AUTO_STOP_IDLE_SEC", "0"), 0.0),
        help="Auto-exit after all AGVs stay idle and no tasks remain for this many seconds; 0 disables.",
    )
    p.add_argument(
        "--auto-stop-min-runtime-sec",
        type=float,
        default=_safe_float(env("SIM_AUTO_STOP_MIN_RUNTIME_SEC", "0"), 0.0),
        help="Minimum wall-clock runtime before idle auto-stop can trigger.",
    )
    p.add_argument(
        "--max-sim-time-sec",
        type=float,
        default=_safe_float(env("SIM_MAX_SIM_TIME_SEC", "0"), 0.0),
        help="Hard cap for simulated time; 0 disables.",
    )

    p.add_argument("--no-vis", action="store_true", help="Disable web visualization")
    p.add_argument("--vis-host", default=env("SIM_V2_VIS_HOST", "127.0.0.1"), help="Web UI host to bind")
    p.add_argument("--vis-port", type=int, default=int(env("SIM_V2_VIS_PORT", "18080")), help="Web UI port to bind")
    p.add_argument("--record-replay", action="store_true", help="Record frames/events for offline replay")
    p.add_argument(
        "--record-dir",
        default=env("SIM_REPLAY_DIR", str(Path(__file__).resolve().parents[1] / "demo" / "sessions")),
        help="Output root directory for replay sessions",
    )
    p.add_argument(
        "--record-session",
        default=env("SIM_REPLAY_SESSION", ""),
        help="Optional replay session name; defaults to a timestamp",
    )
    p.add_argument(
        "--record-interval-ms",
        type=int,
        default=int(env("SIM_REPLAY_INTERVAL_MS", "120")),
        help="Replay frame sampling interval in milliseconds",
    )
    p.add_argument(
        "--record-max-points",
        type=int,
        default=int(env("SIM_REPLAY_MAX_POINTS", "60")),
        help="Max path/trail/subtask points stored per AGV in replay frames",
    )

    p.add_argument("--max-speed-m-s", type=float, default=float(env("SIM_MAX_SPEED_M_S", "0")), help="Override max speed (m/s), 0=use map maxSpeed")
    p.add_argument("--accel-m-s2", type=float, default=float(env("SIM_ACCEL_M_S2", "0.5")), help="Acceleration limit (m/s^2)")
    p.add_argument("--decel-m-s2", type=float, default=float(env("SIM_DECEL_M_S2", "0.5")), help="Deceleration limit (m/s^2)")
    p.add_argument("--turn-slowdown", type=float, default=float(env("SIM_TURN_SLOWDOWN", "0.7")), help="Turn slowdown strength (0..1)")
    p.add_argument("--turn-min-factor", type=float, default=float(env("SIM_TURN_MIN_FACTOR", "0.3")), help="Min speed factor at 180deg turn")
    p.add_argument(
        "--turn-stop-angle-deg",
        type=float,
        default=float(env("SIM_TURN_STOP_ANGLE_DEG", "1")),
        help="If direction change >= this angle, stop and turn in place first.",
    )
    p.add_argument(
        "--turn-rate-deg-s",
        type=float,
        default=float(env("SIM_TURN_RATE_DEG_S", "90")),
        help="In-place turning angular speed (deg/s).",
    )
    p.add_argument(
        "--turn-min-time-s",
        type=float,
        default=float(env("SIM_TURN_MIN_TIME_S", "0.2")),
        help="Minimum in-place turning time (s) when stopping for a turn.",
    )
    p.add_argument(
        "--trail-end-stop",
        type=int,
        default=1 if env_bool("SIM_TRAIL_END_STOP", True) else 0,
        help="For trail-driven motion: 1=force v_end=0 at the last trail point; 0=treat trail end as lookahead (smoother).",
    )
    p.add_argument(
        "--trail-min-drive-points",
        type=int,
        default=_resolve_trail_min_drive_points(),
        help="Trail-driven motion: trail points <= this value trigger braking.",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    root = Path(__file__).resolve().parents[1]

    map_path = Path(args.map_path) if args.map_path else (root / "config" / "grid_20x20.json")
    if not map_path.exists():
        print(f"[sim_v2] map file not found: {map_path}", file=sys.stderr)
        return 2
    map_raw = load_map_raw(map_path)

    graph = sim.load_map_graph(map_path)
    if not graph.nodes:
        print("[sim_v2] map has no nodes", file=sys.stderr)
        return 2

    node_list: List[Dict[str, Any]] = []
    if isinstance(map_raw, dict):
        node_list = map_raw.get("node") or map_raw.get("nodes") or []
    map_forbidden_nodes: Set[int] = set()
    if node_list:
        for n in node_list:
            if _node_passable(n):
                continue
            nid = n.get("id", n.get("nodeId"))
            if nid is None:
                continue
            try:
                map_forbidden_nodes.add(int(nid))
            except Exception:
                continue

    status_list: List[Dict[str, Any]] = []
    tasks_payload: Optional[Dict[str, Any]] = None
    task_pool = TaskPool()
    if args.generate:
        rng = random.Random(int(args.seed))
        bind_ratio = max(0.0, min(1.0, float(args.bind_tasks_ratio)))
        start_candidates: Optional[List[int]] = None
        if node_list:
            start_candidates = [
                int(n.get("id", n.get("nodeId")))
                for n in node_list
                if _node_passable(n) and n.get("id", n.get("nodeId")) is not None
            ]
            if not start_candidates:
                start_candidates = None
        status_list = sim.generate_status_list(
            graph,
            int(args.agv_count),
            rng,
            int(args.map_id),
            str(args.device_prefix),
            start_candidates,
        )
        bind_ids: Optional[List[str]] = None
        start_nodes: Optional[Dict[str, int]] = None
        if args.bind_tasks:
            bind_ids = [s.get("deviceId", "") for s in status_list if s.get("deviceId")]
            bind_ids = [x for x in bind_ids if x]
            start_nodes = {}
            for s in status_list:
                did = s.get("deviceId")
                nid = sim.parse_node_id(s.get("nodeId"))
                if did and nid is not None:
                    start_nodes[did] = nid
        tasks_payload = sim.generate_task_payload(
            graph,
            int(args.task_count),
            rng,
            str(args.map_version),
            bind_to_device_ids=bind_ids,
            start_node_by_device_id=start_nodes,
            allocation_algo=args.alloc_algo or None,
        )
        if args.include_status_in_tasks and tasks_payload is not None:
            tasks_payload["agvStatusList"] = status_list
        if tasks_payload and args.bind_tasks and bind_ratio < 1.0:
            apply_bind_tasks_ratio(tasks_payload, rng, bind_ratio)
    else:
        if args.status_path:
            status_path = Path(args.status_path)
            if not status_path.exists():
                print(f"[sim_v2] status file not found: {status_path}", file=sys.stderr)
                return 2
            loaded = load_json_path(status_path)
            if isinstance(loaded, list):
                status_list = loaded
            elif isinstance(loaded, dict):
                candidate = loaded.get("robotStatusInfos")
                if not isinstance(candidate, list):
                    candidate = loaded.get("agvStatusList")
                if isinstance(candidate, list):
                    status_list = candidate
        if args.tasks_path:
            tasks_path = Path(args.tasks_path)
            if not tasks_path.exists():
                print(f"[sim_v2] tasks file not found: {tasks_path}", file=sys.stderr)
                return 2
            loaded = load_json_path(tasks_path)
            if isinstance(loaded, dict):
                tasks_payload = loaded
        if not status_list and tasks_payload and isinstance(tasks_payload.get("agvStatusList"), list):
            status_list = list(tasks_payload.get("agvStatusList"))

    effective_agv_count = len(status_list) if status_list else int(args.agv_count)
    web_map = build_web_map_payload(map_raw, effective_agv_count)
    max_speed_mm_s = int(web_map.get("info", {}).get("maxSpeed", 1000) or 1000)
    v_max = float(args.max_speed_m_s) if float(args.max_speed_m_s) > 0 else max(0.1, max_speed_mm_s / 1000.0)
    bind_ratio = max(0.0, min(1.0, float(args.bind_tasks_ratio)))
    if tasks_payload:
        occupied_by_node: Dict[int, str] = {}
        forbidden_nodes: Set[int] = set()
        for st in status_list:
            node_id = sim.parse_node_id(st.get("nodeId"))
            device_id = str(st.get("deviceId", "")).strip()
            if node_id is None:
                continue
            forbidden_nodes.add(int(node_id))
            if device_id:
                occupied_by_node[int(node_id)] = device_id
        if map_forbidden_nodes:
            # Ensure simulation never issues tasks that end on allowPass=0 nodes.
            tasks = tasks_payload.get("candidateTasks", []) if isinstance(tasks_payload, dict) else []
            if isinstance(tasks, list):
                filtered: List[Dict[str, Any]] = []
                for task in tasks:
                    if not isinstance(task, dict):
                        continue
                    end_node: Optional[int] = None
                    sub_tasks = task.get("subTasks")
                    if isinstance(sub_tasks, list) and sub_tasks:
                        last = sub_tasks[-1]
                        if isinstance(last, dict):
                            point = last.get("point")
                            if isinstance(point, dict):
                                end_node = sim.parse_node_id(point.get("nodeId"))
                            if end_node is None:
                                end_node = sim.parse_node_id(last.get("nodeId"))
                    if end_node is None or int(end_node) not in map_forbidden_nodes:
                        filtered.append(task)
                if len(filtered) != len(tasks):
                    tasks_payload["candidateTasks"] = filtered
                    tasks_payload["taskNumber"] = len(filtered)
                    batch = tasks_payload.get("batchConfig")
                    if isinstance(batch, dict):
                        batch = dict(batch)
                        batch["maxBatchSize"] = len(filtered)
                        tasks_payload["batchConfig"] = batch
                    else:
                        tasks_payload["batchConfig"] = {"maxBatchSize": len(filtered)}
            forbidden_nodes.update(map_forbidden_nodes)
        task_pool.add_payload(tasks_payload)
        tasks_payload = task_pool.build_payload(
            tasks_payload,
            forbidden_end_nodes=forbidden_nodes,
            occupied_by_node=occupied_by_node,
            allocation_algo=args.alloc_algo or None,
        )

    state = SimStateV2(
        status_list,
        graph,
        int(args.trail_preview),
        drive_mode=str(args.drive_mode),
        v_max_m_s=v_max,
        accel_m_s2=float(args.accel_m_s2),
        decel_m_s2=float(args.decel_m_s2),
        turn_slowdown=float(args.turn_slowdown),
        turn_min_factor=float(args.turn_min_factor),
        turn_stop_angle_deg=float(args.turn_stop_angle_deg),
        turn_rate_deg_s=float(args.turn_rate_deg_s),
        turn_min_time_s=float(args.turn_min_time_s),
        trail_end_stop=bool(int(args.trail_end_stop)),
        trail_min_drive_points=int(args.trail_min_drive_points),
    )
    if tasks_payload:
        state.set_tasks_payload(tasks_payload)

    recorder: Optional[ReplayRecorder] = None
    if args.record_replay:
        session_name = str(args.record_session or "").strip()
        if not session_name:
            session_name = time.strftime("replay_%Y%m%d_%H%M%S", time.localtime())
        recorder = ReplayRecorder(
            Path(args.record_dir),
            session_name,
            web_map,
            record_interval_s=max(0.02, float(args.record_interval_ms) / 1000.0),
            record_max_points=max(10, int(args.record_max_points)),
            metadata={
                "source": "simulation_v2",
                "map_file": str(map_path),
                "map_version": str(args.map_version),
                "map_id": int(args.map_id),
                "agv_count": len(status_list),
                "drive_mode": str(args.drive_mode),
            },
        )
        if tasks_payload:
            recorder.record_tasks_published(tasks_payload, sim_time_s=state.sim_time_s(), source="initial")
        initial_frame = build_live_state_payload(
            state,
            web_map,
            include_routes=True,
            max_points=int(args.record_max_points),
        )
        recorder.maybe_record_frame(initial_frame)
        print(f"[sim_v2] replay recording: {recorder.session_dir}", file=sys.stderr)

    publisher = sim.MQPublisher()
    bootstrap_map_wait_s = _resolve_bootstrap_map_wait_s()
    pool_payload_template: Dict[str, Any] = dict(tasks_payload or {})

    if args.bootstrap:
        sim.send_map(publisher, map_path, str(args.map_version), int(args.map_id))
        print(f"[sim_v2] map sent: path={map_path} version={args.map_version} id={args.map_id}", file=sys.stderr)
        if bootstrap_map_wait_s > 0.0 and (status_list or tasks_payload):
            print(f"[sim_v2] waiting {bootstrap_map_wait_s:.1f}s for receiver map reload", file=sys.stderr)
            time.sleep(bootstrap_map_wait_s)
        if status_list:
            sim.send_status(publisher, state.snapshot_status_list())
            print(f"[sim_v2] status sent: count={len(status_list)}", file=sys.stderr)
        if tasks_payload:
            attach_status = not bool(args.no_attach_status)
            status_payload = state.snapshot_status_list() if attach_status else None
            sim.send_tasks(publisher, tasks_payload, status_payload)
            state.set_tasks_payload(tasks_payload)
            task_count = tasks_payload.get("taskNumber")
            if task_count is None:
                candidate = tasks_payload.get("candidateTasks")
                task_count = len(candidate) if isinstance(candidate, list) else 0
            print(f"[sim_v2] tasks sent: count={task_count}", file=sys.stderr)

    stop_event = threading.Event()
    snap_to_map = not bool(args.no_snap_to_map)
    path_refresh_lock = threading.Lock()
    pending_path_refresh: Set[str] = set()
    trail_refresh_lock = threading.Lock()
    pending_trail_refresh: Set[str] = set()

    def trail_is_contained_in_path(path_points: List[Any], trail_points: List[Any]) -> bool:
        if not trail_points:
            return True
        if not path_points:
            return False
        path_nodes = [p.node_id for p in sim.dedup_consecutive(path_points)]
        trail_nodes = [p.node_id for p in sim.dedup_consecutive(trail_points)]
        if not trail_nodes:
            return True
        if len(trail_nodes) > len(path_nodes):
            return False
        for i in range(0, len(path_nodes) - len(trail_nodes) + 1):
            if path_nodes[i : i + len(trail_nodes)] == trail_nodes:
                return True
        return False

    def on_message(routing_key: str, payload_text: str) -> None:
        try:
            payload = json.loads(payload_text)
        except json.JSONDecodeError:
            return
        if routing_key == env("ALGO_PATH_RESPONSE_ROUTING_KEY", "RobotPathResponse"):
            device_id = payload.get("deviceId", "")
            points = sim.extract_path_points(payload, graph)
            if snap_to_map and points:
                points = sim.snap_points_to_map(graph, points, f"path/{device_id or '-'}")
            if device_id:
                state.set_path(device_id, points)
                if recorder is not None:
                    recorder.record_path_response(payload, points, sim_time_s=state.sim_time_s())
        elif routing_key == env("ALGO_TRAIL_RESPONSE_ROUTING_KEY", "RobotTrailResponse"):
            device_id = payload.get("deviceId", "")
            points = sim.extract_trail_points(payload, graph)
            if snap_to_map and points:
                points = sim.snap_points_to_map(graph, points, f"trail/{device_id or '-'}")
            if device_id:
                state.set_trail(device_id, points)
                if recorder is not None:
                    recorder.record_trail_response(payload, points, sim_time_s=state.sim_time_s())
                if args.path_interval > 0 or args.assigned_path_interval > 0:
                    current_path = state.snapshot_paths().get(device_id, [])
                    if points and not trail_is_contained_in_path(current_path, points):
                        with path_refresh_lock:
                            pending_path_refresh.add(device_id)
        elif routing_key == env("ASSIGN_RESULT_ROUTING_KEY", "AssignmentTaskResponse"):
            state.set_assignment_response(payload)
            _dump_assign_result(payload)
            if recorder is not None:
                recorder.record_assignment_response(payload, sim_time_s=state.sim_time_s())
            task_pool.remove_assigned(payload)
            assigned_ids: List[str] = []
            assignments = payload.get("agv_assignments")
            if isinstance(assignments, list):
                for agv_entry in assignments:
                    if not isinstance(agv_entry, dict):
                        continue
                    device_id = str(agv_entry.get("agv_id", "")).strip()
                    if not device_id:
                        continue
                    tasks = agv_entry.get("assigned_tasks")
                    if isinstance(tasks, list) and tasks:
                        assigned_ids.append(device_id)
            if assigned_ids:
                if args.path_interval > 0 or args.assigned_path_interval > 0:
                    with path_refresh_lock:
                        pending_path_refresh.update(assigned_ids)
                if args.trail_interval > 0 or args.assigned_trail_interval > 0:
                    with trail_refresh_lock:
                        pending_trail_refresh.update(assigned_ids)

    def on_reserved_message(routing_key: str, payload_text: str) -> None:
        try:
            payload = json.loads(payload_text)
        except json.JSONDecodeError:
            return
        reserved = payload.get("reservedNodes")
        if isinstance(reserved, list):
            state.set_reserved_nodes(reserved)
            if recorder is not None:
                recorder.record_reserved_snapshot(payload, sim_time_s=state.sim_time_s())

    consumer = sim.MQConsumer(on_message, stop_event)
    consumer.start()
    reserved_exchange = env("SIM_RESERVED_EXCHANGE", "AlgoSimExchange")
    reserved_exchange_type = env("SIM_RESERVED_EXCHANGE_TYPE", "fanout")
    reserved_consumer = sim.MQConsumer(on_reserved_message, stop_event,
                                       exchange=reserved_exchange,
                                       exchange_type=reserved_exchange_type)
    reserved_consumer.start()

    web_server: Optional[LiveWebServer] = None
    if not args.no_vis:
        web_root = Path(__file__).resolve().parent / "web"
        web_server = LiveWebServer(state, web_map, web_root, str(args.vis_host), int(args.vis_port))
        web_server.start()
        print(f"[sim_v2] web UI: http://{args.vis_host}:{args.vis_port}/", file=sys.stderr)

    next_path_req = time.monotonic()
    next_trail_req = time.monotonic()
    next_assigned_path_req = time.monotonic()
    next_assigned_trail_req = time.monotonic()
    pending_pool_dispatch_enabled = env_bool("SIM_PENDING_TASK_POOL_ENABLE", True)
    pending_pool_dispatch_interval_s = max(1.0, _safe_float(env("SIM_PENDING_TASK_POOL_INTERVAL_SEC", "8.0"), 8.0))
    next_pool_dispatch = time.monotonic() + pending_pool_dispatch_interval_s
    status_interval_s = max(0.0, float(args.status_interval))
    last_tick = time.monotonic()
    run_started_at = last_tick
    loop_rng = random.Random(int(args.seed) + 1000)
    loop_cycle = 0
    loop_deadline: Optional[float] = None
    attach_status = not args.no_attach_status
    auto_stop_idle_s = max(0.0, float(args.auto_stop_idle_sec))
    auto_stop_min_runtime_s = max(0.0, float(args.auto_stop_min_runtime_sec))
    max_sim_time_s = max(0.0, float(args.max_sim_time_sec))
    auto_stop_idle_since: Optional[float] = None
    auto_stop_work_seen = bool(tasks_payload) or task_pool.size() > 0
    if auto_stop_idle_s > 0.0:
        print(
            f"[sim_v2] auto-stop enabled: idle={auto_stop_idle_s:.1f}s min_runtime={auto_stop_min_runtime_s:.1f}s",
            file=sys.stderr,
        )
    if max_sim_time_s > 0.0:
        print(f"[sim_v2] max-sim-time enabled: {max_sim_time_s:.1f}s", file=sys.stderr)
    status_thread: Optional[threading.Thread] = None
    if args.run and status_list and status_interval_s > 0.0:
        def status_loop() -> None:
            status_pub = sim.MQPublisher()
            next_send = time.monotonic()
            sleep_step = min(0.05, status_interval_s)
            while not stop_event.is_set():
                now = time.monotonic()
                if now >= next_send:
                    try:
                        sim.send_status(status_pub, state.snapshot_status_list())
                    except Exception:
                        pass
                    next_send = now + status_interval_s
                stop_event.wait(sleep_step)
            status_pub.close()

        status_thread = threading.Thread(target=status_loop, daemon=True)
        status_thread.start()

    try:
        while args.run:
            now = time.monotonic()
            dt = max(0.0, now - last_tick)
            # Clamp dt to keep turn/motion visually stable even if the main loop stalls.
            dt_cap = max(0.001, float(args.step_interval))
            if dt > dt_cap:
                dt = dt_cap
            last_tick = now

            # Loop task generator (keeps the system busy).
            statuses = state.snapshot_status_by_id()
            paths = state.snapshot_paths()
            trails = state.snapshot_trails()
            assigned_set = set(state.assigned_device_ids())
            all_devices = sorted(statuses.keys())
            idle_devices: List[str] = []
            start_nodes: Dict[str, int] = {}
            current_nodes: Set[int] = set()

            def device_is_idle(device_id: str, st: Dict[str, Any]) -> bool:
                # If the simulator believes this AGV still has assigned tasks/subtasks, do not
                # generate new loop tasks or publish pending historical tasks for it.
                if device_id in assigned_set:
                    return False
                next_dest = st.get("nextDestinationPoint")
                if isinstance(next_dest, dict) and next_dest:
                    if sim.parse_node_id(next_dest.get("nodeId")) is not None:
                        return False
                cur_node = sim.parse_node_id(st.get("nodeId"))
                goal_node = None
                if device_id in paths and paths[device_id]:
                    goal_node = paths[device_id][-1].node_id
                elif device_id in trails and trails[device_id]:
                    goal_node = trails[device_id][-1].node_id
                if goal_node is not None and cur_node is not None and cur_node != goal_node:
                    return False
                return True

            for device_id, st in statuses.items():
                nid = sim.parse_node_id(st.get("nodeId"))
                if nid is not None:
                    start_nodes[device_id] = nid
                    current_nodes.add(int(nid))
                if device_is_idle(device_id, st):
                    idle_devices.append(device_id)
            idle_devices.sort()
            pending_task_count = task_pool.size()
            if assigned_set or pending_task_count > 0 or any(paths.values()) or any(trails.values()):
                auto_stop_work_seen = True

            if auto_stop_idle_s > 0.0 and not args.loop_tasks:
                all_idle = bool(all_devices) and len(idle_devices) == len(all_devices)
                if (
                    auto_stop_work_seen
                    and pending_task_count == 0
                    and not assigned_set
                    and all_idle
                    and (now - run_started_at) >= auto_stop_min_runtime_s
                ):
                    if auto_stop_idle_since is None:
                        auto_stop_idle_since = now
                    elif (now - auto_stop_idle_since) >= auto_stop_idle_s:
                        print(
                            f"[sim_v2] auto-stop: idle for {auto_stop_idle_s:.1f}s after {now - run_started_at:.1f}s runtime",
                            file=sys.stderr,
                        )
                        break
                else:
                    auto_stop_idle_since = None

            if args.loop_tasks:
                trigger_devices: List[str] = []
                if args.loop_mode == "timer":
                    trigger_devices = all_devices
                elif args.loop_mode == "all-idle":
                    if all_devices and len(idle_devices) == len(all_devices):
                        trigger_devices = idle_devices
                else:
                    trigger_devices = idle_devices

                if trigger_devices:
                    min_delay = max(0.0, float(args.loop_task_min_delay))
                    max_delay = max(min_delay, float(args.loop_task_max_delay))
                    if loop_deadline is None:
                        loop_deadline = now + loop_rng.uniform(min_delay, max_delay)
                    if now >= loop_deadline:
                        min_count = max(1, int(args.loop_task_min_count))
                        max_count = max(min_count, int(args.loop_task_max_count))
                        max_send = min(max_count, len(trigger_devices))
                        min_send = min(min_count, max_send) if max_send > 0 else 0
                        task_count = loop_rng.randint(min_send, max_send) if max_send > 0 else 0
                        if task_count > 0:
                            chosen = trigger_devices
                            if task_count < len(trigger_devices):
                                chosen = sorted(loop_rng.sample(trigger_devices, k=task_count))
                            loop_cycle += 1
                            payload = sim.generate_loop_tasks_payload(
                                graph,
                                chosen,
                                loop_rng,
                                str(args.map_version),
                                cycle=loop_cycle,
                                start_node_by_device_id=start_nodes,
                                forbidden_end_nodes=current_nodes | map_forbidden_nodes,
                                bind_tasks=bool(args.bind_tasks),
                                allocation_algo=args.alloc_algo or None,
                            )
                            if args.bind_tasks and bind_ratio < 1.0:
                                apply_bind_tasks_ratio(payload, loop_rng, bind_ratio)
                            task_pool.add_payload(payload)
                            occupied_by_node: Dict[int, str] = {}
                            for device_id, nid in start_nodes.items():
                                occupied_by_node[int(nid)] = device_id
                            pool_payload = task_pool.build_payload(
                                payload,
                                forbidden_end_nodes=current_nodes | map_forbidden_nodes,
                                occupied_by_node=occupied_by_node,
                                allocation_algo=args.alloc_algo or None,
                            )
                            status_payload = state.snapshot_status_list() if attach_status else None
                            sim.send_tasks(publisher, pool_payload, status_payload)
                            state.set_tasks_payload(pool_payload)
                            if recorder is not None:
                                recorder.record_tasks_published(
                                    pool_payload,
                                    sim_time_s=state.sim_time_s(),
                                    source="loop",
                                )
                        loop_deadline = now + loop_rng.uniform(min_delay, max_delay)
                else:
                    loop_deadline = None

            if pending_pool_dispatch_enabled and task_pool.size() > 0 and now >= next_pool_dispatch:
                if idle_devices:
                    occupied_by_node: Dict[int, str] = {}
                    for device_id, nid in start_nodes.items():
                        occupied_by_node[int(nid)] = device_id
                    pool_payload = task_pool.build_payload(
                        pool_payload_template,
                        forbidden_end_nodes=current_nodes | map_forbidden_nodes,
                        occupied_by_node=occupied_by_node,
                        allocation_algo=args.alloc_algo or None,
                    )
                    pending_tasks = pool_payload.get("candidateTasks", [])
                    if isinstance(pending_tasks, list) and pending_tasks:
                        status_payload = state.snapshot_status_list() if attach_status else None
                        sim.send_tasks(publisher, pool_payload, status_payload)
                        state.set_tasks_payload(pool_payload)
                        if recorder is not None:
                            recorder.record_tasks_published(
                                pool_payload,
                                sim_time_s=state.sim_time_s(),
                                source="pool",
                            )
                next_pool_dispatch = now + pending_pool_dispatch_interval_s

            refresh_ids: List[str] = []
            with path_refresh_lock:
                if pending_path_refresh:
                    refresh_ids = sorted(pending_path_refresh)
                    pending_path_refresh.clear()
            for did in refresh_ids:
                sim.send_path_request(publisher, did)

            trail_refresh_ids: List[str] = []
            with trail_refresh_lock:
                if pending_trail_refresh:
                    trail_refresh_ids = sorted(pending_trail_refresh)
                    pending_trail_refresh.clear()
            for did in trail_refresh_ids:
                sim.send_trail_request(publisher, did, sub_task_id=state.next_subtask_id(did))

            if args.assigned_path_interval > 0 and now >= next_assigned_path_req:
                for did in state.assigned_device_ids():
                    sim.send_path_request(publisher, did)
                next_assigned_path_req = now + args.assigned_path_interval

            if args.path_interval > 0 and now >= next_path_req:
                for did in state.device_ids():
                    sim.send_path_request(publisher, did)
                next_path_req = now + args.path_interval

            if args.assigned_trail_interval > 0 and now >= next_assigned_trail_req:
                for did in state.assigned_device_ids():
                    sim.send_trail_request(publisher, did, sub_task_id=state.next_subtask_id(did))
                next_assigned_trail_req = now + args.assigned_trail_interval

            if args.trail_interval > 0 and now >= next_trail_req:
                for did in state.device_ids():
                    sim.send_trail_request(publisher, did, sub_task_id=state.next_subtask_id(did))
                next_trail_req = now + args.trail_interval

            state.advance(dt, edge_duration=float(args.edge_duration))
            if recorder is not None:
                recorder.maybe_record_frame(
                    build_live_state_payload(
                        state,
                        web_map,
                        include_routes=True,
                        max_points=int(args.record_max_points),
                    )
                )
            current_sim_time_s = state.sim_time_s()
            if max_sim_time_s > 0.0 and current_sim_time_s >= max_sim_time_s:
                print(
                    f"[sim_v2] max-sim-time reached: {current_sim_time_s:.1f}s >= {max_sim_time_s:.1f}s",
                    file=sys.stderr,
                )
                break
            if status_list and status_interval_s <= 0.0:
                sim.send_status(publisher, state.snapshot_status_list())

            time.sleep(max(0.01, float(args.step_interval)))
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if status_thread is not None:
            status_thread.join(timeout=1.0)
        publisher.close()
        if web_server is not None:
            web_server.close()
        if recorder is not None:
            recorder.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
