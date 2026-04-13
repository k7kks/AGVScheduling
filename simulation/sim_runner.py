#!/usr/bin/env python3
import argparse
import copy
import itertools
import json
import math
import os
import random
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Set, Tuple

try:
    import pika
except Exception as exc:  # pragma: no cover - runtime dependency
    print(f"pika is required: {exc}", file=sys.stderr)
    sys.exit(2)


def ensure_matplotlib_gui_backend() -> None:
    """
    Best-effort selection of a GUI backend.
    - Respect explicit MPLBACKEND only if it is a GUI backend.
    - Prefer Qt5Agg when available (often behaves better on HiDPI/resize).
    - Fall back to TkAgg.
    """
    if not (os.getenv("DISPLAY") or os.getenv("WAYLAND_DISPLAY")) and sys.platform != "win32":
        return
    try:
        import matplotlib
        from matplotlib import rcsetup
    except Exception:
        return
    forced = os.getenv("MPLBACKEND")
    if forced:
        try:
            backend = str(matplotlib.get_backend() or "")
            interactive_backends = {b.lower() for b in getattr(rcsetup, "interactive_bk", [])}
            if not interactive_backends or backend.lower() in interactive_backends:
                return
            print(
                f"[vis] warning: MPLBACKEND={forced!r} selects non-GUI backend {backend!r}; trying GUI backend instead.",
                file=sys.stderr,
            )
        except Exception:
            pass
    try:
        import PyQt5  # noqa: F401

        matplotlib.use("Qt5Agg", force=True)
        return
    except Exception:
        pass
    try:
        matplotlib.use("TkAgg", force=True)
    except Exception:
        pass


def env(key: str, default: str = "") -> str:
    value = os.getenv(key)
    return value if value not in (None, "") else default


def parse_node_id(value: Any) -> Optional[int]:
    if value is None:
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return int(value)
    if isinstance(value, str):
        try:
            return int(value)
        except ValueError:
            return None
    return None


def parse_point_type(value: Any) -> Optional[int]:
    if value is None:
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return int(value)
    if isinstance(value, str):
        raw = value.strip()
        if not raw:
            return None
        try:
            return int(raw)
        except ValueError:
            pass
        norm = "".join(ch.upper() for ch in raw if not ch.isspace()).replace("-", "_")
        if norm in ("PICKUP", "取货"):
            return 0
        if norm in ("DROPOFF", "DELIVERY", "放货"):
            return 1
        if norm in ("WAYPOINT", "WAY", "经过点"):
            return 2
        if norm in ("CHARGE", "CHARGING", "充电"):
            return 3
        if norm in ("STANDBY", "WAIT", "待命"):
            return 4
        if norm in ("MAINTENANCE", "MAINTENANCE_CALL", "维护"):
            return 5
        return None
    return None


def parse_float(value: Any, default: float = 0.0) -> float:
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return default
    return default


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def iso_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


@dataclass
class MapGraph:
    nodes: Dict[int, Tuple[float, float]]
    edges: List[Tuple[int, int]]
    neighbors: Dict[int, List[int]]
    edge_set: Set[Tuple[int, int]]


def undirected_edge(a: int, b: int) -> Tuple[int, int]:
    return (a, b) if a <= b else (b, a)


def load_map_graph(map_path: Path) -> MapGraph:
    data = load_json(map_path)
    if isinstance(data, dict) and "mapData" in data:
        data = data["mapData"]
    nodes: Dict[int, Tuple[float, float]] = {}
    edges: List[Tuple[int, int]] = []

    node_list = []
    if isinstance(data, dict):
        node_list = data.get("node") or data.get("nodes") or []
    for node in node_list:
        node_id = parse_node_id(node.get("id", node.get("nodeId")))
        if node_id is None:
            continue
        x = parse_float(node.get("x"))
        y = parse_float(node.get("y"))
        nodes[node_id] = (x, y)

    edge_list = []
    if isinstance(data, dict):
        edge_list = data.get("edge") or data.get("edges") or []
    for edge in edge_list:
        start = parse_node_id(
            edge.get("startId", edge.get("startNodeId", edge.get("startNode")))
        )
        end = parse_node_id(
            edge.get("endId", edge.get("endNodeId", edge.get("endNode")))
        )
        if start is None or end is None:
            continue
        if start == end:
            continue
        edges.append((start, end))

    neighbors: Dict[int, List[int]] = {node_id: [] for node_id in nodes.keys()}
    edge_set: Set[Tuple[int, int]] = set()
    for start, end in edges:
        if start not in nodes or end not in nodes:
            continue
        neighbors.setdefault(start, []).append(end)
        neighbors.setdefault(end, []).append(start)
        edge_set.add(undirected_edge(start, end))

    for node_id in neighbors:
        neighbors[node_id] = sorted(set(neighbors[node_id]))
    unique_edges = sorted(edge_set)
    return MapGraph(nodes=nodes, edges=unique_edges, neighbors=neighbors, edge_set=edge_set)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)


def choose_nodes(rng: random.Random, node_ids: List[int], count: int) -> List[int]:
    if not node_ids:
        return []
    if count <= len(node_ids):
        return rng.sample(node_ids, count)
    return [rng.choice(node_ids) for _ in range(count)]


def estimate_congestion_cell_size(graph: MapGraph) -> float:
    if not graph.edges:
        return 0.0
    dists: List[float] = []
    for idx, (a, b) in enumerate(graph.edges):
        if idx >= 2000:
            break
        pa = graph.nodes.get(a)
        pb = graph.nodes.get(b)
        if not pa or not pb:
            continue
        dist = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
        if dist > 0:
            dists.append(dist)
    if not dists:
        return 0.0
    dists.sort()
    return float(dists[len(dists) // 2])


def build_congestion_nodes_by_window(
    graph: MapGraph,
    start_node_by_device_id: Optional[Dict[str, int]],
) -> Set[int]:
    avoid = parse_int_env("SIM_AVOID_CONGESTION", 1) > 0
    if not avoid or not start_node_by_device_id:
        return set()
    min_count = max(1, parse_int_env("SIM_CONGESTION_MIN_COUNT", 2))
    window = max(1, parse_int_env("SIM_CONGESTION_WINDOW", 3))
    if window % 2 == 0:
        window += 1
    half = window // 2
    cell_size = parse_float_env("SIM_CONGESTION_CELL_MM", 0.0)
    if cell_size <= 0.0:
        cell_size = estimate_congestion_cell_size(graph)
    if cell_size <= 0.0:
        cell_size = 1000.0

    def node_cell(node_id: int) -> Optional[Tuple[int, int]]:
        pos = graph.nodes.get(node_id)
        if not pos:
            return None
        return (
            int(math.floor(pos[0] / cell_size)),
            int(math.floor(pos[1] / cell_size)),
        )

    cell_counts: Dict[Tuple[int, int], int] = {}
    for nid in start_node_by_device_id.values():
        try:
            node_id = int(nid)
        except Exception:
            continue
        cell = node_cell(node_id)
        if cell is None:
            continue
        cell_counts[cell] = cell_counts.get(cell, 0) + 1
    if not cell_counts:
        return set()

    window_counts: Dict[Tuple[int, int], int] = {}
    for (cx, cy), cnt in cell_counts.items():
        for dx in range(-half, half + 1):
            for dy in range(-half, half + 1):
                key = (cx + dx, cy + dy)
                window_counts[key] = window_counts.get(key, 0) + cnt
    congested_cells = {cell for cell, cnt in window_counts.items() if cnt >= min_count}
    if not congested_cells:
        return set()

    congested_nodes: Set[int] = set()
    for node_id in graph.nodes.keys():
        cell = node_cell(node_id)
        if cell in congested_cells:
            congested_nodes.add(node_id)
    return congested_nodes


def generate_status_list(
    graph: MapGraph,
    count: int,
    rng: random.Random,
    map_id: int,
    device_prefix: str,
    candidate_nodes: Optional[List[int]] = None,
) -> List[Dict[str, Any]]:
    node_ids = list(candidate_nodes) if candidate_nodes else list(graph.nodes.keys())
    chosen = choose_nodes(rng, node_ids, count)
    statuses: List[Dict[str, Any]] = []
    now_ts = int(time.time())
    for idx, node_id in enumerate(chosen, start=1):
        x, y = graph.nodes.get(node_id, (0.0, 0.0))
        device_id = f"{device_prefix}{idx:02d}"
        statuses.append(
            {
                "deviceId": device_id,
                "mapId": map_id,
                "curArea": "",
                "connection": True,
                "x": x,
                "y": y,
                "angle": rng.choice([0, 90, 180, 270]),
                "speed": 0,
                "nodeId": str(node_id),
                "nextDestinationPoint": {},
                "curTrailPoints": [],
                "taskId": "",
                "taskStatus": 0,
                "taskProgress": 0,
                "estimatedDuration": 0,
                "batteryLevel": rng.randint(60, 100),
                "endurance": rng.randint(60, 180),
                "load": False,
                "errorCode": 0,
                "updateTime": now_ts,
                "agv_type": 1,
            }
        )
    return statuses


def generate_task_payload(
    graph: MapGraph,
    count: int,
    rng: random.Random,
    map_version: str,
    *,
    bind_to_device_ids: Optional[List[str]] = None,
    start_node_by_device_id: Optional[Dict[str, int]] = None,
    allocation_algo: Optional[str] = None,
) -> Dict[str, Any]:
    def pick_end_node(start_node: int) -> int:
        candidates = [n for n in node_ids if n != start_node]
        if not candidates:
            return int(start_node)
        return int(rng.choice(candidates))

    def pick_targets_from_start(start_node: int) -> List[int]:
        end_node = pick_end_node(start_node)
        if end_node == start_node:
            return []
        route = bfs_shortest_route(graph, start_node, end_node)
        if not route or len(route) < 2:
            return []
        max_targets = min(3, len(route) - 1)
        num_targets = int(rng.randint(1, max_targets))
        if num_targets <= 1:
            return [route[-1]]
        pick_count = num_targets - 1
        candidates = list(range(1, len(route) - 1))
        picked = sorted(rng.sample(candidates, k=pick_count))
        targets = [route[i] for i in picked] + [route[-1]]
        # Defensive: route is simple, but ensure no accidental duplicates.
        out: List[int] = []
        seen: Set[int] = set()
        for n in targets:
            if n in seen:
                continue
            seen.add(n)
            out.append(n)
        return out

    def pick_priority() -> int:
        min_pri = max(1, parse_int_env("SIM_TASK_PRIORITY_MIN", 1))
        max_pri = max(min_pri, parse_int_env("SIM_TASK_PRIORITY_MAX", 9))
        return int(rng.randint(min_pri, max_pri))

    node_ids = list(graph.nodes.keys())
    pickup_ms = max(0, parse_int_env("SIM_SUBTASK_PICKUP_DURATION_MS", 0))
    dropoff_ms = max(0, parse_int_env("SIM_SUBTASK_DROPOFF_DURATION_MS", 0))
    waypoint_ms = max(0, parse_int_env("SIM_SUBTASK_WAYPOINT_DURATION_MS", 0))
    tasks: List[Dict[str, Any]] = []
    for idx in range(1, count + 1):
        task_id = f"TASK_{idx:03d}"

        start_node: Optional[int] = None
        if start_node_by_device_id and bind_to_device_ids:
            bind_device = bind_to_device_ids[(idx - 1) % len(bind_to_device_ids)]
            if bind_device in start_node_by_device_id:
                start_node = start_node_by_device_id[bind_device]
        if start_node is None:
            start_node = rng.choice(node_ids)
        targets = pick_targets_from_start(int(start_node))
        if not targets:
            targets = [int(rng.choice([n for n in node_ids if n != start_node] or node_ids))]
        sub_tasks: List[Dict[str, Any]] = []
        total = len(targets)
        for seq, node_id in enumerate(targets, start=1):
            if total <= 1:
                point_type = 1
                est_ms = dropoff_ms
            elif seq == 1:
                point_type = 0
                est_ms = pickup_ms
            elif seq == total:
                point_type = 1
                est_ms = dropoff_ms
            else:
                point_type = 2
                est_ms = waypoint_ms
            sub_tasks.append(
                {
                    "sequence": seq,
                    "pointType": point_type,
                    "location": f"node_{node_id}",
                    "point": {
                        "x": graph.nodes.get(node_id, (0.0, 0.0))[0],
                        "y": graph.nodes.get(node_id, (0.0, 0.0))[1],
                        "angle": 0,
                        "nodeId": str(node_id),
                    },
                    "estimatedDuration": int(est_ms),
                    "pathConstraints": [],
                    "maxSpeed": 0,
                    "subTaskId": f"{task_id}#{seq}",
                }
            )
        tasks.append(
            {
                "taskId": task_id,
                "priority": pick_priority(),
                "minBatteryLevel": 0,
                "subTasks": sub_tasks,
                "agvRequirements": [],
            }
        )
    payload = {
        "schedulingRequestId": f"SIM_REQ_{int(time.time() * 1000)}",
        "requestTimestamp": iso_timestamp(),
        "requestType": 0,
        "taskNumber": len(tasks),
        "triggerContext": {"type": "sim"},
        "triggerAgvId": [],
        "batchConfig": {"maxBatchSize": len(tasks)},
        "timeoutMs": 2000,
        "environmentContext": {
            "mapVersion": map_version,
            "systemLoad": 1,
            "systemLoadFactor": 0.0,
        },
        "candidateTasks": tasks,
    }
    if allocation_algo:
        payload["allocationAlgorithm"] = str(allocation_algo)
    return payload


def generate_loop_tasks_payload(
    graph: MapGraph,
    device_ids: List[str],
    rng: random.Random,
    map_version: str,
    *,
    cycle: int,
    start_node_by_device_id: Optional[Dict[str, int]] = None,
    forbidden_end_nodes: Optional[Set[int]] = None,
    bind_tasks: bool = False,
    allocation_algo: Optional[str] = None,
) -> Dict[str, Any]:
    def pick_end_node(start_node: int, forbidden: Set[int]) -> int:
        candidates = [n for n in node_ids if n != start_node and n not in forbidden]
        if not candidates:
            return int(start_node)
        return int(rng.choice(candidates))

    def pick_targets_from_route(route: List[int]) -> List[int]:
        if not route or len(route) < 2:
            return []
        max_targets = min(3, len(route) - 1)
        num_targets = int(rng.randint(1, max_targets))
        if num_targets <= 1:
            return [route[-1]]
        pick_count = num_targets - 1
        candidates = list(range(1, len(route) - 1))
        picked = sorted(rng.sample(candidates, k=pick_count))
        return [route[i] for i in picked] + [route[-1]]

    def pick_priority() -> int:
        min_pri = max(1, parse_int_env("SIM_TASK_PRIORITY_MIN", 1))
        max_pri = max(min_pri, parse_int_env("SIM_TASK_PRIORITY_MAX", 9))
        return int(rng.randint(min_pri, max_pri))

    node_ids = list(graph.nodes.keys())
    pickup_ms = max(0, parse_int_env("SIM_SUBTASK_PICKUP_DURATION_MS", 0))
    dropoff_ms = max(0, parse_int_env("SIM_SUBTASK_DROPOFF_DURATION_MS", 0))
    waypoint_ms = max(0, parse_int_env("SIM_SUBTASK_WAYPOINT_DURATION_MS", 0))
    start_node_for: Dict[str, int] = {}
    for device_id in device_ids:
        start_node: Optional[int] = None
        if start_node_by_device_id and device_id in start_node_by_device_id:
            start_node = start_node_by_device_id[device_id]
        if start_node is None:
            start_node = rng.choice(node_ids)
        start_node_for[device_id] = int(start_node)

    current_nodes = set(start_node_for.values())
    global_forbidden_nodes: Set[int] = set(forbidden_end_nodes or set())
    global_forbidden_nodes |= current_nodes
    used_end_nodes: Set[int] = set()
    end_node_for: Dict[str, int] = {}
    for device_id in device_ids:
        start_node = start_node_for[device_id]
        forbidden = set(global_forbidden_nodes)
        forbidden.discard(start_node)
        candidates = [n for n in node_ids if n != start_node and n not in forbidden and n not in used_end_nodes]
        if not candidates:
            candidates = [n for n in node_ids if n != start_node and n not in forbidden]
        if not candidates:
            candidates = [n for n in node_ids if n != start_node and n not in used_end_nodes]
        if not candidates:
            candidates = [n for n in node_ids if n != start_node]
        end_node = start_node if not candidates else pick_end_node(start_node, forbidden)
        end_node_for[device_id] = end_node
        used_end_nodes.add(end_node)

    tasks: List[Dict[str, Any]] = []
    for device_id in device_ids:
        task_id = f"LOOP_{cycle:04d}_{device_id}"
        start_node = start_node_for[device_id]
        end_node = end_node_for[device_id]
        route = bfs_shortest_route(graph, start_node, end_node) or [start_node, end_node]
        targets = pick_targets_from_route(route)
        if not targets:
            targets = [end_node]
        sub_tasks: List[Dict[str, Any]] = []
        total = len(targets)
        for seq, node_id in enumerate(targets, start=1):
            if total <= 1:
                point_type = 1
                est_ms = dropoff_ms
            elif seq == 1:
                point_type = 0
                est_ms = pickup_ms
            elif seq == total:
                point_type = 1
                est_ms = dropoff_ms
            else:
                point_type = 2
                est_ms = waypoint_ms
            sub_tasks.append(
                {
                    "sequence": seq,
                    "pointType": point_type,
                    "location": f"node_{node_id}",
                    "point": {
                        "x": graph.nodes.get(node_id, (0.0, 0.0))[0],
                        "y": graph.nodes.get(node_id, (0.0, 0.0))[1],
                        "angle": 0,
                        "nodeId": str(node_id),
                    },
                    "estimatedDuration": int(est_ms),
                    "pathConstraints": [],
                    "maxSpeed": 0,
                    "subTaskId": f"{task_id}#{seq}",
                }
            )
        tasks.append(
            {
                "taskId": task_id,
                "priority": pick_priority(),
                "minBatteryLevel": 0,
                "subTasks": sub_tasks,
                "agvRequirements": [device_id] if bind_tasks else [],
            }
        )

    payload = {
        "schedulingRequestId": f"SIM_LOOP_REQ_{cycle}_{int(time.time() * 1000)}",
        "requestTimestamp": iso_timestamp(),
        "requestType": 0,
        "taskNumber": len(tasks),
        "triggerContext": {"type": "sim-loop"},
        "triggerAgvId": [],
        "batchConfig": {"maxBatchSize": len(tasks)},
        "timeoutMs": 2000,
        "environmentContext": {
            "mapVersion": map_version,
            "systemLoad": 1,
            "systemLoadFactor": 0.0,
        },
        "candidateTasks": tasks,
    }
    if allocation_algo:
        payload["allocationAlgorithm"] = str(allocation_algo)
    return payload


def build_status_entry(
    graph: MapGraph,
    *,
    device_id: str,
    node_id: int,
    map_id: int,
    angle: float = 0.0,
    battery_level: int = 80,
    endurance: int = 120,
    load: bool = False,
    now_ts: Optional[int] = None,
) -> Dict[str, Any]:
    ts = int(time.time()) if now_ts is None else int(now_ts)
    x, y = graph.nodes.get(node_id, (0.0, 0.0))
    return {
        "deviceId": device_id,
        "mapId": map_id,
        "curArea": "",
        "connection": True,
        "x": x,
        "y": y,
        "angle": angle,
        "speed": 0,
        "nodeId": str(node_id),
        "nextDestinationPoint": {},
        "curTrailPoints": [],
        "taskId": "",
        "taskStatus": 0,
        "taskProgress": 0,
        "estimatedDuration": 0,
        "batteryLevel": max(0, min(100, int(battery_level))),
        "endurance": int(endurance),
        "load": bool(load),
        "errorCode": 0,
        "updateTime": ts,
        "agv_type": 1,
    }


def pick_demo_corridor_nodes(graph: MapGraph) -> Optional[Tuple[int, int]]:
    if 12 in graph.nodes and 16 in graph.nodes:
        return 12, 16

    by_y: Dict[float, List[Tuple[float, int]]] = {}
    for node_id, (x, y) in graph.nodes.items():
        by_y.setdefault(round(float(y), 3), []).append((float(x), node_id))

    best: Optional[Tuple[float, int, int]] = None
    for _, nodes in by_y.items():
        if len(nodes) < 4:
            continue
        nodes.sort()
        left = nodes[0][1]
        right = nodes[-1][1]
        if bfs_shortest_route(graph, left, right) is None:
            continue
        span = nodes[-1][0] - nodes[0][0]
        if best is None or span > best[0]:
            best = (span, left, right)

    if best is not None:
        return best[1], best[2]
    return None


def build_two_agv_avoidance_demo(graph: MapGraph, *, map_id: int, map_version: str) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    corridor = pick_demo_corridor_nodes(graph)
    if corridor is None:
        raise RuntimeError("demo-avoidance requires a map with a clear corridor (could not auto-pick nodes)")
    left, right = corridor

    # Prefer an intersection-style conflict:
    # - AGV01 goes along the corridor left -> right (priority).
    # - AGV02 approaches an intermediate corridor node from a side branch (yield).
    # This avoids the "head-on swap into each other's start node" deadlock case.
    corridor_route = bfs_shortest_route(graph, left, right) or [left, right]
    corridor_set = set(corridor_route)
    merge_node: Optional[int] = None
    branch_start: Optional[int] = None
    for node in corridor_route[1:-1]:
        branch_neighbors = [nb for nb in graph.neighbors.get(node, []) if nb not in corridor_set]
        if not branch_neighbors:
            continue
        picked_start: Optional[int] = None
        for entry in branch_neighbors:
            further = [x for x in graph.neighbors.get(entry, []) if x != node and x not in corridor_set]
            if further:
                picked_start = further[0]
                break
        merge_node = node
        branch_start = picked_start or branch_neighbors[0]
        break
    if merge_node is None or branch_start is None:
        # Fallback: the old swap demo (may deadlock on some maps), but keep deterministic.
        merge_node = left
        branch_start = right

    now_ts = int(time.time())
    status_list = [
        build_status_entry(graph, device_id="AGV01", node_id=left, map_id=map_id, angle=0.0, battery_level=90, now_ts=now_ts),
        build_status_entry(graph, device_id="AGV02", node_id=branch_start, map_id=map_id, angle=180.0, battery_level=70, now_ts=now_ts),
    ]

    def task(task_id: str, *, device_id: str, start: int, end: int, priority: int) -> Dict[str, Any]:
        sx, sy = graph.nodes.get(start, (0.0, 0.0))
        ex, ey = graph.nodes.get(end, (0.0, 0.0))
        pickup_ms = max(0, parse_int_env("SIM_SUBTASK_PICKUP_DURATION_MS", 0))
        dropoff_ms = max(0, parse_int_env("SIM_SUBTASK_DROPOFF_DURATION_MS", 0))
        return {
            "taskId": task_id,
            "priority": int(priority),
            "minBatteryLevel": 0,
            "subTasks": [
                {
                    "sequence": 1,
                    "pointType": 0,
                    "location": f"node_{start}",
                    "point": {"x": sx, "y": sy, "angle": 0, "nodeId": str(start)},
                    "estimatedDuration": int(pickup_ms),
                    "pathConstraints": [],
                    "maxSpeed": 0,
                    "subTaskId": f"{task_id}#1",
                },
                {
                    "sequence": 2,
                    "pointType": 1,
                    "location": f"node_{end}",
                    "point": {"x": ex, "y": ey, "angle": 0, "nodeId": str(end)},
                    "estimatedDuration": int(dropoff_ms),
                    "pathConstraints": [],
                    "maxSpeed": 0,
                    "subTaskId": f"{task_id}#2",
                },
            ],
            # This makes the assignment deterministic for the demo.
            "agvRequirements": [device_id],
        }

    tasks = [
        task("DEMO_AGV01_LEFT_TO_RIGHT", device_id="AGV01", start=left, end=right, priority=10),
        task("DEMO_AGV02_YIELD", device_id="AGV02", start=branch_start, end=merge_node, priority=1),
    ]

    tasks_payload: Dict[str, Any] = {
        "schedulingRequestId": f"SIM_DEMO_REQ_{int(time.time() * 1000)}",
        "requestTimestamp": iso_timestamp(),
        "requestType": 0,
        "taskNumber": len(tasks),
        "triggerContext": {"type": "sim-demo-avoidance"},
        "triggerAgvId": [],
        "batchConfig": {"maxBatchSize": len(tasks)},
        "timeoutMs": 2000,
        "environmentContext": {"mapVersion": map_version, "systemLoad": 1, "systemLoadFactor": 0.0},
        "candidateTasks": tasks,
    }
    return status_list, tasks_payload


@dataclass
class PathPoint:
    node_id: int
    x: float
    y: float
    yaw: float


def point_from_dict(pt: Dict[str, Any], graph: MapGraph) -> Optional[PathPoint]:
    node_id = parse_node_id(pt.get("nodeId"))
    if node_id is None:
        return None
    # For map alignment, prefer node coordinates over payload x/y when available.
    if node_id in graph.nodes:
        x, y = graph.nodes[node_id]
    else:
        x = parse_float(pt.get("x"))
        y = parse_float(pt.get("y"))
    yaw = parse_float(pt.get("yaw", pt.get("angle", 0)))
    return PathPoint(node_id=node_id, x=x, y=y, yaw=yaw)


def dedup_consecutive(points: List[PathPoint]) -> List[PathPoint]:
    if not points:
        return []
    result = [points[0]]
    for pt in points[1:]:
        if pt.node_id != result[-1].node_id:
            result.append(pt)
    return result


def route_exists(graph: MapGraph, start: int, end: int) -> bool:
    if start == end:
        return True
    return undirected_edge(start, end) in graph.edge_set


def bfs_shortest_route(graph: MapGraph, start: int, goal: int, max_nodes: int = 20000) -> Optional[List[int]]:
    if start == goal:
        return [start]
    if start not in graph.nodes or goal not in graph.nodes:
        return None
    visited: Dict[int, Optional[int]] = {start: None}
    queue: Deque[int] = deque([start])
    while queue and len(visited) < max_nodes:
        node = queue.popleft()
        for nb in graph.neighbors.get(node, []):
            if nb in visited:
                continue
            visited[nb] = node
            if nb == goal:
                queue.clear()
                break
            queue.append(nb)
    if goal not in visited:
        return None
    route: List[int] = []
    cur: Optional[int] = goal
    while cur is not None:
        route.append(cur)
        cur = visited[cur]
    route.reverse()
    return route


def expand_node_route(graph: MapGraph, nodes: List[int]) -> Tuple[List[int], List[Tuple[int, int]]]:
    if not nodes:
        return [], []
    expanded: List[int] = [nodes[0]]
    missing: List[Tuple[int, int]] = []
    for nxt in nodes[1:]:
        prev = expanded[-1]
        if nxt == prev:
            continue
        if route_exists(graph, prev, nxt):
            expanded.append(nxt)
            continue
        missing.append((prev, nxt))
        # Do not auto-repair/expand across missing edges: report and truncate so
        # the simulator can surface algorithm issues (and avoid drawing/moving
        # along non-existent edges).
        break
    return expanded, missing


def yaws_for_route(graph: MapGraph, nodes: List[int]) -> List[float]:
    if not nodes:
        return []
    yaws: List[float] = []
    for idx, node_id in enumerate(nodes):
        if idx + 1 < len(nodes) and node_id in graph.nodes and nodes[idx + 1] in graph.nodes:
            x1, y1 = graph.nodes[node_id]
            x2, y2 = graph.nodes[nodes[idx + 1]]
            yaw = (math.degrees(math.atan2(y2 - y1, x2 - x1)) + 360.0) % 360.0
        elif yaws:
            yaw = yaws[-1]
        else:
            yaw = 0.0
        yaws.append(yaw)
    return yaws


def snap_points_to_map(graph: MapGraph, points: List[PathPoint], label: str) -> List[PathPoint]:
    points = dedup_consecutive(points)
    if not points:
        return []

    nodes = [pt.node_id for pt in points]
    unknown = sorted({nid for nid in nodes if nid not in graph.nodes})
    missing_edges = []
    for a, b in zip(nodes, nodes[1:]):
        if a in graph.nodes and b in graph.nodes and not route_exists(graph, a, b):
            missing_edges.append((a, b))

    if unknown or missing_edges:
        first_invalid = ""
        prev: Optional[int] = None
        for nid in nodes:
            if nid not in graph.nodes:
                first_invalid = f"first_invalid_node={nid}"
                break
            if prev is not None and prev != nid and not route_exists(graph, prev, nid):
                first_invalid = f"first_invalid_edge=({prev}->{nid})"
                break
            prev = nid

        parts = []
        if unknown:
            shown = unknown[:10]
            parts.append(f"unknown_nodes={shown}{'...' if len(unknown) > len(shown) else ''}")
        if missing_edges:
            shown = missing_edges[:10]
            parts.append(f"missing_edges={shown}{'...' if len(missing_edges) > len(shown) else ''}")
        detail = f" ({first_invalid})" if first_invalid else ""
        preview = nodes[:20]
        print(
            f"[ERROR] {label}: route contains invalid items ({'; '.join(parts)}); "
            f"nodes_preview={preview}{'...' if len(nodes) > len(preview) else ''}; "
            f"truncating at first invalid edge{detail}.",
              file=sys.stderr)

    expanded: List[int] = []
    prev_node: Optional[int] = None
    for nid in nodes:
        if nid not in graph.nodes:
            break
        if prev_node is not None and prev_node != nid and not route_exists(graph, prev_node, nid):
            break
        if not expanded or expanded[-1] != nid:
            expanded.append(nid)
        prev_node = nid

    if not expanded:
        return []
    yaws = yaws_for_route(graph, expanded)
    snapped: List[PathPoint] = []
    for node_id, yaw in zip(expanded, yaws):
        x, y = graph.nodes[node_id]
        snapped.append(PathPoint(node_id=node_id, x=x, y=y, yaw=yaw))
    return snapped


class SimState:
    def __init__(
        self,
        status_list: List[Dict[str, Any]],
        graph: MapGraph,
        trail_preview: int,
        *,
        drive_mode: str = "trail",
    ) -> None:
        self._lock = threading.Lock()
        self.graph = graph
        self.drive_mode = drive_mode
        self.status_by_id: Dict[str, Dict[str, Any]] = {}
        for item in status_list:
            device_id = item.get("deviceId")
            if device_id:
                self.status_by_id[device_id] = item
        self.path_by_id: Dict[str, List[PathPoint]] = {}
        self.trail_by_id: Dict[str, List[PathPoint]] = {}
        self.drive_by_id: Dict[str, List[PathPoint]] = {}
        self.drive_index: Dict[str, int] = {}
        self.drive_progress: Dict[str, float] = {}
        self.drive_source: Dict[str, str] = {}
        self.trail_preview = max(0, trail_preview)
        self._task_defs: Dict[str, List[Dict[str, Any]]] = {}
        self._assigned_task_ids: Dict[str, List[str]] = {}
        self._task_owner_by_id: Dict[str, str] = {}
        self._task_reassign_warned: Set[str] = set()
        self._subtask_markers_by_device: Dict[str, List[Dict[str, Any]]] = {}
        self._sticky_subtask_markers_by_device: Dict[str, List[Dict[str, Any]]] = {}
        self._subtask_reach_tol_mm = max(0.0, float(parse_float_env("SIM_SUBTASK_REACH_TOL_MM", 50.0)))
        # Optional: enforce a minimum dwell time (seconds) at each subtask point, even if the task
        # itself has estimatedDuration=0. Default 0 means "fully follow estimatedDuration".
        self._subtask_min_dwell_s = max(0.0, float(parse_float_env("SIM_SUBTASK_MIN_DWELL_S", 0.0)))
        self._subtask_dwell_remaining_s_by_device: Dict[str, float] = {}
        self._subtask_dwell_total_s_by_device: Dict[str, float] = {}
        self._pending_drive_refresh: Set[str] = set()
        # Metrics (simulation-side KPIs)
        self._metrics_start_s: Optional[float] = None
        self._task_publish_s: Dict[str, float] = {}
        self._task_start_s: Dict[str, float] = {}
        self._task_complete_s: Dict[str, float] = {}
        self._task_distance_mm: Dict[str, float] = {}
        self._completed_task_distance_mm = 0.0
        self._tasks_completed = 0
        self._publish_to_start_sum_s = 0.0
        self._publish_to_start_count = 0
        self._distance_total_mm = 0.0
        self._last_pos_by_device: Dict[str, Tuple[float, float]] = {}
        self._reserved_nodes: List[Dict[str, Any]] = []

    def _metrics_now_s(self) -> float:
        return time.monotonic()

    def _metrics_note_tasks_published_locked(self, task_ids: List[str]) -> None:
        if not task_ids:
            return
        now = self._metrics_now_s()
        if self._metrics_start_s is None:
            self._metrics_start_s = now
        for tid in task_ids:
            if tid and tid not in self._task_publish_s:
                self._task_publish_s[tid] = now

    def _metrics_note_task_start_locked(self, task_id: str) -> None:
        if not task_id or task_id in self._task_start_s:
            return
        now = self._metrics_now_s()
        if self._metrics_start_s is None:
            self._metrics_start_s = now
        self._task_start_s[task_id] = now
        pub = self._task_publish_s.get(task_id)
        if pub is not None:
            self._publish_to_start_sum_s += max(0.0, now - pub)
            self._publish_to_start_count += 1

    def _metrics_note_task_complete_locked(self, task_id: str) -> None:
        if not task_id or task_id in self._task_complete_s:
            return
        now = self._metrics_now_s()
        if self._metrics_start_s is None:
            self._metrics_start_s = now
        self._task_complete_s[task_id] = now
        self._tasks_completed += 1
        self._completed_task_distance_mm += self._task_distance_mm.get(task_id, 0.0)

    def _metrics_update_distances_locked(self) -> None:
        for device_id, status in self.status_by_id.items():
            if "x" not in status or "y" not in status:
                continue
            x = parse_float(status.get("x"), 0.0)
            y = parse_float(status.get("y"), 0.0)
            prev = self._last_pos_by_device.get(device_id)
            if prev is not None:
                dist = math.hypot(x - prev[0], y - prev[1])
                if dist > 0.0:
                    self._distance_total_mm += dist
                    task_id = str(status.get("taskId", "")).strip()
                    if task_id:
                        self._task_distance_mm[task_id] = self._task_distance_mm.get(task_id, 0.0) + dist
            self._last_pos_by_device[device_id] = (x, y)

    def snapshot_metrics(self) -> Dict[str, Any]:
        with self._lock:
            now = self._metrics_now_s()
            if self._metrics_start_s is None:
                elapsed = 0.0
            else:
                elapsed = max(0.0, now - self._metrics_start_s)
            avg_publish_to_start = (
                self._publish_to_start_sum_s / self._publish_to_start_count
                if self._publish_to_start_count > 0
                else 0.0
            )
            throughput = (self._tasks_completed / elapsed * 60.0) if elapsed > 1e-9 else 0.0
            return {
                "tasks_published": len(self._task_publish_s),
                "tasks_started": len(self._task_start_s),
                "tasks_completed": self._tasks_completed,
                "avg_publish_to_start_s": avg_publish_to_start,
                "total_distance_mm": self._distance_total_mm,
                "completed_task_distance_mm": self._completed_task_distance_mm,
                "throughput_tasks_per_min": throughput,
                "elapsed_s": elapsed,
            }

    def _sync_sticky_subtask_markers_locked(self, device_id: str, rebuilt: List[Dict[str, Any]]) -> None:
        if not device_id:
            return
        existing = self._sticky_subtask_markers_by_device.get(device_id)
        if not existing:
            if rebuilt:
                self._sticky_subtask_markers_by_device[device_id] = rebuilt
            return
        if not rebuilt:
            return
        existing_by_sub_id: Dict[str, Dict[str, Any]] = {}
        for m in existing:
            sub_id = str(m.get("sub_task_id", "")).strip()
            if sub_id:
                existing_by_sub_id[sub_id] = m
        first_sub_id = str(existing[0].get("sub_task_id", "")).strip()
        start_idx: Optional[int] = None
        if first_sub_id:
            for i, m in enumerate(rebuilt):
                if str(m.get("sub_task_id", "")).strip() == first_sub_id:
                    start_idx = i
                    break
        new_markers: List[Dict[str, Any]] = []
        if start_idx is not None:
            for m in rebuilt[start_idx:]:
                sub_id = str(m.get("sub_task_id", "")).strip()
                new_markers.append(existing_by_sub_id.get(sub_id) or m)
        else:
            # Preserve current progress and only append truly new markers.
            new_markers = list(existing)
            known = set(existing_by_sub_id.keys())
            for m in rebuilt:
                sub_id = str(m.get("sub_task_id", "")).strip()
                if not sub_id or sub_id in known:
                    continue
                known.add(sub_id)
                new_markers.append(m)
        self._sticky_subtask_markers_by_device[device_id] = new_markers

    def _marker_at_current_node_locked(self, device_id: str, status: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if not device_id:
            return None
        markers = self._sticky_subtask_markers_by_device.get(device_id)
        if not markers:
            return None
        cur_node = parse_node_id(status.get("nodeId"))
        if cur_node is None:
            return None
        node_id = markers[0].get("node_id")
        if node_id is None or int(node_id) != int(cur_node):
            return None
        if self._subtask_reach_tol_mm > 0.0 and "x" in status and "y" in status:
            pos = self.graph.nodes.get(int(cur_node))
            if pos is not None:
                px = parse_float(status.get("x"), float(pos[0]))
                py = parse_float(status.get("y"), float(pos[1]))
                if math.hypot(px - float(pos[0]), py - float(pos[1])) > self._subtask_reach_tol_mm:
                    return None
        return markers[0]

    def _start_subtask_dwell_locked(self, device_id: str, marker: Dict[str, Any]) -> bool:
        if not device_id:
            return False
        dwell_s = parse_float(marker.get("dwell_s"), 0.0)
        dwell_s = max(float(dwell_s), float(self._subtask_min_dwell_s))
        if dwell_s <= 1e-9:
            return False
        self._subtask_dwell_remaining_s_by_device[device_id] = float(dwell_s)
        self._subtask_dwell_total_s_by_device[device_id] = float(dwell_s)
        return True

    def _refresh_status_task_fields_locked(self, device_id: str, status: Dict[str, Any]) -> None:
        if not device_id or not isinstance(status, dict):
            return
        queue = self._assigned_task_ids.get(device_id, [])
        if not queue:
            status["taskId"] = ""
            status["taskStatus"] = 0
            status["taskProgress"] = 0
            return
        current_task_id = str(queue[0])
        status["taskId"] = current_task_id
        status["taskStatus"] = 1
        self._metrics_note_task_start_locked(current_task_id)
        total = len(self._task_defs.get(current_task_id, []) or [])
        if total <= 0:
            status["taskProgress"] = 0
            return
        markers = self._sticky_subtask_markers_by_device.get(device_id) or []
        remaining = 0
        for m in markers:
            if str(m.get("task_id", "")).strip() == current_task_id:
                remaining += 1
        done = max(0, total - remaining)
        pct = int(round((done / total) * 100.0))
        status["taskProgress"] = max(0, min(100, pct))

    def _extract_node_id_from_text(self, value: Any) -> Optional[int]:
        if value is None:
            return None
        if isinstance(value, (int, float, str)):
            direct = parse_node_id(value)
            if direct is not None:
                return direct
        if isinstance(value, str):
            match = re.search(r"(\d+)", value)
            if match:
                try:
                    return int(match.group(1))
                except Exception:
                    return None
        return None

    def _parse_subtask_point(self, subtask: Dict[str, Any]) -> Optional[PathPoint]:
        pt = subtask.get("point")
        if isinstance(pt, dict):
            parsed = point_from_dict(pt, self.graph)
            if parsed is not None:
                return parsed
            node_id = self._extract_node_id_from_text(pt.get("nodeId"))
            if node_id is None:
                node_id = self._extract_node_id_from_text(subtask.get("location"))
            if node_id is None:
                return None
            if node_id in self.graph.nodes:
                x, y = self.graph.nodes[node_id]
            else:
                x = parse_float(pt.get("x"))
                y = parse_float(pt.get("y"))
            yaw = parse_float(pt.get("yaw", pt.get("angle", 0)))
            return PathPoint(node_id=node_id, x=x, y=y, yaw=yaw)

        node_id = self._extract_node_id_from_text(subtask.get("nodeId"))
        if node_id is None:
            node_id = self._extract_node_id_from_text(subtask.get("location"))
        if node_id is None or node_id not in self.graph.nodes:
            return None
        x, y = self.graph.nodes[node_id]
        return PathPoint(node_id=node_id, x=x, y=y, yaw=0.0)

    def _build_subtask_markers_for_device_locked(self, device_id: str, task_ids: List[str]) -> List[Dict[str, Any]]:
        seq = 1
        out: List[Dict[str, Any]] = []
        last_node: Optional[int] = None
        for task_id in task_ids:
            sub_defs = self._task_defs.get(task_id, [])
            for st in sub_defs:
                node_id = st.get("node_id")
                if node_id is None:
                    continue
                if last_node is not None and int(node_id) == int(last_node):
                    continue
                last_node = int(node_id)
                out.append(
                    {
                        "node_id": int(node_id),
                        "x": float(st.get("x", 0.0)),
                        "y": float(st.get("y", 0.0)),
                        "label": f"S{seq}:{int(node_id)}",
                        "task_id": str(task_id),
                        "sub_task_id": str(st.get("sub_task_id", "")),
                        "sequence": int(st.get("sequence", 0) or 0),
                        "point_type": int(st.get("point_type", -1) or -1),
                        "location": str(st.get("location", "") or ""),
                        "dwell_s": float(st.get("dwell_s", 0.0) or 0.0),
                    }
                )
                seq += 1
        return out

    def _rebuild_subtask_markers_locked(self) -> None:
        markers: Dict[str, List[Dict[str, Any]]] = {}
        device_ids = set(self.status_by_id.keys()) | set(self._assigned_task_ids.keys())
        for device_id in device_ids:
            task_ids = self._assigned_task_ids.get(device_id, [])
            markers[device_id] = self._build_subtask_markers_for_device_locked(device_id, task_ids)
        self._subtask_markers_by_device = markers

    def _consume_reached_subtasks_locked(self, device_id: str, status: Dict[str, Any]) -> None:
        if not device_id:
            return
        markers = self._sticky_subtask_markers_by_device.get(device_id)
        if markers is None:
            return
        cur_node = parse_node_id(status.get("nodeId"))
        if cur_node is None:
            return
        if self._subtask_reach_tol_mm > 0.0 and "x" in status and "y" in status:
            pos = self.graph.nodes.get(int(cur_node))
            if pos is not None:
                px = parse_float(status.get("x"), float(pos[0]))
                py = parse_float(status.get("y"), float(pos[1]))
                if math.hypot(px - float(pos[0]), py - float(pos[1])) > self._subtask_reach_tol_mm:
                    return
        changed = False
        popped_task_ids: List[str] = []
        while markers:
            node_id = markers[0].get("node_id")
            if node_id is None or int(node_id) != int(cur_node):
                break
            tid = str(markers[0].get("task_id", "")).strip()
            if tid:
                popped_task_ids.append(tid)
            markers.pop(0)
            changed = True
        if not changed:
            return

        # Pop completed tasks out of the per-AGV queue as soon as their last subtask is consumed.
        remaining_task_ids: Set[str] = set()
        for m in markers:
            tid = str(m.get("task_id", "")).strip()
            if tid:
                remaining_task_ids.add(tid)

        completed_in_order: List[str] = []
        seen_completed: Set[str] = set()
        for tid in popped_task_ids:
            if tid in seen_completed:
                continue
            if tid and tid not in remaining_task_ids:
                seen_completed.add(tid)
                completed_in_order.append(tid)

        if completed_in_order:
            for tid in completed_in_order:
                self._metrics_note_task_complete_locked(tid)
            existing_tasks = self._assigned_task_ids.get(device_id, [])
            if existing_tasks:
                remaining_queue = [tid for tid in existing_tasks if tid not in seen_completed]
                if remaining_queue:
                    self._assigned_task_ids[device_id] = remaining_queue
                else:
                    self._assigned_task_ids.pop(device_id, None)
            for tid in completed_in_order:
                if self._task_owner_by_id.get(tid) == device_id:
                    self._task_owner_by_id.pop(tid, None)

        if not markers:
            self._sticky_subtask_markers_by_device.pop(device_id, None)
        self._refresh_status_task_fields_locked(device_id, status)

    def set_tasks_payload(self, task_payload: Dict[str, Any]) -> None:
        tasks = task_payload.get("candidateTasks", []) if isinstance(task_payload, dict) else []
        task_defs: Dict[str, List[Dict[str, Any]]] = {}
        if isinstance(tasks, list):
            for task in tasks:
                if not isinstance(task, dict):
                    continue
                task_id = str(task.get("taskId", "")).strip()
                if not task_id:
                    continue
                sub_tasks = task.get("subTasks", [])
                parsed_subs: List[Dict[str, Any]] = []
                if isinstance(sub_tasks, list):
                    ordered = []
                    for st in sub_tasks:
                        if isinstance(st, dict):
                            ordered.append(st)
                    ordered.sort(key=lambda x: int(x.get("sequence", 0) or 0))
                    for st in ordered:
                        pp = self._parse_subtask_point(st)
                        if pp is None:
                            continue
                        est_ms = parse_float(st.get("estimatedDuration"), 0.0)
                        dwell_s = 0.0
                        if est_ms > 0.0:
                            dwell_s = max(0.0, float(est_ms) / 1000.0)
                        point_type = parse_point_type(st.get("pointType"))
                        if point_type is None:
                            point_type = parse_point_type(st.get("point_type"))
                        location = str(st.get("location", "") or "").strip()
                        parsed_subs.append(
                            {
                                "node_id": int(pp.node_id),
                                "x": float(pp.x),
                                "y": float(pp.y),
                                "sequence": int(st.get("sequence", 0) or 0),
                                "sub_task_id": str(st.get("subTaskId", "")).strip()
                                or f"{task_id}#{int(st.get('sequence', 0) or 0)}",
                                "point_type": int(point_type) if point_type is not None else -1,
                                "location": location,
                                "dwell_s": float(dwell_s),
                            }
                        )
                task_defs[task_id] = parsed_subs

        with self._lock:
            # Merge so that loop-task batches don't wipe older tasks that are still present.
            # Also: once a task is owned (assigned), keep its definition stable unless we only had a placeholder.
            for task_id, subs in task_defs.items():
                existing = self._task_defs.get(task_id)
                if existing and task_id in self._task_owner_by_id:
                    continue
                self._task_defs[task_id] = subs
            self._rebuild_subtask_markers_locked()
            for device_id, task_ids in self._assigned_task_ids.items():
                if not task_ids:
                    continue
                rebuilt = self._build_subtask_markers_for_device_locked(device_id, task_ids)
                if rebuilt:
                    self._subtask_markers_by_device[device_id] = rebuilt
                self._sync_sticky_subtask_markers_locked(device_id, rebuilt)
                st = self.status_by_id.get(device_id)
                if isinstance(st, dict):
                    self._refresh_status_task_fields_locked(device_id, st)
            self._metrics_note_tasks_published_locked(list(task_defs.keys()))

    def set_assignment_response(self, payload: Dict[str, Any]) -> None:
        if not isinstance(payload, dict):
            return
        assignments = payload.get("agv_assignments")
        if not isinstance(assignments, list):
            return

        assigned: Dict[str, List[str]] = {}
        for agv_entry in assignments:
            if not isinstance(agv_entry, dict):
                continue
            device_id = str(agv_entry.get("agv_id", "")).strip()
            if not device_id:
                continue
            tasks = agv_entry.get("assigned_tasks")
            if not isinstance(tasks, list):
                continue
            seq: List[str] = []
            for t in tasks:
                if not isinstance(t, dict):
                    continue
                tid = str(t.get("task_id", "")).strip()
                if tid:
                    seq.append(tid)
            if seq:
                assigned[device_id] = seq

        with self._lock:
            # Merge assignment into a stable per-AGV queue:
            # - Preserve existing owned tasks (order stays unless emergency insertion is supported).
            # - Append newly-seen tasks.
            # - Reject task reassignment to a different AGV (sim has no emergency preemption support).
            for device_id, seq in assigned.items():
                existing = self._assigned_task_ids.get(device_id, [])
                merged: List[str] = []
                for tid in existing:
                    owner = self._task_owner_by_id.get(tid)
                    if owner is None:
                        self._task_owner_by_id[tid] = device_id
                        owner = device_id
                    if owner == device_id:
                        merged.append(tid)
                for tid in seq:
                    owner = self._task_owner_by_id.get(tid)
                    if owner is None:
                        self._task_owner_by_id[tid] = device_id
                        merged.append(tid)
                    elif owner != device_id:
                        if tid not in self._task_reassign_warned:
                            print(
                                f"[SimAssignWarn] task_id={tid} old_owner={owner} new_owner={device_id} (ignored; no emergency support)",
                                file=sys.stderr,
                            )
                            self._task_reassign_warned.add(tid)
                    elif tid not in merged:
                        merged.append(tid)

                if merged:
                    self._assigned_task_ids[device_id] = merged
                else:
                    self._assigned_task_ids.pop(device_id, None)

                rebuilt = self._build_subtask_markers_for_device_locked(device_id, merged)
                if rebuilt:
                    self._subtask_markers_by_device[device_id] = rebuilt
                self._sync_sticky_subtask_markers_locked(device_id, rebuilt)
                st = self.status_by_id.get(device_id)
                if isinstance(st, dict):
                    self._refresh_status_task_fields_locked(device_id, st)

    def snapshot_subtask_markers(self) -> Dict[str, List[Dict[str, Any]]]:
        with self._lock:
            return {k: [dict(x) for x in v] for k, v in self._sticky_subtask_markers_by_device.items()}

    def assigned_device_ids(self) -> List[str]:
        with self._lock:
            out = [did for did, tids in self._assigned_task_ids.items() if tids]
        out.sort()
        return out

    def next_subtask_id(self, device_id: str) -> Optional[str]:
        if not device_id:
            return None
        with self._lock:
            markers = self._sticky_subtask_markers_by_device.get(device_id, [])
            if not markers:
                return None
            current_node = parse_node_id(self.status_by_id.get(device_id, {}).get("nodeId"))
            for marker in markers:
                node_id = marker.get("node_id")
                if node_id is None:
                    continue
                if current_node is None or int(node_id) != int(current_node):
                    sub_id = str(marker.get("sub_task_id", "")).strip()
                    return sub_id or None
            sub_id = str(markers[-1].get("sub_task_id", "")).strip()
            return sub_id or None

    def device_ids(self) -> List[str]:
        with self._lock:
            return list(self.status_by_id.keys())

    def _clear_drive_locked(self, device_id: str) -> None:
        self.drive_by_id.pop(device_id, None)
        self.drive_index.pop(device_id, None)
        self.drive_progress.pop(device_id, None)
        self.drive_source.pop(device_id, None)
        self._subtask_dwell_remaining_s_by_device.pop(device_id, None)
        self._subtask_dwell_total_s_by_device.pop(device_id, None)

    def _project_progress_ratio(self, status: Dict[str, Any], start_pt: PathPoint, end_pt: PathPoint) -> Optional[float]:
        if "x" not in status or "y" not in status:
            return None
        px = parse_float(status.get("x"))
        py = parse_float(status.get("y"))
        dx = float(end_pt.x) - float(start_pt.x)
        dy = float(end_pt.y) - float(start_pt.y)
        denom = dx * dx + dy * dy
        if denom <= 1e-9:
            return None
        ratio = ((px - float(start_pt.x)) * dx + (py - float(start_pt.y)) * dy) / denom
        return max(0.0, min(1.0, ratio))

    def _refresh_drive_locked(self, device_id: str) -> None:
        if not device_id:
            return
        if self.drive_mode == "path":
            points = self.path_by_id.get(device_id) or []
            if points:
                self._set_drive_locked(device_id, points, source="path")
            else:
                self._clear_drive_locked(device_id)
            return
        if self.drive_mode == "trail":
            points = self.trail_by_id.get(device_id) or []
            if points:
                self._set_drive_locked(device_id, points, source="trail")
            else:
                self._clear_drive_locked(device_id)
            return

        # trail-only (no auto fallback to path)
        trail = self.trail_by_id.get(device_id) or []
        if trail:
            self._set_drive_locked(device_id, trail, source="trail")
        else:
            self._clear_drive_locked(device_id)

    def _set_drive_locked(self, device_id: str, points: List[PathPoint], *, source: str) -> None:
        if not device_id or not points:
            return
        status = self.status_by_id.get(device_id, {})
        current_node = parse_node_id(status.get("nodeId"))
        next_node = None
        next_dest = status.get("nextDestinationPoint")
        if isinstance(next_dest, dict):
            next_node = parse_node_id(next_dest.get("nodeId"))

        existing_route = self.drive_by_id.get(device_id) or []
        existing_idx = int(self.drive_index.get(device_id, 0))
        existing_progress = float(self.drive_progress.get(device_id, 0.0))
        existing_seg: Optional[Tuple[int, int]] = None
        if existing_route and 0 <= existing_idx < len(existing_route) - 1:
            existing_seg = (existing_route[existing_idx].node_id, existing_route[existing_idx + 1].node_id)

        idx = 0
        preserve_progress = False
        if current_node is not None:
            if next_node is not None:
                for i in range(len(points) - 1):
                    if points[i].node_id == current_node and points[i + 1].node_id == next_node:
                        idx = i
                        preserve_progress = True
                        break
            if not preserve_progress:
                candidates = [i for i, pt in enumerate(points) if pt.node_id == current_node]
                if candidates:
                    if existing_route:
                        idx = min(candidates, key=lambda i: abs(i - existing_idx))
                    else:
                        idx = candidates[0]

        if not preserve_progress and existing_seg is not None:
            for i in range(len(points) - 1):
                if points[i].node_id == existing_seg[0] and points[i + 1].node_id == existing_seg[1]:
                    idx = i
                    preserve_progress = True
                    break

        if existing_seg is not None and existing_progress > 1e-3 and not preserve_progress:
            self._pending_drive_refresh.add(device_id)
            return
        self._pending_drive_refresh.discard(device_id)

        self.drive_by_id[device_id] = points
        self.drive_source[device_id] = source
        idx = max(0, min(idx, len(points) - 1))
        self.drive_index[device_id] = idx
        if preserve_progress and idx < len(points) - 1:
            proj_ratio = self._project_progress_ratio(status, points[idx], points[idx + 1])
            if proj_ratio is None:
                prog = float(self.drive_progress.get(device_id, 0.0))
                self.drive_progress[device_id] = max(0.0, min(1.0, prog))
            else:
                self.drive_progress[device_id] = proj_ratio
        else:
            self.drive_progress[device_id] = 0.0

    def set_path(self, device_id: str, points: List[PathPoint]) -> None:
        if not device_id:
            return
        points = dedup_consecutive(points)
        with self._lock:
            if points:
                self.path_by_id[device_id] = points
            else:
                self.path_by_id.pop(device_id, None)
            self._refresh_drive_locked(device_id)

    def set_trail(self, device_id: str, points: List[PathPoint]) -> None:
        if not device_id:
            return
        points = dedup_consecutive(points)
        with self._lock:
            if points:
                self.trail_by_id[device_id] = points
            else:
                self.trail_by_id.pop(device_id, None)
            self._refresh_drive_locked(device_id)

    def set_reserved_nodes(self, nodes: List[Any]) -> None:
        with self._lock:
            normalized: List[Dict[str, Any]] = []
            for item in nodes or []:
                if isinstance(item, dict):
                    normalized.append(dict(item))
                else:
                    normalized.append({"nodeId": item})
            self._reserved_nodes = normalized

    def advance(self, dt: float, edge_duration: float) -> None:
        now = int(time.time())
        dt = max(0.0, float(dt))
        edge_duration = max(0.0, float(edge_duration))
        with self._lock:
            for device_id, status in self.status_by_id.items():
                if device_id in self._pending_drive_refresh:
                    if float(self.drive_progress.get(device_id, 0.0)) <= 1e-6:
                        self._pending_drive_refresh.discard(device_id)
                        self._refresh_drive_locked(device_id)
                dwell_left = float(self._subtask_dwell_remaining_s_by_device.get(device_id, 0.0))
                if dwell_left > 1e-9:
                    dwell_left = max(0.0, dwell_left - dt)
                    if dwell_left <= 1e-9:
                        self._subtask_dwell_remaining_s_by_device.pop(device_id, None)
                        self._subtask_dwell_total_s_by_device.pop(device_id, None)
                        self._consume_reached_subtasks_locked(device_id, status)
                    else:
                        self._subtask_dwell_remaining_s_by_device[device_id] = dwell_left
                    status["speed"] = 0
                    status["updateTime"] = now
                    continue

                route = self.drive_by_id.get(device_id)
                if not route:
                    status["speed"] = 0
                    status["updateTime"] = now
                    status["nextDestinationPoint"] = {}
                    status["curTrailPoints"] = []
                    marker = self._marker_at_current_node_locked(device_id, status)
                    if marker is not None and self._start_subtask_dwell_locked(device_id, marker):
                        continue
                    self._consume_reached_subtasks_locked(device_id, status)
                    continue
                if len(route) == 1:
                    pt = route[0]
                    status["nodeId"] = str(pt.node_id)
                    status["x"] = pt.x
                    status["y"] = pt.y
                    status["angle"] = pt.yaw
                    status["speed"] = 0
                    status["updateTime"] = now
                    status["nextDestinationPoint"] = {}
                    status["curTrailPoints"] = []
                    marker = self._marker_at_current_node_locked(device_id, status)
                    if marker is not None and self._start_subtask_dwell_locked(device_id, marker):
                        continue
                    self._consume_reached_subtasks_locked(device_id, status)
                    continue

                idx = int(self.drive_index.get(device_id, 0))
                idx = max(0, min(idx, len(route) - 1))
                progress = float(self.drive_progress.get(device_id, 0.0))
                duration = edge_duration if edge_duration > 0 else max(dt, 0.01)

                # If already at the end, stay there.
                if idx >= len(route) - 1:
                    pt = route[-1]
                    self.drive_index[device_id] = len(route) - 1
                    self.drive_progress[device_id] = 0.0
                    status["nodeId"] = str(pt.node_id)
                    status["x"] = pt.x
                    status["y"] = pt.y
                    status["angle"] = pt.yaw
                    status["speed"] = 0
                    status["updateTime"] = now
                    status["nextDestinationPoint"] = {}
                    status["curTrailPoints"] = []
                    marker = self._marker_at_current_node_locked(device_id, status)
                    if marker is not None and self._start_subtask_dwell_locked(device_id, marker):
                        continue
                    self._consume_reached_subtasks_locked(device_id, status)
                    continue

                progress += dt / duration
                # Advance across edges if dt is large.
                while progress >= 1.0 and idx + 1 < len(route) - 1:
                    progress -= 1.0
                    idx += 1
                if progress >= 1.0 and idx + 1 == len(route) - 1:
                    idx += 1
                    progress = 0.0

                self.drive_index[device_id] = idx
                self.drive_progress[device_id] = progress

                if idx >= len(route) - 1:
                    pt = route[-1]
                    status["nodeId"] = str(pt.node_id)
                    status["x"] = pt.x
                    status["y"] = pt.y
                    status["angle"] = pt.yaw
                    status["speed"] = 0
                    status["updateTime"] = now
                    status["nextDestinationPoint"] = {}
                    status["curTrailPoints"] = []
                    marker = self._marker_at_current_node_locked(device_id, status)
                    if marker is not None and self._start_subtask_dwell_locked(device_id, marker):
                        continue
                    self._consume_reached_subtasks_locked(device_id, status)
                    continue

                start_pt = route[idx]
                end_pt = route[idx + 1]
                ratio = max(0.0, min(1.0, progress))
                x = start_pt.x + (end_pt.x - start_pt.x) * ratio
                y = start_pt.y + (end_pt.y - start_pt.y) * ratio
                yaw = start_pt.yaw
                if start_pt.x != end_pt.x or start_pt.y != end_pt.y:
                    yaw = (math.degrees(math.atan2(end_pt.y - start_pt.y, end_pt.x - start_pt.x)) + 360.0) % 360.0

                status["nodeId"] = str(start_pt.node_id)
                status["x"] = x
                status["y"] = y
                status["angle"] = yaw
                status["updateTime"] = now

                # Provide a coarse speed hint (same unit as x/y per second).
                distance = math.hypot(end_pt.x - start_pt.x, end_pt.y - start_pt.y)
                if duration > 0:
                    status["speed"] = distance / duration

                status["nextDestinationPoint"] = {
                    "x": end_pt.x,
                    "y": end_pt.y,
                    "angle": yaw,
                    "nodeId": str(end_pt.node_id),
                }
                if self.trail_preview > 0:
                    preview = []
                    for pt in route[idx + 1 : idx + 1 + self.trail_preview]:
                        preview.append(
                            {
                                "x": pt.x,
                                "y": pt.y,
                                "angle": pt.yaw,
                                "nodeId": str(pt.node_id),
                            }
                        )
                    status["curTrailPoints"] = preview
                else:
                    status["curTrailPoints"] = []
                marker = self._marker_at_current_node_locked(device_id, status)
                if marker is not None and self._start_subtask_dwell_locked(device_id, marker):
                    status["speed"] = 0
                    continue
                self._consume_reached_subtasks_locked(device_id, status)
            self._metrics_update_distances_locked()

    def snapshot_status_list(self) -> List[Dict[str, Any]]:
        with self._lock:
            return copy.deepcopy(list(self.status_by_id.values()))

    def snapshot_status_by_id(self) -> Dict[str, Dict[str, Any]]:
        with self._lock:
            return copy.deepcopy(self.status_by_id)

    def snapshot_paths(self) -> Dict[str, List[PathPoint]]:
        with self._lock:
            return {k: list(v) for k, v in self.path_by_id.items()}

    def snapshot_trails(self) -> Dict[str, List[PathPoint]]:
        with self._lock:
            return {k: list(v) for k, v in self.trail_by_id.items()}

    def snapshot_reserved_nodes(self) -> List[Dict[str, Any]]:
        with self._lock:
            return [dict(x) for x in self._reserved_nodes]

    def snapshot_positions(self) -> Dict[str, Tuple[float, float]]:
        with self._lock:
            return {
                k: (parse_float(v.get("x")), parse_float(v.get("y")))
                for k, v in self.status_by_id.items()
            }


def connect_with_retry(
    params: pika.connection.Parameters,
    *,
    purpose: str,
    stop_event: Optional[threading.Event] = None,
) -> pika.BlockingConnection:
    retry_sec = parse_float_env("MQ_CONNECT_RETRY_SEC", 30.0)
    backoff_sec = parse_float_env("MQ_CONNECT_BACKOFF_SEC", 0.5)
    backoff_max_sec = parse_float_env("MQ_CONNECT_BACKOFF_MAX_SEC", 5.0)
    deadline = time.monotonic() + max(0.0, retry_sec)
    delay = max(0.05, backoff_sec)
    attempt = 0
    while True:
        if stop_event is not None and stop_event.is_set():
            raise RuntimeError(f"[MQ] stopped before connect ({purpose})")
        try:
            return pika.BlockingConnection(params)
        except Exception as exc:
            attempt += 1
            if retry_sec <= 0 or time.monotonic() >= deadline:
                raise
            msg = str(exc).strip() or repr(exc)
            print(
                f"[MQ] connect failed ({purpose}) attempt={attempt}: {msg}; retry in {delay:.1f}s",
                file=sys.stderr,
            )
            time.sleep(delay)
            delay = min(backoff_max_sec, delay * 1.6)


class MQPublisher:
    def __init__(self) -> None:
        self.params = build_connection_parameters()
        self.exchange = env("EXT_EXCHANGE", "DispToAlgoExchange")
        self.exchange_type = env("EXT_EXCHANGE_TYPE", "fanout")
        self.message_ttl_ms = parse_int_env("EXT_MESSAGE_TTL_MS", 0)
        self._connection: Optional[pika.BlockingConnection] = None
        self._channel: Optional[pika.adapters.blocking_connection.BlockingChannel] = None

    def ensure_connected(self) -> None:
        if self._connection and not self._connection.is_closed:
            return
        self.close()
        self._connection = connect_with_retry(
            self.params, purpose=f"publisher(exchange={self.exchange})"
        )
        self._channel = self._connection.channel()
        self._channel.exchange_declare(
            exchange=self.exchange, exchange_type=self.exchange_type, durable=True
        )

    def publish_json(self, routing_key: str, body: Any) -> None:
        self.ensure_connected()
        payload = body if isinstance(body, str) else json.dumps(body, ensure_ascii=False)
        props = pika.BasicProperties(
            content_type="application/json",
            delivery_mode=2,
            expiration=str(self.message_ttl_ms) if self.message_ttl_ms > 0 else None,
        )
        assert self._channel is not None
        self._channel.basic_publish(
            exchange=self.exchange,
            routing_key=routing_key,
            body=payload,
            properties=props,
        )

    def close(self) -> None:
        if self._channel and not self._channel.is_closed:
            try:
                self._channel.close()
            except Exception:
                pass
        if self._connection and not self._connection.is_closed:
            try:
                self._connection.close()
            except Exception:
                pass
        self._connection = None
        self._channel = None


class MQConsumer(threading.Thread):
    def __init__(self, on_message, stop_event: threading.Event, *, exchange: Optional[str] = None,
                 exchange_type: str = "fanout") -> None:
        super().__init__(daemon=True)
        self.on_message = on_message
        self.stop_event = stop_event
        self.exchange = exchange or env("ALGO_PUBLISH_EXCHANGE", env("ASSIGN_RESULT_EXCHANGE", "AlgoToDispExchange"))
        self.exchange_type = exchange_type

    def run(self) -> None:
        params = build_connection_parameters()
        while not self.stop_event.is_set():
            connection: Optional[pika.BlockingConnection] = None
            channel: Optional[pika.adapters.blocking_connection.BlockingChannel] = None
            try:
                connection = connect_with_retry(
                    params,
                    purpose=f"consumer(exchange={self.exchange})",
                    stop_event=self.stop_event,
                )
                channel = connection.channel()
                channel.exchange_declare(
                    exchange=self.exchange,
                    exchange_type=self.exchange_type,
                    durable=True,
                )
                result = channel.queue_declare(queue="", exclusive=True)
                queue_name = result.method.queue
                channel.queue_bind(queue=queue_name, exchange=self.exchange, routing_key="#")
                for method, properties, body in channel.consume(
                    queue=queue_name, inactivity_timeout=1.0, auto_ack=False
                ):
                    if self.stop_event.is_set():
                        break
                    if method is None:
                        continue
                    try:
                        text = body.decode("utf-8")
                    except Exception:
                        text = body.decode("utf-8", errors="replace")
                    self.on_message(method.routing_key, text)
                    channel.basic_ack(delivery_tag=method.delivery_tag)
            except Exception as exc:
                if self.stop_event.is_set():
                    break
                print(f"[MQ] consumer error: {exc}; reconnecting in 1s", file=sys.stderr)
                time.sleep(1.0)
            finally:
                if channel and not channel.is_closed:
                    try:
                        channel.close()
                    except Exception:
                        pass
                if connection and not connection.is_closed:
                    try:
                        connection.close()
                    except Exception:
                        pass


class Visualizer:
    def __init__(
        self,
        graph: MapGraph,
        *,
        path_preview: int = 80,
        trail_preview: int = 0,
        show_map_nodes: bool = False,
        show_node_ids: Optional[bool] = None,
    ) -> None:
        ensure_matplotlib_gui_backend()
        try:
            import matplotlib
            import matplotlib.pyplot as plt
            from matplotlib import rcsetup
            from matplotlib.collections import LineCollection
            from matplotlib.lines import Line2D
        except Exception as exc:
            raise RuntimeError(f"matplotlib is required for visualization: {exc}") from exc
        backend = str(matplotlib.get_backend() or "")
        interactive_backends = {b.lower() for b in getattr(rcsetup, "interactive_bk", [])}
        if interactive_backends and backend.lower() not in interactive_backends:
            raise RuntimeError(
                f"matplotlib backend {backend!r} is not a GUI backend; set MPLBACKEND=TkAgg/Qt5Agg (or install Tk/Qt)"
            )
        self.plt = plt
        self.LineCollection = LineCollection
        self.Line2D = Line2D
        self.graph = graph
        self.fig, self.ax = plt.subplots(figsize=(14, 9))
        self.fig.subplots_adjust(left=0.01, right=0.99, bottom=0.01, top=0.96)
        # Use the full axes area and adjust view limits to keep 1:1 scale.
        # This makes the map appear larger (vs shrinking the axes box).
        self.ax.set_aspect("equal", adjustable="datalim")
        self.ax.set_anchor("C")
        self.ax.set_title("AGV Path/Trail Viewer")
        self.ax.set_facecolor("#fafafa")
        self.ax.set_axis_off()

        self._map_xlim: Optional[Tuple[float, float]] = None
        self._map_ylim: Optional[Tuple[float, float]] = None

        self.path_preview = max(0, int(path_preview))
        self.trail_preview = max(0, int(trail_preview))
        self.show_path = True
        self.show_trail = True
        self.show_map = True
        self.show_labels = True
        self.show_subtasks = True
        self._show_map_nodes = bool(show_map_nodes)

        self._colors = itertools.cycle(
            [
                "#1f77b4",
                "#ff7f0e",
                "#2ca02c",
                "#d62728",
                "#9467bd",
                "#8c564b",
                "#e377c2",
                "#7f7f7f",
                "#bcbd22",
                "#17becf",
            ]
        )
        self._color_by_id: Dict[str, str] = {}
        self._map_edges: Optional[Any] = None
        self._map_nodes: Optional[Any] = None
        self._node_texts: List[Any] = []
        self._label_offset = 0.0
        self._path_lines: Dict[str, Any] = {}
        self._trail_lines: Dict[str, Any] = {}
        self._pos_markers: Dict[str, Any] = {}
        self._labels: Dict[str, Any] = {}
        self._goal_markers: Dict[str, Any] = {}
        self._temp_markers: Dict[str, Any] = {}
        self._next_markers: Dict[str, Any] = {}
        self._goal_labels: Dict[str, Any] = {}
        self._temp_labels: Dict[str, Any] = {}
        self._next_labels: Dict[str, Any] = {}
        self._subtask_scatters: Dict[str, Any] = {}
        self._subtask_active_markers: Dict[str, Any] = {}
        self._subtask_texts: Dict[str, List[Any]] = {}

        if show_node_ids is None:
            show_node_ids = len(graph.nodes) <= 60
        self._draw_map(graph, show_node_ids=bool(show_node_ids))
        self._init_legend()
        self._init_hud()
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        plt.ion()
        try:
            self.fig.canvas.manager.set_window_title("AGV Path/Trail Viewer")
        except Exception:
            pass

        # Show first so the window exists before resizing/maximizing.
        try:
            plt.show(block=False)
            plt.pause(0.001)
        except Exception as exc:
            raise RuntimeError(f"failed to open matplotlib window: {exc}") from exc
        try:
            print(f"[vis] started backend={backend}", file=sys.stderr)
        except Exception:
            pass

        self._maybe_maximize_window()
        self._raise_window()
        self._fit_to_map()
        try:
            self.fig.canvas.draw_idle()
        except Exception:
            pass
        try:
            plt.pause(0.001)
        except Exception:
            pass

    def _color_for(self, device_id: str) -> str:
        if device_id not in self._color_by_id:
            self._color_by_id[device_id] = next(self._colors)
        return self._color_by_id[device_id]

    def _draw_map(self, graph: MapGraph, *, show_node_ids: bool) -> None:
        self._node_texts = []
        if graph.edges:
            segments = []
            for start, end in graph.edges:
                if start in graph.nodes and end in graph.nodes:
                    x1, y1 = graph.nodes[start]
                    x2, y2 = graph.nodes[end]
                    segments.append([(x1, y1), (x2, y2)])
            if segments:
                lc = self.LineCollection(
                    segments,
                    colors="#4f4f4f",
                    linewidths=3.4,
                    alpha=0.92,
                    zorder=1,
                )
                self._map_edges = lc
                self.ax.add_collection(lc)
        if graph.nodes:
            xs = [pt[0] for pt in graph.nodes.values()]
            ys = [pt[1] for pt in graph.nodes.values()]

            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            x_span = max(max_x - min_x, 1.0)
            y_span = max(max_y - min_y, 1.0)
            pad_x = x_span * 0.02
            pad_y = y_span * 0.02
            self._label_offset = max(x_span, y_span) * 0.015
            self._map_xlim = (min_x - pad_x, max_x + pad_x)
            self._map_ylim = (min_y - pad_y, max_y + pad_y)
            self._fit_to_map()

            if self._show_map_nodes:
                self._map_nodes = self.ax.scatter(
                    xs,
                    ys,
                    s=110,
                    color="#1f1f1f",
                    alpha=0.98,
                    edgecolors="white",
                    linewidths=0.9,
                    zorder=2,
                )

            if show_node_ids:
                for node_id, (x, y) in graph.nodes.items():
                    txt = self.ax.text(
                        x,
                        y,
                        str(node_id),
                        fontsize=10,
                        color="#2f2f2f",
                        ha="center",
                        va="center",
                        zorder=2,
                        bbox=dict(
                            facecolor="white",
                            alpha=0.65,
                            edgecolor="none",
                            pad=0.25,
                        ),
                    )
                    self._node_texts.append(txt)

    def _maybe_maximize_window(self) -> None:
        try:
            manager = self.plt.get_current_fig_manager()
        except Exception:
            manager = None
        window = getattr(manager, "window", None)
        if window is None:
            try:
                self.fig.set_size_inches(12, 8, forward=True)
            except Exception:
                pass
            return

        # Qt first.
        if hasattr(window, "showMaximized"):
            try:
                window.showMaximized()
                return
            except Exception:
                pass

        # TkAgg (Linux/Windows) best-effort.
        if hasattr(window, "update_idletasks"):
            try:
                window.update_idletasks()
            except Exception:
                pass
        for action in (
            lambda: window.state("zoomed"),
            lambda: window.attributes("-zoomed", True),
        ):
            try:
                action()
                return
            except Exception:
                pass

        # Fallback: size to ~90% of screen on Tk, otherwise a reasonable geometry.
        if hasattr(window, "winfo_screenwidth") and hasattr(window, "winfo_screenheight"):
            try:
                sw = int(window.winfo_screenwidth())
                sh = int(window.winfo_screenheight())
                w = max(900, int(sw * 0.9))
                h = max(700, int(sh * 0.9))
                x = max(0, int((sw - w) / 2))
                y = max(0, int((sh - h) / 2))
                window.geometry(f"{w}x{h}+{x}+{y}")
                return
            except Exception:
                pass
        try:
            window.geometry("1400x900+60+60")
        except Exception:
            pass

    def _raise_window(self) -> None:
        try:
            manager = self.plt.get_current_fig_manager()
        except Exception:
            return
        window = getattr(manager, "window", None)
        if window is None:
            return

        # Qt.
        for action in (
            lambda: window.raise_(),
            lambda: window.activateWindow(),
        ):
            try:
                action()
            except Exception:
                pass

        # Tk.
        for action in (
            lambda: window.lift(),
            lambda: window.attributes("-topmost", True),
            lambda: window.attributes("-topmost", False),
        ):
            try:
                action()
            except Exception:
                pass

    def _fit_to_map(self) -> None:
        if not self._map_xlim or not self._map_ylim:
            return
        self.ax.set_xlim(*self._map_xlim)
        self.ax.set_ylim(*self._map_ylim)

    def _init_legend(self) -> None:
        handles = [
            self.Line2D([0], [0], color="#333333", lw=2.0, linestyle="-", label="Path"),
            self.Line2D([0], [0], color="#333333", lw=2.0, linestyle="--", label="Trail"),
            self.Line2D([0], [0], color="#333333", marker="s", linestyle="None", markersize=8, label="SubTasks"),
            self.Line2D([0], [0], color="#333333", marker="*", linestyle="None", markersize=10, label="Goal(G)"),
            self.Line2D([0], [0], color="#333333", marker="^", linestyle="None", markersize=8, label="Temp(T)"),
            self.Line2D([0], [0], color="#333333", marker="D", linestyle="None", markersize=8, label="Next(N)"),
        ]
        self.ax.legend(
            handles=handles,
            loc="upper right",
            framealpha=0.85,
            facecolor="white",
            edgecolor="#dddddd",
            fontsize=8,
        )

    def _init_hud(self) -> None:
        self._hud = self.ax.text(
            0.01,
            0.99,
            "Keys: p=path  t=trail  m=map  l=labels  s=subtasks  f=fit  r=reset  +=zoom  -=zoom",
            transform=self.ax.transAxes,
            ha="left",
            va="top",
            fontsize=8,
            color="#2f2f2f",
            bbox=dict(facecolor="white", alpha=0.8, edgecolor="none", pad=2),
            zorder=10,
        )

    def _apply_visibility(self) -> None:
        for artist in (self._map_edges, self._map_nodes):
            if artist is not None:
                artist.set_visible(self.show_map)
        for txt in self._node_texts:
            txt.set_visible(self.show_map)
        for line in self._path_lines.values():
            line.set_visible(self.show_path)
        for line in self._trail_lines.values():
            line.set_visible(self.show_trail)
        for marker in self._pos_markers.values():
            marker.set_visible(True)
        for label in self._labels.values():
            label.set_visible(self.show_labels)
        for marker in self._goal_markers.values():
            marker.set_visible(True)
        for marker in self._temp_markers.values():
            marker.set_visible(True)
        for marker in self._next_markers.values():
            marker.set_visible(True)
        for label in self._goal_labels.values():
            label.set_visible(self.show_labels)
        for label in self._temp_labels.values():
            label.set_visible(self.show_labels)
        for label in self._next_labels.values():
            label.set_visible(self.show_labels)
        for scatter in self._subtask_scatters.values():
            scatter.set_visible(self.show_subtasks)
        for marker in self._subtask_active_markers.values():
            marker.set_visible(self.show_subtasks)
        for labels in self._subtask_texts.values():
            for label in labels:
                label.set_visible(self.show_subtasks and self.show_labels and bool(label.get_text()))

    def _on_key(self, event) -> None:
        key = (event.key or "").lower()
        if key == "p":
            self.show_path = not self.show_path
        elif key == "t":
            self.show_trail = not self.show_trail
        elif key == "m":
            self.show_map = not self.show_map
        elif key == "l":
            self.show_labels = not self.show_labels
        elif key == "s":
            self.show_subtasks = not self.show_subtasks
        elif key == "r":
            self.show_path = True
            self.show_trail = True
            self.show_map = True
            self.show_labels = True
            self.show_subtasks = True
            self._fit_to_map()
        elif key == "f":
            self._fit_to_map()
        elif key in {"+", "="}:
            self._zoom(0.8)
        elif key in {"-", "_"}:
            self._zoom(1.25)
        else:
            return
        self._apply_visibility()
        self.fig.canvas.draw_idle()

    def _on_scroll(self, event) -> None:
        # wheel up -> zoom in, wheel down -> zoom out
        if getattr(event, "button", None) == "up":
            factor = 0.85
        elif getattr(event, "button", None) == "down":
            factor = 1.15
        else:
            return
        center = None
        if event.xdata is not None and event.ydata is not None:
            center = (float(event.xdata), float(event.ydata))
        self._zoom(factor, center=center)
        self.fig.canvas.draw_idle()

    def _zoom(self, factor: float, *, center: Optional[Tuple[float, float]] = None) -> None:
        if factor <= 0:
            return
        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        if center is None:
            cx = (x0 + x1) / 2.0
            cy = (y0 + y1) / 2.0
        else:
            cx, cy = center
        half_w = (x1 - x0) / 2.0 * factor
        half_h = (y1 - y0) / 2.0 * factor
        if half_w <= 0 or half_h <= 0:
            return
        self.ax.set_xlim(cx - half_w, cx + half_w)
        self.ax.set_ylim(cy - half_h, cy + half_h)

    def update(self, state: SimState) -> None:
        paths = state.snapshot_paths()
        trails = state.snapshot_trails()
        statuses = state.snapshot_status_by_id()
        subtasks_by_device = state.snapshot_subtask_markers()
        for device_id, status in statuses.items():
            color = self._color_for(device_id)
            if device_id not in self._path_lines:
                (line,) = self.ax.plot(
                    [],
                    [],
                    "-",
                    linewidth=2.6,
                    color=color,
                    alpha=0.65,
                    zorder=3,
                )
                self._path_lines[device_id] = line
            if device_id not in self._trail_lines:
                (line,) = self.ax.plot(
                    [],
                    [],
                    "--",
                    linewidth=3.0,
                    color=color,
                    alpha=0.85,
                    zorder=4,
                )
                self._trail_lines[device_id] = line
            if device_id not in self._pos_markers:
                (marker,) = self.ax.plot(
                    [],
                    [],
                    "o",
                    color=color,
                    markersize=9,
                    markeredgecolor="#111111",
                    markeredgewidth=0.8,
                    zorder=5,
                )
                self._pos_markers[device_id] = marker
            if device_id not in self._labels:
                self._labels[device_id] = self.ax.text(
                    0,
                    0,
                    device_id,
                    fontsize=9,
                    color="#111111",
                    zorder=6,
                    bbox=dict(facecolor="white", alpha=0.75, edgecolor="none", pad=1),
                )
            if device_id not in self._goal_markers:
                (marker,) = self.ax.plot(
                    [],
                    [],
                    marker="*",
                    markersize=16,
                    markeredgecolor="#111111",
                    markeredgewidth=0.8,
                    color=color,
                    zorder=6,
                    linestyle="None",
                )
                self._goal_markers[device_id] = marker
            if device_id not in self._temp_markers:
                (marker,) = self.ax.plot(
                    [],
                    [],
                    marker="^",
                    markersize=12,
                    markeredgecolor="#111111",
                    markeredgewidth=0.8,
                    color=color,
                    zorder=6,
                    linestyle="None",
                    alpha=0.95,
                )
                self._temp_markers[device_id] = marker
            if device_id not in self._goal_labels:
                self._goal_labels[device_id] = self.ax.text(
                    0,
                    0,
                    "G",
                    fontsize=9,
                    color="#111111",
                    zorder=7,
                    bbox=dict(facecolor="white", alpha=0.75, edgecolor="none", pad=1),
                )
            if device_id not in self._temp_labels:
                self._temp_labels[device_id] = self.ax.text(
                    0,
                    0,
                    "T",
                    fontsize=9,
                    color="#111111",
                    zorder=7,
                    bbox=dict(facecolor="white", alpha=0.75, edgecolor="none", pad=1),
                )
            if device_id not in self._next_markers:
                (marker,) = self.ax.plot(
                    [],
                    [],
                    marker="D",
                    markersize=10,
                    markeredgecolor="#111111",
                    markeredgewidth=0.8,
                    color=color,
                    zorder=6,
                    linestyle="None",
                    alpha=0.95,
                )
                self._next_markers[device_id] = marker
            if device_id not in self._next_labels:
                self._next_labels[device_id] = self.ax.text(
                    0,
                    0,
                    "N",
                    fontsize=9,
                    color="#111111",
                    zorder=7,
                    bbox=dict(facecolor="white", alpha=0.75, edgecolor="none", pad=1),
                )
            if device_id not in self._subtask_scatters:
                self._subtask_scatters[device_id] = self.ax.scatter(
                    [],
                    [],
                    s=220,
                    marker="s",
                    facecolors="white",
                    edgecolors=color,
                    linewidths=1.8,
                    alpha=0.95,
                    zorder=4.2,
                )
            if device_id not in self._subtask_active_markers:
                (marker,) = self.ax.plot(
                    [],
                    [],
                    marker="s",
                    markersize=14,
                    markeredgecolor="#111111",
                    markeredgewidth=1.0,
                    markerfacecolor=color,
                    color=color,
                    zorder=6.2,
                    linestyle="None",
                    alpha=0.35,
                )
                self._subtask_active_markers[device_id] = marker
            if device_id not in self._subtask_texts:
                self._subtask_texts[device_id] = []

            path_pts = paths.get(device_id, [])
            current_node = parse_node_id(status.get("nodeId"))
            next_node = None
            next_dest = status.get("nextDestinationPoint")
            if isinstance(next_dest, dict):
                next_node = parse_node_id(next_dest.get("nodeId"))
            if path_pts:
                start_idx = 0
                if current_node is not None:
                    if next_node is not None:
                        for i in range(len(path_pts) - 1):
                            if path_pts[i].node_id == current_node and path_pts[i + 1].node_id == next_node:
                                start_idx = i
                                break
                    if start_idx == 0:
                        for i, pt in enumerate(path_pts):
                            if pt.node_id == current_node:
                                start_idx = i
                                break
                start_idx = max(0, min(start_idx, len(path_pts) - 1))
                view = path_pts[start_idx:]
                if self.path_preview > 0:
                    view = view[: self.path_preview]
                xs = [pt.x for pt in view]
                ys = [pt.y for pt in view]
                self._path_lines[device_id].set_data(xs, ys)
            else:
                self._path_lines[device_id].set_data([], [])

            trail_pts = trails.get(device_id, [])
            if trail_pts:
                view = trail_pts
                if self.trail_preview > 0:
                    view = view[: self.trail_preview]
                xs = [pt.x for pt in view]
                ys = [pt.y for pt in view]
                self._trail_lines[device_id].set_data(xs, ys)
            else:
                self._trail_lines[device_id].set_data([], [])

            x = parse_float(status.get("x"))
            y = parse_float(status.get("y"))
            self._pos_markers[device_id].set_data([x], [y])
            self._labels[device_id].set_position((x + self._label_offset, y + self._label_offset))

            goal = None
            if path_pts:
                goal = path_pts[-1]
            elif trail_pts:
                goal = trail_pts[-1]
            if goal is not None:
                self._goal_markers[device_id].set_data([goal.x], [goal.y])
                self._goal_labels[device_id].set_text(f"G:{goal.node_id}")
                self._goal_labels[device_id].set_position((goal.x + self._label_offset, goal.y + self._label_offset))
            else:
                self._goal_markers[device_id].set_data([], [])
                self._goal_labels[device_id].set_text("")

            temp = None
            if trail_pts:
                temp = trail_pts[-1]
            elif path_pts:
                if start_idx < len(path_pts) - 1:
                    temp = path_pts[start_idx + 1]
            if temp is not None:
                self._temp_markers[device_id].set_data([temp.x], [temp.y])
                self._temp_labels[device_id].set_text(f"T:{temp.node_id}")
                self._temp_labels[device_id].set_position((temp.x + self._label_offset, temp.y + self._label_offset))
            else:
                self._temp_markers[device_id].set_data([], [])
                self._temp_labels[device_id].set_text("")

            nxt = None
            if next_node is not None:
                if next_node in self.graph.nodes:
                    nx, ny = self.graph.nodes[next_node]
                else:
                    nx = parse_float(next_dest.get("x") if isinstance(next_dest, dict) else 0.0)
                    ny = parse_float(next_dest.get("y") if isinstance(next_dest, dict) else 0.0)
                nxt = (float(nx), float(ny), int(next_node))
            if nxt is not None:
                nx, ny, nid = nxt
                self._next_markers[device_id].set_data([nx], [ny])
                self._next_labels[device_id].set_text(f"N:{nid}")
                self._next_labels[device_id].set_position((nx + self._label_offset, ny - self._label_offset))
            else:
                self._next_markers[device_id].set_data([], [])
                self._next_labels[device_id].set_text("")

            # Only show the next (first remaining) subtask point to reduce clutter.
            all_markers = subtasks_by_device.get(device_id, [])
            active = None
            if all_markers:
                active_idx = 0
                if current_node is not None:
                    for i, m in enumerate(all_markers):
                        if int(m.get("node_id", -1)) != int(current_node):
                            active_idx = i
                            break
                active = all_markers[min(active_idx, len(all_markers) - 1)]
            markers = [active] if active is not None else []

            xs = [float(m.get("x", 0.0)) for m in markers]
            ys = [float(m.get("y", 0.0)) for m in markers]
            offsets = list(zip(xs, ys)) if xs else []
            if offsets:
                self._subtask_scatters[device_id].set_offsets(offsets)
                self._subtask_scatters[device_id].set_sizes([220.0] * len(offsets))
            else:
                # Matplotlib expects a (N, 2) array; [] is treated as (0,) and crashes.
                self._subtask_scatters[device_id].set_offsets([(0.0, 0.0)])
                self._subtask_scatters[device_id].set_sizes([0.0])

            if active is not None:
                self._subtask_active_markers[device_id].set_data([float(active.get("x", 0.0))], [float(active.get("y", 0.0))])
            else:
                self._subtask_active_markers[device_id].set_data([], [])

            texts = self._subtask_texts[device_id]
            for i, m in enumerate(markers):
                if i >= len(texts):
                    texts.append(
                        self.ax.text(
                            0,
                            0,
                            "",
                            fontsize=8,
                            color="#111111",
                            zorder=7,
                            bbox=dict(facecolor="white", alpha=0.70, edgecolor="none", pad=1),
                        )
                    )
                txt = texts[i]
                txt.set_text(str(m.get("label", "")))
                tx = float(m.get("x", 0.0)) + self._label_offset
                ty = float(m.get("y", 0.0)) - self._label_offset
                txt.set_position((tx, ty))
            for j in range(len(markers), len(texts)):
                texts[j].set_text("")

            # Enrich the AGV label with a coarse state to make demo behavior obvious.
            label_state = "IDLE"
            if next_node is not None:
                cur = current_node if current_node is not None else "?"
                label_state = f"{cur}->{next_node}"
            elif path_pts and current_node is not None and path_pts[-1].node_id != current_node:
                label_state = "WAIT"
            speed = parse_float(status.get("speed"), 0.0)
            suffix = f"{label_state} v={speed:.1f}"
            self._labels[device_id].set_text(f"{device_id} {suffix}")

        self._apply_visibility()
        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)


def build_connection_parameters() -> pika.connection.Parameters:
    url = env("RABBITMQ_URL")
    if url:
        params = pika.URLParameters(url)
        if os.getenv("AMQP_HEARTBEAT") is not None:
            params.heartbeat = parse_int_env("AMQP_HEARTBEAT", params.heartbeat or 0)
        if os.getenv("AMQP_BLOCKED_TIMEOUT") is not None:
            params.blocked_connection_timeout = parse_int_env(
                "AMQP_BLOCKED_TIMEOUT",
                int(params.blocked_connection_timeout or 0),
            )
        return params
    host = env("AMQP_HOST", "127.0.0.1")
    port = int(env("AMQP_PORT", "5672"))
    user = env("AMQP_USER", "guest")
    password = env("AMQP_PASS", "guest")
    vhost = env("AMQP_VHOST", "/")
    credentials = pika.PlainCredentials(user, password)
    heartbeat = parse_int_env("AMQP_HEARTBEAT", 30)
    blocked_timeout = parse_int_env("AMQP_BLOCKED_TIMEOUT", 10)
    return pika.ConnectionParameters(
        host=host,
        port=port,
        virtual_host=vhost,
        credentials=credentials,
        heartbeat=heartbeat,
        blocked_connection_timeout=blocked_timeout,
    )


def parse_int_env(key: str, default: int) -> int:
    value = env(key)
    if not value:
        return default
    try:
        return max(0, int(value))
    except ValueError:
        return default


def parse_float_env(key: str, default: float) -> float:
    value = env(key)
    if not value:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def resolve_map_path(cli_path: Optional[str]) -> Optional[Path]:
    if cli_path:
        return Path(cli_path)
    env_map = env("MAP_FILE")
    if env_map:
        return Path(env_map)
    root = Path(__file__).resolve().parent
    candidates = [
        root.parent / "config" / "grid_20x20.json",
        root.parent / "config" / "zz_rcs_map_20250912_171049.json",
        root.parent / "config" / "south_20260107.json",
    ]
    for path in candidates:
        if path.exists():
            return path
    return None


def resolve_status_path(cli_path: Optional[str]) -> Optional[Path]:
    if cli_path:
        return Path(cli_path)
    root = Path(__file__).resolve().parent
    candidates = [root / "generated_status.json", root.parent / "script" / "status_simple_warehouse.json"]
    for path in candidates:
        if path.exists():
            return path
    return None


def resolve_tasks_path(cli_path: Optional[str]) -> Optional[Path]:
    if cli_path:
        return Path(cli_path)
    root = Path(__file__).resolve().parent
    candidates = [root / "generated_tasks.json", root.parent / "script" / "generated_test_payload1.json"]
    for path in candidates:
        if path.exists():
            return path
    return None


def routing_key_with_fallback(primary_env: str, default_key: str) -> str:
    override = env(primary_env)
    if override:
        return override
    global_override = env("EXT_ROUTING_KEY")
    if global_override and default_key in {
        "SendMapInfo",
        "SendRobotStatusInfos",
        "RobotPathRequest",
        "RobotTrailRequest",
    }:
        return global_override
    return default_key


def build_map_message(map_path: Path, map_version: str, map_id: Optional[int]) -> Dict[str, Any]:
    data = load_json(map_path)
    if isinstance(data, dict) and "mapData" in data:
        message = data
        if "mapVersion" not in message:
            message["mapVersion"] = map_version
        if map_id is not None and "mapId" not in message:
            message["mapId"] = map_id
        return message
    message: Dict[str, Any] = {"mapVersion": map_version, "mapData": data}
    if map_id is not None:
        message["mapId"] = map_id
    return message


def build_status_payload(status_list: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return status_list


def build_task_payload(task_payload: Dict[str, Any], status_list: Optional[List[Dict[str, Any]]]) -> Dict[str, Any]:
    if status_list is None:
        return task_payload
    payload = dict(task_payload)
    if "agvStatusList" not in payload or not isinstance(payload.get("agvStatusList"), list):
        payload["agvStatusList"] = status_list
    return payload


def extract_path_points(payload: Dict[str, Any], graph: MapGraph) -> List[PathPoint]:
    points: List[PathPoint] = []
    for info in payload.get("pathInfos", []):
        pts = info.get("pathPoints") or []
        if not pts:
            for key in ("startPoint", "endPoint"):
                candidate = info.get(key)
                if isinstance(candidate, dict):
                    pts.append(candidate)
        for pt in pts:
            if not isinstance(pt, dict):
                continue
            parsed = point_from_dict(pt, graph)
            if parsed:
                points.append(parsed)
    return dedup_consecutive(points)


def extract_trail_points(payload: Dict[str, Any], graph: MapGraph) -> List[PathPoint]:
    points: List[PathPoint] = []
    for pt in payload.get("controlPoints", []) or []:
        if not isinstance(pt, dict):
            continue
        parsed = point_from_dict(pt, graph)
        if parsed:
            points.append(parsed)
    return dedup_consecutive(points)


def send_map(publisher: MQPublisher, map_path: Path, map_version: str, map_id: Optional[int]) -> None:
    routing_key = routing_key_with_fallback("MAP_ROUTING_KEY", "SendMapInfo")
    message = build_map_message(map_path, map_version, map_id)
    publisher.publish_json(routing_key, message)


def send_status(publisher: MQPublisher, status_list: List[Dict[str, Any]]) -> None:
    routing_key = routing_key_with_fallback("STATUS_ROUTING_KEY", "SendRobotStatusInfos")
    publisher.publish_json(routing_key, build_status_payload(status_list))


def send_tasks(publisher: MQPublisher, task_payload: Dict[str, Any], status_list: Optional[List[Dict[str, Any]]]) -> None:
    routing_key = env("EXT_ROUTING_KEY", "AssignmentTaskRequest")
    # Refresh task timestamps to prevent age-bonus from collapsing costs
    _refresh_task_timestamps(task_payload)
    publisher.publish_json(routing_key, build_task_payload(task_payload, status_list))


def _refresh_task_timestamps(payload: Dict[str, Any]) -> None:
    """Overwrite task timestamp fields with current time.

    The allocation cost-matrix applies an age-bonus that subtracts
    cost proportional to how long a task has been waiting.  When tasks
    carry build-time or historical timestamps the bonus saturates and
    all costs collapse to the -60 000 ms floor, destroying any
    geographic differentiation and causing single-AGV monopoly.
    """
    now_iso = iso_timestamp()
    for task in payload.get("candidateTasks", []):
        if not isinstance(task, dict):
            continue
        for key in ("timestamp", "expectedStartTime", "expectedCompletionTime",
                    "createTimestamp", "createTimestampISO", "expectedStartTimeISO"):
            if key in task:
                task[key] = now_iso


def send_path_request(publisher: MQPublisher, device_id: str) -> None:
    routing_key = routing_key_with_fallback("PATH_REQUEST_ROUTING_KEY", "RobotPathRequest")
    message = {
        "messageId": f"PATH_REQ_{device_id}_{int(time.time() * 1000)}",
        "deviceId": device_id,
    }
    publisher.publish_json(routing_key, message)


def send_trail_request(publisher: MQPublisher, device_id: str, *, sub_task_id: Optional[str] = None) -> None:
    if not sub_task_id:
        return
    routing_key = routing_key_with_fallback("TRAIL_REQUEST_ROUTING_KEY", "RobotTrailRequest")
    message = {
        "messageId": f"TRAIL_REQ_{device_id}_{int(time.time() * 1000)}",
        "deviceId": device_id,
    }
    message["subTaskId"] = str(sub_task_id)
    publisher.publish_json(routing_key, message)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AGV scheduling simulator adapter")
    parser.add_argument("--map", dest="map_path", default=None, help="Map JSON path")
    parser.add_argument("--map-version", default=env("MAP_VERSION", "sim-map"), help="Map version string")
    parser.add_argument("--map-id", type=int, default=0, help="Map ID for status payloads")
    parser.add_argument("--status", dest="status_path", default=None, help="Status JSON path")
    parser.add_argument("--tasks", dest="tasks_path", default=None, help="Task JSON path")
    parser.add_argument("--device-ids", default="", help="Comma-separated device IDs to simulate")
    parser.add_argument("--trail-preview", type=int, default=5, help="Preview trail points in status payloads")
    parser.add_argument("--generate", action="store_true", help="Generate status/tasks from map")
    parser.add_argument("--out-dir", default=".", help="Output directory for generated payloads")
    parser.add_argument("--agv-count", type=int, default=50, help="Number of AGVs to generate")
    parser.add_argument("--task-count", type=int, default=2, help="Number of tasks to generate")
    parser.add_argument("--seed", type=int, default=7, help="Random seed for generation")
    parser.add_argument("--device-prefix", default="AGV", help="Device ID prefix for generated AGVs")
    parser.add_argument(
        "--bind-tasks",
        action="store_true",
        help="(Deprecated, no-op) Retained for CLI compatibility.",
    )
    parser.add_argument(
        "--demo-avoidance",
        action="store_true",
        help="Two-AGV avoidance demo: fixed scenario intended to trigger two-AGV avoidance logic (if enabled).",
    )
    parser.add_argument("--include-status-in-tasks", action="store_true", help="Embed agvStatusList into tasks payload")
    parser.add_argument(
        "--alloc-algo",
        default=env("SIM_ALLOC_ALGO", ""),
        help="Allocation algorithm name (e.g. posta/greedy/mlp) to include in task payloads.",
    )
    parser.add_argument("--bootstrap", action="store_true", help="Send map/status/tasks before running")
    parser.add_argument("--send-map", action="store_true", help="Send map payload and exit")
    parser.add_argument("--send-status", action="store_true", help="Send status payload and exit")
    parser.add_argument("--send-tasks", action="store_true", help="Send tasks payload and exit")
    parser.add_argument("--run", action="store_true", help="Start simulation loop")
    parser.add_argument(
        "--step-interval",
        type=float,
        default=0.5,
        help="Seconds per simulation tick (status publish interval). If --edge-duration=0, each tick advances one edge.",
    )
    parser.add_argument(
        "--edge-duration",
        type=float,
        default=0.0,
        help="Seconds to traverse one edge (0 = same as step-interval; > step-interval enables smooth motion).",
    )
    parser.add_argument("--path-interval", type=float, default=1.0, help="Seconds between path requests")
    parser.add_argument("--trail-interval", type=float, default=1.0, help="Seconds between trail requests")
    parser.add_argument(
        "--assigned-path-interval",
        type=float,
        default=0.5,
        help="Seconds between path requests for devices with assigned tasks (0 disables).",
    )
    parser.add_argument(
        "--assigned-trail-interval",
        type=float,
        default=0.2,
        help="Seconds between trail requests for devices with assigned tasks (0 disables).",
    )
    parser.add_argument(
        "--loop-tasks",
        action="store_true",
        help="Continuously send new random tasks (default: when any AGV becomes idle; configurable via --loop-mode).",
    )
    parser.add_argument(
        "--loop-mode",
        choices=("any-idle", "all-idle", "timer"),
        default=env("SIM_LOOP_MODE", "any-idle"),
        help="loop-tasks trigger: any-idle=send when any AGV idle; all-idle=wait for all; timer=send periodically regardless of motion.",
    )
    parser.add_argument(
        "--loop-task-min-count",
        type=int,
        default=parse_int_env("SIM_LOOP_TASK_MIN_COUNT", 1),
        help="Min number of tasks per loop batch (default 1).",
    )
    parser.add_argument(
        "--loop-task-max-count",
        type=int,
        default=parse_int_env("SIM_LOOP_TASK_MAX_COUNT", 2),
        help="Max number of tasks per loop batch (default 2).",
    )
    parser.add_argument(
        "--loop-task-min-delay",
        type=float,
        default=parse_float_env("SIM_LOOP_TASK_MIN_DELAY", 1.0),
        help="Seconds to wait (min) before sending the next loop batch (depends on --loop-mode).",
    )
    parser.add_argument(
        "--loop-task-max-delay",
        type=float,
        default=parse_float_env("SIM_LOOP_TASK_MAX_DELAY", 3.0),
        help="Seconds to wait (max) before sending the next loop batch (depends on --loop-mode).",
    )
    parser.add_argument(
        "--loop-pending-timeout",
        type=float,
        default=parse_float_env("SIM_LOOP_PENDING_TIMEOUT", 5.0),
        help="Seconds to wait for path/trail to appear after sending loop tasks before allowing another batch for the same AGV (default 5).",
    )
    parser.add_argument(
        "--stop-when-idle",
        action="store_true",
        help="Exit when all AGVs return idle after having moved (ignored when --loop-tasks).",
    )
    parser.add_argument(
        "--stop-idle-grace",
        type=float,
        default=parse_float_env("SIM_STOP_IDLE_GRACE", 2.0),
        help="Seconds of continuous idle before exiting when --stop-when-idle is set (default 2).",
    )
    parser.add_argument(
        "--drive-mode",
        choices=("path", "trail"),
        default=env("SIM_DRIVE_MODE", "trail"),
        help="How to move AGVs: trail=follow RobotTrailResponse only; path=ignore trail and follow RobotPathResponse.",
    )
    parser.add_argument("--no-vis", action="store_true", help="Disable visualization")
    parser.add_argument(
        "--no-snap-to-map",
        action="store_true",
        help="Disable map-edge validation/truncation and draw raw path/trail points (may include edges not in the map).",
    )
    parser.add_argument(
        "--vis-path-preview",
        type=int,
        default=parse_int_env("VIS_PATH_PREVIEW", 80),
        help="Visualization: draw at most N future path points per AGV (0=all).",
    )
    parser.add_argument(
        "--vis-node-ids",
        action="store_true",
        help="Visualization: show node IDs on the map (auto enabled for small maps).",
    )
    parser.add_argument(
        "--vis-map-nodes",
        action="store_true",
        help="Visualization: draw map nodes as points (disabled by default; lines-only is faster/cleaner for large maps).",
    )
    parser.add_argument("--no-attach-status", action="store_true", help="Do not attach status to task payload")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if getattr(args, "demo_avoidance", False):
        args.generate = True
    map_path = resolve_map_path(args.map_path)
    if not map_path or not map_path.exists():
        print("Map file not found. Use --map or MAP_FILE.", file=sys.stderr)
        return 2

    graph = load_map_graph(map_path)

    tasks_payload: Optional[Dict[str, Any]] = None
    status_list: List[Dict[str, Any]] = []
    if args.generate:
        if not graph.nodes:
            print("Map has no nodes; cannot generate payloads.", file=sys.stderr)
            return 2
        if args.demo_avoidance:
            try:
                status_list, tasks_payload = build_two_agv_avoidance_demo(
                    graph, map_id=args.map_id, map_version=args.map_version
                )
            except Exception as exc:
                print(f"demo-avoidance failed: {exc}", file=sys.stderr)
                return 2
        else:
            rng = random.Random(args.seed)
            status_list = generate_status_list(
                graph, args.agv_count, rng, args.map_id, args.device_prefix
            )
            bind_ids: Optional[List[str]] = None
            start_nodes: Optional[Dict[str, int]] = None

            tasks_payload = generate_task_payload(
                graph,
                args.task_count,
                rng,
                args.map_version,
                bind_to_device_ids=bind_ids,
                start_node_by_device_id=start_nodes,
                allocation_algo=args.alloc_algo or None,
            )
        if args.include_status_in_tasks:
            tasks_payload["agvStatusList"] = status_list
        out_dir = Path(args.out_dir)
        status_out = out_dir / "generated_status.json"
        tasks_out = out_dir / "generated_tasks.json"
        write_json(status_out, status_list)
        write_json(tasks_out, tasks_payload)
        print(f"Generated status -> {status_out}")
        print(f"Generated tasks  -> {tasks_out}")
    else:
        status_path = resolve_status_path(args.status_path)
        if status_path and status_path.exists():
            loaded = load_json(status_path)
            if isinstance(loaded, list):
                status_list = loaded
            elif isinstance(loaded, dict):
                status_list = loaded.get("robotStatusInfos", []) or []

        tasks_path = resolve_tasks_path(args.tasks_path)
        if tasks_path and tasks_path.exists():
            loaded = load_json(tasks_path)
            if isinstance(loaded, dict):
                tasks_payload = loaded

    if args.alloc_algo and tasks_payload:
        tasks_payload["allocationAlgorithm"] = str(args.alloc_algo)

    if args.device_ids:
        allow = {x.strip() for x in args.device_ids.split(",") if x.strip()}
        status_list = [s for s in status_list if s.get("deviceId") in allow]
        if tasks_payload and isinstance(tasks_payload.get("agvStatusList"), list):
            tasks_payload["agvStatusList"] = status_list

    if args.generate and not (args.run or args.bootstrap or args.send_map or args.send_status or args.send_tasks):
        return 0

    state = SimState(
        status_list,
        graph,
        trail_preview=args.trail_preview,
        drive_mode=args.drive_mode,
    )
    if tasks_payload:
        state.set_tasks_payload(tasks_payload)
    publisher = MQPublisher()

    attach_status = not args.no_attach_status

    if args.send_map:
        send_map(publisher, map_path, args.map_version, args.map_id)
        return 0
    if args.send_status:
        send_status(publisher, state.snapshot_status_list())
        return 0
    if args.send_tasks:
        if not tasks_payload:
            print("Task payload not found. Use --tasks.", file=sys.stderr)
            return 2
        status_payload = state.snapshot_status_list() if attach_status else None
        send_tasks(publisher, tasks_payload, status_payload)
        return 0

    if not args.run and not args.bootstrap:
        print("Nothing to do. Use --run, --bootstrap, or send flags.", file=sys.stderr)
        return 2

    if args.bootstrap:
        bootstrap_map_wait_s = max(0.0, parse_float_env("SIM_BOOTSTRAP_MAP_WAIT_SEC", 1.5))
        send_map(publisher, map_path, args.map_version, args.map_id)
        if bootstrap_map_wait_s > 0.0 and (status_list or tasks_payload):
            print(f"[sim] waiting {bootstrap_map_wait_s:.1f}s for receiver map reload", file=sys.stderr)
            time.sleep(bootstrap_map_wait_s)
        if status_list:
            send_status(publisher, state.snapshot_status_list())
        if tasks_payload:
            status_payload = state.snapshot_status_list() if attach_status else None
            send_tasks(publisher, tasks_payload, status_payload)
            state.set_tasks_payload(tasks_payload)

    stop_event = threading.Event()
    snap_to_map = not args.no_snap_to_map
    last_route_sig: Dict[Tuple[str, str], Tuple[int, int, int]] = {}
    loop_rng = random.Random(args.seed + 1000)
    loop_cycle = 0
    loop_deadline: Optional[float] = None
    loop_pending_by_device: Dict[str, float] = {}
    loop_pending_timeout = max(0.0, float(getattr(args, "loop_pending_timeout", 5.0)))
    path_refresh_lock = threading.Lock()
    pending_path_refresh: Set[str] = set()
    trail_refresh_lock = threading.Lock()
    pending_trail_refresh: Set[str] = set()
    tasks_sent_once = bool(args.bootstrap and tasks_payload)
    stop_when_idle = bool(args.stop_when_idle) and not bool(args.loop_tasks)
    idle_grace_sec = max(0.0, float(getattr(args, "stop_idle_grace", 2.0)))
    idle_since: Optional[float] = None
    active_seen = False

    def trail_is_contained_in_path(path_points: List[PathPoint], trail_points: List[PathPoint]) -> bool:
        if not trail_points:
            return True
        if not path_points:
            return False
        path_nodes = [p.node_id for p in dedup_consecutive(path_points)]
        trail_nodes = [p.node_id for p in dedup_consecutive(trail_points)]
        if not trail_nodes:
            return True
        if len(trail_nodes) > len(path_nodes):
            return False
        for i in range(0, len(path_nodes) - len(trail_nodes) + 1):
            if path_nodes[i : i + len(trail_nodes)] == trail_nodes:
                return True
        return False

    def log_route(kind: str, device_id: str, points: List[PathPoint]) -> None:
        if not device_id or not points:
            return
        sig = (len(points), points[0].node_id, points[-1].node_id)
        key = (kind, device_id)
        if last_route_sig.get(key) == sig:
            return
        last_route_sig[key] = sig
        extra = ""
        if kind == "trail" and len(points) <= 1:
            extra = " (WAIT)"
        print(f"[sim] {kind} {device_id} points={len(points)} {points[0].node_id}->{points[-1].node_id}{extra}", file=sys.stderr)

    def device_is_idle(device_id: str, st: Dict[str, Any], paths: Dict[str, List[PathPoint]], trails: Dict[str, List[PathPoint]]) -> bool:
        next_dest = st.get("nextDestinationPoint")
        if isinstance(next_dest, dict) and next_dest:
            if parse_node_id(next_dest.get("nodeId")) is not None:
                return False
        cur_node = parse_node_id(st.get("nodeId"))
        goal_node = None
        if device_id in paths and paths[device_id]:
            goal_node = paths[device_id][-1].node_id
        elif device_id in trails and trails[device_id]:
            goal_node = trails[device_id][-1].node_id
        if goal_node is not None and cur_node is not None and cur_node != goal_node:
            return False
        return True

    def on_message(routing_key: str, payload_text: str) -> None:
        try:
            payload = json.loads(payload_text)
        except json.JSONDecodeError:
            return
        if routing_key == env("ALGO_PATH_RESPONSE_ROUTING_KEY", "RobotPathResponse"):
            device_id = payload.get("deviceId", "")
            points = extract_path_points(payload, graph)
            if snap_to_map and points:
                points = snap_points_to_map(graph, points, f"path/{device_id or '-'}")
            if device_id:
                state.set_path(device_id, points)
                log_route("path", device_id, points)
        elif routing_key == env("ALGO_TRAIL_RESPONSE_ROUTING_KEY", "RobotTrailResponse"):
            device_id = payload.get("deviceId", "")
            points = extract_trail_points(payload, graph)
            if snap_to_map and points:
                points = snap_points_to_map(graph, points, f"trail/{device_id or '-'}")
            if device_id:
                state.set_trail(device_id, points)
                log_route("trail", device_id, points)
                if args.path_interval > 0 or args.assigned_path_interval > 0:
                    current_path = state.snapshot_paths().get(device_id, [])
                    if points and not trail_is_contained_in_path(current_path, points):
                        with path_refresh_lock:
                            pending_path_refresh.add(device_id)
        elif routing_key == env("ASSIGN_RESULT_ROUTING_KEY", "AssignmentTaskResponse"):
            state.set_assignment_response(payload)
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

    consumer = MQConsumer(on_message, stop_event)
    consumer.start()
    visualizer: Optional[Visualizer] = None
    if not args.no_vis:
        try:
            visualizer = Visualizer(
                graph,
                path_preview=args.vis_path_preview,
                show_map_nodes=args.vis_map_nodes,
                show_node_ids=(args.vis_node_ids or None),
            )
        except Exception as exc:
            print(f"[vis] disabled: {exc}", file=sys.stderr)
            backend = os.getenv("MPLBACKEND")
            display = os.getenv("DISPLAY")
            if backend or display:
                print(f"[vis] env MPLBACKEND={backend!r} DISPLAY={display!r}", file=sys.stderr)
            print("[vis] tip: run with --no-vis if headless; for GUI ensure a desktop session + Tk/Qt backend.", file=sys.stderr)
            visualizer = None

    next_path_req = time.monotonic()
    next_trail_req = time.monotonic()
    next_assigned_path_req = time.monotonic()
    next_assigned_trail_req = time.monotonic()
    last_tick = time.monotonic()
    edge_duration = args.edge_duration if args.edge_duration > 0 else args.step_interval
    try:
        while True:
            now = time.monotonic()
            dt = max(0.0, now - last_tick)
            last_tick = now
            if args.loop_tasks:
                statuses = state.snapshot_status_by_id()
                paths = state.snapshot_paths()
                trails = state.snapshot_trails()
                all_devices = sorted(statuses.keys())
                idle_devices: List[str] = []
                start_nodes: Dict[str, int] = {}
                current_nodes: Set[int] = set()

                def device_has_active_route(device_id: str, st: Dict[str, Any]) -> bool:
                    next_dest = st.get("nextDestinationPoint")
                    if isinstance(next_dest, dict) and next_dest:
                        if parse_node_id(next_dest.get("nodeId")) is not None:
                            return True
                    cur_node = parse_node_id(st.get("nodeId"))
                    goal_node = None
                    if device_id in paths and paths[device_id]:
                        goal_node = paths[device_id][-1].node_id
                    elif device_id in trails and trails[device_id]:
                        goal_node = trails[device_id][-1].node_id
                    if goal_node is not None and cur_node is not None and cur_node != goal_node:
                        return True
                    return False

                # Clear pending flags when we observe movement/route, and drop stale pending entries.
                if loop_pending_by_device:
                    for did, st in list(statuses.items()):
                        if did not in loop_pending_by_device:
                            continue
                        if device_has_active_route(did, st):
                            loop_pending_by_device.pop(did, None)
                    if loop_pending_timeout > 0:
                        for did, ts0 in list(loop_pending_by_device.items()):
                            if (now - ts0) >= loop_pending_timeout:
                                loop_pending_by_device.pop(did, None)

                for device_id, st in statuses.items():
                    nid = parse_node_id(st.get("nodeId"))
                    if nid is not None:
                        start_nodes[device_id] = nid
                        current_nodes.add(int(nid))
                    if device_is_idle(device_id, st, paths, trails):
                        idle_devices.append(device_id)
                idle_devices.sort()

                loop_mode = getattr(args, "loop_mode", "any-idle")
                trigger_devices: List[str] = []
                if loop_mode == "timer":
                    trigger_devices = all_devices
                elif loop_mode == "all-idle":
                    if all_devices and len(idle_devices) == len(all_devices):
                        trigger_devices = idle_devices
                else:
                    trigger_devices = idle_devices
                if trigger_devices and loop_pending_by_device:
                    trigger_devices = [d for d in trigger_devices if d not in loop_pending_by_device]

                if trigger_devices:
                    min_delay = max(0.0, float(args.loop_task_min_delay))
                    max_delay = max(0.0, float(args.loop_task_max_delay))
                    if max_delay < min_delay:
                        max_delay = min_delay
                    if loop_deadline is None:
                        loop_deadline = now + loop_rng.uniform(min_delay, max_delay)
                    if now >= loop_deadline:
                        min_count = max(1, int(getattr(args, "loop_task_min_count", 1)))
                        max_count = max(min_count, int(getattr(args, "loop_task_max_count", 2)))
                        max_send = min(max_count, len(trigger_devices))
                        min_send = min(min_count, max_send) if max_send > 0 else 0
                        task_count = loop_rng.randint(min_send, max_send) if max_send > 0 else 0
                        if task_count > 0:
                            chosen = trigger_devices
                            if task_count < len(trigger_devices):
                                chosen = sorted(loop_rng.sample(trigger_devices, k=task_count))
                            loop_cycle += 1
                            payload = generate_loop_tasks_payload(
                                graph,
                                chosen,
                                loop_rng,
                                args.map_version,
                                cycle=loop_cycle,
                                start_node_by_device_id=start_nodes,
                                forbidden_end_nodes=current_nodes,
                                bind_tasks=bool(getattr(args, "bind_tasks", False)),
                                allocation_algo=args.alloc_algo or None,
                            )
                            status_payload = state.snapshot_status_list() if attach_status else None
                            send_tasks(publisher, payload, status_payload)
                            state.set_tasks_payload(payload)
                            for did in chosen:
                                loop_pending_by_device[did] = now
                            print(
                                f"[sim] loop tasks cycle={loop_cycle} tasks={len(payload.get('candidateTasks', []))} devices={chosen}",
                                file=sys.stderr,
                            )
                        loop_deadline = now + loop_rng.uniform(min_delay, max_delay)
                else:
                    loop_deadline = None
            refresh_ids: List[str] = []
            with path_refresh_lock:
                if pending_path_refresh:
                    refresh_ids = sorted(pending_path_refresh)
                    pending_path_refresh.clear()
            for device_id in refresh_ids:
                send_path_request(publisher, device_id)
            trail_refresh_ids: List[str] = []
            with trail_refresh_lock:
                if pending_trail_refresh:
                    trail_refresh_ids = sorted(pending_trail_refresh)
                    pending_trail_refresh.clear()
            for device_id in trail_refresh_ids:
                send_trail_request(publisher, device_id, sub_task_id=state.next_subtask_id(device_id))
            if args.assigned_path_interval > 0 and now >= next_assigned_path_req:
                for device_id in state.assigned_device_ids():
                    send_path_request(publisher, device_id)
                next_assigned_path_req = now + args.assigned_path_interval
            if args.path_interval > 0 and now >= next_path_req:
                for device_id in state.device_ids():
                    send_path_request(publisher, device_id)
                next_path_req = now + args.path_interval
            if args.assigned_trail_interval > 0 and now >= next_assigned_trail_req:
                for device_id in state.assigned_device_ids():
                    send_trail_request(publisher, device_id, sub_task_id=state.next_subtask_id(device_id))
                next_assigned_trail_req = now + args.assigned_trail_interval
            if args.trail_interval > 0 and now >= next_trail_req:
                for device_id in state.device_ids():
                    send_trail_request(publisher, device_id, sub_task_id=state.next_subtask_id(device_id))
                next_trail_req = now + args.trail_interval
            state.advance(dt, edge_duration=edge_duration)
            if status_list:
                send_status(publisher, state.snapshot_status_list())
            if visualizer:
                visualizer.update(state)
            if stop_when_idle and tasks_sent_once:
                statuses = state.snapshot_status_by_id()
                paths = state.snapshot_paths()
                trails = state.snapshot_trails()
                any_active = False
                all_idle = True
                for device_id, st in statuses.items():
                    if not device_is_idle(device_id, st, paths, trails):
                        any_active = True
                        all_idle = False
                        break
                if any_active:
                    active_seen = True
                if active_seen and all_idle:
                    if idle_since is None:
                        idle_since = now
                    elif (now - idle_since) >= idle_grace_sec:
                        print(f"[sim] all devices idle for {idle_grace_sec:.1f}s; stopping.", file=sys.stderr)
                        break
                else:
                    idle_since = None
            time.sleep(max(0.01, args.step_interval))
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        publisher.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
