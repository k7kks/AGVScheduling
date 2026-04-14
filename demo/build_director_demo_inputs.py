#!/usr/bin/env python3
import argparse
import json
import random
import time
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple


ROOT = Path(__file__).resolve().parents[1]


def _load_module(name: str, path: Path):
    spec = spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load module: {path}")
    module = module_from_spec(spec)
    spec.loader.exec_module(module)  # type: ignore[assignment]
    return module


sim_v2 = _load_module("director_sim_v2", ROOT / "simulation_v2" / "sim_runner_v2.py")
sim = sim_v2.sim


def load_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build deterministic director-demo inputs.")
    parser.add_argument("--map", default=str(ROOT / "config" / "A4_from_xml.json"))
    parser.add_argument("--map-version", default="director-demo")
    parser.add_argument("--map-id", type=int, default=20260107)
    parser.add_argument("--agv-count", type=int, default=30)
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--out-dir", default=str(ROOT / "demo" / "inputs" / "generated_director_demo"))
    return parser.parse_args()


def longest_bridge_component(web_map: Dict[str, Any]) -> List[int]:
    bridge_nodes = {int(x) for x in web_map.get("bridgeNodeIds", []) or []}
    if not bridge_nodes:
        return []
    adj: Dict[int, Set[int]] = {nid: set() for nid in bridge_nodes}
    for edge in web_map.get("bridgeEdges", []) or []:
        a = sim.parse_node_id(edge.get("startNode"))
        b = sim.parse_node_id(edge.get("endNode"))
        if a is None or b is None or a not in bridge_nodes or b not in bridge_nodes:
            continue
        adj[a].add(b)
        adj[b].add(a)
    best: List[int] = []
    seen: Set[int] = set()
    for seed in bridge_nodes:
        if seed in seen:
            continue
        stack = [seed]
        seen.add(seed)
        comp: List[int] = []
        while stack:
            cur = stack.pop()
            comp.append(cur)
            for nb in adj.get(cur, set()):
                if nb in seen:
                    continue
                seen.add(nb)
                stack.append(nb)
        if len(comp) > len(best):
            best = comp
    return sorted(best)


def bridge_entries(graph: Any, bridge_nodes: Sequence[int]) -> Optional[Tuple[int, int]]:
    bridge_set = set(int(x) for x in bridge_nodes)
    if not bridge_set:
        return None
    axis_horizontal = True
    xs = [graph.nodes[nid][0] for nid in bridge_set if nid in graph.nodes]
    ys = [graph.nodes[nid][1] for nid in bridge_set if nid in graph.nodes]
    if not xs or not ys:
        return None
    axis_horizontal = (max(xs) - min(xs)) >= (max(ys) - min(ys))
    boundary: List[int] = []
    for nid in bridge_set:
        if any(nb not in bridge_set for nb in graph.neighbors.get(nid, [])):
            boundary.append(nid)
    if len(boundary) < 2:
        boundary = list(bridge_set)
    boundary.sort(key=lambda nid: graph.nodes.get(nid, (0.0, 0.0))[0 if axis_horizontal else 1])
    low_bridge = boundary[0]
    high_bridge = boundary[-1]

    def pick_entry(bridge_node: int) -> int:
        outside = [nb for nb in graph.neighbors.get(bridge_node, []) if nb not in bridge_set]
        if outside:
            outside.sort(key=lambda nid: graph.nodes.get(nid, (0.0, 0.0))[0 if axis_horizontal else 1])
            return int(outside[0] if bridge_node == low_bridge else outside[-1])
        return int(bridge_node)

    return pick_entry(low_bridge), pick_entry(high_bridge)


def pick_secondary_corridor(graph: Any, blocked_nodes: Set[int]) -> Optional[Tuple[int, int]]:
    direct = sim.pick_demo_corridor_nodes(graph)
    if direct is not None and direct[0] not in blocked_nodes and direct[1] not in blocked_nodes:
        return int(direct[0]), int(direct[1])

    by_y: Dict[float, List[Tuple[float, int]]] = {}
    for node_id, (x, y) in graph.nodes.items():
        if node_id in blocked_nodes:
            continue
        by_y.setdefault(round(float(y), 3), []).append((float(x), int(node_id)))
    best: Optional[Tuple[float, int, int]] = None
    for nodes in by_y.values():
        if len(nodes) < 4:
            continue
        nodes.sort()
        left = nodes[0][1]
        right = nodes[-1][1]
        if left in blocked_nodes or right in blocked_nodes:
            continue
        route = sim.bfs_shortest_route(graph, left, right)
        if not route:
            continue
        span = nodes[-1][0] - nodes[0][0]
        if best is None or span > best[0]:
            best = (span, left, right)
    if best is None:
        return None
    return best[1], best[2]


def build_task(graph: Any,
               *,
               task_id: str,
               targets: Sequence[int],
               priority: int,
               agv_requirements: Optional[Sequence[str]] = None) -> Dict[str, Any]:
    pickup_ms = max(0, int(getattr(sim, "parse_int_env", lambda *_: 0)("SIM_SUBTASK_PICKUP_DURATION_MS", 0)))
    dropoff_ms = max(0, int(getattr(sim, "parse_int_env", lambda *_: 0)("SIM_SUBTASK_DROPOFF_DURATION_MS", 0)))
    waypoint_ms = max(0, int(getattr(sim, "parse_int_env", lambda *_: 0)("SIM_SUBTASK_WAYPOINT_DURATION_MS", 0)))
    clean_targets = [int(nid) for nid in targets if nid in graph.nodes]
    sub_tasks: List[Dict[str, Any]] = []
    total = len(clean_targets)
    for seq, node_id in enumerate(clean_targets, start=1):
        x, y = graph.nodes.get(node_id, (0.0, 0.0))
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
                "point": {"x": x, "y": y, "angle": 0, "nodeId": str(node_id)},
                "estimatedDuration": int(est_ms),
                "pathConstraints": [],
                "maxSpeed": 0,
                "subTaskId": f"{task_id}#{seq}",
            }
        )
    reqs = [str(x) for x in (agv_requirements or []) if str(x).strip()]
    return {
        "taskId": task_id,
        "priority": int(priority),
        "minBatteryLevel": 0,
        "subTasks": sub_tasks,
        "agvRequirements": reqs,
        "deviceIds": reqs,
    }


def pick_far_route(graph: Any, start_node: int, rng: random.Random, forbidden_end_nodes: Set[int]) -> List[int]:
    node_ids = [int(nid) for nid in graph.nodes.keys() if int(nid) != int(start_node) and int(nid) not in forbidden_end_nodes]
    rng.shuffle(node_ids)
    best_route: List[int] = []
    for candidate in node_ids[:24]:
        route = sim.bfs_shortest_route(graph, int(start_node), int(candidate)) or []
        if len(route) > len(best_route):
            best_route = route
    if len(best_route) >= 2:
        return best_route
    if not node_ids:
        return [int(start_node)]
    return [int(start_node), int(node_ids[0])]


def route_targets(route: Sequence[int]) -> List[int]:
    if not route:
        return []
    if len(route) <= 3:
        return [int(route[-1])]
    mid = int(route[min(len(route) - 2, max(1, len(route) // 2))])
    end = int(route[-1])
    if mid == end:
        return [end]
    return [mid, end]


def pick_bridge_crossing_targets(graph: Any,
                                 *,
                                 start_node: int,
                                 bridge_nodes: Sequence[int],
                                 low_entry: int,
                                 high_entry: int,
                                 forbidden_end_nodes: Set[int]) -> List[int]:
    bridge_set = set(int(x) for x in bridge_nodes)
    if start_node not in graph.nodes or not bridge_set:
        return []
    xs = [graph.nodes[nid][0] for nid in bridge_set if nid in graph.nodes]
    ys = [graph.nodes[nid][1] for nid in bridge_set if nid in graph.nodes]
    horizontal = (max(xs) - min(xs)) >= (max(ys) - min(ys)) if xs and ys else True
    axis = 0 if horizontal else 1
    low_axis = graph.nodes.get(low_entry, (0.0, 0.0))[axis]
    high_axis = graph.nodes.get(high_entry, (0.0, 0.0))[axis]
    start_axis = graph.nodes.get(start_node, (0.0, 0.0))[axis]
    moving_to_high = abs(start_axis - low_axis) <= abs(start_axis - high_axis)

    def route_crosses_bridge(route: Sequence[int]) -> bool:
        return any(int(nid) in bridge_set for nid in route)

    best_route: List[int] = []
    for node_id in graph.nodes.keys():
        node_id = int(node_id)
        if node_id == int(start_node) or node_id in forbidden_end_nodes:
            continue
        route = sim.bfs_shortest_route(graph, int(start_node), node_id) or []
        if len(route) < 3 or not route_crosses_bridge(route):
            continue
        end_axis = graph.nodes.get(node_id, (0.0, 0.0))[axis]
        if moving_to_high and end_axis <= high_axis:
            continue
        if (not moving_to_high) and end_axis >= low_axis:
            continue
        if len(route) > len(best_route):
            best_route = route
    if not best_route:
        return []

    first_bridge = None
    for nid in best_route:
        if int(nid) in bridge_set:
            first_bridge = int(nid)
            break
    if first_bridge is None:
        return []
    end_node = int(best_route[-1])
    if end_node == first_bridge:
        return [first_bridge]
    return [first_bridge, end_node]


def pick_corridor_crossing_targets(graph: Any,
                                   *,
                                   start_node: int,
                                   opposite_entry: int,
                                   forbidden_end_nodes: Set[int]) -> List[int]:
    if start_node not in graph.nodes or opposite_entry not in graph.nodes:
        return []
    route_to_opposite = sim.bfs_shortest_route(graph, int(start_node), int(opposite_entry)) or []
    if len(route_to_opposite) < 2:
        return []

    best_route: List[int] = []
    for node_id in graph.nodes.keys():
        node_id = int(node_id)
        if node_id == int(start_node) or node_id in forbidden_end_nodes:
            continue
        route = sim.bfs_shortest_route(graph, int(start_node), node_id) or []
        if len(route) < len(route_to_opposite):
            continue
        overlap = False
        for nid in route_to_opposite[1:]:
            if nid in route:
                overlap = True
                break
        if not overlap:
            continue
        if len(route) > len(best_route):
            best_route = route
    if not best_route:
        best_route = route_to_opposite

    first_hop = int(route_to_opposite[min(len(route_to_opposite) - 1, 1)])
    end_node = int(best_route[-1])
    if end_node == first_hop:
        return [first_hop]
    return [first_hop, end_node]


def build_payload(graph: Any,
                  *,
                  map_raw: Dict[str, Any],
                  map_id: int,
                  map_version: str,
                  agv_count: int,
                  rng: random.Random) -> Tuple[List[Dict[str, Any]], Dict[str, Any], Dict[str, Any]]:
    candidate_nodes = sorted(int(nid) for nid in graph.nodes.keys())
    status_list = sim.generate_status_list(graph, agv_count, rng, map_id, "AGV", candidate_nodes)
    now_ts = int(time.time())

    web_map = sim_v2.build_web_map_payload(map_raw, agv_count)
    bridge_component = longest_bridge_component(web_map)
    bridge_pair = bridge_entries(graph, bridge_component)
    bridge_node_set = set(int(x) for x in bridge_component)

    corridor_pair = pick_secondary_corridor(graph, bridge_node_set)
    if corridor_pair is None:
        corridor_pair = bridge_pair

    overrides: Dict[str, int] = {}
    if bridge_pair is not None:
        overrides["AGV01"] = int(bridge_pair[0])
        overrides["AGV02"] = int(bridge_pair[1])
    if corridor_pair is not None:
        overrides["AGV03"] = int(corridor_pair[0])
        overrides["AGV04"] = int(corridor_pair[1])

    for idx, entry in enumerate(status_list):
        device_id = str(entry.get("deviceId", "")).strip()
        if device_id in overrides:
            status_list[idx] = sim.build_status_entry(
                graph,
                device_id=device_id,
                node_id=int(overrides[device_id]),
                map_id=map_id,
                angle=0.0 if device_id in ("AGV01", "AGV03") else 180.0,
                battery_level=90,
                endurance=180,
                now_ts=now_ts,
            )

    start_node_by_device: Dict[str, int] = {}
    for entry in status_list:
        device_id = str(entry.get("deviceId", "")).strip()
        node_id = sim.parse_node_id(entry.get("nodeId"))
        if device_id and node_id is not None:
            start_node_by_device[device_id] = int(node_id)

    tasks: List[Dict[str, Any]] = []
    task_index = 1
    forbidden_end_nodes: Set[int] = set(overrides.values())
    if bridge_pair is not None:
        low_entry, high_entry = bridge_pair
        agv01_targets: List[int] = [int(high_entry)]
        route_after_high = pick_far_route(graph, int(high_entry), rng, forbidden_end_nodes | {int(high_entry)})
        if route_after_high and int(route_after_high[-1]) != int(high_entry):
            agv01_targets.append(int(route_after_high[-1]))
        agv02_targets: List[int] = [int(low_entry)]
        route_after_low = pick_far_route(
            graph,
            int(low_entry),
            rng,
            forbidden_end_nodes | {int(low_entry)} | ({int(agv01_targets[-1])} if len(agv01_targets) > 1 else set()),
        )
        if route_after_low and int(route_after_low[-1]) != int(low_entry):
            agv02_targets.append(int(route_after_low[-1]))
        if agv01_targets:
            forbidden_end_nodes.add(int(agv01_targets[-1]))
            tasks.append(build_task(graph, task_id=f"TASK_{task_index:03d}", targets=agv01_targets, priority=9, agv_requirements=["AGV01"]))
            task_index += 1
        if agv02_targets:
            forbidden_end_nodes.add(int(agv02_targets[-1]))
            tasks.append(build_task(graph, task_id=f"TASK_{task_index:03d}", targets=agv02_targets, priority=9, agv_requirements=["AGV02"]))
            task_index += 1
    if corridor_pair is not None:
        left, right = corridor_pair
        agv03_targets: List[int] = [int(right)]
        route_after_right = pick_far_route(graph, int(right), rng, forbidden_end_nodes | {int(right)})
        if route_after_right and int(route_after_right[-1]) != int(right):
            agv03_targets.append(int(route_after_right[-1]))
        agv04_targets: List[int] = [int(left)]
        route_after_left = pick_far_route(
            graph,
            int(left),
            rng,
            forbidden_end_nodes | {int(left)} | ({int(agv03_targets[-1])} if len(agv03_targets) > 1 else set()),
        )
        if route_after_left and int(route_after_left[-1]) != int(left):
            agv04_targets.append(int(route_after_left[-1]))
        if agv03_targets:
            forbidden_end_nodes.add(int(agv03_targets[-1]))
            tasks.append(build_task(graph, task_id=f"TASK_{task_index:03d}", targets=agv03_targets, priority=8, agv_requirements=["AGV03"]))
            task_index += 1
        if agv04_targets:
            forbidden_end_nodes.add(int(agv04_targets[-1]))
            tasks.append(build_task(graph, task_id=f"TASK_{task_index:03d}", targets=agv04_targets, priority=8, agv_requirements=["AGV04"]))
            task_index += 1

    general_device_ids = [f"AGV{i:02d}" for i in range(5, min(agv_count, 20) + 1)]
    for device_id in general_device_ids:
        start_node = start_node_by_device.get(device_id)
        if start_node is None:
            continue
        route = pick_far_route(graph, start_node, rng, forbidden_end_nodes)
        targets = route_targets(route)
        if not targets:
            continue
        forbidden_end_nodes.add(int(targets[-1]))
        tasks.append(build_task(graph, task_id=f"TASK_{task_index:03d}", targets=targets, priority=5))
        task_index += 1

    payload = {
        "schedulingRequestId": f"DIRECTOR_REQ_{int(time.time() * 1000)}",
        "requestTimestamp": sim.iso_timestamp(),
        "requestType": 0,
        "taskNumber": len(tasks),
        "triggerContext": {"type": "sim-director-demo"},
        "triggerAgvId": [],
        "batchConfig": {"maxBatchSize": len(tasks)},
        "timeoutMs": 2000,
        "environmentContext": {
            "mapVersion": map_version,
            "systemLoad": 1,
            "systemLoadFactor": 0.0,
            "scenario": "director_demo",
        },
        "allocationAlgorithm": "posta",
        "candidateTasks": tasks,
    }
    summary = {
        "bridge_component_nodes": bridge_component,
        "bridge_entries": list(bridge_pair) if bridge_pair is not None else [],
        "corridor_pair": list(corridor_pair) if corridor_pair is not None else [],
        "task_count": len(tasks),
        "agv_count": len(status_list),
    }
    return status_list, payload, summary


def main() -> int:
    args = parse_args()
    map_path = Path(args.map)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    map_raw = load_json(map_path)
    graph = sim.load_map_graph(map_path)
    rng = random.Random(int(args.seed))
    status_list, tasks_payload, summary = build_payload(
        graph,
        map_raw=map_raw,
        map_id=int(args.map_id),
        map_version=str(args.map_version),
        agv_count=int(args.agv_count),
        rng=rng,
    )

    (out_dir / "status.json").write_text(json.dumps(status_list, ensure_ascii=False, indent=2), encoding="utf-8")
    (out_dir / "tasks.json").write_text(json.dumps(tasks_payload, ensure_ascii=False, indent=2), encoding="utf-8")
    (out_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[director-demo] status -> {out_dir / 'status.json'}")
    print(f"[director-demo] tasks  -> {out_dir / 'tasks.json'}")
    print(f"[director-demo] summary -> {out_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
