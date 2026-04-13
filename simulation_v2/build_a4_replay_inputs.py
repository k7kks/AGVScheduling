#!/usr/bin/env python3
import argparse
import csv
import json
import random
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE_DIR = ROOT / "demo" / "source_data" / "A4产线任务单数据-近1月"
DEFAULT_MAP_PATH = ROOT / "config" / "A4_from_xml.json"
DEFAULT_OUT_DIR = ROOT / "demo" / "inputs" / "generated_a4_real"
CSV_FILTER = "csv:Text - txt - csv (StarCalc):44,34,76,1,,0,false,true,false,false"
SHANGHAI_TZ = timezone(timedelta(hours=8))


def _load_sim_v1():
    path = ROOT / "simulation" / "sim_runner.py"
    spec = spec_from_file_location("sim_v1_for_replay_inputs", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load simulator helpers from {path}")
    mod = module_from_spec(spec)
    spec.loader.exec_module(mod)  # type: ignore[assignment]
    return mod


sim = _load_sim_v1()


@dataclass
class MapPoint:
    node_id: int
    x: float
    y: float


def iso_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Build simulator-ready A4 replay inputs from historical XLS exports")
    p.add_argument("--source-dir", default=str(DEFAULT_SOURCE_DIR), help="Directory containing the A4 historical XLS exports")
    p.add_argument("--map", dest="map_path", default=str(DEFAULT_MAP_PATH), help="Map JSON path")
    p.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR), help="Output directory for generated JSON files")
    p.add_argument("--map-version", default="south_20260107", help="Map version string for generated payload")
    p.add_argument("--map-id", type=int, default=20260107, help="Map ID for generated status payload")
    p.add_argument("--task-limit", type=int, default=0, help="Limit selected tasks; 0 means all cleaned tasks")
    p.add_argument(
        "--task-order",
        choices=("created", "latest", "random"),
        default="created",
        help="Ordering when selecting tasks",
    )
    p.add_argument("--seed", type=int, default=7, help="Random seed when task-order=random")
    p.add_argument("--status-count", type=int, default=0, help="Generate this many simulator AGV statuses from the map; 0 keeps historical AGV count")
    p.add_argument("--device-prefix", default="AGV", help="Device prefix used with --status-count")
    p.add_argument("--include-cancelled", action="store_true", help="Keep non-'已结束' tasks in the generated payload")
    p.add_argument("--no-bind-history", action="store_true", help="Do not preserve historical AGV bindings in agvRequirements")
    p.add_argument("--no-attach-status", action="store_true", help="Do not embed agvStatusList into the generated tasks payload")
    p.add_argument("--max-subtasks", type=int, default=0, help="Cap subtask points per task; 0 means no limit")
    return p.parse_args()


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def to_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        return float(text)
    except Exception:
        return None


def to_int(value: Any, default: int = 0) -> int:
    try:
        return int(str(value).strip())
    except Exception:
        return default


def coord_key(x: float, y: float) -> str:
    return f"{round(float(x), 3):.3f},{round(float(y), 3):.3f}"


def load_map_index(map_path: Path) -> Dict[str, MapPoint]:
    data = load_json(map_path)
    if isinstance(data, dict) and isinstance(data.get("mapData"), dict):
        data = data["mapData"]
    nodes = data.get("node") or data.get("nodes") or []
    coord_to_point: Dict[str, MapPoint] = {}
    for node in nodes:
        if not isinstance(node, dict):
            continue
        node_id = node.get("id", node.get("nodeId"))
        x = to_float(node.get("x"))
        y = to_float(node.get("y"))
        if node_id is None or x is None or y is None:
            continue
        coord_to_point[coord_key(x, y)] = MapPoint(node_id=to_int(node_id), x=x, y=y)
    return coord_to_point


def convert_xls_to_csv(source_dir: Path, output_dir: Path) -> List[Path]:
    office = shutil.which("libreoffice") or shutil.which("soffice")
    if not office:
        raise RuntimeError("libreoffice/soffice not found; cannot convert .xls exports")
    files = sorted(source_dir.glob("*.xls"))
    if not files:
        raise RuntimeError(f"no .xls files found under {source_dir}")
    csv_paths: List[Path] = []
    output_dir.mkdir(parents=True, exist_ok=True)
    for file_path in files:
        cmd = [
            office,
            "--headless",
            "--convert-to",
            CSV_FILTER,
            "--outdir",
            str(output_dir),
            str(file_path),
        ]
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if proc.returncode != 0:
            raise RuntimeError(f"failed to convert {file_path.name}: {proc.stderr.strip() or proc.stdout.strip()}")
        csv_path = output_dir / f"{file_path.stem}.csv"
        if not csv_path.exists():
            raise RuntimeError(f"expected converted csv missing: {csv_path}")
        csv_paths.append(csv_path)
    return csv_paths


def read_csv_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        row["_file"] = path.name
    return rows


def parse_task_time(value: str) -> Optional[datetime]:
    text = str(value).strip()
    if not text:
        return None
    try:
        return datetime.strptime(text, "%Y-%m-%d %H:%M:%S").replace(tzinfo=SHANGHAI_TZ)
    except Exception:
        return None


def parse_xml_task_type(message: str) -> str:
    text = str(message or "").strip()
    if not text.startswith("<?xml"):
        return ""
    try:
        root = ET.fromstring(text)
    except Exception:
        return ""
    return str(root.findtext(".//TaskType") or "").strip()


def map_point_from_row(row: Dict[str, str], x_key: str, y_key: str, coord_to_point: Dict[str, MapPoint]) -> Optional[MapPoint]:
    x = to_float(row.get(x_key, ""))
    y = to_float(row.get(y_key, ""))
    if x is None or y is None:
        return None
    return coord_to_point.get(coord_key(x, y))


def point_type_priority(point_type: int) -> int:
    if point_type in (0, 1):
        return 2
    if point_type == 2:
        return 1
    return 0


def append_point(
    out: List[Dict[str, Any]],
    *,
    point: MapPoint,
    point_type: int,
    location: str,
    estimated_ms: int,
) -> None:
    node_text = str(point.node_id)
    if out and str(out[-1]["point"]["nodeId"]) == node_text:
        if point_type_priority(point_type) >= point_type_priority(int(out[-1].get("pointType", 2))):
            out[-1]["pointType"] = int(point_type)
        if estimated_ms > int(out[-1].get("estimatedDuration", 0)):
            out[-1]["estimatedDuration"] = int(estimated_ms)
        if not out[-1].get("location") and location:
            out[-1]["location"] = location
        return
    out.append(
        {
            "sequence": 0,
            "pointType": int(point_type),
            "location": location or f"node_{point.node_id}",
            "point": {
                "x": float(point.x),
                "y": float(point.y),
                "angle": 0,
                "nodeId": node_text,
            },
            "estimatedDuration": int(estimated_ms),
            "pathConstraints": [],
            "maxSpeed": 0,
            "subTaskId": "",
        }
    )


def expand_subtask_points(row: Dict[str, str], coord_to_point: Dict[str, MapPoint]) -> List[Tuple[MapPoint, int, str, int]]:
    xml_task_type = parse_xml_task_type(row.get("任务执行消息", ""))
    start_point = map_point_from_row(row, "起点X坐标", "起点Y坐标", coord_to_point)
    end_point = map_point_from_row(row, "终点X坐标", "终点Y坐标", coord_to_point)
    location = str(row.get("呼叫站点", "")).strip()
    estimated_ms = to_int(row.get("停留时间", ""), 0)
    entries: List[Tuple[MapPoint, int, str, int]] = []
    if xml_task_type == "MOVE_ROBOT":
        if end_point is not None:
            entries.append((end_point, 2, location, estimated_ms))
        return entries
    if xml_task_type in ("GET_POD", "MOVE_POD", "RETURN_POD"):
        if start_point is not None:
            entries.append((start_point, 0, location, estimated_ms))
        if end_point is not None:
            entries.append((end_point, 1, location, estimated_ms))
        return entries
    if start_point is not None:
        entries.append((start_point, 2, location, estimated_ms))
    if end_point is not None:
        entries.append((end_point, 2, location, estimated_ms))
    return entries


def build_task_subtasks(
    task_id: str,
    rows: Sequence[Dict[str, str]],
    coord_to_point: Dict[str, MapPoint],
    max_subtasks: int = 0,
) -> Tuple[List[Dict[str, Any]], Optional[MapPoint]]:
    ordered = sorted(rows, key=lambda item: to_int(item.get("子任务执行顺序", ""), 0))
    built: List[Dict[str, Any]] = []
    initial_status_point: Optional[MapPoint] = None
    for row in ordered:
        row_start = map_point_from_row(row, "起点X坐标", "起点Y坐标", coord_to_point)
        row_end = map_point_from_row(row, "终点X坐标", "终点Y坐标", coord_to_point)
        if initial_status_point is None:
            initial_status_point = row_start or row_end
        for point, point_type, location, estimated_ms in expand_subtask_points(row, coord_to_point):
            append_point(
                built,
                point=point,
                point_type=point_type,
                location=location,
                estimated_ms=estimated_ms,
            )
    if max_subtasks > 0 and len(built) > max_subtasks:
        built = built[:max_subtasks]
    for seq, item in enumerate(built, start=1):
        item["sequence"] = seq
        item["subTaskId"] = f"{task_id}#{seq}"
    return built, initial_status_point


def valid_agv_id(value: str) -> bool:
    text = str(value or "").strip()
    return bool(text) and text != "-1"


def build_status_entry(device_id: str, point: MapPoint, map_id: int) -> Dict[str, Any]:
    now_ts = int(time.time())
    return {
        "deviceId": str(device_id),
        "mapId": int(map_id),
        "curArea": "",
        "connection": True,
        "x": float(point.x),
        "y": float(point.y),
        "angle": 0,
        "speed": 0,
        "nodeId": str(point.node_id),
        "nextDestinationPoint": {},
        "curTrailPoints": [],
        "taskId": "",
        "taskStatus": 0,
        "taskProgress": 0,
        "estimatedDuration": 0,
        "batteryLevel": 80,
        "endurance": 120,
        "load": False,
        "errorCode": 0,
        "updateTime": now_ts,
        "agv_type": 1,
    }


def task_sort_key(task: Dict[str, Any]) -> Tuple[int, str]:
    created_at = parse_task_time(str(task.get("_sourceCreatedAt", "")))
    if created_at is None:
        return (0, str(task.get("taskId", "")))
    return (int(created_at.timestamp()), str(task.get("taskId", "")))


def main() -> int:
    args = parse_args()
    source_dir = Path(args.source_dir)
    map_path = Path(args.map_path)
    out_dir = Path(args.out_dir)

    if not source_dir.exists():
        print(f"[a4_replay] source dir not found: {source_dir}", file=sys.stderr)
        return 2
    if not map_path.exists():
        print(f"[a4_replay] map file not found: {map_path}", file=sys.stderr)
        return 2

    coord_to_point = load_map_index(map_path)
    if not coord_to_point:
        print("[a4_replay] map has no usable nodes", file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory(prefix="a4_replay_csv_") as tmp_dir_name:
        csv_dir = Path(tmp_dir_name)
        csv_paths = convert_xls_to_csv(source_dir, csv_dir)
        main_rows: List[Dict[str, str]] = []
        sub_rows: List[Dict[str, str]] = []
        for path in csv_paths:
            if "子任务" in path.name:
                sub_rows.extend(read_csv_rows(path))
            elif "任务单数据导出" in path.name:
                main_rows.extend(read_csv_rows(path))

    main_by_id: Dict[str, Dict[str, str]] = {}
    for row in main_rows:
        task_id = str(row.get("任务单头", "")).strip()
        if not task_id:
            continue
        if not args.include_cancelled and str(row.get("任务状态", "")).strip() != "已结束":
            continue
        main_by_id.setdefault(task_id, row)

    sub_by_task: Dict[str, List[Dict[str, str]]] = defaultdict(list)
    seen_sub_ids: set[str] = set()
    for row in sub_rows:
        sub_task_id = str(row.get("子任务编号", "")).strip()
        if sub_task_id:
            if sub_task_id in seen_sub_ids:
                continue
            seen_sub_ids.add(sub_task_id)
        task_id = str(row.get("任务单头", "")).strip()
        if not task_id or task_id not in main_by_id:
            continue
        sub_by_task[task_id].append(row)

    tasks: List[Dict[str, Any]] = []
    skipped_missing_subtasks = 0
    skipped_missing_points = 0
    status_seed_by_agv: Dict[str, MapPoint] = {}
    for task_id, main_row in main_by_id.items():
        rows = sub_by_task.get(task_id, [])
        if not rows:
            skipped_missing_subtasks += 1
            continue
        sub_tasks, initial_status_point = build_task_subtasks(task_id, rows, coord_to_point, max_subtasks=int(args.max_subtasks))
        if not sub_tasks:
            skipped_missing_points += 1
            continue
        historical_agv_id = str(main_row.get("AGV编号", "")).strip()
        if valid_agv_id(historical_agv_id) and initial_status_point is not None and historical_agv_id not in status_seed_by_agv:
            status_seed_by_agv[historical_agv_id] = initial_status_point
        priority = max(1, to_int(main_row.get("优先级", ""), 1))
        task: Dict[str, Any] = {
            "taskId": task_id,
            "priority": priority,
            "minBatteryLevel": 0,
            "taskType": str(main_row.get("任务类型", "")).strip(),
            "subTasks": sub_tasks,
            "agvRequirements": [historical_agv_id] if (not args.no_bind_history and valid_agv_id(historical_agv_id)) else [],
            "timestamp": str(main_row.get("创建日期", "")).strip(),
            "_sourceAgvId": historical_agv_id,
            "_sourceCreatedAt": str(main_row.get("创建日期", "")).strip(),
            "_sourceFinishedAt": str(main_row.get("状态变更日期", "")).strip(),
            "_sourceStatus": str(main_row.get("任务状态", "")).strip(),
            "_initialStatusPoint": initial_status_point,
        }
        created_at = parse_task_time(task["_sourceCreatedAt"])
        finished_at = parse_task_time(task["_sourceFinishedAt"])
        if created_at is not None:
            task["expectedStartTime"] = created_at.isoformat(timespec="seconds")
        if created_at is not None and finished_at is not None and finished_at >= created_at:
            task["expectedCompletionTime"] = finished_at.isoformat(timespec="seconds")
        tasks.append(task)

    if args.task_order == "created":
        tasks.sort(key=task_sort_key)
    elif args.task_order == "latest":
        tasks.sort(key=task_sort_key, reverse=True)
    else:
        rng = random.Random(int(args.seed))
        rng.shuffle(tasks)

    selected_tasks = list(tasks)
    if int(args.task_limit) > 0:
        selected_tasks = selected_tasks[: int(args.task_limit)]

    selected_agv_ids: List[str] = []
    seen_agv_ids: set[str] = set()
    selected_status_seed_by_agv: Dict[str, MapPoint] = {}
    for task in selected_tasks:
        source_agv_id = str(task.get("_sourceAgvId", "")).strip()
        if valid_agv_id(source_agv_id) and source_agv_id not in selected_status_seed_by_agv:
            initial_point = task.get("_initialStatusPoint")
            if isinstance(initial_point, MapPoint):
                selected_status_seed_by_agv[source_agv_id] = initial_point
        agv_reqs = task.get("agvRequirements")
        if not isinstance(agv_reqs, list) or len(agv_reqs) != 1:
            continue
        agv_id = str(agv_reqs[0]).strip()
        if not valid_agv_id(agv_id) or agv_id in seen_agv_ids:
            continue
        seen_agv_ids.add(agv_id)
        selected_agv_ids.append(agv_id)

    if not selected_agv_ids:
        historical_ids = sorted(k for k in status_seed_by_agv.keys() if valid_agv_id(k))
        selected_agv_ids = historical_ids

    status_list = [
        build_status_entry(agv_id, selected_status_seed_by_agv.get(agv_id, status_seed_by_agv[agv_id]), int(args.map_id))
        for agv_id in selected_agv_ids
        if agv_id in status_seed_by_agv
    ]
    if int(args.status_count) > 0:
        graph = sim.load_map_graph(map_path)
        status_list = sim.generate_status_list(
            graph,
            int(args.status_count),
            random.Random(int(args.seed)),
            int(args.map_id),
            str(args.device_prefix),
        )
        generated_ids = {str(item.get("deviceId", "")).strip() for item in status_list}
        for task in selected_tasks:
            agv_reqs = task.get("agvRequirements")
            if not isinstance(agv_reqs, list):
                continue
            task["agvRequirements"] = [
                str(agv_id).strip()
                for agv_id in agv_reqs
                if str(agv_id).strip() in generated_ids
            ]


    clean_tasks_payload: Dict[str, Any] = {
        "schedulingRequestId": f"A4_REAL_REQ_{int(time.time() * 1000)}",
        "requestTimestamp": iso_now(),
        "requestType": 0,
        "taskNumber": len(selected_tasks),
        "triggerContext": {"type": "a4-real-history"},
        "triggerAgvId": [],
        "batchConfig": {"maxBatchSize": len(selected_tasks)},
        "timeoutMs": 2000,
        "environmentContext": {
            "mapVersion": str(args.map_version),
            "systemLoad": 1,
            "systemLoadFactor": 0.0,
            "sourceDir": str(source_dir),
            "selectedTaskCount": len(selected_tasks),
            "historicalTaskCount": len(tasks),
            "historicalAgvCount": len(status_list),
        },
        "candidateTasks": [],
    }
    if not args.no_attach_status:
        clean_tasks_payload["agvStatusList"] = status_list

    # Rewrite historical timestamps to current time so age-bonus
    # in the allocator does not collapse all costs to the floor.
    fresh_ts = datetime.now(SHANGHAI_TZ).isoformat(timespec="seconds")
    for task in selected_tasks:
        clean_task = {k: v for k, v in task.items() if not k.startswith("_")}
        for ts_key in ("timestamp", "createTimestamp", "expectedStartTime",
                        "createTimestampISO", "expectedStartTimeISO",
                        "expectedCompletionTime"):
            if ts_key in clean_task:
                clean_task[ts_key] = fresh_ts
        clean_tasks_payload["candidateTasks"].append(clean_task)

    summary = {
        "sourceDir": str(source_dir),
        "mapPath": str(map_path),
        "mapVersion": str(args.map_version),
        "mapId": int(args.map_id),
        "rawMainRows": len(main_rows),
        "rawSubRows": len(sub_rows),
        "cleanedTasksAvailable": len(tasks),
        "selectedTasks": len(selected_tasks),
        "selectedAgvCount": len(status_list),
        "selectedAgvIds": [item["deviceId"] for item in status_list],
        "skippedMissingSubtasks": skipped_missing_subtasks,
        "skippedMissingPoints": skipped_missing_points,
        "taskTypeCounts": dict(Counter(str(t.get("taskType", "")).strip() for t in tasks)),
        "selectedTaskTypeCounts": dict(Counter(str(t.get("taskType", "")).strip() for t in selected_tasks)),
        "taskOrder": str(args.task_order),
        "taskLimit": int(args.task_limit),
        "historyBindings": not bool(args.no_bind_history),
    }

    write_json(out_dir / "summary.json", summary)
    write_json(out_dir / "status.json", status_list)
    write_json(out_dir / "tasks.json", clean_tasks_payload)

    print(f"[a4_replay] tasks  -> {out_dir / 'tasks.json'}")
    print(f"[a4_replay] status -> {out_dir / 'status.json'}")
    print(f"[a4_replay] summary -> {out_dir / 'summary.json'}")
    print(
        f"[a4_replay] selected tasks={len(selected_tasks)} historical tasks={len(tasks)} "
        f"agvs={len(status_list)}",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
