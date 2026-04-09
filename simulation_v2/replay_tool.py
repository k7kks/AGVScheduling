#!/usr/bin/env python3
import argparse
import json
import os
import sys
import urllib.parse
from bisect import bisect_right
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


def build_conflict_event(raw: Dict[str, Any], frames: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
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
            "raw_event_index": safe_int(raw.get("event_index"), 0),
        }
    return None


def build_timeline(raw_events: List[Dict[str, Any]], frames: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
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
            event = build_conflict_event(raw, frames)
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


def build_summary(
    session_meta: Dict[str, Any],
    frames: List[Dict[str, Any]],
    raw_events: List[Dict[str, Any]],
    timeline: List[Dict[str, Any]],
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
    timeline = build_timeline(raw_events, frames)
    summary = build_summary(session_json, frames, raw_events, timeline)
    bundle = {
        "metadata": {
            **session_json,
            "bundle_name": bundle_name,
            "profile": "leader_demo",
            "director_base_rate": 2.8,
        },
        "summary": summary,
        "map": map_json,
        "frames": frames,
        "timeline": timeline,
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
