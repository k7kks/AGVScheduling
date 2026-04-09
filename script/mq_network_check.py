#!/usr/bin/env python3
"""
快速检测 RabbitMQ 网络配置是否可用：
1. 读取环境变量（已 source config/network.env）以获取 host/queue 等。
2. 连接到服务器，尝试被动声明队列验证是否存在。
3. 在限定时间内轮询获取消息并打印基本信息。
"""

import argparse
import json
import os
import sys
import time

import pika


def env(key: str, default: str = "") -> str:
    val = os.getenv(key)
    return val if val not in (None, "") else default


def build_connection_parameters() -> pika.ConnectionParameters:
    url = os.getenv("RABBITMQ_URL")
    if url:
        return pika.URLParameters(url)
    host = env("AMQP_HOST", "127.0.0.1")
    port = int(env("AMQP_PORT", "5672"))
    user = env("AMQP_USER", "guest")
    pwd = env("AMQP_PASS", "guest")
    vhost = env("AMQP_VHOST", "/")
    credentials = pika.PlainCredentials(user, pwd)
    return pika.ConnectionParameters(
        host=host,
        port=port,
        virtual_host=vhost,
        credentials=credentials,
        heartbeat=30,
        blocked_connection_timeout=10,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="连接 RabbitMQ 并验证是否能从指定队列收到消息。"
    )
    parser.add_argument(
        "--queue",
        default=env("EXT_QUEUE", env("ASSIGN_RESULT_QUEUE", "")),
        help="要检测的队列名称，默认读取 EXT_QUEUE 或 ASSIGN_RESULT_QUEUE。",
    )
    parser.add_argument(
        "--mirror",
        action="store_true",
        help="使用镜像（临时）队列监听 exchange，避免影响原队列消费。",
    )
    parser.add_argument(
        "--exchange",
        default=env("EXT_EXCHANGE", ""),
        help="镜像模式下绑定的 exchange，默认读 EXT_EXCHANGE。",
    )
    parser.add_argument(
        "--binding-key",
        default=env("EXT_BINDING_KEY", "#"),
        help="镜像模式下使用的 binding key；默认 '#'",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="期望接收的消息数量，<=0 表示持续监听直到 Ctrl+C。",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=300.0,
        help="等待消息的超时时间（秒，默认 300）。",
    )
    parser.add_argument(
        "--no-ack",
        action="store_true",
        help="仅检测不确认消息（默认会 ack）。",
    )
    parser.add_argument(
        "--show-body",
        action="store_true",
        help="打印完整消息体（否则只给出预览）。",
    )
    parser.add_argument(
        "--skip-routing",
        action="append",
        default=[],
        help="指定 routing key 时跳过输出（仍会 ACK），可重复使用。",
    )
    parser.add_argument(
        "--dump-template",
        action="store_true",
        help="解析 JSON 后输出结构模板（用占位符表示类型，便于对比）。",
    )
    return parser.parse_args()


def ensure_queue(channel: pika.adapters.blocking_connection.BlockingChannel, queue: str) -> None:
    if not queue:
        raise ValueError("未指定队列名称，可通过 --queue 或环境变量 EXT_QUEUE 提供。")
    try:
        channel.queue_declare(queue=queue, passive=True)
    except pika.exceptions.ChannelClosedByBroker as exc:
        raise RuntimeError(f"队列 '{queue}' 无法被动声明，请确认服务器端是否已创建：{exc}") from exc


def main() -> int:
    args = parse_args()
    params = build_connection_parameters()
    print(f"[MQ] connecting to {params.host}:{params.port} vhost='{params.virtual_host}' ...")
    connection = pika.BlockingConnection(params)
    channel = connection.channel()

    queue_name = args.queue
    if args.mirror:
        exch = args.exchange or env("EXT_EXCHANGE", "")
        if not exch:
            print("[MQ] 镜像模式需要提供 --exchange 或设置 EXT_EXCHANGE。")
            connection.close()
            return 2
        binding = args.binding_key or "#"
        result = channel.queue_declare(queue="", exclusive=True)
        queue_name = result.method.queue
        channel.queue_bind(queue=queue_name, exchange=exch, routing_key=binding)
        print(f"[MQ] 镜像监听 exchange='{exch}' binding='{binding}' -> 临时队列 '{queue_name}'")
    else:
        try:
            ensure_queue(channel, queue_name)
        except Exception as err:
            print(f"[MQ] 队列检查失败: {err}")
            connection.close()
            return 2

    goal_desc = "持续监听" if args.count <= 0 else f"目标消息数={args.count}"
    print(f"[MQ] 队列 '{queue_name}' 可用，开始轮询，{goal_desc}，单次等待超时={args.timeout}s")
    received = 0
    last_activity = time.time()

    while args.count <= 0 or received < args.count:
        if (time.time() - last_activity) > args.timeout:
            print("[MQ] 等待超时，继续监听...")
            last_activity = time.time()
        method_frame, header_frame, body = channel.basic_get(queue_name, auto_ack=False)
        if method_frame:
            if method_frame.routing_key in args.skip_routing:
                if not args.no_ack:
                    channel.basic_ack(method_frame.delivery_tag)
                continue
            received += 1
            last_activity = time.time()
            print(f"\n--- 消息 {received} ---")
            print(f"routing_key={method_frame.routing_key} delivery_tag={method_frame.delivery_tag} redelivered={method_frame.redelivered}")
            if header_frame:
                headers = {
                    "content_type": header_frame.content_type,
                    "content_encoding": header_frame.content_encoding,
                    "timestamp": header_frame.timestamp,
                    "message_id": header_frame.message_id,
                }
                print("properties:", {k: v for k, v in headers.items() if v})
            if body:
                try:
                    text = body.decode("utf-8")
                except UnicodeDecodeError:
                    text = body[:200].hex()
                preview = text if args.show_body else text[:200]
                summary = None
                try:
                    parsed = json.loads(text)
                    summary = describe_payload(method_frame.routing_key, parsed)
                    print("json:", json.dumps(parsed, ensure_ascii=False, indent=2))
                    if args.dump_template:
                        template = build_template(parsed)
                        print("[template]:", json.dumps(template, ensure_ascii=False, indent=2))
                except Exception:
                    print(f"body: {preview}")
                if summary:
                    print("summary:", summary)
            else:
                print("body: <empty>")
            if not args.no_ack:
                channel.basic_ack(method_frame.delivery_tag)
        else:
            connection.process_data_events(time_limit=0.2)

    channel.close()
    connection.close()
    print(f"[MQ] 已接收 {received} 条消息，脚本结束。")
    return 0 if received > 0 else 1


def describe_payload(routing_key: str, data) -> str:
    """
    根据 routing key 对消息做概要描述，方便快速了解内容。
    """
    try:
        if routing_key == "AssignmentTaskRequest":
            req_id = data.get("schedulingRequestId", "-")
            req_type = data.get("requestType", "-")
            task_count = len(data.get("candidateTasks", []))
            return f"调度请求 {req_id}, 类型={req_type}, task数={task_count}"
        if routing_key == "SendRobotStatusInfos":
            if isinstance(data, list):
                print(f"[summary] SendRobotStatusInfos list size={len(data)}")
                return ""
            cnt = len(data.get("robotStatusInfos", []))
            print(f"[summary] SendRobotStatusInfos count={cnt}")
            return f"机器人状态批次，数量={cnt}"
        if routing_key == "SendRobotConfigInfos":
            cnt = len(data.get("robotConfigInfos", []))
            return f"机器人配置批次，数量={cnt}"
        if routing_key == "SendMapInfo":
            map_version = data.get("mapVersion", "-")
            map_blob = data.get("mapData", {})
            if isinstance(map_blob, str):
                try:
                    map_blob = json.loads(map_blob)
                except Exception:
                    map_blob = {}
            print("map structure:", summarize_structure(map_blob))
            def count_items(blob: dict, keys) -> int:
                if not isinstance(blob, dict):
                    return 0
                for key in keys:
                    value = blob.get(key)
                    if isinstance(value, list):
                        return len(value)
                    if isinstance(value, dict):
                        return len(value)
                return 0
            nodes = count_items(map_blob, ("nodes", "node"))
            edges = count_items(map_blob, ("edges", "edge"))
            info = map_blob.get("info") if isinstance(map_blob, dict) else None
            if isinstance(info, dict):
                name = info.get("name", "-")
                map_id = info.get("mapId", "-")
                desc = info.get("desc", "")
                return f"地图版本={map_version}, mapId={map_id}, name={name}, nodes={nodes}, edges={edges}, desc={desc}"
            return f"地图版本={map_version}, nodes={nodes}, edges={edges}"
        if routing_key == "RobotPathRequest":
            return f"路径请求 deviceId={data.get('deviceId','-')} msgId={data.get('messageId','-')}"
        if routing_key == "RobotTrailRequest":
            return f"轨迹请求 deviceId={data.get('deviceId','-')} msgId={data.get('messageId','-')}"
        if routing_key == "AssignmentTaskResponse":
            assigned = len(data.get("agv_assignments", []))
            unassigned = len(data.get("unassigned_tasks", []))
            return f"任务分配结果，AGV数={assigned}, 未分配任务={unassigned}"
        if routing_key == "RobotPathResponse":
            return f"路径响应 deviceId={data.get('deviceId','-')} path段数={len(data.get('pathInfos', []))}"
        if routing_key == "RobotTrailResponse":
            return f"轨迹响应 deviceId={data.get('deviceId','-')} 控制点={len(data.get('controlPoints', []))}"
        if routing_key == "SendRunInfo":
            return f"运行日志 deviceId={data.get('id','-')} 描述={data.get('runDesc','')[:30]}"
        if routing_key == "SendAlertInfo":
            return f"告警 deviceId={data.get('id','-')} level={data.get('alertLevel','-')} desc={data.get('alertDesc','')[:30]}"
    except Exception:
        pass
    if isinstance(data, dict):
        keys = list(data.keys())
        return f"通用消息，字段={keys}"
    if isinstance(data, list):
        return f"列表消息，元素数={len(data)}"
    return f"类型={type(data).__name__}"


def summarize_structure(obj, depth: int = 0, max_depth: int = 2, max_items: int = 8) -> str:
    """
    返回对象结构的简短描述，避免输出完整内容。
    """
    if depth > max_depth:
        return type(obj).__name__
    if isinstance(obj, dict):
        parts = []
        for idx, (k, v) in enumerate(obj.items()):
            if idx >= max_items:
                parts.append("...") 
                break
            parts.append(f"{k}:{summarize_structure(v, depth + 1, max_depth, max_items)}")
        return "{" + ", ".join(parts) + "}"
    if isinstance(obj, list):
        if not obj:
            return "list(len=0)"
        sample = summarize_structure(obj[0], depth + 1, max_depth, max_items)
        suffix = "" if len(obj) == 1 else f", len={len(obj)}"
        return f"list({sample}{suffix})"
    return type(obj).__name__


def build_template(obj, depth: int = 0, max_depth: int = 4, max_items: int = 20):
    """
    将任意 JSON 对象转为占位模板，便于分享结构。
    """
    if depth >= max_depth:
        return type(obj).__name__
    if isinstance(obj, dict):
        result = {}
        for idx, (key, value) in enumerate(obj.items()):
            if idx >= max_items:
                result["..."] = "trimmed"
                break
            result[key] = build_template(value, depth + 1, max_depth, max_items)
        return result
    if isinstance(obj, list):
        if not obj:
            return []
        return [build_template(obj[0], depth + 1, max_depth, max_items)]
    if isinstance(obj, str):
        return "<string>"
    if isinstance(obj, bool):
        return False
    if isinstance(obj, int):
        return 0
    if isinstance(obj, float):
        return 0.0
    if obj is None:
        return None
    return str(obj)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[MQ] 中断退出")
        sys.exit(130)
