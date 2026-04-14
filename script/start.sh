#!/usr/bin/env bash
#脚本出现异常会立即退出
set -euo pipefail

# 根目录
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

# 自动 export 配置文件里的赋值，避免“文件里配了但子进程收不到”。
set -a

# 如果存在网络相关环境配置，则加载
if [[ -f "${ROOT_DIR}/config/network.env" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/config/network.env"
fi

if [[ -f "${ROOT_DIR}/settings/production.env" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/settings/production.env"
else
  set +a
  echo "[start] missing ${ROOT_DIR}/settings/production.env" >&2
  exit 2
fi

set +a

# Receiver log switches (default OFF; use START_DEBUG=1 to enable summaries).
START_DEBUG="${START_DEBUG:-0}"
if [[ "${START_DEBUG}" == "1" ]]; then
  export RECEIVER_LOG_ROUTE_SUMMARY="${RECEIVER_LOG_ROUTE_SUMMARY:-1}"
  export RECEIVER_LOG_ROUTE_DETAIL="${RECEIVER_LOG_ROUTE_DETAIL:-0}"
  export RECEIVER_LOG_STATUS_SUMMARY="${RECEIVER_LOG_STATUS_SUMMARY:-1}"
  export RECEIVER_LOG_STARTUP_SUMMARY="${RECEIVER_LOG_STARTUP_SUMMARY:-1}"
  export RECEIVER_LOG_STATUS_NODE_XY="${RECEIVER_LOG_STATUS_NODE_XY:-0}"
  export RECEIVER_LOG_STATUS_SNAPSHOT="${RECEIVER_LOG_STATUS_SNAPSHOT:-0}"
  export RECEIVER_LOG_TRAIL_POINTS="${RECEIVER_LOG_TRAIL_POINTS:-0}"
  export RECEIVER_LOG_EACH_MESSAGE="${RECEIVER_LOG_EACH_MESSAGE:-0}"
  export RECEIVER_LOG_EACH_MESSAGE_STATUS="${RECEIVER_LOG_EACH_MESSAGE_STATUS:-0}"
  export RECEIVER_LOG_TRAFFIC_SUMMARY="${RECEIVER_LOG_TRAFFIC_SUMMARY:-1}"
  export RECEIVER_LOG_TRAFFIC_JSON="${RECEIVER_LOG_TRAFFIC_JSON:-0}"
  export RECEIVER_LOG_TRAFFIC_DETAIL="${RECEIVER_LOG_TRAFFIC_DETAIL:-0}"
  export RECEIVER_LOG_ALLOC_SUMMARY="${RECEIVER_LOG_ALLOC_SUMMARY:-0}"
  export RECEIVER_LOG_ALLOC_PROFILE="${RECEIVER_LOG_ALLOC_PROFILE:-0}"
  export RECEIVER_LOG_ALLOC_RESULT="${RECEIVER_LOG_ALLOC_RESULT:-0}"
  export RECEIVER_LOG_ALLOC_DETAIL="${RECEIVER_LOG_ALLOC_DETAIL:-0}"
  export RECEIVER_LOG_ALLOC_DIAG="${RECEIVER_LOG_ALLOC_DIAG:-0}"
  export RECEIVER_LOG_SCHED_REQUEST="${RECEIVER_LOG_SCHED_REQUEST:-1}"
  export RECEIVER_LOG_SCHED_TASK_DETAIL="${RECEIVER_LOG_SCHED_TASK_DETAIL:-0}"
  export RECEIVER_LOG_REJECT_RESP="${RECEIVER_LOG_REJECT_RESP:-0}"
  export RECEIVER_LOG_ROUTE_WARN="${RECEIVER_LOG_ROUTE_WARN:-0}"
else
  # 强制静音：覆盖外部 shell 里可能残留的 export 值。
  export RECEIVER_LOG_ROUTE_SUMMARY="0"
  export RECEIVER_LOG_ROUTE_DETAIL="0"
  export RECEIVER_LOG_STATUS_SUMMARY="0"
  export RECEIVER_LOG_STARTUP_SUMMARY="0"
  export RECEIVER_LOG_STATUS_NODE_XY="0"
  export RECEIVER_LOG_STATUS_SNAPSHOT="0"
  export RECEIVER_LOG_TRAIL_POINTS="0"
  export RECEIVER_LOG_EACH_MESSAGE="0"
  export RECEIVER_LOG_EACH_MESSAGE_STATUS="0"
  export RECEIVER_LOG_TRAFFIC_SUMMARY="0"
  export RECEIVER_LOG_TRAFFIC_JSON="0"
  export RECEIVER_LOG_TRAFFIC_DETAIL="0"
  export RECEIVER_LOG_ALLOC_SUMMARY="0"
  export RECEIVER_LOG_ALLOC_PROFILE="0"
  export RECEIVER_LOG_ALLOC_RESULT="0"
  export RECEIVER_LOG_ALLOC_DETAIL="0"
  export RECEIVER_LOG_ALLOC_DIAG="0"
  export RECEIVER_LOG_SCHED_REQUEST="0"
  export RECEIVER_LOG_SCHED_TASK_DETAIL="0"
  export RECEIVER_LOG_REJECT_RESP="0"
  export RECEIVER_LOG_ROUTE_WARN="0"
fi

# 其他高频日志默认关闭。
export STATUS_LAG_LOG_SEC="${STATUS_LAG_LOG_SEC:-0}"
export STATUS_LAG_WARN_MS="${STATUS_LAG_WARN_MS:-0}"
export REPLAN_PROFILE="${REPLAN_PROFILE:-0}"
export EXT_PRINT_UNALLOC_CAUSE="${EXT_PRINT_UNALLOC_CAUSE:-0}"
TRAFFIC_DUMP_KEEP_MAX="${TRAFFIC_DUMP_KEEP_MAX:-200}"   # 交通 JSON 保留数量（0=不限制）

if [[ "${DEBUG_DIR}" != /* ]]; then
  DEBUG_DIR="${ROOT_DIR}/${DEBUG_DIR}"
fi
mkdir -p "${DEBUG_DIR}"

TS="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${DEBUG_DIR}/${RECEIVER_LOG_PREFIX}_${TS}.log"
ln -sf "${LOG_FILE}" "${DEBUG_DIR}/external_receiver_latest.log" 2>/dev/null || true

# 默认关闭结果/交通 JSON 落盘；调用者可显式设置目录开启。
if [[ ! -v RESULT_DUMP_DIR ]]; then
  RESULT_DUMP_DIR=""
fi
if [[ -z "${MAP_CACHE_FILE:-}" ]]; then
  MAP_CACHE_FILE="${DEBUG_DIR}/received_map.json"
fi

export DEBUG_DIR
export RESULT_DUMP_DIR
export TRAFFIC_DUMP_KEEP_MAX
export MAP_CACHE_FILE

# 可用时优先使用可执行的编译产物目录
RECEIVER_BIN="${ROOT_DIR}/build/bin/external_receiver"
if [[ ! -x "${RECEIVER_BIN}" && -x "${ROOT_DIR}/build_local/bin/external_receiver" ]]; then
  RECEIVER_BIN="${ROOT_DIR}/build_local/bin/external_receiver"
fi
chmod +x "${RECEIVER_BIN}"

# 启动 external_receiver（按时间滚动日志，避免单文件过大）
if [[ "${RECEIVER_LOG_ROTATE_SEC}" =~ ^[0-9]+$ ]] && (( RECEIVER_LOG_ROTATE_SEC > 0 )); then
  exec > >(
    bash "${ROTATE_SCRIPT}" \
      --log-dir "${DEBUG_DIR}" \
      --prefix "${RECEIVER_LOG_PREFIX}" \
      --first-file "${LOG_FILE}" \
      --rotate-sec "${RECEIVER_LOG_ROTATE_SEC}" \
      --keep "${RECEIVER_LOG_KEEP}" \
      --prune-json \
      --json-dir "${RESULT_DUMP_DIR}" \
      --latest-symlink "${DEBUG_DIR}/external_receiver_latest.log"
  ) 2>&1
else
  exec > >(tee -a "${LOG_FILE}") 2>&1
fi

exec "${RECEIVER_BIN}"
