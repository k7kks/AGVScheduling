#!/usr/bin/env bash
set -euo pipefail

SIM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SIM_ROOT}/.." && pwd)"

# Avoid polluting the repo with __pycache__/ during long-running processes.
# Force-enable even if the caller has exported a different value.
export PYTHONDONTWRITEBYTECODE="1"

is_true() {
  local v="${1:-}"
  shopt -s nocasematch
  case "${v}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

resolve_vis_urls() {
  local host="$1"
  local port="$2"
  local ips=""
  local urls=""
  if [[ "${host}" == "0.0.0.0" || "${host}" == "::" ]]; then
    if command -v ip >/dev/null 2>&1; then
      ips="$(ip -o -4 addr show 2>/dev/null | awk '{print $4}' | cut -d/ -f1)"
    elif command -v hostname >/dev/null 2>&1; then
      ips="$(hostname -I 2>/dev/null || true)"
    fi
    for ip in ${ips}; do
      if [[ "${ip}" == 127.* ]]; then
        continue
      fi
      urls+=$'http://'"${ip}:${port}/"$'\n'
    done
    if [[ -z "${urls}" ]]; then
      urls+="http://127.0.0.1:${port}/"$'\n'
    fi
  else
    urls+="http://${host}:${port}/"$'\n'
  fi
  printf '%s' "${urls}"
}

if [[ ! -f "${ROOT_DIR}/CMakeLists.txt" ]]; then
  echo "[run_all_v2] repo root not found: ${ROOT_DIR}" >&2
  exit 2
fi

if [[ -f "${ROOT_DIR}/config/network.env" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/config/network.env"
else
  echo "[run_all_v2] missing ${ROOT_DIR}/config/network.env" >&2
  exit 2
fi

# Always use repo-level debug/ (do not allow per-run debug subfolders).
export DEBUG_DIR="${ROOT_DIR}/debug"
# Quiet mode: do not dump JSON, keep logs minimal.
export RESULT_DUMP_DIR=""
export RESULT_DUMP_KEEP_MAX="0"
export MAP_CACHE_FILE=""
export START_DEBUG="0"
export RECEIVER_LOG_ROUTE_SUMMARY="0"
export RECEIVER_LOG_ROUTE_DETAIL="0"
export RECEIVER_LOG_STATUS_SUMMARY="0"
export RECEIVER_LOG_STATUS_NODE_XY="0"
export RECEIVER_LOG_TRAIL_POINTS="0"
export RECEIVER_LOG_EACH_MESSAGE="0"
export RECEIVER_LOG_EACH_MESSAGE_STATUS="0"
export RECEIVER_LOG_TRAFFIC_SUMMARY="0"
export RECEIVER_LOG_TRAFFIC_JSON="0"
export RECEIVER_LOG_TRAFFIC_DETAIL="0"
# Enable reserved-node snapshots for the web UI by default.
export ENABLE_SIM_RESERVED_STREAM="${ENABLE_SIM_RESERVED_STREAM:-1}"
export SIM_RESERVED_INTERVAL_MS="${SIM_RESERVED_INTERVAL_MS:-200}"
# Simulation pose stream (optional).
export ENABLE_SIM_POSE_STREAM="${ENABLE_SIM_POSE_STREAM:-0}"
export SIM_POSE_INTERVAL_MS="${SIM_POSE_INTERVAL_MS:-200}"
export SIM_POSE_EXCHANGE="${SIM_POSE_EXCHANGE:-AlgoSimExchange}"
export SIM_POSE_EXCHANGE_TYPE="${SIM_POSE_EXCHANGE_TYPE:-fanout}"
export SIM_POSE_QUEUE="${SIM_POSE_QUEUE:-AlgoSimQueue}"
export SIM_POSE_ROUTING_KEY="${SIM_POSE_ROUTING_KEY:-SimPose}"
export SIM_POSE_BINDING_KEY="${SIM_POSE_BINDING_KEY:-#}"
# Throttle status publishes (ms); reduces MQ backlog with many AGVs.
export SIM_STATUS_INTERVAL_MS="${SIM_STATUS_INTERVAL_MS:-300}"
# Bootstrap map settle wait (s): external_receiver reloads map asynchronously,
# so map/status/tasks sent back-to-back can race on startup.
export SIM_BOOTSTRAP_MAP_WAIT_SEC="${SIM_BOOTSTRAP_MAP_WAIT_SEC:-1.5}"
# Subtask dwell durations (ms). Keep short by default so run_all focuses on traffic flow, not station waits.
export SIM_SUBTASK_PICKUP_DURATION_MS="${SIM_SUBTASK_PICKUP_DURATION_MS:-1000}"
export SIM_SUBTASK_DROPOFF_DURATION_MS="${SIM_SUBTASK_DROPOFF_DURATION_MS:-1000}"
export SIM_SUBTASK_WAYPOINT_DURATION_MS="${SIM_SUBTASK_WAYPOINT_DURATION_MS:-1000}"
# Pika heartbeat tuning (avoid heartbeat timeout under heavy load).
export AMQP_HEARTBEAT="${AMQP_HEARTBEAT:-120}"
export AMQP_BLOCKED_TIMEOUT="${AMQP_BLOCKED_TIMEOUT:-60}"
# Around-path obstacle blocking thresholds (mm).
export AROUND_PATH_NODE_BLOCK_MM="${AROUND_PATH_NODE_BLOCK_MM:-300}"
export AROUND_PATH_EDGE_BLOCK_MM="${AROUND_PATH_EDGE_BLOCK_MM:-300}"
# Dump reserved nodes snapshot to JSON when explicitly set.
export SIM_RESERVED_DUMP_PATH=""
# RobotTrailResponse 截断长度（节点数）。
# 定时补充 trail（毫秒）。设置 100~200 可显著减少“走一下停一下”（rolling window 末端断粮）。
# 置 0 则关闭，仅按 RobotTrailRequest 响应。
# Sim-only: trail points <= this value trigger braking (decoupled from TRAIL_MAX_POINTS).
export SIM_TRAIL_MIN_DRIVE_POINTS="${SIM_TRAIL_MIN_DRIVE_POINTS:-3}"
# 预约窗口默认与 trail 发布长度对齐（避免未发布的长窗口造成“前面空空如也但被预约挡住”）。
export RESERVE_MATCH_TRAIL_POINTS="${RESERVE_MATCH_TRAIL_POINTS:-1}"
# Dump simulator assignment responses to debug/ by default.
export SIM_ASSIGN_RESULT_DIR=""
export SIM_ASSIGN_RESULT_KEEP="0"
# Dump traffic JSONs to debug/ by default.
export TRAFFIC_DUMP_KEEP_MAX="-1"
# Congestion region mode: grid | graph | cluster | partition
# Defaults are handled in code (graph). Override via env if needed.
export CONGESTION_REGION_COUNT="${CONGESTION_REGION_COUNT:-0}"

# Never write simulator stdout/stderr to log files from this wrapper.
SIM_LOG_ENABLE="0"

# Use shared exchanges/queues from config/network.env (no per-run temp resources).
SIM_USE_SHARED_MQ="1"
echo "[run_all_v2] EXT_EXCHANGE=${EXT_EXCHANGE} EXT_QUEUE=${EXT_QUEUE} ASSIGN_RESULT_EXCHANGE=${ASSIGN_RESULT_EXCHANGE}" >&2

MAP_FILE_DEFAULT="${ROOT_DIR}/config/A4_from_xml.json"
MAP_FILE="${MAP_FILE:-${MAP_FILE_DEFAULT}}"
export MAP_FILE
if [[ ! -f "${MAP_FILE}" ]]; then
  echo "[run_all_v2] map file not found: ${MAP_FILE}" >&2
  exit 2
fi

PYTHON_BIN="${PYTHON_BIN:-python3}"
SIM_MAP_VERSION="${SIM_MAP_VERSION:-${MAP_VERSION:-south_20260107}}"
SIM_MAP_ID="${SIM_MAP_ID:-20260107}"

# Primary knobs (override via env):
# - SIM_AGV_COUNT: number of AGVs
# - SIM_TASK_COUNT: initial tasks at bootstrap
# - SIM_TASK_MULTIPLIER: if SIM_TASK_COUNT unset, tasks = agv_count * multiplier
SIM_AGV_COUNT="${SIM_AGV_COUNT:-30}"
export SIM_AGV_COUNT
if ! [[ "${SIM_AGV_COUNT}" =~ ^[0-9]+$ ]]; then
  echo "[run_all_v2] SIM_AGV_COUNT invalid: ${SIM_AGV_COUNT}; using 2" >&2
  SIM_AGV_COUNT=2
fi

SIM_TASK_MULTIPLIER="${SIM_TASK_MULTIPLIER:-3}"
if ! [[ "${SIM_TASK_MULTIPLIER}" =~ ^[0-9]+$ ]]; then
  echo "[run_all_v2] SIM_TASK_MULTIPLIER invalid: ${SIM_TASK_MULTIPLIER}; using 3" >&2
  SIM_TASK_MULTIPLIER=3
fi
if [[ -z "${SIM_TASK_COUNT:-}" ]]; then
  SIM_TASK_COUNT=$(( SIM_AGV_COUNT * SIM_TASK_MULTIPLIER ))
fi
SIM_STATUS_FILE="${SIM_STATUS_FILE:-}"
SIM_TASK_FILE="${SIM_TASK_FILE:-}"
SIM_GENERATE="${SIM_GENERATE:-1}"
if [[ -n "${SIM_STATUS_FILE}" || -n "${SIM_TASK_FILE}" ]]; then
  SIM_GENERATE="0"
fi
SIM_RECORD_REPLAY="${SIM_RECORD_REPLAY:-0}"
SIM_RECORD_SESSION="${SIM_RECORD_SESSION:-}"
SIM_RECORD_DIR="${SIM_RECORD_DIR:-${ROOT_DIR}/demo/sessions}"

SIM_SEED="${SIM_SEED:-7}"
SIM_STEP_INTERVAL="${SIM_STEP_INTERVAL:-0.1}"
SIM_TRAIL_INTERVAL="${SIM_TRAIL_INTERVAL:-1.0}"
SIM_PATH_INTERVAL="${SIM_PATH_INTERVAL:-0}"
SIM_ASSIGNED_PATH_INTERVAL="${SIM_ASSIGNED_PATH_INTERVAL:-0}"
SIM_ASSIGNED_TRAIL_INTERVAL="${SIM_ASSIGNED_TRAIL_INTERVAL:-0.5}"
# Simulator motion: when using rolling trail windows, treat the last trail point as lookahead by default (smoother).
SIM_TRAIL_END_STOP="${SIM_TRAIL_END_STOP:-0}"
export SIM_TRAIL_END_STOP
# Do not stop-and-spin for tiny heading changes in the default simulation profile.
export SIM_TURN_STOP_ANGLE_DEG="${SIM_TURN_STOP_ANGLE_DEG:-30}"
export SIM_TURN_MIN_TIME_S="${SIM_TURN_MIN_TIME_S:-0.1}"
SIM_NO_VIS="${SIM_NO_VIS:-0}"
# Round-robin task binding removed — allocation algorithm handles assignment.

SIM_LOOP_TASKS="${SIM_LOOP_TASKS:-1}"
SIM_LOOP_MODE="${SIM_LOOP_MODE:-timer}"
if [[ -z "${SIM_LOOP_TASK_MIN_COUNT:-}" ]]; then
  SIM_LOOP_TASK_MIN_COUNT=$(( (SIM_AGV_COUNT + 3) / 4 ))
fi
if [[ -z "${SIM_LOOP_TASK_MAX_COUNT:-}" ]]; then
  SIM_LOOP_TASK_MAX_COUNT=$(( (SIM_AGV_COUNT + 1) / 2 ))
fi
if (( SIM_LOOP_TASK_MIN_COUNT < 1 )); then SIM_LOOP_TASK_MIN_COUNT=1; fi
if (( SIM_LOOP_TASK_MAX_COUNT < SIM_LOOP_TASK_MIN_COUNT )); then
  SIM_LOOP_TASK_MAX_COUNT="${SIM_LOOP_TASK_MIN_COUNT}"
fi
SIM_LOOP_TASK_MIN_DELAY="${SIM_LOOP_TASK_MIN_DELAY:-1.0}"
SIM_LOOP_TASK_MAX_DELAY="${SIM_LOOP_TASK_MAX_DELAY:-1.0}"

# Web UI bind address (override via env).
SIM_V2_VIS_HOST="${SIM_V2_VIS_HOST:-0.0.0.0}"
SIM_V2_VIS_PORT="${SIM_V2_VIS_PORT:-18080}"

echo "[run_all_v2] sim: agv=${SIM_AGV_COUNT} tasks=${SIM_TASK_COUNT} (mult=${SIM_TASK_MULTIPLIER}) loop_batch=[${SIM_LOOP_TASK_MIN_COUNT},${SIM_LOOP_TASK_MAX_COUNT}] loop_delay=[${SIM_LOOP_TASK_MIN_DELAY},${SIM_LOOP_TASK_MAX_DELAY}] trail_int=${SIM_TRAIL_INTERVAL} assigned_trail_int=${SIM_ASSIGNED_TRAIL_INTERVAL} vis=http://${SIM_V2_VIS_HOST}:${SIM_V2_VIS_PORT}/" >&2
VIS_URLS="$(resolve_vis_urls "${SIM_V2_VIS_HOST}" "${SIM_V2_VIS_PORT}")"
if [[ -n "${VIS_URLS}" ]]; then
  echo "[run_all_v2] web ui:" >&2
  while IFS= read -r line; do
    if [[ -n "${line}" ]]; then
      echo "[run_all_v2]  ${line}" >&2
    fi
  done <<< "${VIS_URLS}"
fi

# Optional build step (disabled by default). Set SIM_BUILD=1 to enable.
SIM_BUILD="${SIM_BUILD:-0}"
if is_true "${SIM_BUILD}"; then
  echo "[run_all_v2] SIM_BUILD=1: building external_receiver (-j9)..." >&2
  cmake -S "${ROOT_DIR}" -B "${ROOT_DIR}/build_local" >/dev/null
  cmake --build "${ROOT_DIR}/build_local" -j "${BUILD_JOBS:-9}"
else
  echo "[run_all_v2] SIM_BUILD=0: skip build (expect receiver already built)" >&2
fi

if ! [[ -x "${ROOT_DIR}/build/bin/external_receiver" || -x "${ROOT_DIR}/build_local/bin/external_receiver" ]]; then
  echo "[run_all_v2] error: external_receiver missing. Build manually or run with SIM_BUILD=1." >&2
  echo "[run_all_v2] hint: rm -rf build && cmake -S . -B build && cmake --build build -j9" >&2
  exit 2
fi

echo "[run_all_v2] starting external_receiver..." >&2
bash "${ROOT_DIR}/script/start.sh" &
RECEIVER_PID=$!

cleanup() {
  if [[ -n "${RECEIVER_PID:-}" ]]; then
    echo "[run_all_v2] stopping external_receiver (pid=${RECEIVER_PID})" >&2
    kill "${RECEIVER_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

sleep 1

VIS_FLAG=""
if is_true "${SIM_NO_VIS}"; then
  VIS_FLAG="--no-vis"
  echo "[run_all_v2] SIM_NO_VIS=1 (web visualization disabled)" >&2
fi

TASK_BIND_FLAG=""

LOOP_FLAG=""
if is_true "${SIM_LOOP_TASKS}"; then
  LOOP_FLAG="--loop-tasks --loop-mode ${SIM_LOOP_MODE} --loop-task-min-count ${SIM_LOOP_TASK_MIN_COUNT} --loop-task-max-count ${SIM_LOOP_TASK_MAX_COUNT} --loop-task-min-delay ${SIM_LOOP_TASK_MIN_DELAY} --loop-task-max-delay ${SIM_LOOP_TASK_MAX_DELAY}"
fi

cd "${SIM_ROOT}"

if ! "${PYTHON_BIN}" -c "import pika" >/dev/null 2>&1; then
  echo "[run_all_v2] installing python deps from requirements.txt..." >&2
  "${PYTHON_BIN}" -m pip install --user -r "${SIM_ROOT}/requirements.txt"
fi

SIM_RESTART_ON_EXIT="${SIM_RESTART_ON_EXIT:-0}"

run_sim() {
  local -a sim_args=(
    --bootstrap
    --run
    --map "${MAP_FILE}"
    --map-version "${SIM_MAP_VERSION}"
    --map-id "${SIM_MAP_ID}"
    --step-interval "${SIM_STEP_INTERVAL}"
    --path-interval "${SIM_PATH_INTERVAL}"
    --trail-interval "${SIM_TRAIL_INTERVAL}"
    --assigned-path-interval "${SIM_ASSIGNED_PATH_INTERVAL}"
    --assigned-trail-interval "${SIM_ASSIGNED_TRAIL_INTERVAL}"
    --trail-end-stop "${SIM_TRAIL_END_STOP}"
    --vis-host "${SIM_V2_VIS_HOST}"
    --vis-port "${SIM_V2_VIS_PORT}"
  )
  if is_true "${SIM_GENERATE}"; then
    sim_args+=(
      --generate
      --agv-count "${SIM_AGV_COUNT}"
      --task-count "${SIM_TASK_COUNT}"
      --seed "${SIM_SEED}"
    )
    if [[ -n "${TASK_BIND_FLAG}" ]]; then
      :  # reserved for future upstream task binding
    fi
  else
    if [[ -n "${SIM_STATUS_FILE}" ]]; then
      sim_args+=(--status "${SIM_STATUS_FILE}")
    fi
    if [[ -n "${SIM_TASK_FILE}" ]]; then
      sim_args+=(--tasks "${SIM_TASK_FILE}")
    fi
  fi
  if is_true "${SIM_LOOP_TASKS}"; then
    sim_args+=(
      --loop-tasks
      --loop-mode "${SIM_LOOP_MODE}"
      --loop-task-min-count "${SIM_LOOP_TASK_MIN_COUNT}"
      --loop-task-max-count "${SIM_LOOP_TASK_MAX_COUNT}"
      --loop-task-min-delay "${SIM_LOOP_TASK_MIN_DELAY}"
      --loop-task-max-delay "${SIM_LOOP_TASK_MAX_DELAY}"
    )
  fi
  if is_true "${SIM_NO_VIS}"; then
    sim_args+=(--no-vis)
  fi
  if is_true "${SIM_RECORD_REPLAY}"; then
    sim_args+=(--record-replay --record-dir "${SIM_RECORD_DIR}")
    if [[ -n "${SIM_RECORD_SESSION}" ]]; then
      sim_args+=(--record-session "${SIM_RECORD_SESSION}")
    fi
  fi

  if is_true "${SIM_LOG_ENABLE}"; then
    mkdir -p "${SIM_LOG_DIR}"
    "${PYTHON_BIN}" -u sim_runner_v2.py "${sim_args[@]}" 2>&1 | bash "${ROOT_DIR}/script/rotating_tee.sh" \
    --log-dir "${SIM_LOG_DIR}" \
    --prefix "${SIM_LOG_PREFIX}" \
    --rotate-sec "${SIM_LOG_ROTATE_SEC}" \
    --keep "${SIM_LOG_KEEP}" \
    --latest-symlink "${SIM_LOG_DIR}/${SIM_LOG_PREFIX}_latest.log"
  else
    "${PYTHON_BIN}" -u sim_runner_v2.py "${sim_args[@]}"
  fi
}

if is_true "${SIM_RESTART_ON_EXIT}"; then
  echo "[run_all_v2] SIM_RESTART_ON_EXIT=1 (auto-restart simulator on exit)" >&2
  while true; do
    run_sim || true
    echo "[run_all_v2] simulator exited; restarting in 1s (Ctrl-C to stop)..." >&2
    sleep 1
  done
else
  run_sim
fi
