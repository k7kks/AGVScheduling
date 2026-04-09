#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)

if [[ -f "${ROOT_DIR}/config/network.env" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/config/network.env"
else
  echo "[run_two_agv] missing ${ROOT_DIR}/config/network.env" >&2
  exit 2
fi

export MAP_FILE="${MAP_FILE:-${ROOT_DIR}/config/south_20260107.json}"

# Isolate RabbitMQ exchanges/queues per run to avoid cross-talk with other sims.
is_true() {
  local v="${1:-}"
  shopt -s nocasematch
  case "${v}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

SIM_USE_SHARED_MQ="${SIM_USE_SHARED_MQ:-0}"
if ! is_true "${SIM_USE_SHARED_MQ}"; then
  SIM_SESSION="${SIM_SESSION:-$(date +%s)}"
  export EXT_EXCHANGE="DispToAlgoExchange_sim_${SIM_SESSION}"
  export EXT_QUEUE="DispToAlgoQueue_sim_${SIM_SESSION}"
  export ASSIGN_RESULT_EXCHANGE="AlgoToDispExchange_sim_${SIM_SESSION}"
  export ASSIGN_RESULT_QUEUE="AlgoToDispQueue_sim_${SIM_SESSION}"
  export ALGO_PUBLISH_EXCHANGE="${ASSIGN_RESULT_EXCHANGE}"
  export ALGO_PUBLISH_QUEUE="${ASSIGN_RESULT_QUEUE}"
fi

SIM_STEP_INTERVAL="${SIM_STEP_INTERVAL:-0.1}"
SIM_EDGE_DURATION="${SIM_EDGE_DURATION:-0.6}"
SIM_DRIVE_MODE="${SIM_DRIVE_MODE:-trail}"
SIM_LOOP_TASKS="${SIM_LOOP_TASKS:-1}"
SIM_LOOP_MODE="${SIM_LOOP_MODE:-any-idle}"
SIM_LOOP_TASK_MIN_DELAY="${SIM_LOOP_TASK_MIN_DELAY:-1.0}"
SIM_LOOP_TASK_MAX_DELAY="${SIM_LOOP_TASK_MAX_DELAY:-2.0}"
SIM_LOOP_TASK_MIN_COUNT="${SIM_LOOP_TASK_MIN_COUNT:-2}"
SIM_LOOP_TASK_MAX_COUNT="${SIM_LOOP_TASK_MAX_COUNT:-2}"
# Subtask dwell durations (ms) to test static-release behavior.
export SIM_SUBTASK_PICKUP_DURATION_MS="${SIM_SUBTASK_PICKUP_DURATION_MS:-6000}"
export SIM_SUBTASK_DROPOFF_DURATION_MS="${SIM_SUBTASK_DROPOFF_DURATION_MS:-6000}"
export SIM_SUBTASK_WAYPOINT_DURATION_MS="${SIM_SUBTASK_WAYPOINT_DURATION_MS:-6000}"

loop_args=()
if is_true "${SIM_LOOP_TASKS}"; then
  loop_args+=(
    --loop-tasks
    --loop-mode "${SIM_LOOP_MODE}"
    --loop-task-min-delay "${SIM_LOOP_TASK_MIN_DELAY}"
    --loop-task-max-delay "${SIM_LOOP_TASK_MAX_DELAY}"
    --loop-task-min-count "${SIM_LOOP_TASK_MIN_COUNT}"
    --loop-task-max-count "${SIM_LOOP_TASK_MAX_COUNT}"
  )
fi

cd "${ROOT_DIR}"

bash "${ROOT_DIR}/script/start.sh" &
RECEIVER_PID=$!

cleanup() {
  if [[ -n "${RECEIVER_PID:-}" ]]; then
    kill "${RECEIVER_PID}" >/dev/null 2>&1 || true
    wait "${RECEIVER_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

python3 "${SCRIPT_DIR}/sim_runner.py" \
  --generate --agv-count 2 --task-count 2 --bind-tasks \
  --bootstrap --run \
  --drive-mode "${SIM_DRIVE_MODE}" \
  --step-interval "${SIM_STEP_INTERVAL}" --edge-duration "${SIM_EDGE_DURATION}" \
  --trail-interval 0.5 --assigned-trail-interval 0.2 \
  "${loop_args[@]}"
