#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
INPUT_DIR="${DEMO_INPUT_DIR:-${ROOT_DIR}/demo/inputs/generated_a4_presentation}"
STATUS_FILE="${DEMO_STATUS_FILE:-${INPUT_DIR}/status.json}"
TASK_FILE="${DEMO_TASK_FILE:-${INPUT_DIR}/tasks.json}"
SESSION_DIR="${DEMO_SESSION_DIR:-${ROOT_DIR}/demo/sessions}"
DEFAULT_SESSION_NAME="a4_presentation_demo"
SESSION_NAME="${DEMO_SESSION_NAME:-${DEFAULT_SESSION_NAME}}"
AUTO_STOP_IDLE_SEC="${DEMO_AUTO_STOP_IDLE_SEC:-12}"
AUTO_STOP_MIN_RUNTIME_SEC="${DEMO_AUTO_STOP_MIN_RUNTIME_SEC:-60}"
BUILD_BUNDLE="${DEMO_BUILD_BUNDLE:-1}"
MAX_SIM_TIME_SEC="${SIM_MAX_SIM_TIME_SEC:-180}"
TASK_LIMIT="${DEMO_TASK_LIMIT:-80}"
STATUS_COUNT="${DEMO_STATUS_COUNT:-30}"
PREPARE_ALWAYS="${DEMO_PREPARE_ALWAYS:-1}"

if [[ "${PREPARE_ALWAYS}" == "1" || ! -f "${STATUS_FILE}" || ! -f "${TASK_FILE}" ]]; then
  DEMO_TASK_LIMIT="${TASK_LIMIT}" \
  DEMO_STATUS_COUNT="${STATUS_COUNT}" \
  DEMO_INPUT_DIR="${INPUT_DIR}" \
  bash "${ROOT_DIR}/demo/prepare_real_inputs.sh"
fi

resolve_actual_session_dir() {
  local root_dir="$1"
  local session_name="$2"
  if [[ -d "${root_dir}/${session_name}" ]]; then
    printf '%s\n' "${root_dir}/${session_name}"
    return 0
  fi
  find "${root_dir}" -mindepth 1 -maxdepth 1 -type d -name "${session_name}*" -printf '%T@ %p\n' \
    | sort -nr \
    | awk 'NR==1 {sub(/^[^ ]+ /, ""); print}'
}

mkdir -p "${SESSION_DIR}"
rm -rf "${SESSION_DIR:?}/${SESSION_NAME}"
export SIM_STATUS_FILE="${STATUS_FILE}"
export SIM_TASK_FILE="${TASK_FILE}"
export SIM_GENERATE="0"
export SIM_LOOP_TASKS="${SIM_LOOP_TASKS:-0}"
export SIM_BIND_TASKS="${SIM_BIND_TASKS:-0}"
export SIM_RECORD_REPLAY="1"
export SIM_RECORD_DIR="${SESSION_DIR}"
export SIM_RECORD_SESSION="${SESSION_NAME}"
export SIM_NO_VIS="${SIM_NO_VIS:-1}"
export SIM_AUTO_STOP_IDLE_SEC="${SIM_AUTO_STOP_IDLE_SEC:-${AUTO_STOP_IDLE_SEC}}"
export SIM_AUTO_STOP_MIN_RUNTIME_SEC="${SIM_AUTO_STOP_MIN_RUNTIME_SEC:-${AUTO_STOP_MIN_RUNTIME_SEC}}"
export SIM_MAX_SIM_TIME_SEC="${MAX_SIM_TIME_SEC}"
export SIM_ASSIGNED_PATH_INTERVAL="${SIM_ASSIGNED_PATH_INTERVAL:-1.0}"

echo "[demo] recording session=${SESSION_NAME} idle_stop=${SIM_AUTO_STOP_IDLE_SEC}s min_runtime=${SIM_AUTO_STOP_MIN_RUNTIME_SEC}s" >&2
bash "${ROOT_DIR}/simulation_v2/run_all.sh"

ACTUAL_SESSION_DIR="$(resolve_actual_session_dir "${SESSION_DIR}" "${SESSION_NAME}")"
if [[ -z "${ACTUAL_SESSION_DIR}" || ! -d "${ACTUAL_SESSION_DIR}" ]]; then
  echo "[demo] recorded session directory not found under ${SESSION_DIR}" >&2
  exit 2
fi

if [[ "${BUILD_BUNDLE}" == "1" ]]; then
  "${PYTHON_BIN}" "${ROOT_DIR}/simulation_v2/replay_tool.py" build --session "${ACTUAL_SESSION_DIR}" >/dev/null
fi

echo "[demo] session ready: ${ACTUAL_SESSION_DIR}" >&2
if [[ -f "${ACTUAL_SESSION_DIR}/leader_demo_bundle.json" ]]; then
  echo "[demo] bundle ready: ${ACTUAL_SESSION_DIR}/leader_demo_bundle.json" >&2
fi
