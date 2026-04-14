#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
INPUT_DIR="${DEMO_INPUT_DIR:-${ROOT_DIR}/demo/inputs/generated_director_demo}"
STATUS_FILE="${DEMO_STATUS_FILE:-${INPUT_DIR}/status.json}"
TASK_FILE="${DEMO_TASK_FILE:-${INPUT_DIR}/tasks.json}"
SESSION_DIR="${DEMO_SESSION_DIR:-${ROOT_DIR}/demo/sessions}"
SESSION_NAME="${DEMO_SESSION_NAME:-director_demo}"
MAX_SIM_TIME_SEC="${SIM_MAX_SIM_TIME_SEC:-120}"
BUILD_BUNDLE="${DEMO_BUILD_BUNDLE:-1}"

"${PYTHON_BIN}" "${ROOT_DIR}/demo/build_director_demo_inputs.py" --out-dir "${INPUT_DIR}"

mkdir -p "${SESSION_DIR}"
rm -rf "${SESSION_DIR:?}/${SESSION_NAME}"

export SIM_STATUS_FILE="${STATUS_FILE}"
export SIM_TASK_FILE="${TASK_FILE}"
export SIM_GENERATE="0"
export SIM_RECORD_REPLAY="1"
export SIM_RECORD_DIR="${SESSION_DIR}"
export SIM_RECORD_SESSION="${SESSION_NAME}"
export SIM_NO_VIS="${SIM_NO_VIS:-1}"
export SIM_MAX_SIM_TIME_SEC="${MAX_SIM_TIME_SEC}"
export SIM_LOOP_TASKS="${SIM_LOOP_TASKS:-1}"
export SIM_LOOP_MODE="${SIM_LOOP_MODE:-timer}"
export SIM_LOOP_TASK_MIN_COUNT="${SIM_LOOP_TASK_MIN_COUNT:-2}"
export SIM_LOOP_TASK_MAX_COUNT="${SIM_LOOP_TASK_MAX_COUNT:-3}"
export SIM_LOOP_TASK_MIN_DELAY="${SIM_LOOP_TASK_MIN_DELAY:-12}"
export SIM_LOOP_TASK_MAX_DELAY="${SIM_LOOP_TASK_MAX_DELAY:-15}"
export SIM_ASSIGNED_PATH_INTERVAL="${SIM_ASSIGNED_PATH_INTERVAL:-1.0}"
export SIM_ASSIGNED_TRAIL_INTERVAL="${SIM_ASSIGNED_TRAIL_INTERVAL:-0.5}"
export ALLOC_TOPK_RATIO="${ALLOC_TOPK_RATIO:-1.0}"
export ALLOC_TOPK_COVERAGE_AMRS="${ALLOC_TOPK_COVERAGE_AMRS:-3}"
export SIM_FILTER_OCCUPIED_TASKS="${SIM_FILTER_OCCUPIED_TASKS:-0}"

echo "[demo] recording deterministic director session=${SESSION_NAME}" >&2
bash "${ROOT_DIR}/simulation_v2/run_all.sh"

ACTUAL_SESSION_DIR="${SESSION_DIR}/${SESSION_NAME}"
if [[ ! -d "${ACTUAL_SESSION_DIR}" ]]; then
  echo "[demo] recorded session directory not found: ${ACTUAL_SESSION_DIR}" >&2
  exit 2
fi

if [[ "${BUILD_BUNDLE}" == "1" ]]; then
  "${PYTHON_BIN}" "${ROOT_DIR}/simulation_v2/replay_tool.py" build --session "${ACTUAL_SESSION_DIR}" >/dev/null
fi

echo "[demo] session ready: ${ACTUAL_SESSION_DIR}" >&2
if [[ -f "${ACTUAL_SESSION_DIR}/leader_demo_bundle.json" ]]; then
  echo "[demo] bundle ready: ${ACTUAL_SESSION_DIR}/leader_demo_bundle.json" >&2
fi
