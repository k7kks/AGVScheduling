#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
INPUT_DIR="${DEMO_INPUT_DIR:-${ROOT_DIR}/demo/inputs/generated_a4_real}"
STATUS_FILE="${DEMO_STATUS_FILE:-${INPUT_DIR}/status.json}"
TASK_FILE="${DEMO_TASK_FILE:-${INPUT_DIR}/tasks.json}"
SESSION_NAME="${DEMO_SESSION_NAME:-a4_real_30agv_$(date +%Y%m%d_%H%M%S)}"
SESSION_DIR="${DEMO_SESSION_DIR:-${ROOT_DIR}/demo/sessions}"

if [[ ! -f "${STATUS_FILE}" || ! -f "${TASK_FILE}" ]]; then
  bash "${ROOT_DIR}/demo/prepare_real_inputs.sh"
fi

mkdir -p "${SESSION_DIR}"
export SIM_STATUS_FILE="${STATUS_FILE}"
export SIM_TASK_FILE="${TASK_FILE}"
export SIM_GENERATE="0"
export SIM_LOOP_TASKS="${SIM_LOOP_TASKS:-0}"
export SIM_BIND_TASKS="${SIM_BIND_TASKS:-0}"
export SIM_RECORD_REPLAY="1"
export SIM_RECORD_DIR="${SESSION_DIR}"
export SIM_RECORD_SESSION="${SESSION_NAME}"
export SIM_NO_VIS="${SIM_NO_VIS:-1}"

exec bash "${ROOT_DIR}/simulation_v2/run_all.sh"
