#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
SOURCE_DIR="${DEMO_SOURCE_DIR:-${ROOT_DIR}/demo/source_data/A4产线任务单数据-近1月}"
OUT_DIR="${DEMO_INPUT_DIR:-${ROOT_DIR}/demo/inputs/generated_a4_real}"
TASK_LIMIT="${DEMO_TASK_LIMIT:-80}"
STATUS_COUNT="${DEMO_STATUS_COUNT:-30}"
DEVICE_PREFIX="${DEMO_DEVICE_PREFIX:-AGV}"
SEED="${DEMO_SEED:-7}"
MAX_SUBTASKS="${DEMO_MAX_SUBTASKS:-3}"

ROUND_ROBIN="${DEMO_ROUND_ROBIN_BIND:-1}"

EXTRA_ARGS=()
if [[ "${ROUND_ROBIN}" == "1" ]]; then
  EXTRA_ARGS+=(--round-robin-bind)
fi

exec "${PYTHON_BIN}" "${ROOT_DIR}/simulation_v2/build_a4_replay_inputs.py" \
  --source-dir "${SOURCE_DIR}" \
  --out-dir "${OUT_DIR}" \
  --task-limit "${TASK_LIMIT}" \
  --status-count "${STATUS_COUNT}" \
  --device-prefix "${DEVICE_PREFIX}" \
  --seed "${SEED}" \
  --max-subtasks "${MAX_SUBTASKS}" \
  --no-bind-history \
  "${EXTRA_ARGS[@]}"
