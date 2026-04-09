#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
SESSION_ROOT="${DEMO_SESSION_DIR:-${ROOT_DIR}/demo/sessions}"

resolve_session() {
  local arg="${1:-}"
  if [[ -n "${arg}" ]]; then
    if [[ -d "${arg}" ]]; then
      printf '%s\n' "${arg}"
      return 0
    fi
    if [[ -d "${SESSION_ROOT}/${arg}" ]]; then
      printf '%s\n' "${SESSION_ROOT}/${arg}"
      return 0
    fi
    return 1
  fi
  find "${SESSION_ROOT}" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' | sort -nr | awk 'NR==1 {print $2}'
}

SESSION_DIR="$(resolve_session "${1:-}")"
if [[ -z "${SESSION_DIR}" || ! -d "${SESSION_DIR}" ]]; then
  echo "[demo] replay session not found under ${SESSION_ROOT}" >&2
  exit 2
fi

exec "${PYTHON_BIN}" "${ROOT_DIR}/simulation_v2/replay_tool.py" serve --session "${SESSION_DIR}" --rebuild
