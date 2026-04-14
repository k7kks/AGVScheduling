#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
SESSION_ROOT="${DEMO_SESSION_DIR:-${ROOT_DIR}/demo/sessions}"
DEFAULT_PORT="${SIM_REPLAY_PORT:-18181}"
HOST="${SIM_REPLAY_HOST:-127.0.0.1}"

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

port_in_use() {
  local port="$1"
  ss -ltn "( sport = :${port} )" 2>/dev/null | grep -q "[.:]${port}[[:space:]]"
}

resolve_port() {
  local start_port="$1"
  local port="$start_port"
  local attempts=0
  while [[ "${attempts}" -lt 50 ]]; do
    if ! port_in_use "${port}"; then
      printf '%s\n' "${port}"
      return 0
    fi
    port="$((port + 1))"
    attempts="$((attempts + 1))"
  done
  return 1
}

SESSION_DIR="$(resolve_session "${1:-}")"
if [[ -z "${SESSION_DIR}" || ! -d "${SESSION_DIR}" ]]; then
  echo "[demo] replay session not found under ${SESSION_ROOT}" >&2
  exit 2
fi

PORT="$(resolve_port "${DEFAULT_PORT}")"
if [[ -z "${PORT}" ]]; then
  echo "[demo] no free replay port found near ${DEFAULT_PORT}" >&2
  exit 3
fi
if [[ "${PORT}" != "${DEFAULT_PORT}" ]]; then
  echo "[demo] port ${DEFAULT_PORT} is busy, using ${PORT}" >&2
fi
echo "[demo] serving replay session=${SESSION_DIR} at http://${HOST}:${PORT}/" >&2

exec "${PYTHON_BIN}" "${ROOT_DIR}/simulation_v2/replay_tool.py" serve --session "${SESSION_DIR}" --rebuild --host "${HOST}" --port "${PORT}"
