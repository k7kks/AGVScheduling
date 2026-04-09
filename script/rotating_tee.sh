#!/usr/bin/env bash
set -u

LOG_DIR=""
PREFIX=""
FIRST_FILE=""
ROTATE_SEC="300"
KEEP="30"
LATEST_SYMLINK=""
PRUNE_JSON=0
JSON_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --log-dir)
      LOG_DIR="$2"
      shift 2
      ;;
    --prefix)
      PREFIX="$2"
      shift 2
      ;;
    --first-file)
      FIRST_FILE="$2"
      shift 2
      ;;
    --rotate-sec)
      ROTATE_SEC="$2"
      shift 2
      ;;
    --keep)
      KEEP="$2"
      shift 2
      ;;
    --latest-symlink)
      LATEST_SYMLINK="$2"
      shift 2
      ;;
    --prune-json)
      PRUNE_JSON=1
      shift
      ;;
    --json-dir)
      JSON_DIR="$2"
      shift 2
      ;;
    *)
      echo "[rotating_tee] unknown arg: $1" >&2
      exit 2
      ;;
  esac
done

if [[ -z "${LOG_DIR}" || -z "${PREFIX}" ]]; then
  echo "[rotating_tee] --log-dir and --prefix are required" >&2
  exit 2
fi

if [[ "${ROTATE_SEC}" =~ ^[0-9]+$ ]]; then
  rotate_sec="${ROTATE_SEC}"
else
  rotate_sec=0
fi

if [[ "${KEEP}" =~ ^[0-9]+$ ]]; then
  keep="${KEEP}"
else
  keep=0
fi

prefix="${PREFIX:-external_receiver}"
stdout_enabled=1
if [[ -n "${LOG_STDOUT:-}" ]]; then
  case "${LOG_STDOUT}" in
    0|false|no|off) stdout_enabled=0 ;;
  esac
elif [[ -n "${RECEIVER_LOG_STDOUT:-}" ]]; then
  case "${RECEIVER_LOG_STDOUT}" in
    0|false|no|off) stdout_enabled=0 ;;
  esac
fi

if ! mkdir -p "${LOG_DIR}" 2>/dev/null; then
  echo "[rotating_tee] failed to create log dir: ${LOG_DIR}" >&2
  exit 2
fi

log_dir="$(cd "${LOG_DIR}" && pwd)"
if [[ -n "${JSON_DIR}" ]]; then
  if [[ "${JSON_DIR}" != /* ]]; then
    json_dir="$(pwd)/${JSON_DIR}"
  else
    json_dir="${JSON_DIR}"
  fi
else
  json_dir="${log_dir}"
fi
latest_path=""
if [[ -n "${LATEST_SYMLINK}" ]]; then
  if [[ "${LATEST_SYMLINK}" != /* ]]; then
    latest_path="$(pwd)/${LATEST_SYMLINK}"
  else
    latest_path="${LATEST_SYMLINK}"
  fi
  if [[ "$(dirname "${latest_path}")" != "${log_dir}" ]]; then
    latest_path=""
  fi
fi

extract_log_ts() {
  local base="${1##*/}"
  local ts="${base#${prefix}_}"
  ts="${ts%.log}"
  if [[ "${ts}" =~ ^[0-9]{8}_[0-9]{6}$ ]]; then
    echo "${ts}"
    return 0
  fi
  return 1
}

normalize_json_ts() {
  local name="$1"
  if [[ "${name}" == "allocation_result_latest.json" ]]; then
    return 1
  fi
  if [[ "${name}" == allocation_result_*.json ]]; then
    local ts="${name#allocation_result_}"
    ts="${ts%.json}"
    if [[ "${ts}" =~ ^[0-9]{8}_[0-9]{6}$ ]]; then
      echo "${ts}"
      return 0
    fi
  fi
  if [[ "${name}" == traffic_path_*.json ]]; then
    local token="${name#traffic_path_}"
    token="${token%.json}"
    token="${token%_*}"
    if [[ "${token}" =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}-[0-9]{6}$ ]]; then
      local cleaned="${token//-/}"
      local ts="${cleaned:0:8}_${cleaned:8:6}"
      echo "${ts}"
      return 0
    fi
  fi
  return 1
}

prune_json_before_ts() {
  local cutoff="$1"
  if [[ -z "${cutoff}" ]]; then
    return 0
  fi
  if [[ ! -d "${json_dir}" ]]; then
    return 0
  fi
  shopt -s nullglob
  local file base ts
  for file in "${json_dir}"/*.json; do
    base="${file##*/}"
    ts="$(normalize_json_ts "${base}")" || continue
    if [[ "${ts}" < "${cutoff}" ]]; then
      rm -f -- "${file}"
    fi
  done
  shopt -u nullglob
}

prune_logs() {
  if (( keep <= 0 )); then
    return 0
  fi
  shopt -s nullglob
  local files=( "${log_dir}"/"${prefix}"_*.log )
  shopt -u nullglob
  if [[ -n "${latest_path}" ]]; then
    local filtered=()
    local f
    for f in "${files[@]}"; do
      if [[ "${f}" == "${latest_path}" ]]; then
        continue
      fi
      filtered+=( "${f}" )
    done
    files=( "${filtered[@]}" )
  fi
  if (( ${#files[@]} == 0 )); then
    return 0
  fi
  local sorted
  IFS=$'\n' sorted=( $(printf '%s\n' "${files[@]}" | sort -r) )
  local kept_logs=()
  if (( ${#sorted[@]} > 0 )); then
    if (( ${#sorted[@]} > keep )); then
      kept_logs=( "${sorted[@]:0:keep}" )
    else
      kept_logs=( "${sorted[@]}" )
    fi
  fi
  if (( ${#sorted[@]} > keep )); then
  local i
  for (( i=keep; i<${#sorted[@]}; i++ )); do
    rm -f -- "${sorted[$i]}"
  done
  fi
  if (( PRUNE_JSON )); then
    local cutoff_idx cutoff_ts
    cutoff_idx=$(( keep - 1 ))
    if (( cutoff_idx < 0 )); then
      cutoff_idx=0
    fi
    if (( cutoff_idx >= ${#sorted[@]} )); then
      cutoff_idx=$(( ${#sorted[@]} - 1 ))
    fi
    cutoff_ts="$(extract_log_ts "${sorted[$cutoff_idx]}")" || cutoff_ts=""
    prune_json_before_ts "${cutoff_ts}"
  fi
}

update_symlink() {
  local link_path="$1"
  local target_file="$2"
  if [[ -z "${link_path}" ]]; then
    return 0
  fi
  mkdir -p "$(dirname "${link_path}")" 2>/dev/null || return 0
  local target="${target_file}"
  if [[ "$(dirname "${link_path}")" == "${log_dir}" ]]; then
    target="$(basename "${target_file}")"
  fi
  ln -sfn "${target}" "${link_path}" 2>/dev/null && return 0
  local tmp="${link_path}.tmp.$$"
  rm -f -- "${tmp}"
  if ln -s "${target}" "${tmp}" 2>/dev/null; then
    mv -f "${tmp}" "${link_path}" 2>/dev/null || rm -f -- "${tmp}"
  fi
}

make_log_path() {
  local ts
  ts="$(date +%Y%m%d_%H%M%S)"
  echo "${log_dir}/${prefix}_${ts}.log"
}

current_path=""
if [[ -n "${FIRST_FILE}" ]]; then
  if [[ "${FIRST_FILE}" != /* ]]; then
    current_path="$(pwd)/${FIRST_FILE}"
  else
    current_path="${FIRST_FILE}"
  fi
  if [[ "$(dirname "${current_path}")" != "${log_dir}" ]]; then
    current_path="$(make_log_path)"
  fi
else
  current_path="$(make_log_path)"
fi

if ! exec 3>>"${current_path}"; then
  echo "[rotating_tee] failed to open log file: ${current_path}" >&2
  exit 2
fi

update_symlink "${LATEST_SYMLINK}" "${current_path}"
prune_logs

stop=0
trap 'stop=1' INT TERM

if (( rotate_sec > 0 )); then
  next_rotate=$(( $(date +%s) + rotate_sec ))
else
  next_rotate=0
fi

line=""
while IFS= read -r line || [[ -n "${line}" ]]; do
  if (( rotate_sec > 0 )); then
    now="$(date +%s)"
    if (( now >= next_rotate )); then
      exec 3>&- || true
      current_path="$(make_log_path)"
      if ! exec 3>>"${current_path}"; then
        echo "[rotating_tee] failed to open log file: ${current_path}" >&2
        exit 2
      fi
      update_symlink "${LATEST_SYMLINK}" "${current_path}"
      prune_logs
      next_rotate=$(( now + rotate_sec ))
    fi
  fi
  printf '%s\n' "${line}" >&3 || true
  if (( stdout_enabled )); then
    printf '%s\n' "${line}" || true
  fi
  if (( stop )); then
    break
  fi
done

exec 3>&- || true
exit 0
