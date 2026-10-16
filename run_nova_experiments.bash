#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
REMOTE_HOST="${REMOTE_HOST:-isaac}"
REMOTE_REPO="${REMOTE_REPO:-/home/isaac/Projects/TIC-VLA}"
REMOTE_DISPLAY="${REMOTE_DISPLAY:-:1}"
CONFIG="${1:-DynaNav/configs/icra_scenarios_v4/scenario1_5.yaml}"
READY_TIMEOUT_SECONDS="${READY_TIMEOUT_SECONDS:-600}"
BATCH_ID="${BATCH_ID:-$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="${REPO_ROOT}/logs/orchestration/${BATCH_ID}"

[[ "${CONFIG}" =~ ^[A-Za-z0-9_./-]+\.yaml$ ]] || { echo "Invalid relative YAML path." >&2; exit 2; }
[[ "${REMOTE_REPO}" =~ ^/[A-Za-z0-9_./-]+$ ]] || { echo "Invalid REMOTE_REPO." >&2; exit 2; }
[[ "${REMOTE_DISPLAY}" =~ ^:[0-9]+(\.[0-9]+)?$ ]] || { echo "Invalid REMOTE_DISPLAY." >&2; exit 2; }
[[ "${BATCH_ID}" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "Invalid BATCH_ID." >&2; exit 2; }
[[ "${READY_TIMEOUT_SECONDS}" =~ ^[0-9]+$ && "${READY_TIMEOUT_SECONDS}" -gt 0 ]] || {
  echo "READY_TIMEOUT_SECONDS must be a positive integer." >&2
  exit 2
}
[[ -f "${ROS_SETUP}" ]] || { echo "ROS setup not found: ${ROS_SETUP}" >&2; exit 1; }
command -v tmux >/dev/null 2>&1 || { echo "tmux is not installed." >&2; exit 1; }

display_number="${REMOTE_DISPLAY#:}"
display_number="${display_number%%.*}"
mkdir -p "${LOG_DIR}"
active_session=""

cleanup_session() {
  if [[ -n "${active_session}" ]] && tmux has-session -t "${active_session}" 2>/dev/null; then
    tmux kill-session -t "${active_session}"
  fi
  active_session=""
}
trap cleanup_session EXIT
trap 'exit 130' INT TERM

wait_until_ready() {
  local mode="$1" session="$2" deadline=$((SECONDS + READY_TIMEOUT_SECONDS)) pane_output pane_id
  echo "Waiting up to ${READY_TIMEOUT_SECONDS}s for local ${mode} nodes..."
  while (( SECONDS < deadline )); do
    tmux has-session -t "${session}" 2>/dev/null || return 1
    pane_output=""
    while IFS= read -r pane_id; do
      pane_output+="$(tmux capture-pane -p -S -200 -t "${pane_id}" 2>/dev/null || true)"
      pane_output+=$'\n'
    done < <(tmux list-panes -t "${session}:0" -F '#{pane_id}')
    if [[ "${pane_output}" == *"Model ready."* && "${pane_output}" == *"Registered with master node."* ]]; then
      if [[ "${mode}" == "normal" || "${pane_output}" == *"Waiting for simulation episodes"* ]]; then
        echo "Local ${mode} pipeline is ready."
        return 0
      fi
    fi
    sleep 2
  done
  while IFS= read -r pane_id; do
    tmux capture-pane -p -S -100 -t "${pane_id}" >&2 || true
  done < <(tmux list-panes -t "${session}:0" -F '#{pane_id}')
  return 1
}

run_mode() {
  local mode="$1" run_id="${BATCH_ID}_$1" session="omnivla_${BATCH_ID}_$1"
  local status_file="${LOG_DIR}/.${mode}.$$.status" remote_command remote_pane watcher_pid remote_status
  active_session="${session}"
  "${REPO_ROOT}/nova_ros.bash" "${mode}" --detach --session-name "${session}"
  wait_until_ready "${mode}" "${session}"

  remote_command="set -o pipefail; ssh -o BatchMode=yes -o ConnectTimeout=10 ${REMOTE_HOST} 'cd ${REMOTE_REPO} && export DISPLAY=${REMOTE_DISPLAY} && export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-} && DynaNav/run_benchmark.sh ${CONFIG} --run_id ${run_id}' 2>&1 | tee ${LOG_DIR}/${mode}.log; status=\${PIPESTATUS[0]}; echo \${status} > ${status_file}; exit \${status}"
  remote_pane="$(tmux split-window -v -P -F '#{pane_id}' -t "${session}:0" -c "${REPO_ROOT}" /bin/bash -lc "${remote_command}")"
  tmux select-pane -t "${remote_pane}" -T "Isaac ${mode} status"
  tmux select-layout -t "${session}:0" tiled

  (
    while tmux has-session -t "${session}" 2>/dev/null && [[ ! -s "${status_file}" ]]; do sleep 1; done
    if tmux has-session -t "${session}" 2>/dev/null; then sleep 3; tmux kill-session -t "${session}"; fi
  ) &
  watcher_pid=$!

  if [[ -n "${TMUX:-}" ]]; then
    echo "Run this script outside an existing tmux client." >&2
    tmux kill-session -t "${session}"
    wait "${watcher_pid}" || true
    return 1
  fi
  tmux attach-session -t "${session}" || true
  wait "${watcher_pid}" || true
  active_session=""
  [[ -s "${status_file}" ]] || { echo "Remote benchmark did not report status." >&2; return 1; }
  remote_status="$(<"${status_file}")"
  [[ "${remote_status}" == "0" ]] || return "${remote_status}"
  sleep 5
}

ssh -o BatchMode=yes -o ConnectTimeout=10 "${REMOTE_HOST}" \
  "cd ${REMOTE_REPO} && test -S /tmp/.X11-unix/X${display_number} && test -f ${CONFIG} && test -x DynaNav/run_benchmark.sh && export DISPLAY=${REMOTE_DISPLAY} && export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-} && bash -u -c 'source .env.testing'"

echo "Batch ID: ${BATCH_ID}"
echo "Scenario: ${CONFIG}"
echo "Remote simulator: ${REMOTE_HOST}:${REMOTE_REPO}"
run_mode normal
run_mode steered
echo "Both OmniVLA experiments completed. Logs: ${LOG_DIR}"
