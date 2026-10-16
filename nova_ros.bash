#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_CONFIG="${REPO_ROOT}/inference/config/robot.yaml"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
STEERNAV_ROOT="${STEERNAV_ROOT:-/home/jim/Projects/steernav}"
MODE="normal"
DETACH=false
SESSION_NAME=""

if [[ $# -gt 0 && ( "$1" == "normal" || "$1" == "steered" ) ]]; then
  MODE="$1"
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --detach) DETACH=true; shift ;;
    --session-name)
      [[ $# -ge 2 ]] || { echo "--session-name requires a value" >&2; exit 2; }
      SESSION_NAME="$2"
      shift 2
      ;;
    -h|--help)
      echo "Usage: $0 [normal|steered] [--detach] [--session-name NAME]"
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [[ -n "${SESSION_NAME}" && ! "${SESSION_NAME}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "Session name may contain only letters, numbers, dots, underscores, and hyphens." >&2
  exit 2
fi
[[ -f "${ROS_SETUP}" ]] || { echo "ROS setup not found: ${ROS_SETUP}" >&2; exit 1; }
[[ -f "${ROBOT_CONFIG}" ]] || { echo "Robot config not found: ${ROBOT_CONFIG}" >&2; exit 1; }
command -v tmux >/dev/null 2>&1 || { echo "tmux is not installed." >&2; exit 1; }
if [[ "${MODE}" == "steered" && ! -f "${STEERNAV_ROOT}/steernav/ros_inference.py" ]]; then
  echo "Set STEERNAV_ROOT to a SteerNav checkout." >&2
  exit 1
fi

session_name="${SESSION_NAME:-omnivla_nova_$(date +%s)_$$}"
if tmux has-session -t "${session_name}" 2>/dev/null; then
  echo "tmux session already exists: ${session_name}" >&2
  exit 1
fi

tmux new-session -d -s "${session_name}" -c "${REPO_ROOT}"
inference_pane="$(tmux display-message -p -t "${session_name}:0.0" '#{pane_id}')"
controller_pane="$(tmux split-window -h -P -F '#{pane_id}' -t "${inference_pane}" -c "${REPO_ROOT}")"
tmux set-option -t "${session_name}" mouse on
tmux select-pane -t "${inference_pane}" -T "OmniVLA inference"
tmux select-pane -t "${controller_pane}" -T "Waypoint controller"

tmux send-keys -t "${inference_pane}" \
  "source ${ROS_SETUP} && export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && uv run python inference/omnivla_ros.py --robot sim_nova --config ${ROBOT_CONFIG}" C-m

controller_flags="--simulation-episode-control"
if [[ "${MODE}" == "steered" ]]; then
  controller_flags="--steer ${controller_flags}"
  steernav_pane="$(tmux split-window -v -P -F '#{pane_id}' -t "${controller_pane}" -c "${STEERNAV_ROOT}")"
  tmux select-pane -t "${steernav_pane}" -T "SteerNav"
  tmux send-keys -t "${steernav_pane}" \
    "source ${ROS_SETUP} && export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && uv run steernav/ros_inference.py --robot sim_nova" C-m
  tmux select-layout -t "${session_name}:0" tiled
fi

tmux send-keys -t "${controller_pane}" \
  "source ${ROS_SETUP} && export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && uv run python custom_utils/custom_utils/kinematic_controller_naive.py --robot sim_nova --config ${ROBOT_CONFIG} --dt 1.5 ${controller_flags}" C-m

tmux select-pane -t "${inference_pane}"
echo "TMUX_SESSION=${session_name}"
if [[ "${DETACH}" == true ]]; then
  exit 0
elif [[ -n "${TMUX:-}" ]]; then
  tmux switch-client -t "${session_name}"
else
  tmux attach-session -t "${session_name}"
fi
