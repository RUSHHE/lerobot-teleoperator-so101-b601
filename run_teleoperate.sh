#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Load optional machine-local overrides (serial numbers, ports, mapping path) if present.
if [[ -f "${SCRIPT_DIR}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/.env"
  set +a
fi

if [[ -z "${LEROBOT_BIN:-}" ]]; then
  if command -v lerobot-teleoperate >/dev/null 2>&1; then
    LEROBOT_BIN="$(command -v lerobot-teleoperate)"
  elif [[ -n "${CONDA_PREFIX:-}" && -x "${CONDA_PREFIX}/bin/lerobot-teleoperate" ]]; then
    LEROBOT_BIN="${CONDA_PREFIX}/bin/lerobot-teleoperate"
  elif [[ -x "${HOME}/miniconda3/envs/lerobot/bin/lerobot-teleoperate" ]]; then
    LEROBOT_BIN="${HOME}/miniconda3/envs/lerobot/bin/lerobot-teleoperate"
  else
    echo "lerobot-teleoperate not found; activate your lerobot environment or set LEROBOT_BIN." >&2
    exit 1
  fi
fi

LEROBOT_BIN_DIR="$(dirname "${LEROBOT_BIN}")"
if [[ -d "${LEROBOT_BIN_DIR}" && ":${PATH}:" != *":${LEROBOT_BIN_DIR}:"* ]]; then
  export PATH="${LEROBOT_BIN_DIR}:${PATH}"
fi

CAN_PORT="${CAN_PORT:-can0}"
LEADER_PORT="${LEADER_PORT:-/dev/ttyACM0}"
LEADER_ID="${LEADER_ID:-so101_leader}"
FOLLOWER_ID="${FOLLOWER_ID:-so101_b601_follower}"
MAPPING_CONFIG="${MAPPING_CONFIG:-${SCRIPT_DIR}/configs/so101_b601_mapping.json}"
GRIPPER_HOLD_TORQUE="${GRIPPER_HOLD_TORQUE:-3.5}"
MAX_RELATIVE_TARGET="${MAX_RELATIVE_TARGET:-}"
WITH_CAMERAS="${WITH_CAMERAS:-0}"
FPS="${FPS:-30}"

ARGS=(
  --robot.type=rebot_b601_follower
  --robot.motor_family=rs
  --robot.port="${CAN_PORT}"
  --robot.id="${FOLLOWER_ID}"
  --robot.gripper_hold_torque_limit="${GRIPPER_HOLD_TORQUE}"
  --teleop.type=so101_b601_leader
  --teleop.port="${LEADER_PORT}"
  --teleop.id="${LEADER_ID}"
  --teleop.leader_id="${LEADER_ID}"
  --teleop.mapping_config="${MAPPING_CONFIG}"
  --fps="${FPS}"
)

if [[ -n "${MAX_RELATIVE_TARGET}" ]]; then
  ARGS+=(--robot.max_relative_target="${MAX_RELATIVE_TARGET}")
fi

if [[ "${WITH_CAMERAS}" == "1" ]]; then
  export DISPLAY="${DISPLAY:-:1}"
  if [[ -z "${XAUTHORITY:-}" && -f "/run/user/$(id -u)/gdm/Xauthority" ]]; then
    export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
  fi

  CAM_WIDTH="${CAM_WIDTH:-640}"
  CAM_HEIGHT="${CAM_HEIGHT:-480}"
  CAM_FPS="${CAM_FPS:-30}"
  if [[ -z "${CAMERAS_JSON:-}" ]]; then
    if [[ -z "${FRONT_CAM_SERIAL:-}" || -z "${SIDE_CAM_SERIAL:-}" ]]; then
      echo "WITH_CAMERAS=1 requires CAMERAS_JSON or both FRONT_CAM_SERIAL and SIDE_CAM_SERIAL (see .env.example)." >&2
      exit 1
    fi
    CAMERAS_JSON="{front: {type: intelrealsense, serial_number_or_name: \"${FRONT_CAM_SERIAL}\", width: ${CAM_WIDTH}, height: ${CAM_HEIGHT}, fps: ${CAM_FPS}, use_depth: false}, side: {type: intelrealsense, serial_number_or_name: \"${SIDE_CAM_SERIAL}\", width: ${CAM_WIDTH}, height: ${CAM_HEIGHT}, fps: ${CAM_FPS}, use_depth: false}}"
  fi

  ARGS+=(
    --display_data=true
    --robot.cameras="${CAMERAS_JSON}"
  )
fi

exec "${LEROBOT_BIN}" "${ARGS[@]}" "$@"
