#!/usr/bin/env bash
set -euo pipefail
smolvla_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$smolvla_root/build/libopenarm_smolvla_control.so" ]]; then
  echo "Build the policy plugin first; see README.md." >&2
  exit 1
fi
if [[ $# -ne 1 ]]; then
  echo "Usage: bash scripts/start_robot.sh LAPTOP_WIFI_IP" >&2
  exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/openarm_ws/install/setup.bash"
source "$HOME/vive_vr_ws/install/setup.bash"
set -u
exec ros2 launch vive_vr_ros2 openarm_real.launch.py host_ip:="$1" external_control_library:="$smolvla_root/build/libopenarm_smolvla_control.so"
