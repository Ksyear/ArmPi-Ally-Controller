#!/usr/bin/env bash
# Start the protocol mock robot from the ArmPro_Cpp_ROS2 repository (UDP control/discovery, rosbridge, camera).
#   scripts/run_mock.sh /path/to/ArmPro_Cpp_ROS2 [bind-address] [port-base]
# port-base 25000 -> control 25001, discovery 25002, rosbridge 29090, camera 28080 (keeps the default ports free).
# Prints the PID; stop it with `kill <PID>` (not pkill: other tools may run their own mock).
set -euo pipefail
repo="${1:?usage: run_mock.sh /path/to/ArmPro_Cpp_ROS2 [bind] [port-base]}"
bind="${2:-127.0.0.1}"
base="${3:-25000}"
python3 "$repo/protocol_v3/mock_robot.py" --bind "$bind" --port $((base + 1)) --discovery-port $((base + 2)) \
  --rosbridge-port $((base + 4090)) --camera-port $((base + 3080)) --name ally-mock \
  > "${MOCK_LOG:-${TMPDIR:-/tmp}/ally_mock_$base.log}" 2>&1 < /dev/null &
echo "$!"
