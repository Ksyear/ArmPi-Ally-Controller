#!/usr/bin/env bash
# End-to-end test: mock robot + armpi-ally-cli selftest (discovery, drive, arm ramps, reach limit, e-stop,
# rosbridge status/services, camera decode, joint debug, new session). Exit code 0 = all passed.
#   test/e2e_mock.sh /path/to/ArmPro_Cpp_ROS2 [build-dir]
set -euo pipefail
cd "$(dirname "$0")/.."
repo="${1:?usage: e2e_mock.sh /path/to/ArmPro_Cpp_ROS2 [build-dir]}"
build="${2:-build-mac}"
pid="$(scripts/run_mock.sh "$repo" 127.0.0.1 25000)"
trap 'kill "$pid" 2>/dev/null || true' EXIT
sleep 1.5
"$build/armpi-ally-cli" discover --target 127.0.0.1 --discovery-port 25002 --timeout 1.5
"$build/armpi-ally-cli" selftest --robot 127.0.0.1 --port 25001 --rosbridge-port 29090 --camera-port 28080
