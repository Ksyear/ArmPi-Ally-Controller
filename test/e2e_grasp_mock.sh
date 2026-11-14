#!/usr/bin/env bash
# Autograsp end to end: mock robot + synthetic table camera (tools/fake_cam_server.py) + armpi-ally-cli grasp-selftest
# (detect -> plan -> open/hover/descend/close/lift -> verify, then dead-man pause/resume and e-stop abort, then the
# base approach to an object out of reach: the synthetic camera follows the mock robot's odometry).
#   test/e2e_grasp_mock.sh /path/to/ArmPro_Cpp_ROS2 [build-dir]
set -euo pipefail
cd "$(dirname "$0")/.."
repo="${1:?usage: e2e_grasp_mock.sh /path/to/ArmPro_Cpp_ROS2 [build-dir]}"
build="${2:-build-mac}"
work="$(mktemp -d)"
pid="$(scripts/run_mock.sh "$repo" 127.0.0.1 26000)"
python3 tools/fake_cam_server.py --port 28180 --write-calib "$work/vision_calib.json" --follow-robot 127.0.0.1:30090 \
  --objects "cube,red,0.18,0.00;ball,blue,0.15,-0.08;cube,green,0.36,0.08" > "$work/cam.log" 2>&1 < /dev/null &
cam=$!
disown "$cam" 2>/dev/null || true
trap 'kill "$pid" "$cam" 2>/dev/null || true; rm -rf "$work"' EXIT
for _ in $(seq 50); do
  python3 -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:28180/state', timeout=0.5)" 2>/dev/null && break
  sleep 0.2
done
sleep 0.5
"$build/armpi-ally-cli" grasp-selftest --robot 127.0.0.1 --port 26001 --rosbridge-port 30090 \
  --camera-url "http://127.0.0.1:28180/stream" --calib "$work/vision_calib.json" --expect-xy 0.18,0.00 \
  --approach-class "초록 블록" --approach-object 2
