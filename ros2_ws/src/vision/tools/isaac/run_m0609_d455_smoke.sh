#!/usr/bin/env bash
set -euo pipefail

cd /home/rokey/smart_factory_project/cobot3/ros2_ws/src/vision
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-105}"

/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  tools/isaac/m0609_d455_yolo_rqt_smoke.py \
  --gui \
  --execute-goals \
  "$@"
