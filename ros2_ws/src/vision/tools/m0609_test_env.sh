#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Use: source /home/rokey/smart_factory_project/cobot3/ros2_ws/src/vision/tools/m0609_test_env.sh"
  exit 1
fi

cd /home/rokey/smart_factory_project/cobot3/ros2_ws || return
source install/setup.bash
shopt -s expand_aliases 2>/dev/null || true

export ROS_DOMAIN_ID="${VISION_ROS_DOMAIN_ID:-105}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

alias vyolo='ros2 launch vision m0609_d455_yolo.launch.py'
alias vbrain='ros2 launch vision m0609_d455_brain.launch.py'
alias vtop='ros2 launch vision m0609_topview_pipeline.launch.py image_topic:=/camera/rgb/observer_01/compressed image_transport:=compressed'
alias vgateway103='ros2 run vision goal_gateway --source-domain 105 --target-domain 103 --input-topic /m0609/vision/pick_place_goal --output-topic /m0609_vision/pick_place_goal'
alias vmotiongoal='ROS_DOMAIN_ID=103 ros2 topic echo /m0609_vision/pick_place_goal'
alias vrqt='ros2 run rqt_image_view rqt_image_view'
alias vgoal='ros2 topic echo /m0609/vision/pick_place_goal'
alias vimg='ros2 topic hz /camera/rgb/observer_01/compressed'
alias vdebug='ros2 topic hz /m0609/vision/debug_image'
alias vdebugc='ros2 topic hz /m0609/vision/debug_image/compressed'

echo "M0609 vision test env ready: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS}"
echo "Aliases: vyolo, vbrain, vtop, vgateway103, vmotiongoal, vrqt, vgoal, vimg, vdebug, vdebugc"
