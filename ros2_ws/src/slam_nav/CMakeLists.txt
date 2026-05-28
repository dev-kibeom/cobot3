cmake_minimum_required(VERSION 3.5)
project(slam_nav)

find_package(ament_cmake REQUIRED)

# Asset directories — share/slam_nav/<dir>로 설치되어 launch/params/맵/BT/RViz/USD 참조 가능.
install(DIRECTORY
  launch
  params
  behavior_trees
  rviz
  maps
  usd
  config
  DESTINATION share/${PROJECT_NAME})

# Python helper nodes — `ros2 run slam_nav <name>.py` 실행 가능하게 lib/에 설치.
# 2026-05-27: robot1_*/robot2_* 8개 helper → --robot 인자 받는 단일 파일 4개로 통합.
install(PROGRAMS
  scripts/world_tf_bridge.py
  scripts/footprint_pub.py
  scripts/lidar_self_filter.py
  scripts/arrival_stopper.py
  scripts/robot_yield_coordinator.py
  scripts/chain_goal.py
  scripts/chain_waypoint_server.py
  scripts/lift_ramper.py
  DESTINATION lib/${PROJECT_NAME})

# 환경 hook — 워크스페이스 오버레이 소싱 시 ROS_DOMAIN_ID 자동 export.
# (env-hooks/ros_domain_id.sh.in 추가 후 활성화)
# ament_environment_hooks(env-hooks/ros_domain_id.sh.in)

ament_package()
