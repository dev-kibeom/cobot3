cmake_minimum_required(VERSION 3.8)
project(manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 검색
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(cpr REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# 메인 노드 실행 파일 생성
add_executable(bt_scheduler src/bt_main.cpp)

# 헤더 파일 경로(include) 추가
target_include_directories(bt_scheduler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 라이브러리 및 ROS 2 종속성 링크
ament_target_dependencies(bt_scheduler
  rclcpp
  ament_index_cpp
  behaviortree_cpp
  nlohmann_json
  nav2_msgs
  rclcpp_action
)

# 순수 CMake 외부 라이브러리(cpr)는 타겟 링크 라이브러리로 직접 연결해 줍니다.
target_link_libraries(bt_scheduler
  cpr::cpr
)

# 1. 실행 파일을 install/manager/lib/manager 로 복사
install(TARGETS bt_scheduler
  DESTINATION lib/${PROJECT_NAME}
)

# 2. 🚀 config 폴더(XML 트리)를 install 공간으로 복사!
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()