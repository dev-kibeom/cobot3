"""두 iw_hub_ROS 양보 코디네이터 launch.

별도 터미널에서 실행:
    source /opt/ros/humble/setup.bash
    source ~/smart_factory_project/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=101
    ros2 launch slam_nav yield_coordinator.launch.py

전제: robot1_slam.launch.py + robot2_slam.launch.py 둘 다 실행 중.
이 노드가 두 namespace의 TF를 구독해 world 기준 거리 계산 → 임계값 미만이면
iw_hub_ROS_02 cmd_vel jam.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    yield_node = Node(
        package="slam_nav",
        executable="robot_yield_coordinator.py",
        name="robot_yield_coordinator",
        output="screen",
    )
    return LaunchDescription([yield_node])
