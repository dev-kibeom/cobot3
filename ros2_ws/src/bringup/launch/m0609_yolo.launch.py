import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share_dir = get_package_share_directory("bringup")
    params_file = os.path.join(bringup_share_dir, "config", "m0609_params.yaml")

    # 제어 노드 실행
    robot_controller_node = Node(
        package="controller",
        executable="m0609_controller",  # controller/setup.py에 등록된 이름
        name="doosan_robot_controller_node",
        output="screen",
        parameters=[params_file],
    )

    # 비전 노드 실행
    vision_node = Node(
        package="vision",
        executable="yolo_object_detection",  # vision/setup.py에 등록된 이름
        name="vision_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([robot_controller_node, vision_node])
