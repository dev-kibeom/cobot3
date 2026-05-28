import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 🚀 Hub PC IP를 외부에서 주입받을 수 있도록 설정
    hub_ip = LaunchConfiguration("hub_ip", default="127.0.0.1")

    bringup_share_dir = get_package_share_directory("bringup")
    params_file = os.path.join(bringup_share_dir, "config", "m0609_params.yaml")

    # 1. 로봇팔 제어 노드 (PnP 수행)
    robot_controller_node = Node(
        package="controller",
        executable="m0609_controller",
        name="doosan_robot_controller_node",
        output="screen",
        parameters=[params_file],
    )

    # 2. 🚀 이미지 압축 전송 브릿지 (YOLO 연산을 Hub PC로 위임)
    image_bridge_node = Node(
        package="bridge",
        executable="image_bridge",  # bridge/setup.py에 등록된 이름
        name="image_bridge_node",
        output="screen",
    )

    # 3. 🚀 MQTT 수신 브릿지 (FMS 서버의 도착 알림을 수신)
    mqtt_receiver_node = Node(
        package="bridge",
        executable="mqtt_receiver",  # 새로 추가할 수신 스크립트
        name="mqtt_receiver_node",
        output="screen",
        parameters=[
            {"mqtt_broker": hub_ip},  # Hub PC의 주소를 바라보게 설정
            {"mqtt_port": 1883},
            {"mqtt_topic": "smart_factory/amr/arrival"},  # FMS 서버가 쏘는 토픽 이름
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hub_ip",
                default_value="127.0.0.1",
                description="Target Hub PC IP Address",
            ),
            robot_controller_node,
            image_bridge_node,
            mqtt_receiver_node,
        ]
    )
