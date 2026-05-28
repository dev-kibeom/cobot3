import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    hub_ip = LaunchConfiguration("hub_ip", default="127.0.0.1")

    bringup_dir = get_package_share_directory("bringup")
    map_dir = LaunchConfiguration(
        "map", default=os.path.join(bringup_dir, "maps", "smart_factory.yaml")
    )
    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(bringup_dir, "config", "iw_hub_nav2_params_robot2.yaml"),
    )
    rviz_config_dir = os.path.join(bringup_dir, "rviz2", "iw_hub_nav2.rviz")

    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=map_dir,
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=param_dir,
                description="Full path to param file to load",
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation clock"
            ),
            # 1. RViz2 실행
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")
            #     ),
            #     launch_arguments={
            #         "namespace": "",
            #         "use_namespace": "False",
            #         "rviz_config": rviz_config_dir,
            #     }.items(),
            # ),
            # 2. Nav2 엔진 실행 (AMCL, Planner, Controller 등)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_launch_dir, "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_dir,
                    "use_sim_time": use_sim_time,
                    "params_file": param_dir,
                    "log_level": "error",  # 로그 레벨을 ERROR로 설정하여 불필요한 정보 출력 억제
                }.items(),
            ),
            # 3. 브릿지 노드 실행 (MQTT 통신)
            Node(
                package="bridge",
                executable="mqtt_trigger",  # bridge 패키지의 setup.py에 등록된 실행 이름
                name="mqtt_trigger",
                output="screen",
                prefix=["xterm -e"],
                parameters=[
                    {"mqtt_broker": hub_ip},
                    {"mqtt_port": 1883},
                    {"mqtt_topic": "smart_factory/amr/arrival"},
                ],
            ),
            # C++ BT 관제탑 실행 노드
            Node(
                package="manager",
                executable="bt_scheduler",
                name="bt_manager",
                output="screen",
                prefix=["xterm -hold -e"],
                parameters=[
                    {"robot_id": "IW_HUB-02"},  # 로봇 이름 동적 주입
                    {
                        "server_url": [hub_ip, ":8001/api/robots/amr"]
                    },  # 단순 리스트로 묶으면 자동으로 문자열이 합쳐집니다.
                ],
            ),
        ]
    )
