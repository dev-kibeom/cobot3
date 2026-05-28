"""iw_hub_ROS_01 + iw_hub_ROS_02 통합 launch — 하나의 명령으로 두 robot + RViz 2개.

설계 (2026-05-27 통합 리팩토링):
  - helper 스크립트(world_tf_bridge, footprint_pub, lidar_self_filter, arrival_stopper)는
    `--robot iw_hub_ROS_0N` 인자 받는 단일 파일 4개로 통합됨.
  - params/nav.yaml 단일 템플릿 + RViz config(slam.rviz)도 단일 템플릿.
    `<ROBOT>` placeholder는 launch에서 robot bare name으로 치환해 /tmp/slam_nav_{bare}_*.yaml,
    /tmp/slam_nav_{bare}_slam.rviz 로 저장 후 사용.
  - chain_waypoint_server 두 인스턴스 자동 spawn — PC-D(DB)가
    /iw_hub_ROS_01/chain_waypoints, /iw_hub_ROS_02/chain_waypoints에 각각 PoseArray
    publish하면 두 robot이 동시 독립 수행.

TF chain (각 robot):
  world → iw_hub_ROS_0N/map       (world_tf_bridge.py — static)
  iw_hub_ROS_0N/map → odom         (static_transform_publisher — identity)
  iw_hub_ROS_0N/odom → base_link   (USD wheel encoder — live)

추후 RViz 비활성화: launch 인자 `rviz:=false`
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


PLACEHOLDER = "<ROBOT>"


def _materialize(template_path: str, bare: str, suffix: str) -> str:
    """Read template, substitute <ROBOT> → bare, write to /tmp, return path."""
    with open(template_path, "r") as f:
        text = f.read()
    text = text.replace(PLACEHOLDER, bare)
    out_path = os.path.join(tempfile.gettempdir(), f"slam_nav_{bare}_{suffix}")
    with open(out_path, "w") as f:
        f.write(text)
    return out_path


def make_robot_group(robot_num: int, use_sim_time, show_rviz):
    """robot_num=1 또는 2. 해당 robot의 모든 노드 리스트 반환 (RViz 별도)."""
    pkg_share = get_package_share_directory("slam_nav")
    bare = f"iw_hub_ROS_{robot_num:02d}"        # iw_hub_ROS_01, iw_hub_ROS_02
    ns = f"/{bare}"

    nav_template = os.path.join(pkg_share, "params", "nav.yaml")
    rviz_template = os.path.join(pkg_share, "rviz", "slam.rviz")
    basic_map_yaml = os.path.join(pkg_share, "maps", "basic2.yaml")

    # <ROBOT> placeholder 치환 — /tmp에 robot별 인스턴스 yaml/rviz 생성.
    nav_params = _materialize(nav_template, bare, "nav.yaml")
    rviz_config = _materialize(rviz_template, bare, "slam.rviz")

    tf_remaps = [("/tf", f"{ns}/tf"), ("/tf_static", f"{ns}/tf_static")]

    map_to_odom_static = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{bare}_map_to_odom_static",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", f"{bare}/map", f"{bare}/odom"],
        remappings=tf_remaps,
    )

    world_tf_bridge = Node(
        package="slam_nav",
        executable="world_tf_bridge.py",
        name=f"world_to_{bare}_map_bridge",
        output="screen",
        arguments=["--robot", bare],
        remappings=tf_remaps,
    )

    footprint_pub = Node(
        package="slam_nav",
        executable="footprint_pub.py",
        name=f"{bare}_footprint_publisher",
        output="screen",
        arguments=["--robot", bare],
        remappings=tf_remaps,
    )

    lidar_self_filter = Node(
        package="slam_nav",
        executable="lidar_self_filter.py",
        name=f"{bare}_lidar_self_filter",
        output="screen",
        arguments=["--robot", bare],
    )

    arrival_stopper = Node(
        package="slam_nav",
        executable="arrival_stopper.py",
        name=f"{bare}_arrival_stopper",
        output="screen",
        arguments=["--robot", bare],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name=f"{bare}_warehouse_map_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaml_filename": basic_map_yaml,
            "topic_name": f"{ns}/warehouse_map",
            "frame_id": "world",
        }],
    )

    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name=f"{bare}_lifecycle_manager_warehouse_map",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": [f"{bare}_warehouse_map_server"],
        }],
    )

    nav_ns = bare
    # Nav2 노드가 namespace=nav_ns로 띄워지므로 yaml 키도 그 namespace 아래에 있어야 함.
    # _materialize가 <ROBOT> placeholder를 치환한 /tmp yaml을 RewrittenYaml로 root_key 래핑.
    rewritten_nav_params = RewrittenYaml(
        source_file=nav_params,
        root_key=nav_ns,
        param_rewrites={},
        convert_types=True,
    )
    nav_common = [rewritten_nav_params, {"use_sim_time": use_sim_time}]

    nav_nodes = [
        Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server", namespace=nav_ns, output="screen",
            parameters=nav_common,
            remappings=tf_remaps + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_smoother", executable="smoother_server",
            name="smoother_server", namespace=nav_ns, output="screen",
            parameters=nav_common, remappings=tf_remaps,
        ),
        Node(
            package="nav2_planner", executable="planner_server",
            name="planner_server", namespace=nav_ns, output="screen",
            parameters=nav_common, remappings=tf_remaps,
        ),
        Node(
            package="nav2_behaviors", executable="behavior_server",
            name="behavior_server", namespace=nav_ns, output="screen",
            parameters=nav_common, remappings=tf_remaps,
        ),
        Node(
            package="nav2_bt_navigator", executable="bt_navigator",
            name="bt_navigator", namespace=nav_ns, output="screen",
            parameters=nav_common, remappings=tf_remaps,
        ),
        Node(
            package="nav2_waypoint_follower", executable="waypoint_follower",
            name="waypoint_follower", namespace=nav_ns, output="screen",
            parameters=nav_common, remappings=tf_remaps,
        ),
        Node(
            package="nav2_velocity_smoother", executable="velocity_smoother",
            name="velocity_smoother", namespace=nav_ns, output="screen",
            parameters=nav_common,
            remappings=tf_remaps + [
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "cmd_vel"),
            ],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", namespace=nav_ns, output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "controller_server", "smoother_server", "planner_server",
                    "behavior_server", "bt_navigator", "waypoint_follower",
                    "velocity_smoother",
                ],
                "bond_timeout": 4.0,
            }],
        ),
    ]

    lift_ramper = Node(
        package="slam_nav",
        executable="lift_ramper.py",
        name=f"{bare}_lift_ramper",
        output="screen",
        arguments=["--robot", bare],
    )

    chain_server = Node(
        package="slam_nav",
        executable="chain_waypoint_server.py",
        name=f"{bare}_chain_server",
        output="screen",
        arguments=["--robot", bare],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name=f"{bare}_rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        remappings=tf_remaps,
        condition=IfCondition(show_rviz),
    )

    immediate_nodes = [
        map_to_odom_static,
        world_tf_bridge,
        footprint_pub,
        lidar_self_filter,
        arrival_stopper,
        lift_ramper,
        map_server,
        map_lifecycle,
        *nav_nodes,
    ]
    # Nav2 active 후 chain_server + RViz spawn (action server 대기 회피)
    delayed_nodes = [chain_server, rviz]
    return immediate_nodes, delayed_nodes


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    show_rviz = LaunchConfiguration("rviz")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use Isaac Sim clock")
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Spawn RViz windows (set 'false' to skip)")

    immediate1, delayed1 = make_robot_group(1, use_sim_time, show_rviz)
    immediate2, delayed2 = make_robot_group(2, use_sim_time, show_rviz)

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        *immediate1,
        *immediate2,
        TimerAction(period=8.0, actions=[*delayed1, *delayed2]),
    ])
