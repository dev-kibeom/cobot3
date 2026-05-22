from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("cell", default_value="m0609"),
        DeclareLaunchArgument("obb_topic", default_value="/m0609/vision/plate_obb"),
        DeclareLaunchArgument("goal_topic", default_value="/m0609/vision/pick_place_goal"),
        DeclareLaunchArgument("camera_transform_topic", default_value="/m0609/d455/t_world_camera"),
        DeclareLaunchArgument("fx", default_value="500.0"),
        DeclareLaunchArgument("fy", default_value="500.0"),
        DeclareLaunchArgument("cx", default_value="320.0"),
        DeclareLaunchArgument("cy", default_value="240.0"),
        DeclareLaunchArgument("place_x", default_value="0.45"),
        DeclareLaunchArgument("place_y", default_value="-0.22"),
        DeclareLaunchArgument("place_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("plate_top_z", default_value="0.03"),
        DeclareLaunchArgument("cube_top_z", default_value="0.05"),
        DeclareLaunchArgument("min_confidence", default_value="0.35"),
        Node(
            package="vision",
            executable="brain",
            name="m0609_alignment_brain_node",
            output="screen",
            arguments=[
                "--cell", LaunchConfiguration("cell"),
                "--obb-topic", LaunchConfiguration("obb_topic"),
                "--goal-topic", LaunchConfiguration("goal_topic"),
                "--camera-transform-topic", LaunchConfiguration("camera_transform_topic"),
                "--fx", LaunchConfiguration("fx"),
                "--fy", LaunchConfiguration("fy"),
                "--cx", LaunchConfiguration("cx"),
                "--cy", LaunchConfiguration("cy"),
                "--place-x", LaunchConfiguration("place_x"),
                "--place-y", LaunchConfiguration("place_y"),
                "--place-yaw-deg", LaunchConfiguration("place_yaw_deg"),
                "--plate-top-z", LaunchConfiguration("plate_top_z"),
                "--cube-top-z", LaunchConfiguration("cube_top_z"),
                "--min-confidence", LaunchConfiguration("min_confidence"),
            ],
        ),
    ])
