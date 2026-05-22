from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    model = LaunchConfiguration("model").perform(context)
    t_world_camera = LaunchConfiguration("t_world_camera").perform(context)
    aruco_roi_marker_ids = LaunchConfiguration("aruco_roi_marker_ids").perform(context)

    yolo_args = [
        "--cells", "m0609",
        "--image-topic-template", LaunchConfiguration("image_topic").perform(context),
        "--obb-topic-template", LaunchConfiguration("obb_topic").perform(context),
        "--debug-topic-template", LaunchConfiguration("debug_topic").perform(context),
        "--image-transport", LaunchConfiguration("image_transport").perform(context),
        "--imgsz", LaunchConfiguration("imgsz").perform(context),
        "--conf", LaunchConfiguration("conf").perform(context),
        "--device", LaunchConfiguration("device").perform(context),
        "--publish-debug", LaunchConfiguration("publish_debug").perform(context),
        "--allowed-class-ids", LaunchConfiguration("allowed_class_ids").perform(context),
        "--roi", LaunchConfiguration("roi").perform(context),
        "--reject-edge-margin", LaunchConfiguration("reject_edge_margin").perform(context),
        "--min-area-ratio", LaunchConfiguration("min_area_ratio").perform(context),
        "--max-area-ratio", LaunchConfiguration("max_area_ratio").perform(context),
        "--aruco-dict", LaunchConfiguration("aruco_dict").perform(context),
    ]
    if model:
        yolo_args.extend(["--model", model])
    if aruco_roi_marker_ids:
        yolo_args.extend(["--aruco-roi-marker-ids", aruco_roi_marker_ids])

    brain_args = [
        "--cell", "global",
        "--robot-index", LaunchConfiguration("robot_index").perform(context),
        "--obb-topic", LaunchConfiguration("obb_topic").perform(context),
        "--goal-topic", LaunchConfiguration("goal_topic").perform(context),
        "--fx", LaunchConfiguration("fx").perform(context),
        "--fy", LaunchConfiguration("fy").perform(context),
        "--cx", LaunchConfiguration("cx").perform(context),
        "--cy", LaunchConfiguration("cy").perform(context),
        "--camera-x", LaunchConfiguration("camera_x").perform(context),
        "--camera-y", LaunchConfiguration("camera_y").perform(context),
        "--camera-z", LaunchConfiguration("camera_z").perform(context),
        "--place-x", LaunchConfiguration("place_x").perform(context),
        "--place-y", LaunchConfiguration("place_y").perform(context),
        "--place-yaw-deg", LaunchConfiguration("place_yaw_deg").perform(context),
        "--plate-top-z", LaunchConfiguration("plate_top_z").perform(context),
        "--cube-top-z", LaunchConfiguration("cube_top_z").perform(context),
        "--min-confidence", LaunchConfiguration("brain_min_conf").perform(context),
    ]
    if t_world_camera:
        brain_args.extend(["--t-world-camera", t_world_camera])

    return [
        Node(
            package="vision",
            executable="yolo",
            name="m0609_topview_yolo11s_obb_eye_node",
            output="screen",
            arguments=yolo_args,
        ),
        Node(
            package="vision",
            executable="brain",
            name="m0609_topview_alignment_brain_node",
            output="screen",
            arguments=brain_args,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            "FASTDDS_BUILTIN_TRANSPORTS",
            EnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", default_value="UDPv4"),
        ),
        DeclareLaunchArgument("image_topic", default_value="/camera/rgb/observer_01/compressed"),
        DeclareLaunchArgument("obb_topic", default_value="/m0609/vision/plate_obb"),
        DeclareLaunchArgument("debug_topic", default_value="/m0609/vision/debug_image"),
        DeclareLaunchArgument("goal_topic", default_value="/m0609/vision/pick_place_goal"),
        DeclareLaunchArgument("image_transport", default_value="compressed"),
        DeclareLaunchArgument("model", default_value=""),
        DeclareLaunchArgument("publish_debug", default_value="true"),
        DeclareLaunchArgument("imgsz", default_value="960"),
        DeclareLaunchArgument("conf", default_value="0.25"),
        DeclareLaunchArgument("device", default_value="0"),
        DeclareLaunchArgument("allowed_class_ids", default_value="0,1"),
        DeclareLaunchArgument("roi", default_value="0.0,0.0,1.0,1.0"),
        DeclareLaunchArgument("reject_edge_margin", default_value="0.0"),
        DeclareLaunchArgument("min_area_ratio", default_value="0.0003"),
        DeclareLaunchArgument("max_area_ratio", default_value="0.80"),
        DeclareLaunchArgument("aruco_roi_marker_ids", default_value=""),
        DeclareLaunchArgument("aruco_dict", default_value="DICT_6X6_250"),
        DeclareLaunchArgument("robot_index", default_value="1"),
        DeclareLaunchArgument("fx", default_value="500.0"),
        DeclareLaunchArgument("fy", default_value="500.0"),
        DeclareLaunchArgument("cx", default_value="320.0"),
        DeclareLaunchArgument("cy", default_value="240.0"),
        DeclareLaunchArgument("camera_x", default_value="0.35"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="1.35"),
        DeclareLaunchArgument("t_world_camera", default_value=""),
        DeclareLaunchArgument("place_x", default_value="0.45"),
        DeclareLaunchArgument("place_y", default_value="-0.22"),
        DeclareLaunchArgument("place_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("plate_top_z", default_value="0.03"),
        DeclareLaunchArgument("cube_top_z", default_value="0.05"),
        DeclareLaunchArgument("brain_min_conf", default_value="0.35"),
        OpaqueFunction(function=_nodes),
    ])
