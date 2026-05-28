from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    model = LaunchConfiguration("model").perform(context)
    aruco_roi_marker_ids = LaunchConfiguration("aruco_roi_marker_ids").perform(context)
    args = [
        "--cells", LaunchConfiguration("cell").perform(context),
        "--image-topic-template", LaunchConfiguration("image_topic").perform(context),
        "--obb-topic-template", LaunchConfiguration("obb_topic").perform(context),
        "--debug-topic-template", LaunchConfiguration("debug_topic").perform(context),
        "--trigger-topic-template", LaunchConfiguration("trigger_topic").perform(context),
        "--image-transport", LaunchConfiguration("image_transport").perform(context),
        "--imgsz", LaunchConfiguration("imgsz").perform(context),
        "--conf", LaunchConfiguration("conf").perform(context),
        "--device", LaunchConfiguration("device").perform(context),
        "--publish-debug", LaunchConfiguration("publish_debug").perform(context),
        "--fallback-detector", LaunchConfiguration("fallback_detector").perform(context),
        "--fallback-min-confidence", LaunchConfiguration("fallback_min_confidence").perform(context),
        "--fallback-min-area-px", LaunchConfiguration("fallback_min_area_px").perform(context),
        "--fallback-plate-aspect-min", LaunchConfiguration("fallback_plate_aspect_min").perform(context),
        "--fallback-lab-delta-threshold", LaunchConfiguration("fallback_lab_delta_threshold").perform(context),
        "--allowed-class-ids", LaunchConfiguration("allowed_class_ids").perform(context),
        "--roi", LaunchConfiguration("roi").perform(context),
        "--reject-edge-margin", LaunchConfiguration("reject_edge_margin").perform(context),
        "--min-area-ratio", LaunchConfiguration("min_area_ratio").perform(context),
        "--max-area-ratio", LaunchConfiguration("max_area_ratio").perform(context),
        "--aruco-dict", LaunchConfiguration("aruco_dict").perform(context),
    ]
    if model:
        args.extend(["--model", model])
    if LaunchConfiguration("augment").perform(context).lower() in ("1", "true", "yes", "on"):
        args.append("--augment")
    if aruco_roi_marker_ids:
        args.extend(["--aruco-roi-marker-ids", aruco_roi_marker_ids])
    return [
        Node(
            package="vision",
            executable="yolo",
            name="m0609_yolo11s_obb_eye_node",
            output="screen",
            arguments=args,
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            "FASTDDS_BUILTIN_TRANSPORTS",
            EnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", default_value="UDPv4"),
        ),
        DeclareLaunchArgument("cell", default_value="m0609"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_rgb"),
        DeclareLaunchArgument("obb_topic", default_value="/m0609/vision/plate_obb"),
        DeclareLaunchArgument("debug_topic", default_value="/m0609/vision/debug_image"),
        DeclareLaunchArgument("trigger_topic", default_value=""),
        DeclareLaunchArgument("image_transport", default_value="auto"),
        DeclareLaunchArgument("model", default_value=""),
        DeclareLaunchArgument("publish_debug", default_value="true"),
        DeclareLaunchArgument("imgsz", default_value="960"),
        DeclareLaunchArgument("conf", default_value="0.45"),
        DeclareLaunchArgument("device", default_value="0"),
        DeclareLaunchArgument("augment", default_value="false"),
        DeclareLaunchArgument("fallback_detector", default_value="color_obb"),
        DeclareLaunchArgument("fallback_min_confidence", default_value="0.20"),
        DeclareLaunchArgument("fallback_min_area_px", default_value="120.0"),
        DeclareLaunchArgument("fallback_plate_aspect_min", default_value="1.35"),
        DeclareLaunchArgument("fallback_lab_delta_threshold", default_value="12.0"),
        DeclareLaunchArgument("allowed_class_ids", default_value="0,1"),
        DeclareLaunchArgument("roi", default_value="0.05,0.05,0.88,0.95"),
        DeclareLaunchArgument("reject_edge_margin", default_value="0.02"),
        DeclareLaunchArgument("min_area_ratio", default_value="0.001"),
        DeclareLaunchArgument("max_area_ratio", default_value="0.20"),
        DeclareLaunchArgument("aruco_roi_marker_ids", default_value=""),
        DeclareLaunchArgument("aruco_dict", default_value="DICT_6X6_250"),
        OpaqueFunction(function=_nodes),
    ])
