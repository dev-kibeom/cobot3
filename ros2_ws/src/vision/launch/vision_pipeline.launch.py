from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node_args(context):
    cell = LaunchConfiguration('cell').perform(context)
    image_topic = LaunchConfiguration('image_topic').perform(context) or f'/{cell}/top_camera/image'
    obb_topic = LaunchConfiguration('obb_topic').perform(context) or f'/{cell}/vision/plate_obb'
    goal_topic = LaunchConfiguration('goal_topic').perform(context) or f'/{cell}/vision/pick_place_goal'
    model = LaunchConfiguration('model').perform(context)

    yolo_args = [
        '--cells', cell,
        '--image-topic-template', image_topic,
        '--obb-topic-template', obb_topic,
        '--debug-topic-template', f'/{cell}/vision/debug_image',
        '--trigger-topic-template', LaunchConfiguration('trigger_topic').perform(context),
        '--image-transport', LaunchConfiguration('image_transport').perform(context),
        '--imgsz', LaunchConfiguration('imgsz').perform(context),
        '--conf', LaunchConfiguration('conf').perform(context),
        '--device', LaunchConfiguration('device').perform(context),
        '--publish-debug', LaunchConfiguration('publish_debug').perform(context),
    ]
    if model:
        yolo_args.extend(['--model', model])
    if LaunchConfiguration('augment').perform(context).lower() in ('1', 'true', 'yes', 'on'):
        yolo_args.append('--augment')

    brain_args = [
        '--cell', cell,
        '--obb-topic', obb_topic,
        '--goal-topic', goal_topic,
        '--fx', LaunchConfiguration('fx').perform(context),
        '--fy', LaunchConfiguration('fy').perform(context),
        '--cx', LaunchConfiguration('cx').perform(context),
        '--cy', LaunchConfiguration('cy').perform(context),
        '--camera-x', LaunchConfiguration('camera_x').perform(context),
        '--camera-y', LaunchConfiguration('camera_y').perform(context),
        '--camera-z', LaunchConfiguration('camera_z').perform(context),
        '--place-x', LaunchConfiguration('place_x').perform(context),
        '--place-y', LaunchConfiguration('place_y').perform(context),
        '--place-yaw-deg', LaunchConfiguration('place_yaw_deg').perform(context),
        '--plate-top-z', LaunchConfiguration('plate_top_z').perform(context),
        '--cube-top-z', LaunchConfiguration('cube_top_z').perform(context),
        '--min-confidence', LaunchConfiguration('brain_min_conf').perform(context),
    ]
    t_world_camera = LaunchConfiguration('t_world_camera').perform(context)
    if t_world_camera:
        brain_args.extend(['--t-world-camera', t_world_camera])

    return [
        Node(package='vision', executable='yolo', name='yolo11s_obb_eye_node', output='screen', arguments=yolo_args),
        Node(package='vision', executable='brain', name='alignment_brain_node', output='screen', arguments=brain_args),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cell', default_value='global'),
        DeclareLaunchArgument('image_topic', default_value=''),
        DeclareLaunchArgument('image_transport', default_value='auto'),
        DeclareLaunchArgument('obb_topic', default_value=''),
        DeclareLaunchArgument('goal_topic', default_value=''),
        DeclareLaunchArgument('trigger_topic', default_value=''),
        DeclareLaunchArgument('model', default_value=''),
        DeclareLaunchArgument('publish_debug', default_value='false'),
        DeclareLaunchArgument('imgsz', default_value='960'),
        DeclareLaunchArgument('conf', default_value='0.25'),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('augment', default_value='false'),
        DeclareLaunchArgument('fx', default_value='500.0'),
        DeclareLaunchArgument('fy', default_value='500.0'),
        DeclareLaunchArgument('cx', default_value='320.0'),
        DeclareLaunchArgument('cy', default_value='240.0'),
        DeclareLaunchArgument('camera_x', default_value='0.35'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='1.35'),
        DeclareLaunchArgument('t_world_camera', default_value=''),
        DeclareLaunchArgument('place_x', default_value='0.45'),
        DeclareLaunchArgument('place_y', default_value='-0.22'),
        DeclareLaunchArgument('place_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('plate_top_z', default_value='0.03'),
        DeclareLaunchArgument('cube_top_z', default_value='0.05'),
        DeclareLaunchArgument('brain_min_conf', default_value='0.35'),
        OpaqueFunction(function=_node_args),
    ])
