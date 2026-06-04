import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _load_ros_params(params_file_path: str, yolo_share_dir: str) -> dict:
    with open(params_file_path, "r", encoding="utf-8") as params_file:
        params = yaml.safe_load(params_file) or {}

    ros_params = params.get("/**", {}).get("ros__parameters", {})
    weight_file = ros_params.get("weight_file")

    if isinstance(weight_file, str) and weight_file and not os.path.isabs(weight_file):
        ros_params["weight_file"] = os.path.join(yolo_share_dir, "weights", weight_file)

    return ros_params


def _as_bool(value, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def _join_namespace(namespace: str, node_name: str) -> str:
    namespace = str(namespace).strip("/")
    if namespace:
        return "/" + namespace + "/" + node_name
    return "/" + node_name


def _launch_setup(context):
    params_file = LaunchConfiguration('yolo_params_file').perform(context)
    yolo_share_dir = FindPackageShare('yolo_ros').perform(context)
    ros_params = _load_ros_params(params_file, yolo_share_dir)
    namespace = str(ros_params.get('namespace', 'yolo_ros')).strip('/')
    autostart_lifecycle = str(
        LaunchConfiguration('autostart_lifecycle').perform(context)
    ).strip().lower() in ('true',)
    auto_configure_2d = autostart_lifecycle and _as_bool(ros_params.get('auto_configure_2d'), True)
    auto_activate_2d  = autostart_lifecycle and _as_bool(ros_params.get('auto_activate_2d'),  True)
    auto_configure_3d = autostart_lifecycle and _as_bool(ros_params.get('auto_configure_3d'), True)
    auto_activate_3d  = autostart_lifecycle and _as_bool(ros_params.get('auto_activate_3d'),  True)
    object_3d_poses_topic = _join_namespace(namespace, 'bbox_to_3d/object_3d_poses')

    yolo_node = Node(
        package='yolo_ros',
        executable='yolo_node',
        name='yolo_node',
        namespace=namespace,
        output='screen',
        parameters=[
            ros_params,
            {
                'auto_configure': auto_configure_2d,
                'auto_activate': auto_activate_2d,
            },
        ],
        remappings=[
            (object_3d_poses_topic, '/sobits_follower/body_3d_poses')
        ]
    )

    bbox_to_3d_cmd = Node(
        package='image_to_position',
        executable='bbox_to_3d',
        name='bbox_to_3d',
        namespace=namespace,
        output='screen',
        parameters=[ros_params],
        remappings=[
            (object_3d_poses_topic, '/sobits_follower/body_3d_poses')
        ]
    )

    node_full_path = _join_namespace(namespace, 'bbox_to_3d')

    configure_node = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            f'until ros2 lifecycle get "{node_full_path}" >/dev/null 2>&1; do sleep 0.2; done; '
            f'ros2 lifecycle set "{node_full_path}" configure',
        ],
        output='screen'
    )

    configure_3d_condition = IfCondition("true" if auto_configure_3d or auto_activate_3d else "false")

    activate_node = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            f'until ros2 lifecycle get "{node_full_path}" 2>/dev/null | grep -q "inactive"; do sleep 0.2; done; '
            f'ros2 lifecycle set "{node_full_path}" activate',
        ],
        output='screen'
    )

    return [
        yolo_node, 
        bbox_to_3d_cmd,
        TimerAction(period=0.5, actions=[configure_node], condition=configure_3d_condition),
        TimerAction(period=1.0, actions=[activate_node], condition=IfCondition("true" if auto_activate_3d else "false"))
    ]


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    default_param_file = PathJoinSubstitution([
        FindPackageShare('sobits_follower'), 
        'param',
        robot_type,
        'body_detection_param.yaml',
    ])
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='sobit_edu',
        description='Robot type for selecting YOLO param file.'
    )
    params_file_arg = DeclareLaunchArgument(
        'yolo_params_file',
        default_value=default_param_file,
        description='Full path to the YOLO parameter file.'
    )
    autostart_lifecycle_arg = DeclareLaunchArgument(
        'autostart_lifecycle',
        default_value='true',
        description='Whether to automatically configure and activate yolo lifecycle nodes',
    )
    return LaunchDescription([
        robot_type_arg,
        params_file_arg,
        autostart_lifecycle_arg,
        OpaqueFunction(function=_launch_setup),
    ])
