import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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


def _launch_setup(context):
    params_file = LaunchConfiguration('yolo_params_file').perform(context)
    yolo_share_dir = FindPackageShare('yolo_ros').perform(context)
    ros_params = _load_ros_params(params_file, yolo_share_dir)

    yolo_node = Node(
        package='yolo_ros',
        executable='yolo_node',
        name='yolo_ros',
        output='screen',
        parameters=[ros_params],
        remappings=[
            ('/yolo_ros/bbox_to_3d/object_3d_poses', '/sobits_follower/body_3d_poses')
        ]
    )

    bbox_to_3d_cmd = Node(
        package='image_to_position',
        executable='bbox_to_3d',
        name='bbox_to_3d',
        namespace='yolo_ros',
        output='screen',
        parameters=[ros_params],
        remappings=[
            ('/yolo_ros/bbox_to_3d/object_3d_poses', '/sobits_follower/body_3d_poses')
        ]
    )

    return [
        yolo_node, 
        bbox_to_3d_cmd,
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

    return LaunchDescription([
        robot_type_arg,
        params_file_arg,
        OpaqueFunction(function=_launch_setup),
    ])
