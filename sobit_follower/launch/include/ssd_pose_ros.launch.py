import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    sobit_follower_dir = get_package_share_directory('sobit_follower')
    ssd_ros_share_dir = get_package_share_directory('ssd_ros')
    ssd_ros_launch_path = os.path.join(ssd_ros_share_dir, 'launch', 'ssd_ros.launch.py')
    ssd_param_path = os.path.join(sobit_follower_dir, 'param', 'ssd_param.yaml')

    with open(ssd_param_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

    # === get params from YAML ===
    image_topic = yaml_data.get('ssd_image_topic_name', '/camera/rgb/image_raw')
    cloud_topic = yaml_data.get('ssd_cloud_topic_name', '/points2')
    target_frame = yaml_data.get('target_frame', 'base_footprint')
    img_show_flag = str(yaml_data.get('ssd_img_show_flag', False)).lower()
    execute_default = str(yaml_data.get('ssd_execute_default', True)).lower()
    in_scale_factor = str(yaml_data.get('ssd_in_scale_factor', 0.007843))
    confidence_threshold = str(yaml_data.get('ssd_confidence_threshold', 0.5))
    object_specified_enabled = str(yaml_data.get('object_specified_enabled', True)).lower()
    specified_object_name = yaml_data.get('specified_object_name', 'person')

    # === LaunchArguments ===
    launch_arguments = [
        DeclareLaunchArgument("image_topic_name", default_value=image_topic),
        DeclareLaunchArgument("point_cloud_topic_name", default_value=cloud_topic),
        DeclareLaunchArgument("img_show_flag", default_value=img_show_flag),
        DeclareLaunchArgument("execute_default", default_value=execute_default),
        DeclareLaunchArgument("in_scale_factor", default_value=in_scale_factor),
        DeclareLaunchArgument("confidence_threshold", default_value=confidence_threshold),
        DeclareLaunchArgument("object_specified_enabled", default_value=object_specified_enabled),
        DeclareLaunchArgument("specified_object_name", default_value=specified_object_name),
    ]

    # Include ssd_ros.launch.py
    include_ssd_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ssd_ros_launch_path),
        launch_arguments=[
            ("image_topic_name", LaunchConfiguration("image_topic_name")),
            ("point_cloud_topic_name", LaunchConfiguration("point_cloud_topic_name")),
            ("img_show_flag", LaunchConfiguration("img_show_flag")),
            ("execute_default", LaunchConfiguration("execute_default")),
            ("in_scale_factor", LaunchConfiguration("in_scale_factor")),
            ("confidence_threshold", LaunchConfiguration("confidence_threshold")),
            ("object_specified_enabled", LaunchConfiguration("object_specified_enabled")),
            ("specified_object_name", LaunchConfiguration("specified_object_name")),
        ]
    )

    return LaunchDescription(launch_arguments + [include_ssd_ros])
