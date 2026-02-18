import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():

    default_param_file = os.path.join(
        get_package_share_directory('sobit_follower'),
        'param',
        'hsrb',
        'ssd_param.yaml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_param_file,
        description='Full path to the SSD parameter file.'
    )

    params_file = LaunchConfiguration('params_file')

    ssd_node = Node(
        package='ssd_ros',
        executable='single_shot_multibox_detector',
        name='ssd_ros',
        parameters=[
            params_file, 
            {
                "ssd_prototxt_name": os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object.prototxt'),
                "ssd_caffemodel_name": os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object.caffemodel'),
                "ssd_class_names_file": os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object_names.txt'),    
            },
        ],

    )

    return LaunchDescription([
        params_file_arg,
        ssd_node
    ])