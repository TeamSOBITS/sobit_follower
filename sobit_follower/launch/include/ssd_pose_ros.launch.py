import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    default_param_file = PathJoinSubstitution([
        FindPackageShare('sobit_follower'),
        'param',
        robot_type,
        'ssd_param.yaml',
    ])
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='hsrb',
        description='Robot type for selecting SSD param file.',
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

    bbox_to_3d_cmd = Node(
        package='image_to_position',
        executable='bbox_to_3d',
        name='bbox_to_3d',
        namespace='ssd_ros',
        output='screen',
        parameters=[
            params_file
        ]
    )

    return LaunchDescription([
        robot_type_arg,
        params_file_arg,
        ssd_node,
        bbox_to_3d_cmd
    ])
