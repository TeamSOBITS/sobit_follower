import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


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
        description='Robot type for selecting body detection param file.',
    )

    params_file_arg = DeclareLaunchArgument(
        'ssd_params_file',
        default_value=default_param_file,
        description='Full path to the body detection parameter file.'
    )

    params_file = LaunchConfiguration('ssd_params_file')

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
        ],
        remappings=[
            ('object_3d_poses', '/sobits_follower/body_3d_poses') 
        ]
    )

    return LaunchDescription([
        robot_type_arg,
        params_file_arg,
        ssd_node,
        bbox_to_3d_cmd
    ])
