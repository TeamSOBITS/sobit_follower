from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    default_param_file = PathJoinSubstitution([
        FindPackageShare('sobit_follower'), 
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
        'params_file',
        default_value=default_param_file,
        description='Full path to the YOLO parameter file.'
    )

    params_file = LaunchConfiguration('params_file')

    yolo_node = Node(
        package='yolo_ros',
        executable='yolo_node',
        name='yolo_ros',
        output='screen',
        parameters=[
            params_file, 
        ],
        remappings=[
            ('object_3d_poses', '/sobit_follower/body_3d_poses')
        ]
    )

    bbox_to_3d_cmd = Node(
        package='image_to_position',
        executable='bbox_to_3d',
        name='bbox_to_3d',
        namespace='yolo_ros',
        output='screen',
        parameters=[
            params_file
        ],
        remappings=[
            ('object_3d_poses', '/sobit_follower/body_3d_poses')
        ]
    )

    return LaunchDescription([
        robot_type_arg,
        params_file_arg,
        yolo_node,
        bbox_to_3d_cmd,
    ])
