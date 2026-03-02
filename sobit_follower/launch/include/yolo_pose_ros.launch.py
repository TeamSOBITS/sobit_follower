import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # 引数の定義
    robot_type = LaunchConfiguration('robot_type')
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='sobit_edu', # ここを変えるだけで切り替えたい
        description='Robot type: sobit_edu or sobit_pro'
    )

    # ロボットごとのパラメータ定義
    # 既存のコードに手を加えず、この辞書に使用したい設定をまとめます
    configs = {
        'sobit_edu': {
            'image': '/sobit_edu/head_camera/rgb/image_raw',
            'points': '/sobit_edu/head_camera/depth_registered/points',
            'depth': '/sobit_edu/head_camera/depth/image_raw',
            'info': '/sobit_edu/head_camera/rgb/camera_info',
            'frame': 'sobit_edu/base_footprint',
        },
        'sobit_pro': {
            'image': '/sobit_pro/head_camera/rgb/image_raw',
            'points': '/sobit_pro/head_camera/depth_registered/points',
            'depth': '/sobit_pro/head_camera/depth/image_raw',
            'info': '/sobit_pro/head_camera/rgb/camera_info',
            'frame': 'sobit_pro/base_footprint',
        }
    }

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yolo_ros'),
                'launch',
                'yolo.launch.py'
            )
        ),
        launch_arguments={
            # PythonExpressionを使って、robot_typeに応じて辞書から値を取得する
            'image_topic_name': PythonExpression([f"'{configs['sobit_edu']['image']}' if '", robot_type, "' == 'sobit_edu' else '", configs['sobit_pro']['image'], "'"]),
            'point_cloud_topic': PythonExpression([f"'{configs['sobit_edu']['points']}' if '", robot_type, "' == 'sobit_edu' else '", configs['sobit_pro']['points'], "'"]),
            'depth_image_topic_name': PythonExpression([f"'{configs['sobit_edu']['depth']}' if '", robot_type, "' == 'sobit_edu' else '", configs['sobit_pro']['depth'], "'"]),
            'info_topic_name': PythonExpression([f"'{configs['sobit_edu']['info']}' if '", robot_type, "' == 'sobit_edu' else '", configs['sobit_pro']['info'], "'"]),
            'base_frame_name': PythonExpression([f"'{configs['sobit_edu']['frame']}' if '", robot_type, "' == 'sobit_edu' else '", configs['sobit_pro']['frame'], "'"]),

            'use_3d': 'True',
            'execute_default': 'True',
            'positioning_detection_mode_object': 'fast_point',
            'namespace': 'yolo_ros',
            'threshold': '0.50',
            'weight_file': 'yolo11n.pt'
        }.items()
    )

    return LaunchDescription([
        robot_type_arg,
        yolo_launch
    ])