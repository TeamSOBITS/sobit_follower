import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='sobit_edu',
        description='Robot type for selecting YOLO config.'
    )

    # YOLOのコアとなる launch ファイルを呼び出し、引数でトピック名などを上書きする
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yolo_ros'),
                'launch',
                'yolo.launch.py' # yolo_ros パッケージ内の yolo.launch.py へのパス
            )
        ),
        launch_arguments={
            # SOBIT PRO用のカメラトピック設定 (元のssd_param.yamlの値を参照)
            'image_topic_name': '/sobit_pro/head_camera/rgb/image_raw',
            'point_cloud_topic': '/sobit_pro/head_camera/depth_registered/points',
            'depth_image_topic_name': '/sobit_pro/head_camera/depth/image_raw',
            'info_topic_name': '/sobit_pro/head_camera/rgb/camera_info',
            'base_frame_name': 'sobit_pro/base_footprint',

        # launch_arguments={
        #     # SOBIT EDU用のカメラトピック設定に変更
        #     'image_topic_name': '/sobit_edu/head_camera/rgb/image_raw',
        #     'point_cloud_topic': '/sobit_edu/head_camera/depth_registered/points',
        #     'depth_image_topic_name': '/sobit_edu/head_camera/depth/image_raw',
        #     'info_topic_name': '/sobit_edu/head_camera/rgb/camera_info',
        #     'base_frame_name': 'sobit_edu/base_footprint',

            # YOLOと3D変換の設定
            'use_3d': 'True',
            'execute_default': 'True',
            'positioning_detection_mode_object': 'fast_point',
            'namespace': 'yolo_ros',
            'threshold': '0.50',
            # 使用するモデルに合わせて適宜変更してください (例: yolo11n.pt)
            'weight_file': 'yolo11n.pt'
        }.items()
    )

    return LaunchDescription([
        robot_type_arg,
        yolo_launch
    ])