import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    # モデルファイルのパス設定
    ssd_ros_share = get_package_share_directory('ssd_ros')
    voc_object_prototxt_path = os.path.join(ssd_ros_share, 'models', 'voc_object.prototxt')
    voc_object_caffemodel_path = os.path.join(ssd_ros_share, 'models', 'voc_object.caffemodel')
    voc_object_names_path = os.path.join(ssd_ros_share, 'models', 'voc_object_names.txt')

    # --- 引数の定義 ---
    robot_type = LaunchConfiguration("robot_type")
    robot_type_cmd = DeclareLaunchArgument(
        "robot_type",
        default_value="sobit_edu",
        description="Type of the robot (sobit_edu, sobit_pro, hsrb)"
    )

    image_show_flag = LaunchConfiguration("image_show_flag")
    image_show_flag_cmd = DeclareLaunchArgument(
        "image_show_flag",
        default_value="true",
        description="is image show?"
    )

    execute_default = LaunchConfiguration("execute_default")
    execute_default_cmd = DeclareLaunchArgument(
        "execute_default",
        default_value="true",
        description="Whether to start SSD enabled"
    )

    base_frame_name = LaunchConfiguration("base_frame_name")
    base_frame_name_cmd = DeclareLaunchArgument(
        "base_frame_name",
        default_value="base_footprint",
        description="Base frame name for TF and 3D detection"
    )

    # --- トピック名の動的切り替え設定 ---
    # HSRBかそれ以外(SOBIT系)かでデフォルト値を分岐
    image_topic_name = LaunchConfiguration("image_topic_name")
    image_topic_name_cmd = DeclareLaunchArgument(
        "image_topic_name",
        default_value=PythonExpression([
            "'/hsrb/head_rgbd_sensor/rgb/image_raw' if '", robot_type, "' == 'hsrb' else '/camera/rgb/image_raw'"
        ])
    )

    point_cloud_topic_name = LaunchConfiguration("point_cloud_topic_name")
    point_cloud_topic_name_cmd = DeclareLaunchArgument(
        "point_cloud_topic_name",
        default_value=PythonExpression([
            "'/hsrb/head_rgbd_sensor/depth_registered/rectified_points' if '", robot_type, "' == 'hsrb' else '/camera/depth_registered/points'"
        ])
    )

    depth_image_topic_name = LaunchConfiguration("depth_image_topic_name")
    depth_image_topic_name_cmd = DeclareLaunchArgument(
        "depth_image_topic_name",
        default_value=PythonExpression([
            "'/hsrb/head_rgbd_sensor/depth_registered/image_raw' if '", robot_type, "' == 'hsrb' else '/camera/depth/image_raw'"
        ])
    )

    info_topic_name = LaunchConfiguration("info_topic_name")
    info_topic_name_cmd = DeclareLaunchArgument(
        "info_topic_name",
        default_value=PythonExpression([
            "'/hsrb/head_rgbd_sensor/rgb/camera_info' if '", robot_type, "' == 'hsrb' else '/camera/rgb/camera_info'"
        ])
    )

    # --- その他のパラメータ ---
    in_scale_factor = LaunchConfiguration("in_scale_factor")
    in_scale_factor_cmd = DeclareLaunchArgument(
        "in_scale_factor",
        default_value="0.007843",
        description="Caffemodelで扱う際の変換時スケールパラメータ"
    )

    confidence_threshold = LaunchConfiguration("confidence_threshold")
    confidence_threshold_cmd = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.5",
        description="Minimum probability of a detection to be published"
    )

    ssd_prototxt_name = LaunchConfiguration("ssd_prototxt_name")
    ssd_prototxt_name_cmd = DeclareLaunchArgument(
        "ssd_prototxt_name",
        default_value=voc_object_prototxt_path
    )

    ssd_caffemodel_name = LaunchConfiguration("ssd_caffemodel_name")
    ssd_caffemodel_name_cmd = DeclareLaunchArgument(
        "ssd_caffemodel_name",
        default_value=voc_object_caffemodel_path
    )

    ssd_class_names_file = LaunchConfiguration("ssd_class_names_file")
    ssd_class_names_file_cmd = DeclareLaunchArgument(
        "ssd_class_names_file",
        default_value=voc_object_names_path
    )

    object_specified_enabled = LaunchConfiguration("object_specified_enabled")
    object_specified_enabled_cmd = DeclareLaunchArgument(
        "object_specified_enabled",
        default_value='true'
    )

    specified_object_name = LaunchConfiguration("specified_object_name")
    specified_object_name_cmd = DeclareLaunchArgument(
        "specified_object_name",
        default_value='person'
    )

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="ssd_ros"
    )

    positioning_detection_mode = LaunchConfiguration("positioning_detection_mode")
    positioning_detection_mode_cmd = DeclareLaunchArgument(
        "positioning_detection_mode",
        default_value="fast_point"
    )

    # --- ノードの定義 ---
    ssd_ros_node_cmd = Node(
        package="ssd_ros",
        executable="single_shot_multibox_detector",
        name="ssd_ros",
        namespace=namespace,
        parameters=[
            {
                "image_show_flag": image_show_flag,
                "execute_default": execute_default,
                "image_topic_name": image_topic_name,
                "in_scale_factor": in_scale_factor,
                "confidence_threshold": confidence_threshold,
                "ssd_prototxt_name": ssd_prototxt_name,
                "ssd_caffemodel_name": ssd_caffemodel_name,
                "ssd_class_names_file": ssd_class_names_file,
                "object_specified_enabled": object_specified_enabled,
                "specified_object_name": specified_object_name,
            },
        ],
        output="screen"
    )

    use_3d = LaunchConfiguration("use_3d")
    use_3d_cmd = DeclareLaunchArgument(
        "use_3d", default_value="true", description="Whether to activate 3D detections"
    )

    bbox_to_3d_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("image_to_position"),
                "launch",
                "bbox_to_3d.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "base_frame_name": base_frame_name,
            "bbox_topic_name": "/ssd_ros/objects_rect",
            "cloud_topic_name": point_cloud_topic_name,
            "depth_image_topic_name": depth_image_topic_name,
            "info_topic_name": info_topic_name,
            "execute_default": execute_default,
            "enable_id": "False",
            "positioning_detection_mode": positioning_detection_mode,
        }.items(),
        condition=IfCondition(use_3d),
    )

    return LaunchDescription(
        [
            robot_type_cmd,
            image_show_flag_cmd,
            execute_default_cmd,
            image_topic_name_cmd,
            point_cloud_topic_name_cmd,
            depth_image_topic_name_cmd,
            info_topic_name_cmd,
            in_scale_factor_cmd,
            confidence_threshold_cmd,
            ssd_prototxt_name_cmd,
            ssd_caffemodel_name_cmd,
            ssd_class_names_file_cmd,
            object_specified_enabled_cmd,
            specified_object_name_cmd,
            namespace_cmd,
            positioning_detection_mode_cmd,
            base_frame_name_cmd,
            ssd_ros_node_cmd,
            use_3d_cmd,
            bbox_to_3d_cmd,
        ]
    )
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
# from launch import LaunchDescription


# def generate_launch_description():
#     robot_type = LaunchConfiguration('robot_type')
#     default_param_file = PathJoinSubstitution([
#         FindPackageShare('sobit_follower'),
#         'param',
#         robot_type,
#         'ssd_param.yaml',
#     ])
#     robot_type_arg = DeclareLaunchArgument(
#         'robot_type',
#         default_value='hsrb',
#         description='Robot type for selecting SSD param file.',
#     )

#     params_file_arg = DeclareLaunchArgument(
#         'params_file',
#         default_value=default_param_file,
#         description='Full path to the SSD parameter file.'
#     )

#     params_file = LaunchConfiguration('params_file')

#     ssd_node = Node(
#         package='ssd_ros',
#         executable='single_shot_multibox_detector',
#         name='ssd_ros',
#         parameters=[
#             params_file, 
#             {
#                 "ssd_prototxt_name": os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object.prototxt'),
#                 "ssd_caffemodel_name": os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object.caffemodel'),
#                 "ssd_class_names_file": os.path.join(get_package_share_directory('ssd_ros'), 'models', 'voc_object_names.txt'),    
#             },
#         ],
#     )

#     bbox_to_3d_cmd = Node(
#         package='image_to_position',
#         executable='bbox_to_3d',
#         name='bbox_to_3d',
#         namespace='ssd_ros',
#         output='screen',
#         parameters=[
#             params_file
#         ]
#     )

#     return LaunchDescription([
#         robot_type_arg,
#         params_file_arg,
#         ssd_node,
#         bbox_to_3d_cmd
#     ])
