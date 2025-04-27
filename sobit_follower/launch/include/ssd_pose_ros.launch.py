from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ssd_ros_pkg = FindPackageShare("ssd_ros")
    ssd_opl_cml = FindPackageShare("robocup_opl_cml")
    ssd_launch_file = PathJoinSubstitution([ssd_opl_cml, "launch", "ssd_ros.launch.py"])

    return LaunchDescription([

        DeclareLaunchArgument("image_topic_name", default_value="/sobit_pro/head_camera/rgb/image_raw"),
        DeclareLaunchArgument("cloud_topic_name", default_value="/sobit_pro/head_camera/depth_registered/points"),
        DeclareLaunchArgument("in_scale_factor", default_value="0.007843"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.5"),
        DeclareLaunchArgument("ssd_prototxt_name", default_value=PathJoinSubstitution([ssd_ros_pkg, "models", "voc_object.prototxt"])),
        DeclareLaunchArgument("ssd_caffemodel_name", default_value=PathJoinSubstitution([ssd_ros_pkg, "models", "voc_object.caffemodel"])),
        DeclareLaunchArgument("ssd_class_names_file", default_value=PathJoinSubstitution([ssd_ros_pkg, "models", "voc_object_names.txt"])),
        DeclareLaunchArgument("object_specified_enabled", default_value="true"),
        DeclareLaunchArgument("specified_object_name", default_value="person"),
        DeclareLaunchArgument("image_show_flag", default_value="false"),
        DeclareLaunchArgument("execute_default", default_value="true"),
        DeclareLaunchArgument("namespace", default_value="ssd_ros"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ssd_launch_file),
            launch_arguments={
                "image_topic_name": LaunchConfiguration("image_topic_name"),
                "cloud_topic_name": LaunchConfiguration("cloud_topic_name"),
                "in_scale_factor": LaunchConfiguration("in_scale_factor"),
                "confidence_threshold": LaunchConfiguration("confidence_threshold"),
                "ssd_prototxt_name": LaunchConfiguration("ssd_prototxt_name"),
                "ssd_caffemodel_name": LaunchConfiguration("ssd_caffemodel_name"),
                "ssd_class_names_file": LaunchConfiguration("ssd_class_names_file"),
                "object_specified_enabled": LaunchConfiguration("object_specified_enabled"),
                "specified_object_name": LaunchConfiguration("specified_object_name"),
                "image_show_flag": LaunchConfiguration("image_show_flag"),
                "execute_default": LaunchConfiguration("execute_default"),
                "namespace": LaunchConfiguration("namespace"),
            }.items()
        )
    ])