from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_type = LaunchConfiguration("robot_type")
    dr_spaam_params_file = LaunchConfiguration("dr_spaam_params_file")
    auto_configure = LaunchConfiguration("auto_configure")
    auto_activate = LaunchConfiguration("auto_activate")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_type",
            default_value="sobit_edu",
            description="Type of robot for selecting DR-SPAAM params",
        ),
        DeclareLaunchArgument(
            "dr_spaam_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("sobits_follower"),
                "param",
                robot_type,
                "dr_spaam_param.yaml",
            ]),
            description="Path to DR-SPAAM parameter file",
        ),
        DeclareLaunchArgument(
            "auto_configure",
            default_value="true",
            description="Automatically configure the dr_spaam_ros lifecycle node on startup",
        ),
        DeclareLaunchArgument(
            "auto_activate",
            default_value="true",
            description="Automatically activate the dr_spaam_ros lifecycle node on startup",
        ),
        Node(
            package="dr_spaam_ros",
            executable="dr_spaam_ros",
            name="dr_spaam_ros",
            namespace="dr_spaam",
            output="screen",
            parameters=[
                dr_spaam_params_file,
                {
                    "auto_configure": ParameterValue(auto_configure, value_type=bool),
                    "auto_activate":  ParameterValue(auto_activate,  value_type=bool),
                },
            ],
        ),
    ])
