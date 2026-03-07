from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    robot_type = LaunchConfiguration("robot_type")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_type",
            default_value="sobit_edu",
            description="Type of robot for selecting DR-SPAAM params",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("sobit_follower"),
                "param",
                robot_type,
                "dr_spaam_param.yaml",
            ]),
            description="Path to DR-SPAAM parameter file",
        ),
        Node(
            package="dr_spaam_ros",
            executable="dr_spaam_ros",
            name="dr_spaam_ros",
            namespace="dr_spaam",
            output="screen",
            parameters=[params_file],
        ),
    ])
