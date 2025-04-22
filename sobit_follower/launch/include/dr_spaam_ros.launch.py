from launch import LaunchDescription
from launch.actions import GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        GroupAction([
            Node(
                package="dr_spaam_ros",
                executable="dr_spaam_ros",
                name="dr_spaam_ros",
                namespace="dr_spaam",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare('sobit_follower'), "param", 'dr_spaam_param.yaml'])
                ]
            )
        ])
    ])