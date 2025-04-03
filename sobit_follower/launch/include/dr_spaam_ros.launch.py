from launch import LaunchDescription
from launch.actions import GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    weight_file = LaunchConfiguration("weight_file")

    return LaunchDescription([
        GroupAction([
            Node(
                package="dr_spaam_ros",
                executable="node.py",
                name="dr_spaam_ros",
                namespace="dr_spaam",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare('sobit_follower'), "launch", 'config', 'dr_spaam_param.yaml'])
                ]
            )
        ])
    ])