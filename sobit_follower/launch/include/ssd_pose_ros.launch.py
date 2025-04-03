import os
import yaml
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    return LaunchDescription([
        GroupAction([
            Node(
                package="ssd_ros",
                executable="single_shot_multibox_detector",
                name="ssd_node",
                output="screen",
                parameters=[PathJoinSubstitution([FindPackageShare("sobit_follower"), "param", "ssd_param.yaml"])]
            )
        ])
    ])