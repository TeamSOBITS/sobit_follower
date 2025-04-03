import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ssd_share = FindPackageShare("ssd_ros")

    ssd_params = PathJoinSubstitution(["sobit_follower", "param", "ssd_param.yaml"])
    model_path = PathJoinSubstitution([ssd_share, "models"])

    return LaunchDescription([
        GroupAction([
            Node(
                package="ssd_ros",
                executable="single_shot_multibox_detector",
                name="ssd_node",
                output="screen",
                parameters=[
                    ssd_params,
                    {
                        "ssd_prototxt_name": PathJoinSubstitution([model_path, "voc_object.prototxt"]),
                        "ssd_caffemodel_name": PathJoinSubstitution([model_path, "voc_object.caffemodel"]),
                        "ssd_class_names_file": PathJoinSubstitution([model_path, "voc_object_names.txt"]),
                    }
                ]
            )
        ])
    ])
