from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    weight_file = LaunchConfiguration("weight_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "weight_file",
            default_value=os.path.join(
                os.getenv("HOME"),
                "colcon_ws/src/sobit_follower/weights/ckpt_jrdb_ann_ft_dr_spaam_e20.pth"
            )
        ),

        GroupAction([
            Node(
                package="dr_spaam_ros",
                executable="node.py",
                name="dr_spaam_ros",
                namespace="dr_spaam",
                output="screen",
                parameters=[
                    {"weight_file": weight_file},
                    os.path.join(
                        os.getenv("HOME"),
                        "colcon_ws/src/sobit_follower/param/dr_spaam_param.yaml"
                    )
                ]
            )
        ])
    ])
