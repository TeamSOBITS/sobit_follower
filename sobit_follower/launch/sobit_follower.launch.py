from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import os

def generate_launch_description():

    sobit_follower_share = FindPackageShare("sobit_follower")

    # Launch Arguments
    DeclareLaunchArgument("robot_type", default_value="sobit_edu"), 
        # sobit_edu
        # sobit_pro
        # hsrb
    DeclareLaunchArgument("rviz", default_value="false"),
    DeclareLaunchArgument("rviz_cfg", default_value=PathJoinSubstitution([sobit_follower_share, "config", "rviz","sobit_follower.rviz"])),
    DeclareLaunchArgument("use_rotate", default_value="true"),
    DeclareLaunchArgument("following_method", default_value="0"),
        # 0 : VirtualSpringModel-DynamicWindowApproach
        # 1 : VirtualSpringModel
        # 2 : DynamicWindowApproach
        # 3 : PIDController
    DeclareLaunchArgument("use_smoother", default_value="true"),

    robot_type = LaunchConfiguration("robot_type")
    use_rviz = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    use_rotate = LaunchConfiguration("use_rotate")
    following_method = LaunchConfiguration("following_method")
    use_smoother = LaunchConfiguration("use_smoother")

    person_tracker_params = [
        PathJoinSubstitution([sobit_follower_share, "param", robot_type, "tracker_param.yaml"]),
    ]
    sensor_rotator_params = [
        PathJoinSubstitution([sobit_follower_share, "param", robot_type, "sensor_rotator_param.yaml"]),
        {
            "use_rotate": use_rotate,
            "display_marker": use_rviz,
        }
    ]

    person_following_control_params = [
        PathJoinSubstitution([sobit_follower_share, "param", robot_type, "following_control_param.yaml"]),
        {
            "following_method": following_method,
            "use_smoother": use_smoother,
        }
    ]

    return LaunchDescription([ 

        # RViz Node
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", rviz_cfg],
            output="screen"
        ),

        # DR-SPAAM & SSD launch includes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sobit_follower_share, "launch", "include","dr_spaam_ros.launch.py"])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sobit_follower_share, "launch", "include","ssd_pose_ros.launch.py"])
            )
        ),

        # Component Container
        ComposableNodeContainer(
            name="sobit_follower_container",
            namespace="sobit_follower",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                # Tracker Component
                ComposableNode(
                    package="multiple_sensor_person_tracking",
                    plugin="multiple_sensor_person_tracking::PersonTracker",
                    name="person_tracker",
                    parameters=person_tracker_params,
                ),
                # Sensor Rotator Component
                ComposableNode(
                    package="multiple_sensor_person_tracking",
                    plugin="multiple_sensor_person_tracking::PersonAimSensorRotator",
                    name="person_aim_sensor_rotator",
                    parameters=sensor_rotator_params,
                ),
                # Following Control Component
                ComposableNode(
                    package="person_following_control",
                    plugin="person_following_control::PersonFollowing",
                    name="person_following_control",
                    parameters=person_following_control_params,
                ),
            ]
        )
    ])
