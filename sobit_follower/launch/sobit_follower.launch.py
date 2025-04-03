from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import os

def generate_launch_description():
    robot_type = LaunchConfiguration("robot_type")
    use_rviz = LaunchConfiguration("rviz")
    use_rotate = LaunchConfiguration("use_rotate")
    following_method = LaunchConfiguration("following_method")
    use_smoother = LaunchConfiguration("use_smoother")
    rviz_cfg = LaunchConfiguration("rviz_cfg")

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument("robot_type", default_value="sobit_edu"),
            # sobit_edu
            # sobit_pro
            # hsrb
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("rviz_cfg", default_value=os.path.join(
            os.getenv("HOME"), "colcon_ws/src/sobit_follower/config/rviz/sobit_follower.rviz")), #TODO
        DeclareLaunchArgument("use_rotate", default_value="true"),
        DeclareLaunchArgument("following_method", default_value="0"),
            # 0 : VirtualSpringModel-DynamicWindowApproach
            # 1 : VirtualSpringModel
            # 2 : DynamicWindowApproach
            # 3 : PIDController
        DeclareLaunchArgument("use_smoother", default_value="true"),

        # RViz Node
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", rviz_cfg],
            output="screen"
        ),

        # Tracker & SSD pose launch includes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    os.getenv("HOME"),
                    "colcon_ws/src/sobit_follower/launch/include/dr_spaam_ros.launch.py"
                )
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    os.getenv("HOME"),
                    "colcon_ws/src/sobit_follower/launch/include/ssd_pose_ros.launch.py"
                )
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

                # Sensor Rotator Component
                ComposableNode(
                    condition=IfCondition(LaunchConfiguration("robot_type").perform(None) == "sobit_edu"),
                    package="multiple_sensor_person_tracking",
                    plugin="multiple_sensor_person_tracking::SobitEduPersonAimSensorRotator",
                    name="sensor_rotator",
                    parameters=[{
                        "use_rotate": use_rotate,
                        "display_marker": use_rviz,
                    }]
                ),
                ComposableNode(
                    condition=IfCondition(LaunchConfiguration("robot_type").perform(None) == "sobit_pro"),
                    package="multiple_sensor_person_tracking",
                    plugin="multiple_sensor_person_tracking::SobitProPersonAimSensorRotator",
                    name="sensor_rotator",
                    parameters=[{
                        "use_rotate": use_rotate,
                        "display_marker": use_rviz,
                    }]
                ),
                ComposableNode(
                    condition=IfCondition(LaunchConfiguration("robot_type").perform(None) == "hsrb"),
                    package="multiple_sensor_person_tracking",
                    plugin="multiple_sensor_person_tracking::HSRbPersonAimSensorRotator",
                    name="sensor_rotator",
                    parameters=[{
                        "use_rotate": use_rotate,
                        "display_marker": use_rviz,
                    }]
                ),

                # Following control node
                ComposableNode(
                    package="person_following_control",
                    plugin="person_following_control::PersonFollowing",
                    name="following_control",
                    parameters=[{
                        "following_method": following_method,
                    }]
                ),
            ]
        )
    ])
