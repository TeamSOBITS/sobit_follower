from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    sobit_follower_share = FindPackageShare("sobit_follower")

    robot_type = LaunchConfiguration("robot_type")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    body_detector = LaunchConfiguration("body_detector")
    person_tracker_params = LaunchConfiguration("person_tracker_params")
    sensor_rotator_params = LaunchConfiguration("sensor_rotator_params")
    person_following_control_params = LaunchConfiguration("person_following_control_params")

    launch_args = [
        DeclareLaunchArgument(
            "robot_type",
            description="Type of the robot",
            default_value="sobit_edu",
            # default_value="sobit_pro",
            # default_value="hsrb",
        ), 
        DeclareLaunchArgument(
            "use_rviz", 
            description="Whether to launch RViz",
            default_value="true"
        ),
        DeclareLaunchArgument(
            "rviz_cfg", 
            description="Path to the RViz configuration file",
            default_value=PathJoinSubstitution([sobit_follower_share, "config", "rviz","sobit_follower_hsrb.rviz"])
        ),
        DeclareLaunchArgument(
            "body_detector", 
            default_value="yolo", 
            description="Select the body detector type: 'yolo', 'ssd'"
        ),
        DeclareLaunchArgument(
            "person_tracker_params", 
            description="Path to the person tracker parameter file",
            default_value=PathJoinSubstitution([
                sobit_follower_share, 
                "param", 
                robot_type, 
                "tracker_param.yaml"
            ])
        ),
        DeclareLaunchArgument(
            "sensor_rotator_params", 
            description="Path to the sensor rotator parameter file",
            default_value=PathJoinSubstitution([
                sobit_follower_share, 
                "param", 
                robot_type, 
                "sensor_rotator_param.yaml"
            ])
        ),
        DeclareLaunchArgument(
            "person_following_control_params", 
            description="Path to the person following control parameter file",
            default_value=PathJoinSubstitution([
                sobit_follower_share, 
                "param", 
                robot_type, 
                "following_control_param.yaml"
            ])
        ),
    ]

    # DR-SPAAM launch includes
    dr_spaam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                sobit_follower_share, 
                "launch", 
                "include",
                "dr_spaam_ros.launch.py"])
        ),
        launch_arguments={
            'robot_type': robot_type,
        }.items()
    )
    

    # SSD launch includes
    ssd_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                sobit_follower_share,
                "launch",
                "include",
                "ssd_pose_ros.launch.py",
            ])
        ),
        launch_arguments={
            'robot_type': robot_type,
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", body_detector, "'.lower() == 'ssd'"])
        )
    )

    # YOLO launch includes
    yolo_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                sobit_follower_share,
                "launch",
                "include",
                "yolo_pose_ros.launch.py",
            ])
        ),
        launch_arguments={
            'robot_type': robot_type,
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", body_detector, "'.lower() == 'yolo'"])
        )
    )

    # Component Container
    sobits_follower = ComposableNodeContainer(
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
                namespace="sobit_follower",
                parameters=[
                    person_tracker_params,
                    {
                        "body_detection_topic_name": "/sobit_follower/body_3d_poses"
                    }
                ],
            ),
            # Sensor Rotator Component
            ComposableNode(
                package="multiple_sensor_person_tracking",
                plugin="multiple_sensor_person_tracking::PersonAimSensorRotator",
                name="person_aim_sensor_rotator",
                namespace="sobit_follower",
                parameters=[sensor_rotator_params],
            ),
            # Following Control Component
            ComposableNode(
                package="person_following_control",
                plugin="person_following_control::PersonFollowing",
                name="person_following_control",
                namespace="sobit_follower",
                parameters=[person_following_control_params],
            ),
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription(
        launch_args + [
            rviz_node,
            ssd_ros_launch,
            yolo_ros_launch,
            dr_spaam_launch,
            sobits_follower,
        ]
    )
