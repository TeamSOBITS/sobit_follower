from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import yaml


def _load_detection_mode(params_path: str) -> str:
    detection_mode = "body_leg"
    try:
        with open(params_path, "r", encoding="utf-8") as f:
            params = yaml.safe_load(f) or {}
        detection_mode = params.get("/**", {}).get("ros__parameters", {}).get("detection_mode", detection_mode)
    except Exception:
        return detection_mode

    detection_mode = str(detection_mode).strip().lower()
    if detection_mode not in ("body", "leg", "body_leg"):
        detection_mode = "body_leg"
    return detection_mode


def _launch_setup(context):
    sobit_follower_share = FindPackageShare("sobit_follower")

    robot_type = LaunchConfiguration("robot_type")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    person_tracker_params = LaunchConfiguration("person_tracker_params")
    sensor_rotator_params = LaunchConfiguration("sensor_rotator_params")
    person_following_control_params = LaunchConfiguration("person_following_control_params")

    tracker_params_path = person_tracker_params.perform(context)
    detection_mode = _load_detection_mode(tracker_params_path)

    dr_spaam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                sobit_follower_share,
                "launch",
                "include",
                "dr_spaam_ros.launch.py",
            ])
        ),
        launch_arguments={
            "robot_type": robot_type,
            "params_file": PathJoinSubstitution([
                sobit_follower_share,
                "param",
                robot_type,
                "dr_spaam_param.yaml",
            ]),
        }.items(),
    )

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
            "robot_type": robot_type,
            "params_file": PathJoinSubstitution([
                sobit_follower_share,
                "param",
                robot_type,
                "ssd_param.yaml",
            ]),
        }.items(),
    )

    sobits_follower = ComposableNodeContainer(
        name="sobit_follower_container",
        namespace="sobit_follower",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="multiple_sensor_person_tracking",
                plugin="multiple_sensor_person_tracking::PersonTracker",
                name="person_tracker",
                namespace="sobit_follower",
                parameters=[person_tracker_params],
            ),
            ComposableNode(
                package="multiple_sensor_person_tracking",
                plugin="multiple_sensor_person_tracking::PersonAimSensorRotator",
                name="person_aim_sensor_rotator",
                namespace="sobit_follower",
                parameters=[sensor_rotator_params],
            ),
            ComposableNode(
                package="person_following_control",
                plugin="person_following_control::PersonFollowing",
                name="person_following_control",
                namespace="sobit_follower",
                parameters=[person_following_control_params],
            ),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(use_rviz),
    )

    actions = [rviz_node]
    if detection_mode != "body":
        actions.append(dr_spaam_launch)
    if detection_mode != "leg":
        actions.append(ssd_ros_launch)
    actions.append(sobits_follower)
    return actions

def generate_launch_description():
    sobit_follower_share = FindPackageShare("sobit_follower")

    robot_type = LaunchConfiguration("robot_type")

    launch_args = [
        DeclareLaunchArgument(
            "robot_type",
            description="Type of the robot",
            # default_value="sobit_edu",
            # default_value="sobit_pro",
            default_value="hsrb",
        ), 
        DeclareLaunchArgument(
            "use_rviz", 
            description="Whether to launch RViz",
            default_value="true"
        ),
        DeclareLaunchArgument(
            "rviz_cfg", 
            description="Path to the RViz configuration file",
            # Auto-select RViz config by robot_type (still overridable via rviz_cfg:=...)
            default_value=PathJoinSubstitution([
                sobit_follower_share,
                "config",
                "rviz",
                PythonExpression([
                    "'sobit_follower_' + '",
                    robot_type,
                    "' + '.rviz'"
                ]),
            ])
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
    return LaunchDescription(launch_args + [OpaqueFunction(function=_launch_setup)])
