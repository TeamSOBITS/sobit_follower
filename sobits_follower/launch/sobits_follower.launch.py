from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def _load_ros_parameters(params_path: str) -> dict:
    try:
        with open(params_path, "r", encoding="utf-8") as f:
            params = yaml.safe_load(f) or {}
    except Exception:
        return {}

    return params.get("/**", {}).get("ros__parameters", {}) or {}


def _load_detection_mode(params_path: str) -> str:
    detection_mode = "body_leg"
    try:
        detection_mode = _load_ros_parameters(params_path).get("detection_mode", detection_mode)
    except Exception:
        return detection_mode

    detection_mode = str(detection_mode).strip().lower()
    if detection_mode not in ("body", "leg", "body_leg"):
        detection_mode = "body_leg"
    return detection_mode


def _launch_setup(context):
    sobits_follower_share = FindPackageShare("sobits_follower")

    robot_type = LaunchConfiguration("robot_type")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    body_detector = LaunchConfiguration("body_detector")
    person_tracker_params = LaunchConfiguration("person_tracker_params")
    sensor_rotator_params = LaunchConfiguration("sensor_rotator_params")
    person_following_control_params = LaunchConfiguration("person_following_control_params")
    velocity_smoother_params = LaunchConfiguration("velocity_smoother_params")
    autostart_lifecycle = str(
        LaunchConfiguration("autostart_lifecycle").perform(context)
    ).strip().lower() in ("true", "1", "yes", "on")

    tracker_params_path = person_tracker_params.perform(context)
    detection_mode = _load_detection_mode(tracker_params_path)
    body_detector_value = body_detector.perform(context).strip().lower()

    velocity_smoother_config = _load_ros_parameters(velocity_smoother_params.perform(context))
    use_velocity_smoother = str(
        velocity_smoother_config.get("use_velocity_smoother", True)
    ).strip().lower() in ("true", "1", "yes", "on")
    raw_cmd_vel_topic = str(velocity_smoother_config.get("raw_cmd_vel_topic", "sobits_follower/velocity_smoother/raw_cmd_vel")).strip()
    output_cmd_vel_topic = str(velocity_smoother_config.get("output_cmd_vel_topic", "/cmd_vel")).strip()

    person_following_control_overrides = {}
    if use_velocity_smoother:
        person_following_control_overrides["command_velocity_topic_name"] = raw_cmd_vel_topic
        person_following_control_overrides["stop_command_velocity_topic_name"] = output_cmd_vel_topic

    dr_spaam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                sobits_follower_share,
                "launch",
                "include",
                "dr_spaam_ros.launch.py",
            ])
        ),
        launch_arguments={
            "robot_type": robot_type,
            "dr_spaam_params_file": PathJoinSubstitution([
                sobits_follower_share,
                "param",
                robot_type,
                "dr_spaam_param.yaml",
            ]),
            "auto_configure": "true" if autostart_lifecycle else "false",
            "auto_activate":  "true" if autostart_lifecycle else "false",
        }.items(),
    )

    ssd_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                sobits_follower_share,
                "launch",
                "include",
                "ssd_pose_ros.launch.py",
            ])
        ),
        launch_arguments={"robot_type": robot_type}.items(),
    )

    yolo_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                sobits_follower_share,
                "launch",
                "include",
                "yolo_pose_ros.launch.py",
            ])
        ),
        launch_arguments={
            "robot_type": robot_type,
            "autostart_lifecycle": "true" if autostart_lifecycle else "false",
        }.items(),
    )

    velocity_smoother_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("person_following_control"),
                "launch",
                "velocity_smoother.launch.py",
            ])
        ),
        launch_arguments={
            "use_velocity_smoother": "true" if use_velocity_smoother else "false",
            "velocity_smoother_params": velocity_smoother_params,
            "autostart_lifecycle": "true" if autostart_lifecycle else "false",
        }.items(),
    )

    sobits_follower = ComposableNodeContainer(
        name="sobits_follower_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            # Tracker Component
            ComposableNode(
                package="multiple_sensor_person_tracking",
                plugin="multiple_sensor_person_tracking::PersonTracker",
                name="person_tracker",
                namespace="",
                parameters=[
                    person_tracker_params,
                    {
                        "body_detection_topic_name": "sobits_follower/body_3d_poses"
                    }
                ],
            ),
            # Sensor Rotator Component
            ComposableNode(
                package="multiple_sensor_person_tracking",
                plugin="multiple_sensor_person_tracking::PersonAimSensorRotator",
                name="person_aim_sensor_rotator",
                namespace="",
                parameters=[sensor_rotator_params],
            ),
            # Following Control Component
            ComposableNode(
                package="person_following_control",
                plugin="person_following_control::PersonFollowing",
                name="person_following_control",
                namespace="",
                parameters=[person_following_control_params, person_following_control_overrides],
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

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="sobits_follower_lifecycle_manager",
        namespace="",
        output="screen",
        parameters=[{
            "autostart": autostart_lifecycle,
            "bond_timeout": 0.0,
            "node_names": [
                "person_tracker",
                "person_aim_sensor_rotator",
                "person_following_control",
            ],
        }],
    )

    actions = []
    actions.append(rviz_node)
    if detection_mode != "body":
        actions.append(dr_spaam_launch)
    if detection_mode != "leg":
        if body_detector_value == "ssd":
            actions.append(ssd_ros_launch)
        elif body_detector_value == "yolo":
            actions.append(yolo_ros_launch)
    actions.append(sobits_follower)
    actions.append(lifecycle_manager)
    actions.append(velocity_smoother_launch)
    return actions


def generate_launch_description():
    sobits_follower_share = FindPackageShare("sobits_follower")

    robot_type = LaunchConfiguration("robot_type")

    launch_args = [
        DeclareLaunchArgument(
            "robot_type",
            description="Type of the robot",
            # default_value="sobit_edu",
            # default_value="sobit_pro",
            default_value="sobit_home",
            # default_value="hsrb",
        ), 
        DeclareLaunchArgument(
            "use_rviz",
            description="Whether to launch RViz",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "rviz_cfg",
            description="Path to the RViz configuration file",
            default_value=PathJoinSubstitution([
                sobits_follower_share,
                "config",
                "rviz",
                PythonExpression([
                    "'sobits_follower_' + '",
                    robot_type,
                    "' + '.rviz'",
                ]),
            ]),
        ),
        DeclareLaunchArgument(
            "body_detector",
            default_value="yolo",
            description="Select the body detector type: 'yolo', 'ssd'",
        ),
        DeclareLaunchArgument(
            "person_tracker_params",
            description="Path to the person tracker parameter file",
            default_value=PathJoinSubstitution([
                sobits_follower_share,
                "param",
                robot_type,
                "tracker_param.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "sensor_rotator_params",
            description="Path to the sensor rotator parameter file",
            default_value=PathJoinSubstitution([
                sobits_follower_share,
                "param",
                robot_type,
                "sensor_rotator_param.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "person_following_control_params",
            description="Path to the person following control parameter file",
            default_value=PathJoinSubstitution([
                sobits_follower_share,
                "param",
                robot_type,
                "following_control_param.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "velocity_smoother_params",
            description="Path to the velocity smoother parameter file",
            default_value=PathJoinSubstitution([
                sobits_follower_share,
                "param",
                robot_type,
                "velocity_smoother_param.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "autostart_lifecycle",
            description="Whether to automatically configure and activate sobits_follower lifecycle nodes",
            default_value="true",
        ),
    ]
    return LaunchDescription(launch_args + [OpaqueFunction(function=_launch_setup)])
