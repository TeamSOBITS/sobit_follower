from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
import yaml


def _load_ros_parameters(params_path: str) -> dict:
    try:
        with open(params_path, "r", encoding="utf-8") as f:
            params = yaml.safe_load(f) or {}
    except Exception:
        return {}

    return params.get("/**", {}).get("ros__parameters", {}) or {}


def _resolve_remap_topic(topic: str, namespace: str) -> str:
    topic = str(topic).strip()
    namespace = str(namespace).strip("/")
    if not topic or topic.startswith("/"):
        return topic
    if namespace and (topic == namespace or topic.startswith(namespace + "/")):
        return "/" + topic
    return topic


def _launch_setup(context):
    use_velocity_smoother = str(LaunchConfiguration("use_velocity_smoother").perform(context)).strip().lower() in ("true", "1", "yes", "on")
    if not use_velocity_smoother:
        return []

    config = _load_ros_parameters(LaunchConfiguration("velocity_smoother_params").perform(context))

    speed_lim_v = float(config.get("speed_lim_v", 0.8))
    speed_lim_w = float(config.get("speed_lim_w", 5.4))
    accel_lim_v = float(config.get("accel_lim_v", 0.3))
    accel_lim_w = float(config.get("accel_lim_w", 2.0))
    decel_factor = float(config.get("decel_factor", 1.0))
    robot_feedback = int(config.get("robot_feedback", 0))
    scale_velocities = str(config.get("scale_velocities", False)).strip().lower() in ("true", "1", "yes", "on")
    enable_stamped_cmd_vel = str(config.get("enable_stamped_cmd_vel", False)).strip().lower() in ("true", "1", "yes", "on")
    use_realtime_priority = str(config.get("use_realtime_priority", False)).strip().lower() in ("true", "1", "yes", "on")

    nav2_velocity_smoother_params = {
        "smoothing_frequency": float(config.get("frequency", 20.0)),
        "scale_velocities": scale_velocities,
        "feedback": "CLOSED_LOOP" if robot_feedback != 0 else "OPEN_LOOP",
        "max_velocity": [speed_lim_v, 0.0, speed_lim_w],
        "min_velocity": [-speed_lim_v, 0.0, -speed_lim_w],
        "max_accel": [accel_lim_v, 0.0, accel_lim_w],
        "max_decel": [-accel_lim_v * decel_factor, 0.0, -accel_lim_w * decel_factor],
        "deadband_velocity": [
            float(config.get("deadband_velocity_x", 0.0)),
            float(config.get("deadband_velocity_y", 0.0)),
            float(config.get("deadband_velocity_theta", 0.0)),
        ],
        "velocity_timeout": float(config.get("velocity_timeout", 1.0)),
        "odom_topic": str(config.get("odom_topic", "/odom")),
        "enable_stamped_cmd_vel": enable_stamped_cmd_vel,
        "use_realtime_priority": use_realtime_priority,
    }

    namespace = str(config.get("namespace", "sobits_follower"))
    raw_cmd_vel_topic = _resolve_remap_topic(
        config.get("raw_cmd_vel_topic", "sobits_follower/velocity_smoother/raw_cmd_vel"),
        namespace,
    )
    output_cmd_vel_topic = _resolve_remap_topic(
        config.get("output_cmd_vel_topic", "/cmd_vel"),
        namespace,
    )

    return [
        LifecycleNode(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            namespace=namespace,
            output="screen",
            parameters=[nav2_velocity_smoother_params],
            remappings=[
                ("cmd_vel", raw_cmd_vel_topic),
                ("cmd_vel_smoothed", output_cmd_vel_topic),
            ],
            condition=IfCondition(LaunchConfiguration("use_velocity_smoother")),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="velocity_smoother_manager",
            namespace=namespace,
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["velocity_smoother"],
            }],
            condition=IfCondition(LaunchConfiguration("use_velocity_smoother")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_velocity_smoother",
            default_value="true",
            description="Whether to launch the velocity smoother wrapper",
        ),
        DeclareLaunchArgument(
            "velocity_smoother_params",
            default_value="",
            description="Path to the velocity smoother parameter file",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
