#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "multiple_sensor_person_tracking/msg/following_position.hpp"
#include "multiple_observation_kalman_filter/multiple_observation_kalman_filter.hpp"

using multiple_sensor_person_tracking::msg::FollowingPosition;

namespace multiple_sensor_person_tracking {
    class PersonAimSensorRotator : public rclcpp::Node {
        private:
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
            rclcpp::Subscription<multiple_sensor_person_tracking::msg::FollowingPosition>::SharedPtr sub_following_position_;

			tf2_ros::Buffer tfBuffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_sub_;

			std::shared_ptr<geometry_msgs::msg::Point> tracking_position_;
			double pre_tilt_;
			double pre_pan_;
			double tilt_angle_min_;
			double tilt_angle_max_;
			double person_height_;
			double smoothing_gain_;
			bool use_rotate_;
			bool use_smoothing_;
			bool display_marker_;
            std::string head_pan_joint_name_;
            std::string head_tilt_joint_name_;

			void makeMarker( const double pan_angle, const double tilt_angle, const double distance );
            void callbackData (
                const std::shared_ptr<const FollowingPosition> &following_position_msg
            );

        public:
            explicit PersonAimSensorRotator(const rclcpp::NodeOptions & options)
            : rclcpp::Node("person_aim_sensor_rotator", options),
            tfBuffer_(this->get_clock()),
            tf_sub_(std::make_shared<tf2_ros::TransformListener>(tfBuffer_))
            {
                onInit();
            }

            void onInit();
    };
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::makeMarker( const double pan_angle, const double tilt_angle, const double distance ) {
	visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "sensor_direction";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
	marker.scale.x = distance; marker.scale.y = 0.05; marker.scale.z = 0.05;
	marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
	marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.8;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, -tilt_angle, pan_angle);
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(quat_tf, quat_msg);
    marker.pose.orientation = quat_msg;
    pub_marker_->publish ( marker );
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::callbackData (
    const std::shared_ptr<const multiple_sensor_person_tracking::msg::FollowingPosition> &following_position_msg) {
	geometry_msgs::msg::Point pt;
    if ( use_smoothing_ ) {
        tracking_position_->x = smoothing_gain_ * tracking_position_->x + ( 1.0 - smoothing_gain_ ) * following_position_msg->rotation_position.x;
        tracking_position_->y = smoothing_gain_ * tracking_position_->y + ( 1.0 - smoothing_gain_ ) * following_position_msg->rotation_position.y;
    } else *tracking_position_ = following_position_msg->rotation_position;

    pt = *tracking_position_;
	double distance = std::hypotf( pt.x, pt.y );
	double angle = std::atan2( pt.y, pt.x );
	double pan_angle, tilt_angle;
	double sec = ( distance == 0.0 ) ? 0.5 : 0.05;

	tilt_angle = std::atan2( person_height_, distance );
	if ( distance == 0.0 ) tilt_angle = 0.2;
	else if ( tilt_angle < tilt_angle_min_ ) tilt_angle = tilt_angle_min_;
	else if ( tilt_angle > tilt_angle_max_ ) tilt_angle = tilt_angle_max_;
    pan_angle = angle;

	RCLCPP_INFO(this->get_logger(), "\033[1mRotator\033[m               :\tpan = %8.3f[deg],\ttilt = %8.3f [deg]", pan_angle*180/M_PI, tilt_angle*180/M_PI);

    if ( use_rotate_ ) {
        auto goal_msg = sobits_interfaces::action::MoveJoint::Goal();
        goal_msg.target_joint_names = { head_pan_joint_name_, head_tilt_joint_name_ };
        goal_msg.target_joint_rad = { pan_angle, tilt_angle };
        goal_msg.time_allowance.sec = static_cast<int>(sec);
        goal_msg.time_allowance.nanosec = static_cast<int>((sec - static_cast<int>(sec)) * 1e9);

        auto send_goal_options = rclcpp_action::Client<sobits_interfaces::action::MoveJoint>::SendGoalOptions();
        send_goal_options.result_callback = [this](auto result_future) {
            auto result = result_future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "[Action Result] %s", result->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "[Action Failed] %s", result->message.c_str());
            }
        };

        head_pantilt_ctr_->async_send_goal(goal_msg, send_goal_options);
    }
	if ( display_marker_ ) makeMarker( pan_angle, tilt_angle, distance );

	return;
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::onInit() {
    
    // Declare parameters
    this->declare_parameter<std::string>("following_position_topic_name", "/following_position");
    this->declare_parameter<bool>("use_rotate", true);
    this->declare_parameter<bool>("use_smoothing", true);
    this->declare_parameter<double>("tilt_angle_min_deg", -15.0);
    this->declare_parameter<double>("tilt_angle_max_deg", 15.0);
    this->declare_parameter<double>("person_height", 1.7);
    this->declare_parameter<double>("smoothing_gain", 0.5);
    this->declare_parameter<bool>("display_marker", true);
    this->declare_parameter<std::string>("head_pantilt_action_client_name", "person_aim_sensor_rotator");
    this->declare_parameter<std::string>("head_pan_joint_name", "head_camera_pan_joint");
    this->declare_parameter<std::string>("head_tilt_joint_name", "head_camera_tilt_joint");

    // Retrieve parameter values
    std::string following_position_topic_name;
    this->get_parameter("following_position_topic_name", following_position_topic_name);
    this->get_parameter("use_rotate", use_rotate_);
    this->get_parameter("use_smoothing", use_smoothing_);
    this->get_parameter("tilt_angle_min_deg", tilt_angle_min_);
    this->get_parameter("tilt_angle_max_deg", tilt_angle_max_);
    this->get_parameter("person_height", person_height_);
    this->get_parameter("smoothing_gain", smoothing_gain_);
    this->get_parameter("display_marker", display_marker_);
    this->get_parameter("head_pantilt_action_client_name", head_pantilt_action_client_name);
    this->get_parameter("head_pan_joint_name", head_pan_joint_name_);
    this->get_parameter("head_tilt_joint_name", head_tilt_joint_name_);

    // Convert degrees to radians for tilt angles
    tilt_angle_min_ = tilt_angle_min_ * M_PI / 180.0;
    tilt_angle_max_ = tilt_angle_max_ * M_PI / 180.0;

    RCLCPP_INFO(this->get_logger(), "Parameters loaded: following_position_topic_name=%s, use_rotate=%d, use_smoothing=%d, tilt_angle_min=%f rad, tilt_angle_max=%f rad",
    following_position_topic_name.c_str(), use_rotate_, use_smoothing_, tilt_angle_min_, tilt_angle_max_);

    // Initialize class members
    tf_sub_.reset(new tf2_ros::TransformListener(tfBuffer_));

    pub_marker_ = create_publisher< visualization_msgs::msg::Marker >( "rotator_marker", 1 );

    head_pantilt_ctr_ = rclcpp_action::create_client<sobits_interfaces::action::MoveJoint>( this, head_pantilt_action_client_name );
    
    while (!head_pantilt_ctr_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for action server...");
    }
    tracking_position_ = std::make_shared<geometry_msgs::msg::Point>();

    sub_following_position_ = this->create_subscription<FollowingPosition>(
        following_position_topic_name, 1, std::bind(&PersonAimSensorRotator::callbackData, this, std::placeholders::_1));

    if (use_rotate_) {
        auto pose_goal = sobits_interfaces::action::MoveToPose::Goal();
        pose_goal.pose_name = "initial_pose";
        pose_goal.time_allowance.sec = 1;
        pose_goal.time_allowance.nanosec = 0;
    
        auto pose_client = rclcpp_action::create_client<sobits_interfaces::action::MoveToPose>( this, "move_to_pose" );
        while (!pose_client->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for move_to_pose action server...");
        }
        pose_client->async_send_goal(pose_goal);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(multiple_sensor_person_tracking::PersonAimSensorRotator)
