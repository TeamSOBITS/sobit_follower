#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sobits_interfaces/action/move_joint.hpp>
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

            rclcpp_action::Client<sobits_interfaces::action::MoveJoint>::SharedPtr head_pantilt_ctr_;

			std::shared_ptr<geometry_msgs::msg::Point> tracking_position_;
			double pre_tilt_;
			double pre_pan_;
				double tilt_angle_min_;
				double tilt_angle_max_;
                double pan_angle_min_;
                double pan_angle_max_;
				double person_height_;
				double smoothing_gain_;
                double pan_command_deadband_rad_;
                double tilt_command_deadband_rad_;
                double pan_max_speed_rad_s_;
                double tilt_max_speed_rad_s_;
                double min_action_time_sec_;
                double max_action_time_sec_;
            bool use_rotate_;
				bool use_smoothing_;
				bool display_marker_;
            std::string head_pantilt_action_name_;
            std::string head_pan_joint_name_;
            std::string head_tilt_joint_name_;
            bool goal_in_flight_;

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
    const double input_x = following_position_msg->rotation_position.x;
    const double input_y = following_position_msg->rotation_position.y;
    if (!std::isfinite(input_x) || !std::isfinite(input_y)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Skipping rotator update because target position is non-finite.");
        return;
    }

    if (!std::isfinite(tracking_position_->x) || !std::isfinite(tracking_position_->y)) {
        tracking_position_->x = input_x;
        tracking_position_->y = input_y;
    }

    if ( use_smoothing_ ) {
        tracking_position_->x = smoothing_gain_ * tracking_position_->x + ( 1.0 - smoothing_gain_ ) * input_x;
        tracking_position_->y = smoothing_gain_ * tracking_position_->y + ( 1.0 - smoothing_gain_ ) * input_y;
    } else {
        tracking_position_->x = input_x;
        tracking_position_->y = input_y;
    }

	    pt = *tracking_position_;
		double distance = std::hypotf(pt.x, pt.y);
		double angle = std::atan2( pt.y, pt.x );
		double pan_angle, tilt_angle;

    if (!std::isfinite(distance) || !std::isfinite(angle)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Skipping rotator update because computed angle/distance is non-finite.");
        return;
    }

		tilt_angle = std::atan2( person_height_, distance );
	if (distance < 1.0e-6) {
        tilt_angle = 0.2;
    }
    tilt_angle = std::clamp(tilt_angle, tilt_angle_min_, tilt_angle_max_);
    pan_angle = std::clamp(angle, pan_angle_min_, pan_angle_max_);

    if (!std::isfinite(pan_angle) || !std::isfinite(tilt_angle)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Skipping rotator update because pan/tilt command is non-finite.");
        return;
    }

    const bool has_previous_command = std::isfinite(pre_pan_) && std::isfinite(pre_tilt_);
    const double delta_pan = has_previous_command ? std::fabs(pan_angle - pre_pan_) : 0.0;
    const double delta_tilt = has_previous_command ? std::fabs(tilt_angle - pre_tilt_) : 0.0;
    if (has_previous_command &&
        delta_pan < pan_command_deadband_rad_ &&
        delta_tilt < tilt_command_deadband_rad_) {
        if ( display_marker_ ) makeMarker( pan_angle, tilt_angle, distance );
        return;
    }

    const double time_from_pan = delta_pan / pan_max_speed_rad_s_;
    const double time_from_tilt = delta_tilt / tilt_max_speed_rad_s_;
    const double sec = std::clamp(
        std::max(time_from_pan, time_from_tilt),
        min_action_time_sec_,
        max_action_time_sec_);

	RCLCPP_INFO(this->get_logger(), "\033[1mRotator\033[m               :\tpan = %8.3f[deg],\ttilt = %8.3f [deg]", pan_angle*180/M_PI, tilt_angle*180/M_PI);

		    if ( use_rotate_ ) {
		        if (goal_in_flight_) {
		            return;
		        }
		        if (!head_pantilt_ctr_->action_server_is_ready()) {
		            RCLCPP_WARN_THROTTLE(
		                this->get_logger(), *this->get_clock(), 2000,
		                "Head action server is not ready. Skipping pan/tilt command.");
		            return;
	        }

        auto goal_msg = sobits_interfaces::action::MoveJoint::Goal();
        goal_msg.target_joint_names = { head_pan_joint_name_, head_tilt_joint_name_ };
        goal_msg.target_joint_rad = { pan_angle, tilt_angle };
        goal_msg.time_allowance.sec = static_cast<int>(sec);
        goal_msg.time_allowance.nanosec = static_cast<int>((sec - static_cast<int>(sec)) * 1e9);
        RCLCPP_INFO(
            this->get_logger(),
            "Sending MoveJoint goal: action=%s joints=[%s, %s] target=[%.3f, %.3f] rad time=%.2f s",
            head_pantilt_action_name_.c_str(),
            head_pan_joint_name_.c_str(),
            head_tilt_joint_name_.c_str(),
            pan_angle,
            tilt_angle,
            sec);

	        auto send_goal_options = rclcpp_action::Client<sobits_interfaces::action::MoveJoint>::SendGoalOptions();
	        send_goal_options.goal_response_callback =
	            [this](const rclcpp_action::ClientGoalHandle<sobits_interfaces::action::MoveJoint>::SharedPtr & goal_handle) {
	                if (!goal_handle) {
	                    goal_in_flight_ = false;
	                    RCLCPP_WARN(this->get_logger(), "[Action Failed] MoveJoint goal rejected.");
	                    return;
	                }
	                RCLCPP_INFO(this->get_logger(), "MoveJoint goal accepted.");
	            };
	        send_goal_options.result_callback = [this](auto result_future) {
	            auto result = result_future;
	            goal_in_flight_ = false;
	            if (!result.result) {
	                RCLCPP_WARN(this->get_logger(), "[Action Failed] Empty result payload from MoveJoint.");
	                return;
	            }
            if (result.result->success) {
                RCLCPP_INFO(this->get_logger(), "[Action Result] %s", result.result->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "[Action Failed] %s", result.result->message.c_str());
            }
        };

	        goal_in_flight_ = true;
	        head_pantilt_ctr_->async_send_goal(goal_msg, send_goal_options);
            pre_pan_ = pan_angle;
            pre_tilt_ = tilt_angle;
	    }
	if ( display_marker_ ) makeMarker( pan_angle, tilt_angle, distance );

	return;
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::onInit() {
    
    // Declare parameters
    this->declare_parameter<std::string>("following_position_topic_name", "/following_position");
    this->declare_parameter<bool>("use_rotate", true);
    this->declare_parameter<bool>("use_smoothing", true);
    this->declare_parameter<double>("pan_angle_min_deg", -90.0);
    this->declare_parameter<double>("pan_angle_max_deg", 90.0);
    this->declare_parameter<double>("tilt_angle_min_deg", -15.0);
    this->declare_parameter<double>("tilt_angle_max_deg", 15.0);
    this->declare_parameter<double>("person_height", 1.7);
    this->declare_parameter<double>("camera2person_height", 0.2);
    this->declare_parameter<double>("smoothing_gain", 0.5);
    this->declare_parameter<double>("pan_command_deadband_deg", 0.8);
    this->declare_parameter<double>("tilt_command_deadband_deg", 0.8);
    this->declare_parameter<double>("pan_max_speed_deg_s", 120.0);
    this->declare_parameter<double>("tilt_max_speed_deg_s", 90.0);
    this->declare_parameter<double>("min_action_time_sec", 0.05);
    this->declare_parameter<double>("max_action_time_sec", 0.60);
    this->declare_parameter<bool>("display_marker", true);
    this->declare_parameter<std::string>("head_pantilt_action_name", "move_joint");
    this->declare_parameter<std::string>("head_pan_joint_name", "head_camera_pan_joint");
    this->declare_parameter<std::string>("head_tilt_joint_name", "head_camera_tilt_joint");

    // Retrieve parameter values
    std::string following_position_topic_name;
    this->get_parameter("following_position_topic_name", following_position_topic_name);
    this->get_parameter("use_rotate", use_rotate_);
    this->get_parameter("use_smoothing", use_smoothing_);
    this->get_parameter("pan_angle_min_deg", pan_angle_min_);
    this->get_parameter("pan_angle_max_deg", pan_angle_max_);
    this->get_parameter("tilt_angle_min_deg", tilt_angle_min_);
    this->get_parameter("tilt_angle_max_deg", tilt_angle_max_);
    this->get_parameter("person_height", person_height_);
    this->get_parameter("camera2person_height", person_height_);
    this->get_parameter("smoothing_gain", smoothing_gain_);
    this->get_parameter("pan_command_deadband_deg", pan_command_deadband_rad_);
    this->get_parameter("tilt_command_deadband_deg", tilt_command_deadband_rad_);
    this->get_parameter("pan_max_speed_deg_s", pan_max_speed_rad_s_);
    this->get_parameter("tilt_max_speed_deg_s", tilt_max_speed_rad_s_);
    this->get_parameter("min_action_time_sec", min_action_time_sec_);
    this->get_parameter("max_action_time_sec", max_action_time_sec_);
    this->get_parameter("display_marker", display_marker_);
    this->get_parameter("head_pantilt_action_name", head_pantilt_action_name_);
    this->get_parameter("head_pan_joint_name", head_pan_joint_name_);
    this->get_parameter("head_tilt_joint_name", head_tilt_joint_name_);

    if (pan_angle_min_ > pan_angle_max_) std::swap(pan_angle_min_, pan_angle_max_);
    if (tilt_angle_min_ > tilt_angle_max_) std::swap(tilt_angle_min_, tilt_angle_max_);
    smoothing_gain_ = std::clamp(smoothing_gain_, 0.0, 1.0);
    if (min_action_time_sec_ > max_action_time_sec_) std::swap(min_action_time_sec_, max_action_time_sec_);
    min_action_time_sec_ = std::max(0.0, min_action_time_sec_);
    max_action_time_sec_ = std::max(min_action_time_sec_, max_action_time_sec_);
    pan_command_deadband_rad_ = std::max(0.0, pan_command_deadband_rad_);
    tilt_command_deadband_rad_ = std::max(0.0, tilt_command_deadband_rad_);
    pan_max_speed_rad_s_ = std::max(1.0e-3, pan_max_speed_rad_s_);
    tilt_max_speed_rad_s_ = std::max(1.0e-3, tilt_max_speed_rad_s_);

    // Convert degrees to radians for pan/tilt angles
    pan_angle_min_ = pan_angle_min_ * M_PI / 180.0;
    pan_angle_max_ = pan_angle_max_ * M_PI / 180.0;
    // Convert degrees to radians for tilt angles
    tilt_angle_min_ = tilt_angle_min_ * M_PI / 180.0;
    tilt_angle_max_ = tilt_angle_max_ * M_PI / 180.0;
    pan_command_deadband_rad_ = pan_command_deadband_rad_ * M_PI / 180.0;
    tilt_command_deadband_rad_ = tilt_command_deadband_rad_ * M_PI / 180.0;
    pan_max_speed_rad_s_ = pan_max_speed_rad_s_ * M_PI / 180.0;
    tilt_max_speed_rad_s_ = tilt_max_speed_rad_s_ * M_PI / 180.0;

    RCLCPP_INFO(this->get_logger(),
        "Parameters loaded: following_position_topic_name=%s, use_rotate=%d, use_smoothing=%d, pan_min=%f rad, pan_max=%f rad, tilt_min=%f rad, tilt_max=%f rad",
        following_position_topic_name.c_str(), use_rotate_, use_smoothing_, pan_angle_min_, pan_angle_max_, tilt_angle_min_, tilt_angle_max_);

    // Initialize class members
    tf_sub_.reset(new tf2_ros::TransformListener(tfBuffer_));

    pub_marker_ = create_publisher< visualization_msgs::msg::Marker >( "rotator_marker", 1 );

    head_pantilt_ctr_ = rclcpp_action::create_client<sobits_interfaces::action::MoveJoint>( this, head_pantilt_action_name_ );
    
    while (rclcpp::ok() && !head_pantilt_ctr_->wait_for_action_server(std::chrono::milliseconds(500))) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Waiting for action server...");
    }
    if (!rclcpp::ok()) {
        RCLCPP_INFO(this->get_logger(), "Shutdown requested while waiting for action server.");
        return;
    }
    goal_in_flight_ = false;
    tracking_position_ = std::make_shared<geometry_msgs::msg::Point>();
    pre_pan_ = std::numeric_limits<double>::quiet_NaN();
    pre_tilt_ = std::numeric_limits<double>::quiet_NaN();

    sub_following_position_ = this->create_subscription<FollowingPosition>(
        following_position_topic_name, 1, std::bind(&PersonAimSensorRotator::callbackData, this, std::placeholders::_1));
}

RCLCPP_COMPONENTS_REGISTER_NODE(multiple_sensor_person_tracking::PersonAimSensorRotator)
