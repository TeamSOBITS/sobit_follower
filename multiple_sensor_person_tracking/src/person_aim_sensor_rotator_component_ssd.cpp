#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sobits_interfaces/action/move_joint.hpp>
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "multiple_observation_kalman_filter/multiple_observation_kalman_filter.hpp"

namespace multiple_sensor_person_tracking {
    class PersonAimSensorRotator : public rclcpp::Node {
        private:
            rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr sub_ssd_;

			tf2_ros::Buffer tfBuffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_sub_;

            rclcpp_action::Client<sobits_interfaces::action::MoveJoint>::SharedPtr head_pantilt_ctr_;

			std::shared_ptr<geometry_msgs::msg::Point> tracking_position_;
			double pre_tilt_;
			double pre_pan_;
			double tilt_angle_min_;
			double tilt_angle_max_;
			double person_height_;
			double smoothing_gain_;
			bool use_rotate_;
			bool use_smoothing_;
            std::string head_pantilt_action_name_;
            std::string head_pan_joint_name_;
            std::string head_tilt_joint_name_;

            void callbackData (
                const std::shared_ptr<const vision_msgs::msg::Detection3DArray> &ssd_msg
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

void multiple_sensor_person_tracking::PersonAimSensorRotator::callbackData (
    const std::shared_ptr<const vision_msgs::msg::Detection3DArray> &ssd_msg) {
	geometry_msgs::msg::Point pt;
    double min_distance = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point closest_point;

    for (const auto& detection : ssd_msg->detections) {
        double x = detection.bbox.center.position.x;
        double y = detection.bbox.center.position.y;
        double distance = std::hypot(x, y);

        if (distance < min_distance) {
            min_distance = distance;
            closest_point.x = x;
            closest_point.y = y;
        }
    }

    if (use_smoothing_) {
        tracking_position_->x = smoothing_gain_ * tracking_position_->x + (1.0 - smoothing_gain_) * closest_point.x;
        tracking_position_->y = smoothing_gain_ * tracking_position_->y + (1.0 - smoothing_gain_) * closest_point.y;
    } else {
        *tracking_position_ = closest_point;
    }

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
            auto result = result_future;
            if (result.result->success) {
                RCLCPP_INFO(this->get_logger(), "[Action Result] %s", result.result->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "[Action Failed] %s", result.result->message.c_str());
            }
        };

        head_pantilt_ctr_->async_send_goal(goal_msg, send_goal_options);
    }

	return;
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::onInit() {
    
    // Declare parameters
    this->declare_parameter<std::string>("ssd_topic_name", "/ssd_ros/object_3d_poses");
    this->declare_parameter<bool>("use_rotate", true);
    this->declare_parameter<bool>("use_smoothing", true);
    this->declare_parameter<double>("tilt_angle_min_deg", -15.0);
    this->declare_parameter<double>("tilt_angle_max_deg", 15.0);
    this->declare_parameter<double>("person_height", 1.7);
    this->declare_parameter<double>("smoothing_gain", 0.5);
    this->declare_parameter<std::string>("head_pantilt_action_name", "move_joint");
    this->declare_parameter<std::string>("head_pan_joint_name", "head_camera_pan_joint");
    this->declare_parameter<std::string>("head_tilt_joint_name", "head_camera_tilt_joint");

    // Retrieve parameter values
    std::string ssd_topic_name;
    this->get_parameter("ssd_topic_name", ssd_topic_name);
    this->get_parameter("use_rotate", use_rotate_);
    this->get_parameter("use_smoothing", use_smoothing_);
    this->get_parameter("tilt_angle_min_deg", tilt_angle_min_);
    this->get_parameter("tilt_angle_max_deg", tilt_angle_max_);
    this->get_parameter("person_height", person_height_);
    this->get_parameter("smoothing_gain", smoothing_gain_);
    this->get_parameter("head_pantilt_action_name", head_pantilt_action_name_);
    this->get_parameter("head_pan_joint_name", head_pan_joint_name_);
    this->get_parameter("head_tilt_joint_name", head_tilt_joint_name_);

    // Convert degrees to radians for tilt angles
    tilt_angle_min_ = tilt_angle_min_ * M_PI / 180.0;
    tilt_angle_max_ = tilt_angle_max_ * M_PI / 180.0;

    RCLCPP_INFO(this->get_logger(), "Parameters loaded: ssd_topic_name=%s, use_rotate=%d, use_smoothing=%d, tilt_angle_min=%f rad, tilt_angle_max=%f rad",
        ssd_topic_name.c_str(), use_rotate_, use_smoothing_, tilt_angle_min_, tilt_angle_max_);

    RCLCPP_INFO(this->get_logger(), "Parameters loaded: head_pantilt_action_name=%s, head_pan_joint_name=%s, head_tilt_joint_name=%s",
        head_pantilt_action_name_.c_str(), head_pan_joint_name_.c_str(), head_tilt_joint_name_.c_str());

    // Initialize class members
    tf_sub_.reset(new tf2_ros::TransformListener(tfBuffer_));

    head_pantilt_ctr_ = rclcpp_action::create_client<sobits_interfaces::action::MoveJoint>( this, head_pantilt_action_name_ );
    
    while (!head_pantilt_ctr_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for action server...");
    }
    tracking_position_ = std::make_shared<geometry_msgs::msg::Point>();

    sub_ssd_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
        ssd_topic_name, 1, std::bind(&PersonAimSensorRotator::callbackData, this, std::placeholders::_1));
}

RCLCPP_COMPONENTS_REGISTER_NODE(multiple_sensor_person_tracking::PersonAimSensorRotator)