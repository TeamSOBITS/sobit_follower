#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>

// #include "sobit_pro_library/sobit_pro_joint_controller.h"
#include "multiple_sensor_person_tracking/msg/following_position.hpp"
#include "multiple_observation_kalman_filter/multiple_observation_kalman_filter.hpp"

using multiple_sensor_person_tracking::msg::FollowingPosition;

namespace multiple_sensor_person_tracking {
    class SobitProPersonAimSensorRotator : public rclcpp::Node {
        private:
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
            rclcpp::Subscription<multiple_sensor_person_tracking::msg::FollowingPosition>::SharedPtr sub_following_position_;

            tf2_ros::Buffer tfBuffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_sub_;

			// std::unique_ptr<sobit_pro::SobitProJointController> sobit_pro_ctr_;

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

			void makeMarker( const double pan_angle, const double tilt_angle, const double distance );
            void callbackData (
                const std::shared_ptr<const FollowingPosition> &following_position_msg
                /*const nav_msgs::OdometryConstPtr &odom_msg*/
            );

        public:
            explicit SobitProPersonAimSensorRotator(const rclcpp::NodeOptions & options)
            : rclcpp::Node("sobit_pro_person_aim_sensor_rotator", options),
            tfBuffer_(this->get_clock()),
            tf_sub_(std::make_shared<tf2_ros::TransformListener>(tfBuffer_))
            {
                onInit();
            }

            void onInit();
    };
}

void multiple_sensor_person_tracking::SobitProPersonAimSensorRotator::makeMarker( const double pan_angle, const double tilt_angle, const double distance ) {
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

void multiple_sensor_person_tracking::SobitProPersonAimSensorRotator::callbackData (
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

	// if ( use_rotate_ ) sobit_pro_ctr_->moveHeadPanTilt ( pan_angle, tilt_angle, sec, false );
	if ( display_marker_ ) makeMarker( pan_angle, tilt_angle, distance );

	return;
}

void multiple_sensor_person_tracking::SobitProPersonAimSensorRotator::onInit() {

    // Declare parameters
    this->declare_parameter<std::string>("following_position_topic_name", "/following_position");
    this->declare_parameter<bool>("use_rotate", true);
    this->declare_parameter<bool>("use_smoothing", true);
    this->declare_parameter<double>("tilt_angle_min_deg", -15.0);
    this->declare_parameter<double>("tilt_angle_max_deg", 15.0);
    this->declare_parameter<double>("person_height", 1.7);
    this->declare_parameter<double>("smoothing_gain", 0.5);
    this->declare_parameter<bool>("display_marker", true);

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

    // Convert degrees to radians for tilt angles
    tilt_angle_min_ = tilt_angle_min_ * M_PI / 180.0;
    tilt_angle_max_ = tilt_angle_max_ * M_PI / 180.0;

    RCLCPP_INFO(this->get_logger(), "Parameters loaded: following_position_topic_name=%s, use_rotate=%d, use_smoothing=%d, tilt_angle_min=%f rad, tilt_angle_max=%f rad",
    following_position_topic_name.c_str(), use_rotate_, use_smoothing_, tilt_angle_min_, tilt_angle_max_);

    // Initialize class members
    tf_sub_.reset(new tf2_ros::TransformListener(tfBuffer_));

    pub_marker_ = create_publisher< visualization_msgs::msg::Marker >( "rotator_marker", 1 );

    // sobit_pro_ctr_.reset( new sobit_pro::SobitProJointController );
    tracking_position_ = std::make_shared<geometry_msgs::msg::Point>();

    sub_following_position_ = this->create_subscription<FollowingPosition>(
        following_position_topic_name, 1, std::bind(&SobitProPersonAimSensorRotator::callbackData, this, std::placeholders::_1));

    if ( !use_rotate_ ) return;
    // sobit_pro_ctr_->moveToPose( "initial_pose" );
    // sobit_pro_ctr_->moveHeadPanTilt ( 0.0, 0.2, 0.3, false );
}

RCLCPP_COMPONENTS_REGISTER_NODE(multiple_sensor_person_tracking::SobitProPersonAimSensorRotator)