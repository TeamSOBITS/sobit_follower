#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/time.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <sobit_education_library/sobit_education_controller.hpp>

#include <dynamic_reconfigure/server.h>
#include <multiple_sensor_person_tracking/SensorRotatorParameterConfig.h>
#include <person_following_control/FollowingPosition.h>

namespace multiple_sensor_person_tracking {
    class PersonAimSensorRotator : public nodelet::Nodelet {
        private:
			ros::NodeHandle nh_;
			ros::NodeHandle pnh_;
			// ros::Publisher pub_marker_;
            ros::Subscriber sub_tracking_position_;
			tf::TransformListener tf_listener_;

			dynamic_reconfigure::Server<multiple_sensor_person_tracking::SensorRotatorParameterConfig>* server_;
            dynamic_reconfigure::Server<multiple_sensor_person_tracking::SensorRotatorParameterConfig>::CallbackType f_;

			std::unique_ptr<sobit_education::SobitEducationController> sobit_edu_ctr_;

			geometry_msgs::PointPtr tracking_position_;
			double pre_tilt_;
			double pre_pan_;
			double tilt_angle_min_;
			double tilt_angle_max_;
			double person_height_;
			double smoothing_gain_;
			bool use_rotate_;
			bool use_smoothing_;

			void callbackDynamicReconfigure(multiple_sensor_person_tracking::SensorRotatorParameterConfig& config, uint32_t level);
			void callbackTargetPosition( const person_following_control::FollowingPositionConstPtr& msg );

        public:
            virtual void onInit();

    };
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::callbackDynamicReconfigure(multiple_sensor_person_tracking::SensorRotatorParameterConfig& config, uint32_t level) {
	use_rotate_ = config.use_rotate;
	tilt_angle_min_ = config.tilt_angle_min_deg * M_PI / 180.0;
	tilt_angle_max_ = config.tilt_angle_max_deg * M_PI / 180.0;
	person_height_ = config.camera2person_height;
	use_smoothing_ = config.use_smoothing;
	smoothing_gain_ = config.smoothing_gain;

	if ( !use_rotate_ ) {
		sub_tracking_position_.shutdown();
	} else {
		sub_tracking_position_ = nh_.subscribe( pnh_.param<std::string>( "following_position_topic_name", "/following_position" ), 1, &multiple_sensor_person_tracking::PersonAimSensorRotator::callbackTargetPosition, this);
	}
	return;
}


void multiple_sensor_person_tracking::PersonAimSensorRotator::callbackTargetPosition( const person_following_control::FollowingPositionConstPtr& msg ) {
	geometry_msgs::Point pt;
	if ( use_smoothing_ ) {
		tracking_position_->x = smoothing_gain_ * tracking_position_->x + ( 1.0 - smoothing_gain_ ) * msg->pose.position.x;
        tracking_position_->y = smoothing_gain_ * tracking_position_->y + ( 1.0 - smoothing_gain_ ) * msg->pose.position.y;
		pt = *tracking_position_;
	} else {
		pt = msg->pose.position;
	}

	double distance = std::hypotf( pt.x, pt.y );
	double angle = std::atan2( pt.y, pt.x );
	double tilt_angle, pan_angle;
	double sec = ( distance == 0.0 ) ? 0.5 : 0.033;

	tilt_angle = std::atan2( person_height_, distance );
	if ( distance == 0.0 ) tilt_angle = 0.2;
	else if ( tilt_angle < tilt_angle_min_ ) tilt_angle = tilt_angle_min_;
	else if ( tilt_angle > tilt_angle_max_ ) tilt_angle = tilt_angle_max_;
    pan_angle = -angle;

	NODELET_INFO("Rotator:\tpan = %8.3f[deg],\ttilt = %8.3f [deg]", pan_angle*180/M_PI, tilt_angle*180/M_PI);
	sobit_edu_ctr_->moveXtionPanTilt ( pan_angle, tilt_angle, sec, true );

    // visualization_msgs::Marker target_marker;
    // target_marker.header.frame_id = "base_footprint";
    // target_marker.header.stamp = ros::Time::now();
    // target_marker.ns = "target_marker";
    // target_marker.id =  1;
    // target_marker.type = visualization_msgs::Marker::CUBE;
    // target_marker.action = visualization_msgs::Marker::ADD;
    // target_marker.scale.x = 0.15;target_marker.scale.y = 0.15;target_marker.scale.z = 0.15;
    // target_marker.color.r = 1.0; target_marker.color.g = 1.0; target_marker.color.b = 0.0; target_marker.color.a = 1.0;
    // target_marker.pose.position = *tracking_position_;
    // target_marker.pose.position.z = 0.2;
    // target_marker.pose.orientation.w = 1.0;
    // target_marker.lifetime = ros::Duration(1.0);
	// pub_marker_.publish( target_marker );
	return;
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

	// pub_marker_ = nh_.advertise< visualization_msgs::Marker >( "rotator_marker", 1 );

	sobit_edu_ctr_.reset( new sobit_education::SobitEducationController );
	tracking_position_.reset( new geometry_msgs::Point );

    server_ = new dynamic_reconfigure::Server<multiple_sensor_person_tracking::SensorRotatorParameterConfig>(pnh_);
    f_ = boost::bind(&multiple_sensor_person_tracking::PersonAimSensorRotator::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);

	sub_tracking_position_ = nh_.subscribe( pnh_.param<std::string>( "following_position_topic_name", "/following_position" ), 1, &multiple_sensor_person_tracking::PersonAimSensorRotator::callbackTargetPosition, this);

	if ( !use_rotate_ ) return;
	sobit_edu_ctr_->movePose( "initial_pose" );
	sobit_edu_ctr_->moveXtionPanTilt ( 0.0, 0.2, 0.3, false );
}

PLUGINLIB_EXPORT_CLASS(multiple_sensor_person_tracking::PersonAimSensorRotator, nodelet::Nodelet);
