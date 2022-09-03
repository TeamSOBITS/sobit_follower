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
#include <multiple_sensor_person_tracking/FollowingPosition.h>

namespace multiple_sensor_person_tracking {
    class PersonAimSensorRotator : public nodelet::Nodelet {
        private:
			ros::NodeHandle nh_;
			ros::NodeHandle pnh_;
			ros::Publisher pub_marker_;
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
			bool display_marker_;

			void makeMarker( const double pan_angle, const double tilt_angle, const double distance );
			void callbackDynamicReconfigure(multiple_sensor_person_tracking::SensorRotatorParameterConfig& config, uint32_t level);
			void callbackTargetPosition( const multiple_sensor_person_tracking::FollowingPositionConstPtr& msg );

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
	display_marker_ = config.display_marker;
	return;
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::makeMarker( const double pan_angle, const double tilt_angle, const double distance ) {
	visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = ros::Time::now();
    marker.ns = "sensor_direction";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = distance; marker.scale.y = 0.05; marker.scale.z = 0.05;
	marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
    marker.lifetime = ros::Duration(0.1);
	marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.8;
    tf::Quaternion quat = tf::createQuaternionFromRPY(0, -tilt_angle, pan_angle);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    marker.pose.orientation = geometry_quat;
    pub_marker_.publish ( marker );
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::callbackTargetPosition( const multiple_sensor_person_tracking::FollowingPositionConstPtr& msg ) {
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
	double pan_angle, tilt_angle;
	double sec = ( distance == 0.0 ) ? 0.5 : 0.01;

	tilt_angle = std::atan2( person_height_, distance );
	if ( distance == 0.0 ) tilt_angle = 0.2;
	else if ( tilt_angle < tilt_angle_min_ ) tilt_angle = tilt_angle_min_;
	else if ( tilt_angle > tilt_angle_max_ ) tilt_angle = tilt_angle_max_;
    pan_angle = angle;

	// NODELET_INFO("\033[1mRotator\033[m               :\tpan = %8.3f[deg],\ttilt = %8.3f [deg]", pan_angle*180/M_PI, tilt_angle*180/M_PI);

	if ( use_rotate_ ) sobit_edu_ctr_->moveHeadPanTilt ( pan_angle, tilt_angle, sec, true );
	if ( display_marker_ ) makeMarker( pan_angle, tilt_angle, distance );

	return;
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

	pub_marker_ = nh_.advertise< visualization_msgs::Marker >( "rotator_marker", 1 );

	sobit_edu_ctr_.reset( new sobit_education::SobitEducationController );
	tracking_position_.reset( new geometry_msgs::Point );

    server_ = new dynamic_reconfigure::Server<multiple_sensor_person_tracking::SensorRotatorParameterConfig>(pnh_);
    f_ = boost::bind(&multiple_sensor_person_tracking::PersonAimSensorRotator::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);

	sub_tracking_position_ = nh_.subscribe( pnh_.param<std::string>( "following_position_topic_name", "/following_position" ), 1, &multiple_sensor_person_tracking::PersonAimSensorRotator::callbackTargetPosition, this);

	if ( !use_rotate_ ) return;
	sobit_edu_ctr_->moveToPose( "initial_pose" );
	sobit_edu_ctr_->moveHeadPanTilt ( 0.0, 0.2, 0.3, false );
}

PLUGINLIB_EXPORT_CLASS(multiple_sensor_person_tracking::PersonAimSensorRotator, nodelet::Nodelet);
