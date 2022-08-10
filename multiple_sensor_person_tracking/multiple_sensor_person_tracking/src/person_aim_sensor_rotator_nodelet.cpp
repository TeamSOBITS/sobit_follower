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

namespace multiple_sensor_person_tracking {
    class PersonAimSensorRotator : public nodelet::Nodelet {
        private:
			ros::NodeHandle nh_;
			ros::NodeHandle pnh_;
			ros::Publisher pub_marker_;
            ros::Subscriber sub_tracking_position_;
			ros::Timer timer_;
			tf::TransformListener tf_listener_;

			dynamic_reconfigure::Server<multiple_sensor_person_tracking::SensorRotatorParameterConfig>* server_;
            dynamic_reconfigure::Server<multiple_sensor_person_tracking::SensorRotatorParameterConfig>::CallbackType f_;

			std::unique_ptr<sobit_education::SobitEducationController> sobit_edu_ctr_;

			geometry_msgs::PointStampedPtr tracking_position_;
			double pre_tilt_;
			double pre_pan_;
			double tilt_angle_min_;
			double tilt_angle_max_;
			double person_height_;
			double smoothing_gain_;
			bool use_rotate_;
			bool use_smoothing_;

			void callbackDynamicReconfigure(multiple_sensor_person_tracking::SensorRotatorParameterConfig& config, uint32_t level);
			void callbackTargetPosition( const geometry_msgs::PointStampedConstPtr& msg );
			void callbackTimer( const ros::TimerEvent& e );

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
		timer_.stop();
	} else {
		sub_tracking_position_ = nh_.subscribe("/person_follower/rotate_position", 1, &multiple_sensor_person_tracking::PersonAimSensorRotator::callbackTargetPosition, this);
		timer_ = nh_.createTimer( ros::Duration(0.033), &multiple_sensor_person_tracking::PersonAimSensorRotator::callbackTimer, this );
	}
	return;
}


void multiple_sensor_person_tracking::PersonAimSensorRotator::callbackTargetPosition( const geometry_msgs::PointStampedConstPtr& msg ) {
	if ( use_smoothing_ ) {
		tracking_position_->point.x = smoothing_gain_ * tracking_position_->point.x + ( 1.0 - smoothing_gain_ ) * msg->point.x;
        tracking_position_->point.y = smoothing_gain_ * tracking_position_->point.y + ( 1.0 - smoothing_gain_ ) * msg->point.y;
	} else {
		*tracking_position_ = *msg;
	}
    NODELET_INFO("Rotator: x = %8.3f [m],\ty = %8.3f [m]", tracking_position_->point.x, tracking_position_->point.y);

    visualization_msgs::Marker target_marker;
    target_marker.header.frame_id = "base_footprint";
    target_marker.header.stamp = ros::Time::now();
    target_marker.ns = "target_marker";
    target_marker.id =  1;
    target_marker.type = visualization_msgs::Marker::CUBE;
    target_marker.action = visualization_msgs::Marker::ADD;
    target_marker.scale.x = 0.15;target_marker.scale.y = 0.15;target_marker.scale.z = 0.15;
    target_marker.color.r = 1.0; target_marker.color.g = 1.0; target_marker.color.b = 0.0; target_marker.color.a = 1.0;
    target_marker.pose.position = tracking_position_->point;
    target_marker.pose.position.z = 0.2;
    target_marker.pose.orientation.w = 1.0;
    target_marker.lifetime = ros::Duration(1.0);
	pub_marker_.publish( target_marker );
	return;
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::callbackTimer( const ros::TimerEvent& e ) {
	double distance = std::hypotf( tracking_position_->point.x, tracking_position_->point.y );
	double angle = std::atan2( tracking_position_->point.y, tracking_position_->point.x );
	double tilt_angle, pan_angle;
	double sec = ( distance == 0.0 ) ? 0.5 : 0.033;

	tilt_angle = std::atan2( person_height_, distance );
	if ( distance == 0.0 ) tilt_angle = 0.2;
	else if ( tilt_angle < tilt_angle_min_ ) tilt_angle = tilt_angle_min_;
	else if ( tilt_angle > tilt_angle_max_ ) tilt_angle = tilt_angle_max_;
    pan_angle = -angle;

	sobit_edu_ctr_->moveXtionPanTilt ( pan_angle, tilt_angle, sec, true );
	return;
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

	pub_marker_ = nh_.advertise< visualization_msgs::Marker >( "rotator_marker", 1 );

	sobit_edu_ctr_.reset( new sobit_education::SobitEducationController );
	tracking_position_.reset( new geometry_msgs::PointStamped );

	tilt_angle_min_ = pnh_.param<double>( "tilt_angle_min_deg", 0.0 ) * M_PI / 180.0;
	tilt_angle_max_ = pnh_.param<double>( "tilt_angle_max_deg", 20.0 ) * M_PI / 180.0;
	person_height_ = pnh_.param<double>( "camera2person_height", 0.5 );
	smoothing_gain_ = pnh_.param<double>( "smoothing_gain", 0.95 );
	use_rotate_ = pnh_.param<bool>( "use_rotate", true );
	use_smoothing_ = pnh_.param<bool>( "use_smoothing", true );

    server_ = new dynamic_reconfigure::Server<multiple_sensor_person_tracking::SensorRotatorParameterConfig>(pnh_);
    f_ = boost::bind(&multiple_sensor_person_tracking::PersonAimSensorRotator::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);

	sub_tracking_position_ = nh_.subscribe("/multiple_sensor_person_tracking/tracking_position", 1, &multiple_sensor_person_tracking::PersonAimSensorRotator::callbackTargetPosition, this);

	if ( !use_rotate_ ) return;
	sobit_edu_ctr_->movePose( "initial_pose" );
	sobit_edu_ctr_->moveXtionPanTilt ( 0.0, 0.2, 0.3, false );

	timer_ = nh_.createTimer( ros::Duration(0.033), &multiple_sensor_person_tracking::PersonAimSensorRotator::callbackTimer, this );
}

PLUGINLIB_EXPORT_CLASS(multiple_sensor_person_tracking::PersonAimSensorRotator, nodelet::Nodelet);
