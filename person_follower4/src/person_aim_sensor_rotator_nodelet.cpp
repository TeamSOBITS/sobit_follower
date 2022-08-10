#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/time.h>
#include <iostream>
#include <tf/transform_listener.h>  
#include <sobit_education_library/sobit_education_controller.hpp>
#include <person_tracking4/RotatePosition.h>

constexpr int NON_WORKING = -1;
constexpr int DETECTION = 1;
constexpr int TRACKING = 2;
constexpr int PREDICTION = 3;
constexpr int SVDD_LEG_DETECTION= 4;

namespace person_follower4 {
    class PersonAimSensorRotator : public nodelet::Nodelet {
        private:
			ros::NodeHandle nh_;
			ros::NodeHandle pnh_;
            ros::Subscriber sub_;
			tf::TransformListener tf_listener_;
			sobit_education::SobitEducationController sobit_edu_ctr_;

			std::vector<double> tilt_angle_range_;
			double person_height_;
			
			double pre_tilt_;
			double pre_pan_;
			int pre_flag_;

			bool use_sobit_mini_;
			bool use_rotate_;
			
			double getAngleFrameDifference( std::string org_frame, std::string target_frame );
			void rotateSensor ( const int process_flag, const double target_distance, const double target_angle );
			void rotateSensor ( const double target_distance, const double target_angle );

        public:
            virtual void onInit();
            void callbackRotatePosition( const person_tracking4::RotatePositionConstPtr& rp_msg );
    };
}

void person_follower4::PersonAimSensorRotator::rotateSensor ( const int process_flag, const double target_distance, const double target_angle ) {
	if ( !use_rotate_ ) return;
	double tilt_angle = 0.0, pan_angle = 0.0;
	tilt_angle = std::atan2( person_height_, target_distance );
	if ( target_distance == 0.0 ) tilt_angle = 0.2;
	else if ( tilt_angle < tilt_angle_range_[0] ) tilt_angle = tilt_angle_range_[0];
	else if ( tilt_angle > tilt_angle_range_[1] ) tilt_angle = tilt_angle_range_[1];

	if ( use_sobit_mini_ ) {
		double angle_frame_diff = getAngleFrameDifference( "base_footprint", "body_roll_link" );
		pan_angle = ( process_flag != DETECTION ) ? -(target_angle - angle_frame_diff) : 0.0;
	} else {
		pan_angle = ( process_flag != DETECTION ) ? -target_angle : 0.0;
	}

	if ( pre_flag_ == DETECTION && process_flag == TRACKING ) pre_pan_ = pan_angle;
	pan_angle = 0.95 * pan_angle + 0.05 * pre_pan_;

	if ( process_flag == DETECTION ) pan_angle = 0.0;
	
	double sec = ( process_flag != DETECTION ) ? 0.1 : 1.0;
	if( process_flag == PREDICTION ) sobit_edu_ctr_.moveXtionPanTilt ( pre_pan_, pre_tilt_, sec, false );
	else { 
		sobit_edu_ctr_.moveXtionPanTilt ( pan_angle, tilt_angle, sec, false );
		pre_pan_ = pan_angle;
		pre_tilt_ = tilt_angle;
	}
	pre_flag_ = process_flag;
	return;
}

void person_follower4::PersonAimSensorRotator::rotateSensor ( const double target_distance, const double target_angle ) {
	if ( !use_rotate_ ) return;
	double tilt_angle, pan_angle;
	tilt_angle = std::atan2( person_height_, target_distance );
	if ( target_distance == 0.0 ) tilt_angle = 0.2;
	else if ( tilt_angle < tilt_angle_range_[0] ) tilt_angle = tilt_angle_range_[0];
	else if ( tilt_angle > tilt_angle_range_[1] ) tilt_angle = tilt_angle_range_[1];

	if ( use_sobit_mini_ ) {
		double angle_frame_diff = getAngleFrameDifference( "base_footprint", "body_roll_link" );
		pan_angle = -(target_angle - angle_frame_diff);
	} else {
		pan_angle = -target_angle;
	}
	double sec = 1.5;
	sobit_edu_ctr_.moveXtionPanTilt ( pan_angle, tilt_angle, sec, true );
	pre_pan_ = pan_angle;
	pre_tilt_ = tilt_angle;
	return;
}

void person_follower4::PersonAimSensorRotator::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

	NODELET_INFO("\n%s\n", pnh_.getNamespace().c_str());
	use_rotate_ = pnh_.param<bool>( "use_rotate", true );
	use_sobit_mini_ = pnh_.param<bool>( "use_sobit_mini", false );
	tilt_angle_range_ = pnh_.param<std::vector<double>>( "tilt_angle_range", { 0.0, 0.3} );
	person_height_ = pnh_.param<double>( "camera2person_height", 0.5 );
	std::cout << "\n========================================\n[ PersonAimSensorRotator parameters ]"
		<< "\n  * use_rotate        : " << ( use_rotate_ ? "True" : "False" )
		<< "\n  * use_sobit_mini    : " << ( use_sobit_mini_ ? "True" : "False" )
        << "\n  * tilt_angle_range : " << tilt_angle_range_[0] << ", " << tilt_angle_range_[1]  
		<< "\n  * person_height    : " << person_height_ 
        << "\n========================================\n"
    << std::endl;

	if ( !use_rotate_ ) return;
	sub_ = nh_.subscribe("/person_follower/rotate_position", 1, &PersonAimSensorRotator::callbackRotatePosition, this);
	sobit_edu_ctr_.movePose( "initial_pose" );
	sobit_edu_ctr_.moveXtionPanTilt ( 0.0, 0.2, 0.3, false );
	pre_pan_ = 0.0;
	pre_tilt_ = -0.4; 
}

void person_follower4::PersonAimSensorRotator::callbackRotatePosition( const person_tracking4::RotatePositionConstPtr& rp_msg ) {
	std::cout << "[ Rotate Sensor ] " << std::endl;
	std::cout << " * Angele [deg] :" << rp_msg->target_angle * 180 / M_PI << "\n * Distance [m] : " << rp_msg->target_distance << "\n" << std::endl;
	if( rp_msg->process_flag == SVDD_LEG_DETECTION ) rotateSensor ( rp_msg->target_distance, rp_msg->target_angle );
	else rotateSensor ( rp_msg->process_flag, rp_msg->target_distance, rp_msg->target_angle );
	return;
}

PLUGINLIB_EXPORT_CLASS(person_follower4::PersonAimSensorRotator, nodelet::Nodelet);