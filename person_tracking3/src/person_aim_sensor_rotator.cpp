#include <person_tracking3/person_aim_sensor_rotator.hpp>

using namespace person_tracking3;

double PersonAimSensorRotator::getAngleFrameDifference( std::string org_frame, std::string target_frame ) {
	tf::StampedTransform transform;
	try {
		tf_listener_.waitForTransform( org_frame, target_frame, ros::Time(0), ros::Duration(1.0) );
		tf_listener_.lookupTransform( org_frame, target_frame, ros::Time(0), transform );
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		return 0.0;
	}
	tf::Quaternion quat = transform.getRotation();
	double roll, pitch, yaw;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	double angle_frame_diff = yaw;
	return angle_frame_diff;
}

PersonAimSensorRotator::PersonAimSensorRotator() : nh_(), pnh_("~") {
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
	sobit_edu_ctr_.movePose( "initial_pose" );
	sobit_edu_ctr_.moveXtionPanTilt ( 0.0, 0.2, 0.3, false );
	pre_pan_ = 0.0;
	pre_tilt_ = -0.4; 
}

void PersonAimSensorRotator::rotateSensor ( const int process_flag, const double target_distance, const double target_angle ) {
	if ( !use_rotate_ ) return;
	double tilt_angle = 0.0, pan_angle = 0.0;
	tilt_angle = std::atan2( person_height_, target_distance );
	if ( target_distance == 0.0 ) tilt_angle = 0.2;
	else if ( tilt_angle < tilt_angle_range_[0] ) tilt_angle = tilt_angle_range_[0];
	else if ( tilt_angle > tilt_angle_range_[1] ) tilt_angle = tilt_angle_range_[1];

	if ( use_sobit_mini_ ) {
		double angle_frame_diff = getAngleFrameDifference( "base_footprint", "body_roll_link" );
		pan_angle = ( process_flag != 1 ) ? -(target_angle - angle_frame_diff) : 0.0;
	} else {
		pan_angle = ( process_flag != 1 ) ? -target_angle : 0.0;
	}

	if ( pre_flag_ == 1 && process_flag == 2 ) pre_pan_ = pan_angle;
	pan_angle = 0.95 * pan_angle + 0.05 * pre_pan_;

	if ( process_flag == 1 ) pan_angle = 0.0;
	
	double sec = ( process_flag != 1 ) ? 0.1 : 1.0;
	if( process_flag == 3 ) sobit_edu_ctr_.moveXtionPanTilt ( pre_pan_, pre_tilt_, sec, false );
	else { 
		sobit_edu_ctr_.moveXtionPanTilt ( pan_angle, tilt_angle, sec, false );
		pre_pan_ = pan_angle;
		pre_tilt_ = tilt_angle;
	}
	pre_flag_ = process_flag;
	return;
}

void PersonAimSensorRotator::rotateSensor ( const double target_distance, const double target_angle ) {
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