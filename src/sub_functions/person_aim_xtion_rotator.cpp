#include <iostream>
#include <ros/ros.h> 
#include <ros/time.h>
#include <ros/duration.h>  
#include <tf/transform_listener.h>  
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <sobit_common_msg/gripper_ctrl.h>
#include <sobit_follower/FollowPosition.h>
#include <sobit_education_library/sobit_education_controller.hpp>

constexpr int DETECTION = 1;
constexpr int TRACKING = 2;
constexpr int PREDICTION = 3;

class PersonAimXtionRotator {
	private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber sub_fp_;
        tf::TransformListener tf_listener_;
		bool is_sobit_mini_;
		std::vector<double> tilt_angle_range_;
		double person_height_;
		sobit_education::SobitEducationController sobit_edu_ctr_;
		double pre_tilt_;
		double pre_pan_;
		
		double getAngleFrameDifference( std::string org_frame, std::string target_frame );
		void callbackFollowPosition  ( const sobit_follower::FollowPositionConstPtr &input );

	public:
		PersonAimXtionRotator();
};


double PersonAimXtionRotator::getAngleFrameDifference( std::string org_frame, std::string target_frame ) {
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

void PersonAimXtionRotator::callbackFollowPosition  ( const sobit_follower::FollowPositionConstPtr &input ) {
	int process_flag = input->process_flag;
	double tilt_angle, pan_angle;
	tilt_angle = -1.0 *  std::atan2( person_height_, input->target_distance );
	if ( input->target_distance == 0.0 ) tilt_angle = tilt_angle_range_[1];
	else if ( tilt_angle < tilt_angle_range_[0] ) tilt_angle = tilt_angle_range_[0];
	else if ( tilt_angle > tilt_angle_range_[1] ) tilt_angle = tilt_angle_range_[1];

	if ( is_sobit_mini_ ) {
		double angle_frame_diff = getAngleFrameDifference( "base_footprint", "body_roll_link" );
		pan_angle = ( process_flag != DETECTION ) ? input->target_angle - angle_frame_diff : 0.0;
	} else {
		pan_angle = ( process_flag != DETECTION ) ? input->target_angle : 0.0;
	}
	double sec = ( process_flag != DETECTION ) ? 0.1 : 1.0;
	if( process_flag == PREDICTION ) sobit_edu_ctr_.moveXtionPanTilt ( pre_pan_, pre_tilt_, sec, false );
	else { 
		sobit_edu_ctr_.moveXtionPanTilt ( pan_angle, tilt_angle, sec, false );
		pre_pan_ = pan_angle;
		pre_tilt_ = tilt_angle;
	}
}

PersonAimXtionRotator::PersonAimXtionRotator() : nh_(), pnh_("~") {
	sub_fp_ = nh_.subscribe("/follow_position", 1, &PersonAimXtionRotator::callbackFollowPosition, this);   
	is_sobit_mini_ = pnh_.param<bool>( "is_sobit_mini", false );
	tilt_angle_range_ = pnh_.param<std::vector<double>>( "tilt_angle_range", {-0.69, -0.25} );
	person_height_ = pnh_.param<double>( "camera2person_height", 0.5 );
	sobit_edu_ctr_.movePose( "initial_pose" );
	sobit_edu_ctr_.moveXtionPanTilt ( 0.0, 0.4, 0.4, false );
	pre_pan_ = 0.0;
	pre_tilt_ = 0.4; 
}


int main(int argc, char *argv[]) {
	ros::init(argc, argv, "person_aim_xtion_rotator");
	PersonAimXtionRotator person_aim_xtion_rotator;
	ros::spin();
}
