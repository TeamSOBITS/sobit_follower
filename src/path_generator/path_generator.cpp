#include <iostream>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <sobit_follower/FollowPosition.h>
#include "sobit_follower/path_generator/dynamic_window_approach.h"
#include "sobit_follower/path_generator/pid_controller.h"

// Path Generator Mode
constexpr int STOP = 4;
constexpr int FOLLOW = 5;
constexpr int ATTENTION = 6;

// Path Type
constexpr int DWA_PATH = 7;
constexpr int PID_ROTATE = 8;
constexpr int PID_REVESE = 9;
constexpr int PATH_STOP = 10;

class PathGenerator {
    private :
		ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
		ros::Publisher pub_cmd_vel_;
		ros::Publisher pub_path_type_;
		ros::Subscriber sub_fp_;
		ros::Subscriber sub_odom_;
		ros::Subscriber sub_switch_path_;
		PathPlan::DynamicWindowApproach dwa_;
		PathPlan::PIDController pid_;
		double max_keep_dist_;
		double min_keep_dist_;
		double ang_rotate_stop_;
		double ang_rotate_move_;
		geometry_msgs::Twist curt_vel_;
		std::vector<double> linear_histories_;
		double pre_time_;
		int generate_mode_;

		void setParameters ( );
		int selectVelocityControl ( const bool exists_target, const bool is_nearby_obstacle ,const double target_distance, const double target_angle );
		void velocityCompensation ( const bool should_smoothing, geometry_msgs::Twist* output_vel  );
        void callbackFollowPosition  ( const sobit_follower::FollowPositionConstPtr &input );
        void callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg );
		void callbackSwitchPath ( const std_msgs::Int64ConstPtr &input );
    public :
        PathGenerator( );
};

void PathGenerator::setParameters ( ) {
	std::string target_frame = pnh_.param<std::string>( "target_frame", "base_footprint" );
	dwa_.setTargetFrame( target_frame );
	max_keep_dist_ = pnh_.param<double>( "max_keep_distance", 1.0 );
	min_keep_dist_ = pnh_.param<double>( "min_keep_distance", 0.7 );
	ang_rotate_stop_ = pnh_.param<double>( "angle_rotate_stop", 0.1 );
	ang_rotate_move_ = pnh_.param<double>( "angle_rotate_move", 0.65 );
	int predict_step = pnh_.param<int>( "predict_step", 30 );
	double sampling_time = pnh_.param<double>( "sampling_time", 0.1 );
	double vel_step = pnh_.param<double>( "velocity_step", 7.0 );
	double ang_vel_step = pnh_.param<double>( "angle_velocity_step", 15.0 );
	dwa_.setStepValue( predict_step, sampling_time, vel_step, ang_vel_step );
	std::vector<double> ang_range = pnh_.param<std::vector<double>>( "angle_range", { -1.0, 1.0 } );
	double max_acc = pnh_.param<double>( "max_acceration", 1.5 );
	double max_ang_acc = pnh_.param<double>( "max_ang_acceration", 1.5 );
	double acc_gain = pnh_.param<double>( "acceleration_gain", 1.1 );
	dwa_.setVelocityLimit( ang_range, max_acc, max_ang_acc, acc_gain );
	double goal = pnh_.param<double>( "weight_goal", 1.0 );
	double obs = pnh_.param<double>( "weight_obstacle", 1.0 );
	double ang = pnh_.param<double>( "weight_angle", 2.0 );
	double vel = pnh_.param<double>( "weight_velocity", 1.0 );
	dwa_.setWeight( goal, obs, ang, vel );
	double obs_cnt_rad = pnh_.param<double>( "obstacle_center_radius", 0.35 );
	double obs_pts_rad = pnh_.param<double>( "obstacle_points_radius", 0.35 );
	dwa_.setCostDistance ( obs_cnt_rad, obs_pts_rad );
	bool is_display_path = pnh_.param<bool>( "is_display_path", false );
	bool is_display_all_path = pnh_.param<bool>( "is_display_all_path", false );
	dwa_.setDisplayFlag( is_display_path, is_display_all_path );
	std::cout << "\n========================================\n[ path generator parameters ]"
		<< "\n  * target_frame : " << target_frame
		<< "\n  * max_keep_distance : " << max_keep_dist_
		<< "\n  * min_keep_distance : " << min_keep_dist_
		<< "\n  * angle_rotate_stop : " << ang_rotate_stop_
		<< "\n  * angle_rotate_move : " << ang_rotate_move_
		<< "\n\n< DWA : StepValue >"
		<< "\n  * predict_step : " << predict_step
		<< "\n  * sampling_time : " << sampling_time
		<< "\n  * velocity_step : " << vel_step
		<< "\n  * angle_velocity_step : " << ang_vel_step
		<< "\n\n< DWA : VelocityLimit >"
		<< "\n  * angle_range : " << ang_range[0] << ", " << ang_range[1]
		<< "\n  * max_acceration : " << max_acc
		<< "\n  * max_ang_acceration : " << max_acc
		<< "\n  * acceleration_gain : " << max_acc
		<< "\n\n< DWA : Weight >"
		<< "\n  * weight_goal : " << goal
		<< "\n  * weight_obstacle : " << obs
		<< "\n  * weight_angle : " << ang
		<< "\n  * weight_velocity : " << vel
		<< "\n\n< DWA : CostDistance >"
		<< "\n  * obstacle_center_radius : " << obs_cnt_rad
		<< "\n  * obstacle_points_radius : " << obs_pts_rad
		<< "\n\n< DWA : DisplayFlag >"
		<< "\n  * is_display_path : " << ( is_display_path ? "True" : "False" )
		<< "\n  * is_display_all_path : " << ( is_display_all_path ? "True" : "False" )
		<< "\n========================================\n"
	<< std::endl;
}
inline int PathGenerator::selectVelocityControl ( const bool exists_target, const bool is_nearby_obstacle ,const double target_distance, const double target_angle ) {
	switch (generate_mode_) {
		case STOP:
			return PATH_STOP;
			break;
		case FOLLOW:
			if ( !exists_target ) return PATH_STOP;
			if ( target_distance > max_keep_dist_ ) return DWA_PATH;
			else if ( target_distance <= max_keep_dist_ && target_distance > min_keep_dist_ && is_nearby_obstacle ) return DWA_PATH;
			else return PID_ROTATE;
			break;
		case ATTENTION:
			if ( !exists_target ) return PATH_STOP;
			else return PID_ROTATE;
			break;
	}

}
void PathGenerator::velocityCompensation ( const bool should_smoothing, geometry_msgs::Twist* output_vel  ) {
	geometry_msgs::Twist tmp_vel = *output_vel;
	linear_histories_.push_back ( tmp_vel.linear.x );
	if ( linear_histories_.size() > 10 ) linear_histories_.erase( linear_histories_.begin() );

	if ( ( tmp_vel.linear.x > 0.0 && curt_vel_.linear.x < 0.0 ) ) tmp_vel.linear.x = 0.0;
		
	if ( should_smoothing && tmp_vel.linear.x == 0.0 && fabs ( curt_vel_.linear.x ) > 0.0 ) {
		double vel_ave = 0.0;
		for ( auto& vel : linear_histories_ ) vel_ave += vel;
		vel_ave = vel_ave / double(linear_histories_.size());
		tmp_vel.linear.x = vel_ave;
		*output_vel = tmp_vel;
	}
	return;
}
void PathGenerator::callbackFollowPosition  ( const sobit_follower::FollowPositionConstPtr &input ) {
	sobit_follower::FollowPosition fp = *input;
	int control = selectVelocityControl ( fp.exists_target, fp.is_nearby_obstacle , fp.target_distance, fp.target_angle );
	geometry_msgs::Twist vel;
	bool should_smoothing = true;
	if ( control == PATH_STOP ) {
		ROS_INFO("PATH_STOP");
		vel.linear.x = 0.0;
		vel.angular.z = 0.0;
	} else if ( control == PID_ROTATE ) {
		ROS_INFO("PID_ROTATE");
		should_smoothing = false;
		pid_.generatePIDRotate( pre_time_, curt_vel_.angular.z, fp.target_angle, &vel );
	} else if ( control == DWA_PATH ) {
		ROS_INFO("DWA_PATH");
		if( !dwa_.generatePath2Target ( curt_vel_, input, &vel ) ) {			
			control = PID_REVESE;
			ROS_INFO( "\n< PID_REVESE >\nLinear  : -0.3 [m/s]\nAngular : 0.0 [rad/s]\n" );
			std_msgs::Int64 ctr;
			ctr.data = control;
			pub_path_type_.publish(ctr);
			pid_.reversePID();
			return;
		}
	} else {
		ROS_ERROR("ERROR");
		vel.linear.x = 0.0;
		vel.angular.z = 0.0;
	}
	velocityCompensation ( should_smoothing, &vel );
	pub_cmd_vel_.publish( vel );
	pre_time_ = ros::Time::now().toSec();
	std_msgs::Int64 ctr;
	ctr.data = control;
	pub_path_type_.publish(ctr);
	return;
}
inline void PathGenerator::callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg ) { curt_vel_ = odom_msg->twist.twist; }
inline void PathGenerator::callbackSwitchPath ( const std_msgs::Int64ConstPtr &input ){ generate_mode_ = input->data; }
PathGenerator::PathGenerator( ) : nh_(), pnh_("~") {
	setParameters ( );   
	curt_vel_.linear.x = 0.0;
	curt_vel_.angular.z = 0.0;
	// Subscriber :
	sub_fp_ = nh_.subscribe( "/follow_position", 1, &PathGenerator::callbackFollowPosition, this );
	sub_odom_ = nh_.subscribe( "/odom", 1, &PathGenerator::callbackOdometry, this );
	sub_switch_path_ = nh_.subscribe("/follow_me_handle/path_mode", 1, &PathGenerator::callbackSwitchPath, this); 
	// Publisher :
	pub_cmd_vel_ = nh_.advertise< geometry_msgs::Twist >( "/cmd_vel_mux/input/teleop", 1 );
	pub_path_type_ = nh_.advertise< std_msgs::Int64 >( "path_type", 1 );
	linear_histories_.clear();
	pre_time_ = ros::Time::now().toSec();
	generate_mode_ = FOLLOW;
	//generate_mode_ = STOP;
}
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "path_generator_node");
	PathGenerator path_generator;
    ros::spin();
}
