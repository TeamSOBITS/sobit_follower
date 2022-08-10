#include <follow_path_generator3/path_generator.hpp>

using namespace follow_path_generator3;

void PathGenerator::setParameters ( ) {
	std::string target_frame = pnh_.param<std::string>( "target_frame", "base_footprint" );
	dwa_.setTargetFrame( target_frame );
	max_keep_dist_ = pnh_.param<double>( "max_keep_distance", 1.0 );
	min_keep_dist_ = pnh_.param<double>( "min_keep_distance", 0.7 );
	ang_rotate_stop_ = pnh_.param<double>( "angle_rotate_stop", 0.1 );
	ang_rotate_move_ = pnh_.param<double>( "angle_rotate_move", 0.65 );
	allows_reverse_ = pnh_.param<bool>( "allows_reverse", false );
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
	use_omni_ = pnh_.param<bool>( "use_omni", false );
	std::cout << "\n========================================\n[ path generator parameters ]"
		<< "\n  * target_frame           : " << target_frame
		<< "\n  * max_keep_distance      : " << max_keep_dist_
		<< "\n  * min_keep_distance      : " << min_keep_dist_
		<< "\n  * angle_rotate_stop      : " << ang_rotate_stop_
		<< "\n  * angle_rotate_move      : " << ang_rotate_move_
		<< "\n  * allows_reverse         : " << ( allows_reverse_ ? "True" : "False" )
		<< "\n\n< DWA : StepValue >"
		<< "\n  * predict_step           : " << predict_step
		<< "\n  * sampling_time          : " << sampling_time
		<< "\n  * velocity_step          : " << vel_step
		<< "\n  * angle_velocity_step    : " << ang_vel_step
		<< "\n\n< DWA : VelocityLimit >"
		<< "\n  * angle_range            : " << ang_range[0] << ", " << ang_range[1]
		<< "\n  * max_acceration         : " << max_acc
		<< "\n  * max_ang_acceration     : " << max_acc
		<< "\n  * acceleration_gain      : " << max_acc
		<< "\n\n< DWA : Weight >"
		<< "\n  * weight_goal            : " << goal
		<< "\n  * weight_obstacle        : " << obs
		<< "\n  * weight_angle           : " << ang
		<< "\n  * weight_velocity        : " << vel
		<< "\n\n< DWA : CostDistance >"
		<< "\n  * obstacle_center_radius : " << obs_cnt_rad
		<< "\n  * obstacle_points_radius : " << obs_pts_rad
		<< "\n\n< DWA : DisplayFlag >"
		<< "\n  * is_display_path        : " << ( is_display_path ? "True" : "False" )
		<< "\n  * is_display_all_path    : " << ( is_display_all_path ? "True" : "False" )
		<< "\n  * use_omni                : " << ( use_omni_ ? "True" : "False" )
		<< "\n========================================\n"
	<< std::endl;
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

PathGenerator::PathGenerator( ) : nh_(), pnh_("~") {
	setParameters ( );   
	curt_vel_.linear.x = 0.0;
	curt_vel_.angular.z = 0.0;
	// Publisher :
	pub_cmd_vel_ = nh_.advertise< geometry_msgs::Twist >( "/cmd_vel_mux/input/teleop", 1 );
	linear_histories_.clear();
	pre_time_ = ros::Time::now().toSec();
}

void PathGenerator::genetatePath ( const person_tracking3::FollowPosition& fp ) {
	try {
		int control = selectVelocityControl ( fp.exists_target, fp.is_nearby_obstacle , fp.target_distance, fp.target_angle );
		geometry_msgs::Twist vel;
		curt_vel_ = pid_.getVelocity();
		bool should_smoothing = true;

		if ( control == PATH_STOP ) {
			std::cout << "[ PATH_STOP ]" << std::endl;
			vel.linear.x = 0.0;
			vel.angular.z = 0.0;
		} else if ( control == PID_ROTATE ) {
			std::cout << "[ PID_ROTATE ]" << std::endl;
			should_smoothing = false;
			pid_.generatePIRotate( pre_time_, curt_vel_.angular.z, fp.target_angle, &vel );
		} else if ( control == DWA_PATH ) {
			std::cout << "[ DWA_PATH ]" << std::endl;
			if( !dwa_.generatePath2Target ( curt_vel_, fp, &vel ) ) {			
				control = PID_REVESE;
				std::cout << "[ PID_REVESE ]" << std::endl;
				std::cout << " * Linear  : " << 0.3 << "\n * Angular : " << 0.0 << std::endl;
				if( allows_reverse_ ) pid_.controlWheelLinear( -0.3 );
				return;
			}
		} else {
			std::cout << "[ ERROR ]" << std::endl;
			vel.linear.x = 0.0;
			vel.angular.z = 0.0;
		}
		velocityCompensation ( should_smoothing, &vel );
		pub_cmd_vel_.publish( vel );
		pre_time_ = ros::Time::now().toSec();
		std::cout << " * Linear  : " << vel.linear.x << "\n * Angular : " << vel.angular.z << std::endl;
		return;
	} catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return;
    }   
}
