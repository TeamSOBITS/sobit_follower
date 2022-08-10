#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <follow_path_generator4/dynamic_window_approach.hpp>
#include <follow_path_generator4/pid_controller.hpp>

// Path Generator Mode
constexpr int STOP = 4;
constexpr int FOLLOW = 5;
constexpr int ATTENTION = 6;

// Path Type
constexpr int DWA_PATH = 7;
constexpr int PID_ROTATE = 8;
constexpr int PID_REVESE = 9;
constexpr int PATH_STOP = 10;

namespace person_follower4 {
    class PathGenerator : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
			ros::Subscriber sub_fp_;
            ros::Publisher pub_cmd_vel_;

            follow_path_generator4::DynamicWindowApproach dwa_;
            follow_path_generator4::PIDController pid_;

            double max_keep_dist_;
            double min_keep_dist_;
            double ang_rotate_stop_;
            double ang_rotate_move_;
            bool allows_reverse_;

            geometry_msgs::TwistPtr curt_vel_;
            std::vector<double> linear_histories_;
            double pre_time_;

            int selectVelocityControl ( const bool exists_target, const double target_distance, const double target_angle );
            void velocityCompensation ( const bool should_smoothing, geometry_msgs::TwistPtr output_vel  );

        public:
            virtual void onInit();
            void callbackFollowPosition( const person_tracking4::FollowPositionConstPtr &fp_msg );
    };
}

int person_follower4::PathGenerator::selectVelocityControl ( const bool exists_target, const double target_distance, const double target_angle ) {
    if ( !exists_target ) return PATH_STOP;
    if ( target_distance > max_keep_dist_ ) return DWA_PATH;
    else if ( target_distance <= max_keep_dist_ && target_distance > min_keep_dist_ ) return DWA_PATH;
    else return PID_ROTATE;
}

void person_follower4::PathGenerator::velocityCompensation ( const bool should_smoothing, geometry_msgs::TwistPtr output_vel  ) {
	geometry_msgs::Twist tmp_vel = *output_vel;
	linear_histories_.push_back ( tmp_vel.linear.x );
	if ( linear_histories_.size() > 10 ) linear_histories_.erase( linear_histories_.begin() );

	if ( ( tmp_vel.linear.x > 0.0 && curt_vel_->linear.x < 0.0 ) ) tmp_vel.linear.x = 0.0;
		
	if ( should_smoothing && tmp_vel.linear.x == 0.0 && fabs ( curt_vel_->linear.x ) > 0.0 ) {
		double vel_ave = 0.0;
		for ( auto& vel : linear_histories_ ) vel_ave += vel;
		vel_ave = vel_ave / double(linear_histories_.size());
		tmp_vel.linear.x = vel_ave;
		*output_vel = tmp_vel;
	}
	return;
}

void person_follower4::PathGenerator::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    sub_fp_ = nh_.subscribe("/person_follower/follow_position", 10, &PathGenerator::callbackFollowPosition, this);
	pub_cmd_vel_ = nh_.advertise< geometry_msgs::Twist >( "velocity", 1 );
	linear_histories_.clear();
	pre_time_ = ros::Time::now().toSec();

	curt_vel_.reset( new geometry_msgs::Twist );
	curt_vel_->linear.x = 0.0;
	curt_vel_->angular.z = 0.0;
}

void person_follower4::PathGenerator::callbackFollowPosition( const person_tracking4::FollowPositionConstPtr &fp_msg ) {
    int control = selectVelocityControl ( fp_msg->exists_target, fp_msg->target_distance, fp_msg->target_angle );
    geometry_msgs::TwistPtr vel ( new geometry_msgs::Twist );
    *curt_vel_ = pid_.getVelocity();
    bool should_smoothing = true;

    if ( control == PATH_STOP ) {
        std::cout << "[ PATH_STOP ]" << std::endl;
        vel->linear.x = 0.0;
        vel->angular.z = 0.0;
    } else if ( control == PID_ROTATE ) {
        std::cout << "[ PID_ROTATE ]" << std::endl;
        should_smoothing = false;
        pid_.generatePIRotate( pre_time_, curt_vel_->angular.z, fp_msg->target_angle, vel );
    } else if ( control == DWA_PATH ) {
        std::cout << "[ DWA_PATH ]" << std::endl;
		PointCloud::Ptr cloud_obstacle ( new PointCloud() );
	    pcl::fromROSMsg<PointT>( fp_msg->obstacles, *cloud_obstacle );
        if( !dwa_.generatePath2Target ( fp_msg->target, cloud_obstacle, curt_vel_, vel ) ) {			
            control = PID_REVESE;
            if( allows_reverse_ ) {
				std::cout << "[ PID_REVESE ]" << std::endl;
				std::cout << " * Linear  : " << 0.3 << "\n * Angular : " << 0.0 << std::endl;
				pid_.controlWheelLinear( -0.3 );
			} else std::cout << "[ NO PATH ] : No Path found (too many obstacles) " << std::endl;
        }
    } else {
        std::cout << "[ ERROR ]" << std::endl;
        vel->linear.x = 0.0;
        vel->angular.z = 0.0;
    }
    //velocityCompensation ( should_smoothing, vel );
    pub_cmd_vel_.publish( vel );
    pre_time_ = ros::Time::now().toSec();
    std::cout << " * Linear  : " << vel->linear.x << "\n * Angular : " << vel->angular.z << std::endl;
    return;
}

PLUGINLIB_EXPORT_CLASS(person_follower4::PathGenerator, nodelet::Nodelet);
