#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace follow_path_generator3 {
    class PIDController {
		private : 
			ros::NodeHandle nh_;
			ros::Publisher pub_cmd_vel_;
            ros::Subscriber sub_odom_;
            nav_msgs::OdometryPtr curt_odom_;

			void callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg );
		public :
			PIDController ( );
			bool generatePIRotate ( double pre_time ,double curt_vel_ang, double target_angle, geometry_msgs::Twist* output_vel );
			bool controlWheelLinear ( const double distance );
			geometry_msgs::Twist getVelocity();
    };
	inline void PIDController::callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg ) { *curt_odom_ = *odom_msg; }
	inline geometry_msgs::Twist PIDController::getVelocity(){ return curt_odom_->twist.twist; }
}
#endif