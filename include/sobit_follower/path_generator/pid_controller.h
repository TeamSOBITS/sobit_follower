#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sobit_education_library/sobit_education_controller.hpp>

namespace PathPlan {
    class PIDController {
	private : 
		ros::NodeHandle nh_;
		sobit_education::SobitEducationController sobit_edu_ctr_;
	public :
		PIDController ( );
		bool generatePIDRotate ( double pre_time ,double curt_vel_ang, double target_angle, geometry_msgs::Twist* output_vel );
		bool reversePID ();
    };
}
#endif