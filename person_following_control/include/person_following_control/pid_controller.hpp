#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace person_following_control {
    class PIDController {
		private :

		public :
			PIDController ( );
			bool generatePIRotate ( double pre_time ,double curt_vel_ang, double target_angle, geometry_msgs::TwistPtr output_vel );
    };
}
#endif
