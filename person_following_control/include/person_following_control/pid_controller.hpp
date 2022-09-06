#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace person_following_control {
    class PIDController {
		private :
            double p_gain_;
            double i_gain_;
            double d_gain_;
            double max_angular_;

		public :
			PIDController ( );
            void setGain( const double p_gain, const double i_gain, const double d_gain );
            void setMaxAngular( const double max_angular_rad );
			bool generatePIRotate ( const double pre_time , const double curt_vel_ang, const double target_angle, geometry_msgs::TwistPtr output_vel );
    };

    inline void PIDController::setGain( const double p_gain, const double i_gain, const double d_gain ) {
        p_gain_ = p_gain;
        i_gain_ = i_gain;
        d_gain_ = d_gain;
        return;
    }
    inline void PIDController::setMaxAngular( const double max_angular_rad ) {
        max_angular_ = max_angular_rad;
        return;
    }
}
#endif
