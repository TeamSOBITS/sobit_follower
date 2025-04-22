#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#define _USE_MATH_DEFINES

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

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
			bool generatePIRotate ( const rclcpp::Duration time_diff , const double curt_vel_ang, const double target_angle, geometry_msgs::msg::Twist &output_vel );
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
#endif // PID_CONTROLLER_HPP