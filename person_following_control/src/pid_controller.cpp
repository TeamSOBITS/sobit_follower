#include <person_following_control/pid_controller.hpp>

namespace person_following_control {

    PIDController::PIDController ( ) {
        setGain( 2.8, 0.1, 0.0 );
        setMaxAngular( 90.0 * M_PI / 180.0 );
    }
    bool PIDController::generatePIRotate ( const double pre_time, const double curt_vel_ang, const double target_angle, geometry_msgs::TwistPtr output_vel ) {
        geometry_msgs::Twist vel;
        double now_time = ros::Time::now().toSec();
        double time_diff = now_time - pre_time;
        double angle_abs = std::abs( target_angle );
        double proportional = 0.0, integral = 0.0, differential = 0.0, ctl_qty = 0.0;

        // if ( angle_abs < 0.523599 ) {
        proportional = p_gain_ * angle_abs;
        integral = i_gain_ * ( ( curt_vel_ang + proportional ) * time_diff / 2 );
        differential = d_gain_ * ( angle_abs / time_diff );
        ctl_qty = proportional + integral - differential;
        // } else {
        //     proportional = p_gain_/2 * angle_abs;
        //     integral = i_gain_ * ( ( curt_vel_ang + proportional ) * time_diff / 2 );
        //     differential = d_gain_ * ( angle_abs / time_diff );
        //     ctl_qty = proportional + integral - differential;
        // }
        if ( ctl_qty > max_angular_ ) ctl_qty = max_angular_;
        if ( target_angle < 0.0 ) ctl_qty = -ctl_qty;
        vel.angular.z = ctl_qty;
        *output_vel = vel;
        return true;
    }
}
