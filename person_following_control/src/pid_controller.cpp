#include <person_following_control/pid_controller.hpp>

namespace person_following_control {

    PIDController::PIDController ( ) {

    }
    bool PIDController::generatePIRotate ( double pre_time ,double curt_vel_ang, double target_angle, geometry_msgs::TwistPtr output_vel ) {
        geometry_msgs::Twist vel;
        double now_time = ros::Time::now().toSec();
        double time_diff = now_time - pre_time;
        double angle_abs = std::abs( target_angle );
        double proportinal = 2.8 * angle_abs;
        double integral = 0.1 * ( ( curt_vel_ang + proportinal ) * time_diff / 2 );
        double differential = 0 * ( angle_abs / time_diff );
        double ctl_qty = proportinal + integral - differential;
        if ( ctl_qty > M_PI/2 ) ctl_qty = M_PI/2;
        if ( target_angle < 0.0 ) ctl_qty = -ctl_qty;
        vel.angular.z = ctl_qty;
        *output_vel = vel;
        return true;
    }
}
