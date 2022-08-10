#include "sobit_follower/path_generator/pid_controller.h"

namespace PathPlan {

    PIDController::PIDController ( ) : nh_() {
        
    }
    bool PIDController::generatePIDRotate ( double pre_time ,double curt_vel_ang, double target_angle, geometry_msgs::Twist* output_vel ) {
        try {
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
        } catch ( std::exception& ex ) {
            ROS_ERROR("%s", ex.what());
            output_vel->linear.x = 0.0;
            output_vel->angular.z = 0.0;
            return false;
        }
    }
    bool PIDController::reversePID () {
        sobit_edu_ctr_.controlWheelLinear( -0.3 );
        return true;
    }
    
}