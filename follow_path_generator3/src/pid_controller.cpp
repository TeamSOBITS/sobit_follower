#include <follow_path_generator3/pid_controller.hpp>

namespace follow_path_generator3 {

    PIDController::PIDController ( ) : nh_() {
        sub_odom_ = nh_.subscribe( "/odom", 1, &PIDController::callbackOdometry, this );
        pub_cmd_vel_ = nh_.advertise< geometry_msgs::Twist >( "/cmd_vel_mux/input/teleop", 1 );
        curt_odom_.reset( new nav_msgs::Odometry );
    }
    bool PIDController::generatePIRotate ( double pre_time ,double curt_vel_ang, double target_angle, geometry_msgs::Twist* output_vel ) {
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
    bool PIDController::controlWheelLinear ( const double distance ) {
        try {
            double start_time = ros::Time::now().toSec();
            geometry_msgs::Twist output_vel;
            nav_msgs::Odometry init_odom = *curt_odom_;
            double moving_distance = 0.0;
            double target_distance = std::fabs( distance );
            double Kp = 0.1;
            double Ki = 0.4;
            double Kd = 0.8;
            double velocity_differential = Kp * distance;
            ros::Rate loop_rate(20);
            while ( moving_distance < target_distance  ) {
                ros::spinOnce();
                double end_time = ros::Time::now().toSec();
                double elapsed_time = end_time - start_time;
                double vel_linear = 0.0;
                // PID
                if ( target_distance <= 0.1 ) {
                    vel_linear =  Kp * ( target_distance + 0.001 - moving_distance ) - Kd * velocity_differential + Ki / 0.8 * ( target_distance + 0.001 - moving_distance ) * std::pow( elapsed_time, 2 );
                } else {
                    vel_linear =  Kp * ( target_distance + 0.001 - moving_distance ) - Kd * velocity_differential + Ki / ( 8.0 / target_distance ) * ( target_distance + 0.001 - moving_distance ) * std::pow( elapsed_time, 2 );
                } 
                output_vel.linear.x = ( distance > 0 ) ? vel_linear : -vel_linear;
                velocity_differential = vel_linear;
                pub_cmd_vel_.publish( output_vel );
                double x_diif = curt_odom_->pose.pose.position.x - init_odom.pose.pose.position.x;
                double y_diif = curt_odom_->pose.pose.position.y - init_odom.pose.pose.position.y;
                moving_distance = std::hypotf( x_diif, y_diif );
                ROS_INFO("target_distance = %f\tmoving_distance = %f", target_distance, moving_distance );
                loop_rate.sleep();
            }
            return true;
        } catch ( const std::exception& ex ) {
            ROS_ERROR("%s", ex.what());
            return false;
        }

    }
    
}