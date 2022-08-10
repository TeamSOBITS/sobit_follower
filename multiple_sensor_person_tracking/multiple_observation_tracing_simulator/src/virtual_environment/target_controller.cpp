#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <multiple_observation_tracing_simulator/TargetControllerParameterConfig.h>
#include<bits/stdc++.h>

#define FREE 0
#define LINE 1
#define CIRCLE 2
#define RANDOM 3

namespace multiple_observation_tracing_simulator {
    class TargetController {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_teleop_;
            ros::Timer timer_;

            unsigned int moving_mode_;
            double linear_speed_;
            double circle_theta_deg_;

            std::random_device rnd_;
            std::unique_ptr<std::mt19937> mt_;
            std::unique_ptr<std::uniform_real_distribution<double>> rand_theta_deg_;

            double pre_time_;
            double rand_ang_;

            dynamic_reconfigure::Server<multiple_observation_tracing_simulator::TargetControllerParameterConfig>* server_;
            dynamic_reconfigure::Server<multiple_observation_tracing_simulator::TargetControllerParameterConfig>::CallbackType f_;

            void callbackDynamicReconfigure(multiple_observation_tracing_simulator::TargetControllerParameterConfig& config, uint32_t level);
            void callbackTimer( const ros::TimerEvent& e );

        public:
            TargetController( );
    };
}

void multiple_observation_tracing_simulator::TargetController::callbackDynamicReconfigure(multiple_observation_tracing_simulator::TargetControllerParameterConfig& config, uint32_t level) {
    moving_mode_ = config.moving_mode;
    linear_speed_ = config.linear_speed;
    circle_theta_deg_ = config.circle_theta_deg;
    rand_theta_deg_.reset( new std::uniform_real_distribution<double>( -config.random_theta_deg, config.random_theta_deg ) );
}

void multiple_observation_tracing_simulator::TargetController::callbackTimer( const ros::TimerEvent& e ) {
    geometry_msgs::TwistPtr vel ( new geometry_msgs::Twist );
    if( moving_mode_ == FREE ) return;
    else if( moving_mode_ == LINE ) {
        vel->linear.x = linear_speed_;
    } else if( moving_mode_ == CIRCLE ) {
        vel->linear.x = linear_speed_;
        vel->angular.z = circle_theta_deg_ * M_PI / 180.0;
    } else if ( moving_mode_ == RANDOM ) {
        double curt_time = ros::Time::now().toSec();
        vel->linear.x = linear_speed_;
        if ( curt_time - pre_time_ > 1.0 ) {
            pre_time_ = curt_time;
            rand_ang_ = (*rand_theta_deg_)(*mt_) * M_PI / 180.0;
        } else vel->angular.z = rand_ang_;
    }
    pub_teleop_.publish( vel );
    return;
}

multiple_observation_tracing_simulator::TargetController::TargetController( ) : nh_(), pnh_("~") {
    pub_teleop_ = nh_.advertise<geometry_msgs::Twist>("/target/teleop", 1);
    timer_ = nh_.createTimer( ros::Duration(0.033), &multiple_observation_tracing_simulator::TargetController::callbackTimer, this );

    server_ = new dynamic_reconfigure::Server<multiple_observation_tracing_simulator::TargetControllerParameterConfig>(pnh_);
    f_ = boost::bind(&multiple_observation_tracing_simulator::TargetController::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);

    mt_.reset( new std::mt19937(rnd_()) );
    pre_time_ = ros::Time::now().toSec();
    rand_ang_ = 0.0;
}

int main(int argc, char *argv[])  {
    ros::init(argc, argv, "topic_publisher_template");
    multiple_observation_tracing_simulator::TargetController target_controller;
    ros::spin();
    return 0;
}
