#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include "multiple_observation_tracing_simulator/virtual_spring_model.hpp"
#include "multiple_observation_tracing_simulator/VirtualSpringModelParameterConfig.h"

namespace following_control {
    class FollowingControl {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_vel_;
            ros::Subscriber sub_tgt_pose_;
            ros::Subscriber sub_odom_;

            dynamic_reconfigure::Server<multiple_observation_tracing_simulator::VirtualSpringModelParameterConfig>* server_param_;
            dynamic_reconfigure::Server<multiple_observation_tracing_simulator::VirtualSpringModelParameterConfig>::CallbackType f_;

            multiple_observation_tracing_simulator::VirtualSpringModel vsm_;

            void callbackParameter( const multiple_observation_tracing_simulator::VirtualSpringModelParameterConfig& config, uint32_t level);
            void callbackTarget ( const geometry_msgs::PoseStampedConstPtr &pose_msg ) ;
            void callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg );

            geometry_msgs::TwistPtr curt_vel_;
        public:
            FollowingControl( );
    };
}

void following_control::FollowingControl::callbackParameter( const multiple_observation_tracing_simulator::VirtualSpringModelParameterConfig& config, uint32_t level){
    vsm_.setFollowParamater( config.following_angle_deg, config.following_distance );
    vsm_.setSpringParamater( config.spring_constant_linear, config.spring_constant_angular );
    vsm_.setFrictionParamater( config.viscous_friction_linear, config.viscous_friction_angular );
    vsm_.setRobotParamater( config.weight_robot, config.radius_robot );
    vsm_.setMomentParamater( config.moment_inertia );

    std::cout  << "\n============================================================"
            << "\n[ Virtual Spring Model Paramater(Dynamic Reconfigure)  ]"
            << "\n* following_angle_deg      [deg]      : " << config.following_angle_deg
            << "\n* following_distance       [m]        : " << config.following_distance
            << "\n* spring_constant_linear   [N/m]      : " << config.spring_constant_linear
            << "\n* spring_constant_angular  [N・m/rad] : " << config.spring_constant_angular
            << "\n* weight_robot             [Kg]       : " << config.weight_robot
            << "\n* moment_inertia           [Kg・m^2]  : " << config.moment_inertia
            << "\n* viscous_friction_linear  [N・s/m]   : " << config.viscous_friction_linear
            << "\n* viscous_friction_angular [N・s/rad] : " << config.viscous_friction_angular
            << "\n* radius_robot             [m]        : " << config.radius_robot
    << std::endl;
    ros::Duration(1.0).sleep();
}

void following_control::FollowingControl::callbackTarget ( const geometry_msgs::PoseStampedConstPtr &pose_msg )  {
    geometry_msgs::TwistPtr vel ( new  geometry_msgs::Twist );
    vsm_.compute( pose_msg, curt_vel_->linear.x, curt_vel_->angular.z, vel );
    pub_vel_.publish(vel);
    return;
}

void following_control::FollowingControl::callbackOdometry ( const nav_msgs::OdometryConstPtr &odom_msg ) {
    *curt_vel_ = odom_msg->twist.twist;
    return;
}

following_control::FollowingControl::FollowingControl( ) : nh_(), pnh_("~") {
    pub_vel_ = nh_.advertise< geometry_msgs::Twist >( "/cmd_vel_mux/input/teleop", 1 );

    sub_tgt_pose_ = nh_.subscribe( "/target_pose", 10, &FollowingControl::callbackTarget, this );
    sub_odom_ = nh_.subscribe( "/odom", 10, &FollowingControl::callbackOdometry, this );

    server_param_ = new dynamic_reconfigure::Server<multiple_observation_tracing_simulator::VirtualSpringModelParameterConfig>(pnh_);
    f_ = boost::bind(&FollowingControl::callbackParameter, this, _1, _2);
    server_param_->setCallback(f_);

    curt_vel_.reset( new  geometry_msgs::Twist );
}

int main(int argc, char *argv[])  {
    ros::init(argc, argv, "virtual_spring_model_following_control");
    following_control::FollowingControl following_control;
    ros::spin();
    return 0;
}
