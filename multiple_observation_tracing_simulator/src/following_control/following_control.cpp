#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
// #include <dynamic_reconfigure/server.h>
#include "multiple_observation_tracing_simulator/virtual_spring_model.hpp"
// #include "multiple_observation_tracing_simulator/VirtualSpringModelParameterConfig.h"

namespace following_control {
    class FollowingControl : public rclcpp::Node {
        private:
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_tgt_pose_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

            // dynamic_reconfigure::Server<multiple_observation_tracing_simulator::VirtualSpringModelParameterConfig>* server_param_;
            // dynamic_reconfigure::Server<multiple_observation_tracing_simulator::VirtualSpringModelParameterConfig>::CallbackType f_;

            multiple_observation_tracing_simulator::VirtualSpringModel vsm_;

            // void callbackParameter( const multiple_observation_tracing_simulator::VirtualSpringModelParameterConfig& config, uint32_t level);
            void callbackTarget ( const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg ) ;
            void callbackOdometry ( const nav_msgs::msg::Odometry::SharedPtr odom_msg );

            geometry_msgs::msg::Twist::SharedPtr curt_vel_;
        public:
            FollowingControl( );
    };
}

// void following_control::FollowingControl::callbackParameter( const multiple_observation_tracing_simulator::VirtualSpringModelParameterConfig& config, uint32_t level){
//     vsm_.setFollowParameter( config.following_angle_deg, config.following_distance );
//     vsm_.setSpringParameter( config.spring_constant_linear, config.spring_constant_angular );
//     vsm_.setFrictionParameter( config.viscous_friction_linear, config.viscous_friction_angular );
//     vsm_.setRobotParameter( config.weight_robot, config.radius_robot );
//     vsm_.setMomentParameter( config.moment_inertia );

//     std::cout  << "\n============================================================"
//             << "\n[ Virtual Spring Model Parameter(Dynamic Reconfigure)  ]"
//             << "\n* following_angle_deg      [deg]      : " << config.following_angle_deg
//             << "\n* following_distance       [m]        : " << config.following_distance
//             << "\n* spring_constant_linear   [N/m]      : " << config.spring_constant_linear
//             << "\n* spring_constant_angular  [N・m/rad] : " << config.spring_constant_angular
//             << "\n* weight_robot             [Kg]       : " << config.weight_robot
//             << "\n* moment_inertia           [Kg・m^2]  : " << config.moment_inertia
//             << "\n* viscous_friction_linear  [N・s/m]   : " << config.viscous_friction_linear
//             << "\n* viscous_friction_angular [N・s/rad] : " << config.viscous_friction_angular
//             << "\n* radius_robot             [m]        : " << config.radius_robot
//     << std::endl;
//     ros::Duration(1.0).sleep();
// }

void following_control::FollowingControl::callbackTarget ( const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg )  {
    auto vel = std::make_shared<geometry_msgs::msg::Twist>();
    vsm_.compute( pose_msg, curt_vel_->linear.x, curt_vel_->angular.z, *vel );
    pub_vel_->publish(*vel);
    return;
}

void following_control::FollowingControl::callbackOdometry ( const nav_msgs::msg::Odometry::SharedPtr odom_msg ) {
    *curt_vel_ = odom_msg->twist.twist;
    return;
}

following_control::FollowingControl::FollowingControl( ) : Node("virtual_spring_model_following_control") {
    pub_vel_ = this->create_publisher< geometry_msgs::msg::Twist >( "/cmd_vel_mux/input/teleop", 1 );

    sub_tgt_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( "/target_pose", 10, std::bind(&FollowingControl::callbackTarget, this, std::placeholders::_1) );
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>( "/odom", 10, std::bind(&FollowingControl::callbackOdometry, this, std::placeholders::_1) );

    // server_param_ = new dynamic_reconfigure::Server<multiple_observation_tracing_simulator::VirtualSpringModelParameterConfig>(pnh_);
    // f_ = boost::bind(&FollowingControl::callbackParameter, this, _1, _2);
    // server_param_->setCallback(f_);

    // Load parameters
    this->declare_parameter("following_angle_deg", 0.0);
    this->declare_parameter("following_distance", 0.7);
    this->declare_parameter("spring_constant_linear", 3.0);
    this->declare_parameter("spring_constant_angular", 0.001);
    this->declare_parameter("weight_robot", 30.0);
    this->declare_parameter("radius_robot", 0.3);
    this->declare_parameter("moment_inertia", 15.0);
    this->declare_parameter("viscous_friction_linear", 30.0);
    this->declare_parameter("viscous_friction_angular", 20.0);

    vsm_.setFollowParameter(
        this->get_parameter("following_angle_deg").as_double(),
        this->get_parameter("following_distance").as_double());

    vsm_.setSpringParameter(
        this->get_parameter("spring_constant_linear").as_double(),
        this->get_parameter("spring_constant_angular").as_double());

    vsm_.setRobotParameter(
        this->get_parameter("weight_robot").as_double(),
        this->get_parameter("radius_robot").as_double());

    vsm_.setMomentParameter(
        this->get_parameter("moment_inertia").as_double());

    vsm_.setFrictionParameter(
        this->get_parameter("viscous_friction_linear").as_double(),
        this->get_parameter("viscous_friction_angular").as_double());

    curt_vel_ = std::make_shared<geometry_msgs::msg::Twist>();
}

int main(int argc, char *argv[])  {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<following_control::FollowingControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}