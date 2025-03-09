#ifndef VIRTUAL_SPRING_MODEL
#define VIRTUAL_SPRING_MODEL

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2/transform_datatypes.h>
#include <Eigen/Core>


namespace multiple_observation_tracing_simulator{
    class VirtualSpringModel : public rclcpp::Node {
        private :
            // rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_mrk_tgt_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_mrk_path_;

            double ang_follow_;                 // theta_sh [rad]
            double dist_follow_;                // l_0 [m]

            double spring_constant_linear_;     // k_1 [N/m]
            double spring_constant_angular_;    // k_2 [N・m/rad]
            double weight_robot_;               // M [Kg]
            double moment_inertia_;             // I [Kg・m^2]
            double viscous_friction_linear_;    // k_3 [N・s/m]
            double viscous_friction_angular_;   // k_4 [N・s/rad]
            double radius_robot_;               // L [m]

            void displayTargetMarker ( const geometry_msgs::msg::Twist& vel, const double curt_vel_angular );

        public :
            explicit VirtualSpringModel ( );

            void setFollowParameter( double ang_follow, double dist_follow );
            void setSpringParameter( double spring_constant_linear, double spring_constant_angular );
            void setFrictionParameter( double viscous_friction_linear, double viscous_friction_angular );
            void setRobotParameter( double weight_robot, double radius_robot );
            void setMomentParameter( double moment_inertia );

            void compute ( const geometry_msgs::msg::PoseStamped::SharedPtr& pose_msg, const double curt_vel_linear, const double curt_vel_angular, geometry_msgs::msg::Twist& output_vel );
    };

    inline void VirtualSpringModel::setFollowParameter( double ang_follow, double dist_follow ){
        ang_follow_ = ang_follow * M_PI / 180.0;
        dist_follow_ = dist_follow; 
    }
    inline void VirtualSpringModel::setSpringParameter( double spring_constant_linear, double spring_constant_angular ){
        spring_constant_linear_ = spring_constant_linear;
        spring_constant_angular_ = spring_constant_angular; 
    }
    inline void VirtualSpringModel::setFrictionParameter( double viscous_friction_linear, double viscous_friction_angular ){
        viscous_friction_linear_ = viscous_friction_linear;
        viscous_friction_angular_ = viscous_friction_angular; 
    }
    inline void VirtualSpringModel::setRobotParameter( double weight_robot, double radius_robot ){
        weight_robot_ = weight_robot;
        radius_robot_ = radius_robot; 
    }
    inline void VirtualSpringModel::setMomentParameter( double moment_inertia ){
        moment_inertia_ = -1.0 * moment_inertia;
    }

}

#endif