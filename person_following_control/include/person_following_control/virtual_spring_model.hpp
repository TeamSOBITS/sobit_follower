#ifndef VIRTUAL_SPRING_MODEL
#define VIRTUAL_SPRING_MODEL

#include <ros/ros.h>
#include<bits/stdc++.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>


namespace person_following_control{
    class VirtualSpringModel {
        private :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_mrk_tgt_;
            ros::Publisher pub_mrk_path_;

            float ang_follow_;                 // theta_sh [rad]
            float dist_follow_;                // l_0 [m]

            float spring_constant_linear_;     // k_1 [N/m]
            float spring_constant_angular_;    // k_2 [N・m/rad]
            float weight_robot_;               // M [Kg]
            float moment_inertia_;             // I [Kg・m^2]
            float viscous_friction_linear_;    // k_3 [N・s/m]
            float viscous_friction_angular_;   // k_4 [N・s/rad]
            float radius_robot_;               // L [m]

			bool display_vsm_path_;
            bool display_target_;

            void displayVirtualSpringPathMarker ( float vel, float ang_vel );
            visualization_msgs::Marker displayTargetMarker ( const Eigen::Vector3f& pt, const std::string& name, const float r, const float g, const float b);

        public :
            VirtualSpringModel ( );

            void setFollowParamater( float ang_follow, float dist_follow );
            void setSpringParamater( float spring_constant_linear, float spring_constant_angular );
            void setFrictionParamater( float viscous_friction_linear, float viscous_friction_angular );
            void setRobotParamater( float weight_robot, float radius_robot );
            void setMomentParamater( float moment_inertia );
			void setDisplayFlag ( const bool display_vsm_path, const bool display_target );

            void compute ( const geometry_msgs::Pose &pose_msg, const float curt_vel_linear, const float curt_vel_angular, geometry_msgs::TwistPtr &output_vel );
    };
}
inline void person_following_control::VirtualSpringModel::setFollowParamater( float ang_follow, float dist_follow ){
    ang_follow_ = ang_follow * M_PI / 180.0;
    dist_follow_ = dist_follow;
}
inline void person_following_control::VirtualSpringModel::setSpringParamater( float spring_constant_linear, float spring_constant_angular ){
    spring_constant_linear_ = spring_constant_linear;
    spring_constant_angular_ = spring_constant_angular;
}
inline void person_following_control::VirtualSpringModel::setFrictionParamater( float viscous_friction_linear, float viscous_friction_angular ){
    viscous_friction_linear_ = viscous_friction_linear;
    viscous_friction_angular_ = viscous_friction_angular;
}
inline void person_following_control::VirtualSpringModel::setRobotParamater( float weight_robot, float radius_robot ){
    weight_robot_ = weight_robot;
    radius_robot_ = radius_robot;
}
inline void person_following_control::VirtualSpringModel::setMomentParamater( float moment_inertia ){
    moment_inertia_ = -1.0 * moment_inertia;
}
inline void person_following_control::VirtualSpringModel::setDisplayFlag ( const bool display_vsm_path, const bool display_target ) {
	display_vsm_path_ = display_vsm_path;
    display_target_ = display_target;
}

#endif