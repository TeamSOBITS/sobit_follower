#ifndef PATH_GENERATOR_HPP
#define PATH_GENERATOR_HPP

#include <ros/ros.h>
#include <iostream>
#include <follow_path_generator3/dynamic_window_approach_omni.hpp>
#include <follow_path_generator3/pid_controller.hpp>

// Path Generator Mode
constexpr int STOP = 4;
constexpr int FOLLOW = 5;
constexpr int ATTENTION = 6;

// Path Type
constexpr int DWA_PATH = 7;
constexpr int PID_ROTATE = 8;
constexpr int PID_REVESE = 9;
constexpr int PATH_STOP = 10;

namespace follow_path_generator3 {
    class PathGenerator {
        private :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_cmd_vel_;

            follow_path_generator3::DynamicWindowApproachOmni dwa_;
            follow_path_generator3::PIDController pid_;

            double max_keep_dist_;
            double min_keep_dist_;
            double ang_rotate_stop_;
            double ang_rotate_move_;
            bool allows_reverse_;

            geometry_msgs::Twist curt_vel_;
            std::vector<double> linear_histories_;
            double pre_time_;
            bool use_omni_;

            void setParameters ( );
            int selectVelocityControl ( const bool exists_target, const bool is_nearby_obstacle ,const double target_distance, const double target_angle );
            void velocityCompensation ( const bool should_smoothing, geometry_msgs::Twist* output_vel  );

        public :
            PathGenerator( );
            void genetatePath ( const person_tracking3::FollowPosition& fp );
    };
    
    inline int PathGenerator::selectVelocityControl ( const bool exists_target, const bool is_nearby_obstacle , const double target_distance, const double target_angle ) {
        if ( !exists_target ) return PATH_STOP;
        if ( target_distance > max_keep_dist_ ) return DWA_PATH;
        else if ( target_distance <= max_keep_dist_ && target_distance > min_keep_dist_ && is_nearby_obstacle ) return DWA_PATH;
        else return PID_ROTATE;
    }
}

#endif
