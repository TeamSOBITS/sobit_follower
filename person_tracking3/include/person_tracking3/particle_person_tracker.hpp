#ifndef PARTICLE_PERSON_TRACKER
#define PARTICLE_PERSON_TRACKER

#include <ros/ros.h>
#include <person_tracking3/pcl_handle.hpp>
#include <point_cloud_tracker/point_cloud_tracker.hpp>
#include <person_tracking3/follow_position_handle.hpp>
#include <person_tracking3/person_aim_sensor_rotator.hpp>
#include <visualization_msgs/MarkerArray.h>

constexpr int NON_WORKING = -1;
constexpr int DETECTION = 1;
constexpr int TRACKING = 2;
constexpr int PREDICTION = 3;

namespace person_tracking3 {
    class ParticlePersonTracker {
        protected :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            person_tracking3::PCLHandle pch_;
            point_cloud_tracker::PointCloudTracker pct_;
            person_tracking3::FollowPositionHandle fph_;
            person_tracking3::PersonAimSensorRotator sensor_rotator_;
            tf::TransformListener tf_listener_; 

            ros::Publisher pub_marker_;
            ros::Publisher pub_tgt_; 
            ros::Publisher pub_laser_;

            int process_flag_;
            std::string target_frame_;

            geometry_msgs::Point pre_tgt_pt_;
            geometry_msgs::Point predicted_pt_odom_;

            double detect_allowable_dist_;
            double tracking_allowable_dist_;
            double prediction_allowable_dist_;

            double search_radius_;
            std::vector<double> cluster_size_large_obs_;

            double curt_time_;
            double predicted_exec_time_;
            double predicted_start_time_;
            int cnt_lost_tgt_;
            
            bool should_output_marker_;

            void setParameters ( );
            visualization_msgs::Marker makeMarker ( const geometry_msgs::Point &target_pt, const int flag, const ros::Time& stamp );
            void sortLaserCloud ( PointCloud::Ptr input_laser );
            geometry_msgs::Point transformPoint ( 
                std::string org_frame, 
                std::string target_frame, 
                geometry_msgs::Point point );

            bool detectPersonPosition(  
                const PointCloud::Ptr laser, 
                const std::vector<geometry_msgs::Point>& person_pts,
                PointCloud::Ptr cloud_obs, 
                PointCloud::Ptr cloud_tgt, 
                geometry_msgs::Point* tgt_pt );

            int trackPersonPosition(    
                const PointCloud::Ptr laser,  
                const std::vector<geometry_msgs::Point>& person_pts,
                PointCloud::Ptr cloud_obs, 
                PointCloud::Ptr cloud_tgt, 
                geometry_msgs::Point* tgt_pt );

            int predictPersonPosition(  
                const PointCloud::Ptr laser,  
                const std::vector<geometry_msgs::Point>& person_pts,
                PointCloud::Ptr cloud_obs, 
                PointCloud::Ptr cloud_tgt, 
                geometry_msgs::Point* tgt_pt );

            bool decideTargetPerson (   
                const std::vector<geometry_msgs::Point>& person_pts, 
                geometry_msgs::Point& search_pt, 
                geometry_msgs::Point* target_pt, 
                const int process_flag = DETECTION );

        public :
            ParticlePersonTracker ();
            int computeParticle(    
                PointCloud::Ptr laser,  
                const std::vector<geometry_msgs::Point>& person_pts, 
                const ros::Time& stamp,
                person_tracking3::FollowPosition* follow_position );
    };
}

#endif