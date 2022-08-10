#ifndef POINT_CLOUD_TRACKER
#define POINT_CLOUD_TRACKER

#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef pcl::tracking::ParticleFilterTracker<PointT, ParticleT> ParticleFilter;

namespace mypcl {

    class PointCloudTracker {
        private :
            ParticleFilter *tracker_;
            PointCloud::Ptr cloud_tracked_target_;
            double moving_distance_;
            double moving_angle_;
            geometry_msgs::Point current_predicted_pt_;
            geometry_msgs::Point previous_measured_pt_;
            std::vector<geometry_msgs::Point> target_trajectory_;

        public :
            PointCloudTracker ( );
            // Set Particle Filter :
            bool setTrackTarget ( const PointCloud::Ptr ref_cloud, const geometry_msgs::Point& ref_pt );
            // change track target :
            bool changeTrackTarget ( const PointCloud::Ptr ref_cloud, const geometry_msgs::Point& ref_pt );
            // Get Particle Filter Tracking Result :
            bool getTrackResult ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, geometry_msgs::Pose* output_pose );
            // Get Particle Filter Tracking Result :
            bool getTrackResult ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, geometry_msgs::Point* output_pt );
            // Get the current particles :
            bool getParticles ( PointCloud::Ptr output_cloud );
            // Get the current target motion(2D ver) :
            bool getTargetMotion2D ( double* distance, double* orientation );
            // Add the target Trajectory :
            void addTargetTrajectory ( const geometry_msgs::Point& current_measured_pt );
            // Reset the target Trajectory :
            void resetTargetTrajectory ( );
            // set current_predicted_pt :
            void getSmoothedTarget( geometry_msgs::Point* smoothed_target );
    };  

    inline bool PointCloudTracker::getTargetMotion2D ( double* distance, double* orientation ) {
        *distance = moving_distance_;
        *orientation = moving_angle_ ;
        //ROS_INFO("Distance = %.5f [m]", moving_distance_);
        //ROS_INFO("Angle    = %.5f [rad] ( %.5f [deg] )\n", moving_angle_, (180 * moving_angle_) / M_PI);
    }

    inline void PointCloudTracker::resetTargetTrajectory ( ) {
        geometry_msgs::Point tmp, tmp2;
        current_predicted_pt_ = tmp;
        previous_measured_pt_ = tmp2;
        moving_distance_ = -1.0;
        moving_angle_ = -1.0;
    }

    inline void PointCloudTracker::getSmoothedTarget( geometry_msgs::Point* smoothed_target ) { *smoothed_target = current_predicted_pt_; }
}

#endif