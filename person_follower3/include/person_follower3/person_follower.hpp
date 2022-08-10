#ifndef PERSON_FOLLOWER3
#define PERSON_FOLLOWER3

#include <person_tracking3/svdd_particle_person_tracker.hpp>
#include <person_tracking3/person_detector.hpp>
#include <follow_path_generator3/path_generator.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

constexpr int PARTICLE_TRACKER = 1;
constexpr int SVDD_PARTICLE_TRACKER = 2;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::LaserScan> MySyncPolicy;

namespace person_follower3 {
    class PersonFollower {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Subscriber sub_object_rect_ssd_;

            std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_cloud_;
            std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub_laser_;
            std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

            pcl_handle::PCLHandle pch_;
            person_tracking3::PersonDetector person_detector_;
            person_tracking3::SVDDParticlePersonTracker svdd_particle_tracker_;
            follow_path_generator3::PathGenerator path_generator_;
            
            tf::TransformListener tf_listener_; 
            std::string target_frame_;
            ssd_node::BoundingBoxesPtr obj_bbox_array_ssd_;
            int tracking_method_;
            int process_flag_;
            bool should_output_marker_;

            void callbackSenserData ( const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const sensor_msgs::LaserScanConstPtr &laser_msg );
            void callbackSSD (const ssd_node::BoundingBoxesConstPtr &msg);
        
        public:
            PersonFollower( );
    };
    inline void PersonFollower::callbackSSD (const ssd_node::BoundingBoxesConstPtr &msg) { *obj_bbox_array_ssd_ = *msg; }
}

#endif