#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <person_tracking4/svdd_particle_person_tracker.hpp>
#include <person_tracking4/person_detector.hpp>
#include <person_tracking4/RotatePosition.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

constexpr int PARTICLE_TRACKER = 1;
constexpr int SVDD_PARTICLE_TRACKER = 2;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::LaserScan> MySyncPolicy;

namespace person_follower4 {
    class PersonTracker : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_follow_pos_;
            ros::Publisher pub_rotate_pos_;
            ros::Subscriber sub_ssd_;

            std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_cloud_;
            std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub_laser_;
            std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

            pcl_handle::PCLHandle pch_;
            person_tracking3::PersonDetector person_detector_;
            person_tracking4::SVDDParticlePersonTracker svdd_particle_tracker_;
            sobit_common_msg::BoundingBoxesPtr object_bbox_array_;

            tf::TransformListener tf_listener_; 
            std::string target_frame_;

            int tracking_method_;
            int process_flag_;
            bool should_output_marker_;

        public:
            virtual void onInit();
            void callbackSSD (const sobit_common_msg::BoundingBoxesConstPtr &msg);
            void callbackSenserData ( const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const sensor_msgs::LaserScanConstPtr &laser_msg );
    };
}

void person_follower4::PersonTracker::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
    pub_follow_pos_ = nh_.advertise<person_tracking4::FollowPosition>("follow_position", 1);
    pub_rotate_pos_ = nh_.advertise<person_tracking4::RotatePosition>("rotate_position", 1);
    sub_ssd_ = nh_.subscribe ( pnh_.param<std::string>( "ssd_topic_name", "/person_follower/objects_rect" ), 1, &PersonTracker::callbackSSD, this );    //SSD_NODE
    process_flag_ = DETECTION;
    // message_filters :
    sub_cloud_ .reset ( new message_filters::Subscriber<sensor_msgs::PointCloud2> ( nh_, pnh_.param<std::string>( "cloud_topic_name", "/camera/depth/points" ), 1 ) );
    sub_laser_ .reset ( new message_filters::Subscriber<sensor_msgs::LaserScan> ( nh_, pnh_.param<std::string>( "laser_topic_name", "/scan" ), 1 ) );
    sync_ .reset ( new message_filters::Synchronizer<MySyncPolicy> ( MySyncPolicy(100), *sub_cloud_, *sub_laser_ ) );
    sync_ ->registerCallback ( boost::bind( &PersonTracker::callbackSenserData, this, _1, _2 ) );
    tracking_method_ = pnh_.param<int>( "tracking_method", PARTICLE_TRACKER );
    std::cout << "\n========================================\n[ PersonTracker parameters ]"
        << "\n  * ssd_topic_name         : " << pnh_.param<std::string>( "ssd_topic_name", "/person_follower/objects_rect" ) 
        << "\n  * laser_topic_name       : " << pnh_.param<std::string>( "laser_topic_name", "/scan" ) 
        << "\n  * cloud_topic_name       : " << pnh_.param<std::string>( "cloud_topic_name", "/camera/depth/points" )         
        << "\n  * target_frame           : " << target_frame_
        << "\n  * tracking_method        : " << ( ( tracking_method_ == PARTICLE_TRACKER ) ? "Particle Filter base" : "Particle Filter + Support Vector Data Description" )
        << "\n========================================\n"
    << std::endl;
    object_bbox_array_ .reset ( new sobit_common_msg::BoundingBoxes );
}

void person_follower4::PersonTracker::callbackSSD (const sobit_common_msg::BoundingBoxesConstPtr &msg) { *object_bbox_array_ = *msg; }

void person_follower4::PersonTracker::callbackSenserData ( const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const sensor_msgs::LaserScanConstPtr &laser_msg ) {
    std::cout <<"========================================" << std::endl;
    PointCloud::Ptr cloud (new PointCloud());
    PointCloud::Ptr laser (new PointCloud());
    std::vector<geometry_msgs::Point> person_pts;
    person_tracking4::FollowPositionPtr fp (new person_tracking4::FollowPosition);
    person_tracking4::RotatePositionPtr rp (new person_tracking4::RotatePosition);

    if ( !pch_.transformFramePointCloud( target_frame_, cloud_msg, cloud, laser_msg, laser ) ) return;
    person_detector_.findPersonsPosition( cloud, &person_pts, cloud_msg->width, cloud_msg->height, object_bbox_array_ );
    if ( tracking_method_ == PARTICLE_TRACKER ) svdd_particle_tracker_.computeParticle( laser, person_pts, laser_msg->header.stamp, fp );
    else svdd_particle_tracker_.computeSVDDParticle( laser, person_pts, laser_msg->header.stamp, fp );
    pub_follow_pos_.publish(fp);
    rp->target_distance = fp->target_distance;
    rp->target_angle = fp->target_angle;
    rp->process_flag = fp->process_flag;
    pub_rotate_pos_.publish(rp);
    std::cout << " * X [m] : " << fp->target.x << "\n * Y [m] : " << fp->target.y << "\n" << std::endl;
    return;
}

PLUGINLIB_EXPORT_CLASS(person_follower4::PersonTracker, nodelet::Nodelet);