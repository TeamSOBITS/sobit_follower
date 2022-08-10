#include <person_follower3/person_follower.hpp>

using namespace person_follower3;

void PersonFollower::callbackSenserData ( const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const sensor_msgs::LaserScanConstPtr &laser_msg ) {
    try {
        std::cout <<"========================================" << std::endl;
        geometry_msgs::Point tgt_pt;
        PointCloud::Ptr cloud (new PointCloud());
        PointCloud::Ptr laser (new PointCloud());
        std::vector<geometry_msgs::Point> person_pts;
        person_tracking3::FollowPosition fp;

        if ( !pch_.transformFramePointCloud( target_frame_, cloud_msg, cloud, laser_msg, laser ) ) return;
        person_detector_.findPersonsPosition( cloud, &person_pts, cloud_msg->width, cloud_msg->height, obj_bbox_array_ssd_ );
        if ( tracking_method_ == PARTICLE_TRACKER ) svdd_particle_tracker_.computeParticle( laser, person_pts, laser_msg->header.stamp, &fp );
        else svdd_particle_tracker_.computeSVDDParticle( laser, person_pts, laser_msg->header.stamp, &fp );
        path_generator_.genetatePath( fp );
        return;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return;
    }   
}

PersonFollower::PersonFollower( ) : nh_(), pnh_("~")  {
    target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
    // Subscriber :
    sub_object_rect_ssd_ = nh_.subscribe ( "/ssd_object_detect/objects_rect", 1, &PersonFollower::callbackSSD, this );    //SSD_NODE
    
    process_flag_ = DETECTION;
    obj_bbox_array_ssd_ .reset ( new ssd_node::BoundingBoxes );
    // message_filters :
    sub_cloud_ .reset ( new message_filters::Subscriber<sensor_msgs::PointCloud2> ( nh_, pnh_.param<std::string>( "cloud_topic_name", "/camera/depth/points" ), 1 ) );
    sub_laser_ .reset ( new message_filters::Subscriber<sensor_msgs::LaserScan> ( nh_, pnh_.param<std::string>( "laser_topic_name", "/scan" ), 1 ) );
    sync_ .reset ( new message_filters::Synchronizer<MySyncPolicy> ( MySyncPolicy(100), *sub_cloud_, *sub_laser_ ) );
    sync_ ->registerCallback ( boost::bind( &PersonFollower::callbackSenserData, this, _1, _2 ) );
    tracking_method_ = pnh_.param<int>( "tracking_method", PARTICLE_TRACKER );
    std::cout << "\n========================================\n[ PersonFollower parameters ]"
        << "\n  * cloud_topic_name : " << pnh_.param<std::string>( "cloud_topic_name", "/camera/depth/points" )  
        << "\n  * laser_topic_name : " << pnh_.param<std::string>( "laser_topic_name", "/scan" ) 
        << "\n  * target_frame     : " << target_frame_
        << "\n  * tracking_method  : " << ( ( tracking_method_ == PARTICLE_TRACKER ) ? "Particle Filter base" : "Particle Filter + Support Vector Data Description" )
        << "\n========================================\n"
    << std::endl;
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "person_tracker_node");
    PersonFollower person_tracker;
    ros::spin();
}