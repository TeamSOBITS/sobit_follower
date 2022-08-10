#include "sobit_follower/sub_functions/standard_point_cloud_handle.h"

using namespace mypcl;

StandardPointCloudHandle::StandardPointCloudHandle() {
    is_set_voxel_param_ = false;
    is_set_cloud_cluster_param_ = false;
    tree_ .reset ( new pcl::search::KdTree<PointT>() );
}
// Transform a coordinate frame :
bool StandardPointCloudHandle::transformFramePointCloud ( const std::string target_frame, const sensor_msgs::PointCloud2ConstPtr &input_cloud, PointCloud::Ptr output_cloud ) {
    PointCloud cloud_src;
    pcl::fromROSMsg<PointT>( *input_cloud, cloud_src );
    if (target_frame.empty() == false ){
        try {
            tf_listener_.waitForTransform(target_frame, cloud_src.header.frame_id, ros::Time(0), ros::Duration(1.0));
            pcl_ros::transformPointCloud(target_frame, ros::Time(0), cloud_src, cloud_src.header.frame_id,  *output_cloud, tf_listener_);
            output_cloud->header.frame_id = target_frame;
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    } else ROS_ERROR("Please set the target frame.");
    return true;
}
// Transform a coordinate frame (laser) :
bool StandardPointCloudHandle::transformFramePointCloud ( const std::string target_frame, const sensor_msgs::LaserScanConstPtr &input_laser, PointCloud::Ptr output_cloud ) {
    sensor_msgs::PointCloud2 cloud_laser;
    cloud_laser.header = input_laser->header;
    projector_.transformLaserScanToPointCloud(target_frame, *input_laser, cloud_laser, tf_listener_);  
    PointCloud cloud_src;  
    pcl::fromROSMsg<PointT>(cloud_laser, cloud_src);  
    if (target_frame.empty() == false ){
        try {
            tf_listener_.waitForTransform(target_frame, cloud_src.header.frame_id, ros::Time(0), ros::Duration(1.0));
            pcl_ros::transformPointCloud(target_frame, ros::Time(0), cloud_src, cloud_src.header.frame_id,  *output_cloud, tf_listener_);
            output_cloud->header.frame_id = target_frame;
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    } else ROS_ERROR("Please set the target frame.");
    return true;
}
// Transform a coordinate frame( Cloud & Laser ) :
bool StandardPointCloudHandle::transformFramePointCloud ( const std::string target_frame, const sensor_msgs::PointCloud2ConstPtr &input_cloud, PointCloud::Ptr output_cloud,
                                    const sensor_msgs::LaserScanConstPtr &input_laser, PointCloud::Ptr output_laser ) {
    sensor_msgs::PointCloud2 cloud_laser;
    cloud_laser.header = input_laser->header;
    projector_.transformLaserScanToPointCloud(target_frame, *input_laser, cloud_laser, tf_listener_);
    PointCloud cloud_src;
    PointCloud cloud_laser_src; 
    pcl::fromROSMsg<PointT>( *input_cloud, cloud_src );
    pcl::fromROSMsg<PointT>(cloud_laser, cloud_laser_src);  
    if (target_frame.empty() == false ){
        try {
            tf_listener_.waitForTransform(target_frame, cloud_src.header.frame_id, ros::Time(0), ros::Duration(1.0));
            pcl_ros::transformPointCloud(target_frame, ros::Time(0), cloud_src, cloud_src.header.frame_id,  *output_cloud, tf_listener_);
            output_cloud->header.frame_id = target_frame;
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
        try {
            tf_listener_.waitForTransform(target_frame, cloud_laser_src.header.frame_id, ros::Time(0), ros::Duration(1.0));
            pcl_ros::transformPointCloud(target_frame, ros::Time(0), cloud_laser_src, cloud_laser_src.header.frame_id,  *output_laser, tf_listener_);
            output_laser->header.frame_id = target_frame;
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    } else ROS_ERROR("Please set the target frame.");
    return true;
}
// Extract point cloud specified range :
bool StandardPointCloudHandle::passThrough ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, 
                    const std::string &axis, const float &limit_min, const float &limit_max ) {
    try {
        PointCloud::Ptr tmp ( new PointCloud() );
        pass_.setFilterFieldName( axis );
        pass_.setFilterLimits( limit_min, limit_max);
        pass_.setInputCloud( input_cloud );
        pass_.filter( *tmp );
        *output_cloud = *tmp;
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
// Downsample a point cloud :
bool StandardPointCloudHandle::voxelGrid ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud ) { 
    if ( !is_set_voxel_param_ ) {
        ROS_ERROR("Please set the VoxelGridParameter. ClusteringParameters can be set by using 'setVoxelGridParameter()'.");
        return false;
    }
    try {
        PointCloud::Ptr tmp ( new PointCloud() );
        voxel_.setInputCloud( input_cloud ); 
        voxel_.filter( *tmp );
        *output_cloud = *tmp;
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
// clustering extraction (Cloud):
bool StandardPointCloudHandle::cloudClusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices ) {  
    if( !is_set_cloud_cluster_param_ ) {
        ROS_ERROR("Please set the ClusteringParameters. ClusteringParameters can be set by using 'setClusteringParameters()'.");
        return false;
    }
    try {
        tree_->setInputCloud( input_cloud );
        ec_.setInputCloud( input_cloud );
        ec_.extract( *output_indices );
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
// Extract specified Indices from point cloud :
bool StandardPointCloudHandle::extractIndices( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, const pcl::PointIndices::Ptr indices, bool negative ) {
    try {
        PointCloud::Ptr tmp ( new PointCloud() );
        extract_.setInputCloud( input_cloud );
        extract_.setIndices( indices );
        extract_.setNegative( negative );
        extract_.filter( *tmp );
        *output_cloud = *tmp;
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
// radius Search :
bool StandardPointCloudHandle::radiusSearch ( PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr output_indices, 
    const geometry_msgs::Point& search_pt, const double radius, bool is_accept_add_point ) {
    bool is_match = false;
    PointT searchPoint;
    searchPoint .x = search_pt .x;
    searchPoint .y = search_pt .y;
    searchPoint .z = input_cloud ->points [0] .z;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    flann_ .setInputCloud ( input_cloud );

    if ( flann_.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance, 0) > 0 ) {
        int size = pointIdxRadiusSearch.size();
        for ( size_t i = 0; i < size; ++i ) { output_indices ->indices .push_back ( pointIdxRadiusSearch [i] ); }
        is_match = true;
    } 
    if ( !is_match && is_accept_add_point ) {
        input_cloud->points.push_back(searchPoint);
        output_indices ->indices .push_back ( input_cloud->points.size() - 1 );
        is_match = true;
    }
    return is_match;
}

