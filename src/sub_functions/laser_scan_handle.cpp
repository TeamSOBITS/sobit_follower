#include "sobit_follower/sub_functions/laser_scan_handle.h"

using namespace mypcl;




LaserScanHandle::LaserScanHandle () {
    tree_ .reset ( new pcl::search::KdTree<PointT>() );
}

bool LaserScanHandle::clusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices ) {
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
bool LaserScanHandle::largeClusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices ) {
    try {
        tree_->setInputCloud( input_cloud );
        ec_large_.setInputCloud( input_cloud );
        ec_large_.extract( *output_indices );
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
// Remove outlier point cloud :
bool LaserScanHandle::radiusOutlierRemoval ( const PointCloud::Ptr input_cloud,  PointCloud::Ptr output_cloud ) {
    try {
        PointCloud::Ptr tmp ( new PointCloud() );
        outrem_.setInputCloud( input_cloud );
        outrem_.filter (*tmp);
        *output_cloud = *tmp;
        output_cloud->header.frame_id = input_cloud->header.frame_id;
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}