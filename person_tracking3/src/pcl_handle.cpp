#include <person_tracking3/pcl_handle.hpp>

person_tracking3::PCLHandle::PCLHandle() {
    tree_ .reset ( new pcl::search::KdTree<PointT>() );
}

bool person_tracking3::PCLHandle::largeClusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices ) {
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