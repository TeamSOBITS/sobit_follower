#include <person_tracking4/pcl_handle.hpp>

person_tracking4::PCLHandle::PCLHandle() {
    tree_ .reset ( new pcl::search::KdTree<PointT>() );
}

bool person_tracking4::PCLHandle::largeClusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices ) {
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