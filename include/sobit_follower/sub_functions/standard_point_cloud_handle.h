#ifndef STANDARD_POINT_CLOUD_HANDLE
#define STANDARD_POINT_CLOUD_HANDLE

#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>  
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>  
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace mypcl {

class StandardPointCloudHandle {
    private:
        /* tf */
        tf::TransformListener tf_listener_;
        /* Point Cloud Library */
        pcl::PassThrough<PointT> pass_;
        pcl::VoxelGrid<PointT> voxel_;
        pcl::search::KdTree<PointT>::Ptr tree_;
        pcl::EuclideanClusterExtraction<PointT> ec_;
        pcl::ExtractIndices<PointT> extract_;
        pcl::KdTreeFLANN<pcl::PointXYZ> flann_;
        /* LaserProjection */
		laser_geometry::LaserProjection projector_;  
        /* flag */
        bool is_set_cloud_cluster_param_;
        bool is_set_voxel_param_;

    public:
        StandardPointCloudHandle();
        // Seting Func :
        void setVoxelGridParameter( const float leaf_size );
        void setCloudClusteringParameters ( const float tolerance, const int min_size, const int max_size );
        // Transform a coordinate frame (cloud) :
        bool transformFramePointCloud ( const std::string target_frame, const sensor_msgs::PointCloud2ConstPtr &input_cloud, PointCloud::Ptr output_cloud );
        // Transform a coordinate frame (laser) :
        bool transformFramePointCloud ( const std::string target_frame, const sensor_msgs::LaserScanConstPtr &input_laser, PointCloud::Ptr output_cloud );
        // Transform a coordinate frame( Cloud & Laser ) :
        bool transformFramePointCloud ( const std::string target_frame, const sensor_msgs::PointCloud2ConstPtr &input_cloud, PointCloud::Ptr output_cloud,
                                         const sensor_msgs::LaserScanConstPtr &input_laser, PointCloud::Ptr output_laser );
        // Extract point cloud specified range :
        bool passThrough ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, 
                            const std::string &axis, const float &limit_min, const float &limit_max );
        // Downsample a point cloud :
        bool voxelGrid ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud );
        // clustering extraction (Cloud):
        bool cloudClusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices );
        // Extract specified Indices from point cloud :
        bool extractIndices( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, const pcl::PointIndices::Ptr indices, bool negative );
        // radius Search :
		bool radiusSearch ( PointCloud::Ptr input_cloud, pcl::PointIndices::Ptr output_indices, 
            const geometry_msgs::Point& search_pt, const double radius, bool is_accept_add_point );
};

// Seting Func :
inline void StandardPointCloudHandle::setVoxelGridParameter( const float leaf_size ) { 
    voxel_.setLeafSize( leaf_size, leaf_size, leaf_size );
    is_set_voxel_param_ = true;
}
inline void StandardPointCloudHandle::setCloudClusteringParameters ( const float tolerance, const int min_size, const int max_size ) {
    ec_.setClusterTolerance( tolerance );
    ec_.setMinClusterSize( min_size );
    ec_.setMaxClusterSize( max_size );
    ec_.setSearchMethod( tree_ );
    is_set_cloud_cluster_param_ = true;
}

}

#endif