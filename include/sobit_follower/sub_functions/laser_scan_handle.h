#ifndef LASER_SCAN_HANDLE
#define LASER_SCAN_HANDLE

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace mypcl {
    class LaserScanHandle {
        private :
            pcl::search::KdTree<PointT>::Ptr tree_;
            pcl::EuclideanClusterExtraction<PointT> ec_;
            pcl::EuclideanClusterExtraction<PointT> ec_large_;
            pcl::ExtractIndices<PointT> extract_;
            pcl::RadiusOutlierRemoval<PointT> outrem_;
            
        public :
            LaserScanHandle ();
            void setClusteringParameters ( const float tolerance, const int min_size, const int max_size );
            void setLargeClusteringParameters ( const float tolerance, const int min_size, const int max_size );
            void setRadiusOutlierRemovalParameters ( const double radius, const int min_pts, const bool keep_organized );
            // clustering extraction (noromal):
            bool clusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices );
            // clustering extraction (large):
            bool largeClusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices );
            // Remove outlier point cloud :
            bool radiusOutlierRemoval ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud  );
    };

    inline void LaserScanHandle::setClusteringParameters ( const float tolerance, const int min_size, const int max_size ) {
        ec_.setClusterTolerance( tolerance );
        ec_.setMinClusterSize( min_size );
        ec_.setMaxClusterSize( max_size );
        ec_.setSearchMethod( tree_ );
    }
    inline void LaserScanHandle::setLargeClusteringParameters ( const float tolerance, const int min_size, const int max_size ) {
        ec_large_.setClusterTolerance( tolerance );
        ec_large_.setMinClusterSize( min_size );
        ec_large_.setMaxClusterSize( max_size );
        ec_large_.setSearchMethod( tree_ );
    }
    inline void LaserScanHandle::setRadiusOutlierRemovalParameters ( const double radius, const int min_pts, const bool keep_organized ) {
        outrem_.setRadiusSearch( radius );
        outrem_.setMinNeighborsInRadius ( min_pts );
        outrem_.setKeepOrganized( keep_organized );
    }
}

#endif