#ifndef PCL_HANDLE_FOLLOWER
#define PCL_HANDLE_FOLLOWER

#include <pcl_handle/pcl_handle.hpp>

namespace person_tracking4 {
    class PCLHandle : public  pcl_handle::PCLHandle {
        private :
            pcl::EuclideanClusterExtraction<PointT> ec_large_;
        public :
            PCLHandle();
            void setLargeClusteringParameters ( const float tolerance, const int min_size, const int max_size );
            // clustering extraction (large):
            bool largeClusterExtraction ( const PointCloud::Ptr input_cloud, std::vector<pcl::PointIndices>* output_indices );
    };
    inline void PCLHandle::setLargeClusteringParameters ( const float tolerance, const int min_size, const int max_size ) {
        ec_large_.setClusterTolerance( tolerance );
        ec_large_.setMinClusterSize( min_size );
        ec_large_.setMaxClusterSize( max_size );
        ec_large_.setSearchMethod( tree_ );
    }
}

#endif