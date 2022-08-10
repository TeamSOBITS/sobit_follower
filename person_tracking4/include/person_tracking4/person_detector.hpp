#ifndef PERSON_DETECTOR
#define PERSON_DETECTOR

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
// for msg pub
#include <sobit_common_msg/StringArray.h>
#include <sobit_common_msg/BoundingBox.h>
#include <sobit_common_msg/BoundingBoxes.h>
#include <sobit_common_msg/ObjectPoseArray.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace person_tracking3 {
    class PersonDetector {
        private :

        public :
            PersonDetector ( );
            // find Person Position  :
            int findPersonsPosition ( const PointCloud::Ptr input_cloud, std::vector<geometry_msgs::Point>* person_pt, 
                uint32_t width,  uint32_t height, const sobit_common_msg::BoundingBoxesConstPtr &ssd_msg );
            // Create Persons PointCloud  :
            int createPersonsPointCloud ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, 
                uint32_t width,  uint32_t height, const sobit_common_msg::BoundingBoxesConstPtr &ssd_msg );
    };
}


#endif