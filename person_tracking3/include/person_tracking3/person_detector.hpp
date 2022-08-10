#ifndef PERSON_DETECTOR
#define PERSON_DETECTOR

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <ssd_node/stringArray.h>
#include <ssd_node/BoundingBox.h>
#include <ssd_node/BoundingBoxes.h>
#include <ssd_node/BoundingBoxesStamp.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace person_tracking3 {
    class PersonDetector {
        private :

        public :
            PersonDetector ( );
            // find Person Position  :
            int findPersonsPosition ( const PointCloud::Ptr input_cloud, std::vector<geometry_msgs::Point>* person_pt, 
                uint32_t width,  uint32_t height, const ssd_node::BoundingBoxesConstPtr &ssd_msg );
            // Create Persons PointCloud  :
            int createPersonsPointCloud ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, 
                uint32_t width,  uint32_t height, const ssd_node::BoundingBoxesConstPtr &ssd_msg );
    };
}


#endif