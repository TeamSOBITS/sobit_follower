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

constexpr int NON_WORKING = -1;
constexpr int DETECTION = 1;
constexpr int TRACKING = 2;
constexpr int PREDICTION = 3;

namespace mypcl {
    class PersonDetector {
        private :
            double detect_allowable_dist_;
            double tracking_allowable_dist_;
            double prediction_allowable_dist_;
        public :
            PersonDetector ( );
            // Set Allowable distance :
            void setAllowableDistance ( const double detect, const double tracking, const double prediction );
            // Create Persons PointCloud from service results :
            int createPersonsPositionPointCloud ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, 
                uint32_t width,  uint32_t height, const ssd_node::BoundingBoxesConstPtr &ssd_msg );
            int createPersonsPointCloud ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, 
                uint32_t width,  uint32_t height, const ssd_node::BoundingBoxesConstPtr &ssd_msg );
            
            // Decide Target Person :
            bool decideTargetPerson ( const PointCloud::Ptr persons_cloud, geometry_msgs::Point& search_pt, geometry_msgs::Point* target_pt, int flag );
    };

    // Set Allowable distance :
    inline void PersonDetector::setAllowableDistance ( const double detect, const double tracking, const double prediction ) {
        detect_allowable_dist_ = detect;
        tracking_allowable_dist_ = tracking;
        prediction_allowable_dist_ = prediction;
    }
}


#endif