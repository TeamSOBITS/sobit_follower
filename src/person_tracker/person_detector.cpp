#include "sobit_follower/person_tracker/person_detector.h"

using namespace mypcl;
PersonDetector::PersonDetector ( ) {
    tracking_allowable_dist_ = 0.2;
    prediction_allowable_dist_ = 0.4;
}
// Create Persons PointCloud from service results :
int PersonDetector::createPersonsPositionPointCloud ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, 
    uint32_t width,  uint32_t height, const ssd_node::BoundingBoxesConstPtr &ssd_msg ) {
    int cnt_create = 0;
    int x_init,y_init,bbox_width,bbox_height,x_ctr,y_ctr;
    int array_num;
    int persons_num = 0;
    std::vector<bool> is_used( input_cloud->points.size(), false );
    for( auto &ssd : ssd_msg->boundingBoxes ) {
        if( ssd.Class == "person" ) {
            x_init = ssd .x;
            y_init = ssd .y;
            bbox_width = ssd .width;
            bbox_height = ssd .height;
            x_ctr = ( x_init + ( x_init + bbox_width - 1 ) ) / 2;
            y_ctr =  ( y_init + ( y_init + bbox_height - 1 ) ) / 2;
            array_num = ( width * y_ctr ) + x_ctr;
            if( is_used[array_num] ) continue;
            if (!pcl_isfinite (input_cloud ->points [ array_num ].x) ||
                !pcl_isfinite (input_cloud ->points [ array_num ].y) ||
                !pcl_isfinite (input_cloud ->points [ array_num ].z)) continue;
            output_cloud->points.push_back( input_cloud->points[ array_num ] );
            is_used[array_num] = true;
            cnt_create++;
        }
    }
    output_cloud->header.frame_id = input_cloud->header.frame_id;
    return cnt_create;
}
int PersonDetector::createPersonsPointCloud ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, 
    uint32_t width,  uint32_t height, const ssd_node::BoundingBoxesConstPtr &ssd_msg ) {
    int cnt_create = 0;
    int x_init,y_init,x_limit,y_limit;
    int array_num;
    int persons_num = 0;
    std::vector<bool> is_used( input_cloud->points.size(), false );
    ROS_INFO("createPersonsPointCloud");
    for( auto &ssd : ssd_msg->boundingBoxes ) {
        if( ssd.Class == "person" ) {
            x_init = ( ssd.x < 0 ) ? 0 : ssd.x;
            y_init = ( ssd.y < 0 ) ? 0 : ssd.y;
            x_limit = ( x_init + ssd .width <= width ) ? x_init + ssd .width : width; 
            y_limit = ( y_init + ssd .height <= height ) ? y_init + ssd .height : height; 
            std::cout << "x_init      = " << x_init << std::endl; 
            std::cout << "y_init      = " << y_init << std::endl; 
            std::cout << "ssd .width  = " << ssd .width << std::endl; 
            std::cout << "ssd .height = " << ssd .height << std::endl; 
            std::cout << "x_limit     = " << x_limit << std::endl;
            std::cout << "y_limit     = " << y_limit << std::endl;  
            std::cout << "width       = " << width << std::endl; 
            std::cout << "height      = " << height << std::endl; 
            for ( int y = y_init; y < y_limit; y++ ) {
                for ( int x = x_init; x < x_limit; x++ ) {
                    array_num = ( width * y ) + x;
                    if( array_num > width*height ) ROS_ERROR("ERROR : %d", array_num);
                    if( is_used[array_num] ) continue;
                    if (!pcl_isfinite (input_cloud ->points [ array_num ].x) ||
                        !pcl_isfinite (input_cloud ->points [ array_num ].y) ||
                        !pcl_isfinite (input_cloud ->points [ array_num ].z)) continue;
                    output_cloud->points.push_back( input_cloud->points[ array_num ] );
                    is_used[array_num] = true;
                }
            }
            cnt_create++;
        }
    }
    output_cloud->header.frame_id = input_cloud->header.frame_id;
    return cnt_create;
}
// Decide Target Person :
bool PersonDetector::decideTargetPerson ( const PointCloud::Ptr persons_cloud, geometry_msgs::Point& search_pt, geometry_msgs::Point* target_pt, int flag ) {
    bool is_decide = false;
    double target_distance = DBL_MAX;
    if ( flag == DETECTION ) { search_pt.x = 0.0; search_pt.y = 0.0; search_pt.z = 0.0; }
    double allowable_dist = (flag == DETECTION) ? detect_allowable_dist_ : (flag == TRACKING) ? tracking_allowable_dist_ : prediction_allowable_dist_;
    for ( auto &person : persons_cloud->points ) {
        double distance = std::hypotf( search_pt.x - person.x, search_pt.y - person.y ); 
        if ( distance > allowable_dist ) continue;
        if ( distance < target_distance ) {
            target_pt->x = person.x;
            target_pt->y = person.y;
            target_pt->z = person.z;
            target_distance =  distance;
            is_decide = true;
        }
    }
    return is_decide;
}
