#include <person_tracking4/person_detector.hpp>

using namespace person_tracking3;
PersonDetector::PersonDetector ( ) { }

int PersonDetector::findPersonsPosition ( 
    const PointCloud::Ptr input_cloud, 
    std::vector<geometry_msgs::Point>* person_pt, 
    uint32_t width,  
    uint32_t height, 
    const sobit_common_msg::BoundingBoxesConstPtr &ssd_msg ) 
{
    int cnt_create = 0;
    std::vector<geometry_msgs::Point> tmp_pos;
    std::vector<bool> is_used( input_cloud->points.size(), false );

    for( const auto &obj_bbox : ssd_msg->bounding_boxes ) {
        if( obj_bbox.Class != "person" ) continue;
        int x_ctr = ( obj_bbox.xmin + obj_bbox.xmax ) / 2;
        int y_ctr = ( obj_bbox.ymin + obj_bbox.ymax ) / 2;
        int array_num = ( width * y_ctr ) + x_ctr;
        if( is_used[array_num] ) continue;
        if (!pcl_isfinite (input_cloud ->points [ array_num ].x) ||
            !pcl_isfinite (input_cloud ->points [ array_num ].y) ||
            !pcl_isfinite (input_cloud ->points [ array_num ].z)) continue;

        geometry_msgs::Point pt;
        pt.x = input_cloud->points[ array_num ].x;
        pt.y = input_cloud->points[ array_num ].y;
        pt.z = input_cloud->points[ array_num ].z;
        is_used[array_num] = true;
        tmp_pos.push_back( pt );
        cnt_create++;
    }

    *person_pt = tmp_pos;
    return cnt_create;
}

int PersonDetector::createPersonsPointCloud ( 
    const PointCloud::Ptr input_cloud, 
    PointCloud::Ptr output_cloud, 
    uint32_t width,  
    uint32_t height, 
    const sobit_common_msg::BoundingBoxesConstPtr &ssd_msg ) 
{
    int cnt_create = 0;
    uint32_t array_num;
    std::vector<bool> is_used( input_cloud->points.size(), false );
    ROS_INFO("createPersonsPointCloud");
    for( const auto &obj_bbox : ssd_msg->bounding_boxes ) {
        if( obj_bbox.Class != "person" ) continue;
        uint32_t x_init = ( obj_bbox.xmin < 0 ) ? 0 : obj_bbox.xmin;
        uint32_t y_init = ( obj_bbox.ymin < 0 ) ? 0 : obj_bbox.ymin;
        uint32_t x_limit = ( (uint32_t)obj_bbox.xmax <= width ) ? obj_bbox.xmax : width; 
        uint32_t y_limit = ( (uint32_t)obj_bbox.ymax <= height ) ? obj_bbox.ymax : height; 
        for ( uint32_t y = y_init; y < y_limit; y++ ) {
            for ( uint32_t x = x_init; x < x_limit; x++ ) {
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
    output_cloud->header.frame_id = input_cloud->header.frame_id;
    return cnt_create;
}
