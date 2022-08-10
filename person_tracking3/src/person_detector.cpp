#include <person_tracking3/person_detector.hpp>

using namespace person_tracking3;
PersonDetector::PersonDetector ( ) { }

int PersonDetector::findPersonsPosition ( 
    const PointCloud::Ptr input_cloud, 
    std::vector<geometry_msgs::Point>* person_pt, 
    uint32_t width,  
    uint32_t height, 
    const ssd_node::BoundingBoxesConstPtr &ssd_msg ) 
{
    int cnt_create = 0;
    std::vector<geometry_msgs::Point> tmp_pos;
    std::vector<bool> is_used( input_cloud->points.size(), false );

    for( const auto &ssd : ssd_msg->boundingBoxes ) {
        if( ssd.Class != "person" ) continue;
        int x_ctr = ( ssd .x + ( ssd .x + ssd .width - 1 ) ) / 2;
        int y_ctr = ( ssd .y + ( ssd .y + ssd .height - 1 ) ) / 2;
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
    const ssd_node::BoundingBoxesConstPtr &ssd_msg ) 
{
    int cnt_create = 0;
    uint32_t array_num;
    std::vector<bool> is_used( input_cloud->points.size(), false );
    ROS_INFO("createPersonsPointCloud");
    for( const auto &ssd : ssd_msg->boundingBoxes ) {
        if( ssd.Class != "person" ) continue;
        uint32_t x_init = ( ssd.x < 0 ) ? 0 : ssd.x;
        uint32_t y_init = ( ssd.y < 0 ) ? 0 : ssd.y;
        uint32_t x_limit = ( x_init + ssd .width <= width ) ? x_init + ssd .width : width; 
        uint32_t y_limit = ( y_init + ssd .height <= height ) ? y_init + ssd .height : height; 
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
