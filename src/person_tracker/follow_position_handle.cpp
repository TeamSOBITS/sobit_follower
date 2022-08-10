#include "sobit_follower/person_tracker/follow_position_handle.h"

using namespace mypcl;
FollowPositionHandle::FollowPositionHandle ( ) { 
    follow_position_.reset ( new sobit_follower::FollowPosition );
    follow_position_->is_nearby_obstacle = false;
    dist_nearby_obs_ = 0.35;
    initFollowPosition ( ); 
}

void FollowPositionHandle::addObstacle ( const PointCloud::Ptr obstacle_cloud, const Eigen::Vector4f& center ) {
    double dist_nearby_obs = dist_nearby_obs_;
    bool is_nearby_obstacle = false;
    sobit_follower::Obstacle obstacle;
    geometry_msgs::Point cnt;
    cnt.x = center.x();
    cnt.y = center.y();
    obstacle.center = cnt;
    double dist = std::hypotf( cnt.x, cnt.y );
    if( dist < dist_nearby_obs ) is_nearby_obstacle = true; 
    for ( auto& point : obstacle_cloud->points ) {
        geometry_msgs::Point pt;
        pt.x = point.x;
        pt.y = point.y;
        obstacle.points.push_back( pt );
    }
    follow_position_->obstacles.push_back( obstacle );
    if ( is_nearby_obstacle ) follow_position_->is_nearby_obstacle = is_nearby_obstacle;
}
void FollowPositionHandle::addObstacle ( const PointCloud::Ptr obstacle_cloud, const std::vector<pcl::PointIndices>& cluster_indices ) {
    sobit_follower::FollowPosition tmp_fp = *follow_position_;
    double dist_nearby_obs = dist_nearby_obs_;
    bool is_nearby_obstacle = false;
    for ( auto &cluster : cluster_indices ) {
        sobit_follower::Obstacle obstacle;
        Eigen::Vector4f centroid;  
        pcl::compute3DCentroid( *obstacle_cloud, cluster, centroid );
        geometry_msgs::Point cnt;
        cnt.x = centroid.x();
        cnt.y = centroid.y();
        obstacle.center = cnt;
        double dist = std::hypotf( cnt.x, cnt.y );
        if( dist < dist_nearby_obs ) is_nearby_obstacle = true; 
        for ( auto& i : cluster.indices ) {
            geometry_msgs::Point pt;
            pt.x = obstacle_cloud->points[i].x;
            pt.y = obstacle_cloud->points[i].y;
            obstacle.points.push_back( pt ); 
        }
        tmp_fp.obstacles.push_back( obstacle );
    }
    if ( is_nearby_obstacle ) tmp_fp.is_nearby_obstacle = is_nearby_obstacle;
    *follow_position_ = tmp_fp;
}
