#include "person_tracking3/follow_position_handle.hpp"

using namespace person_tracking3;
FollowPositionHandle::FollowPositionHandle ( ) { 
    follow_position_.reset ( new person_tracking3::FollowPosition() );
    follow_position_->is_nearby_obstacle = false;
    dist_nearby_obs_ = 0.35;
    initFollowPosition ( ); 
}

void FollowPositionHandle::addObstacle ( const PointCloud::Ptr obstacle_cloud, const Eigen::Vector4f& center ) {
    double dist_nearby_obs = dist_nearby_obs_;
    bool is_nearby_obstacle = false;
    person_tracking3::Obstacle obstacle;
    geometry_msgs::Point cnt;
    cnt.x = center.x();
    cnt.y = center.y();
    obstacle.center = cnt;
    double dist = std::hypotf( cnt.x, cnt.y );
    if( dist < dist_nearby_obs ) is_nearby_obstacle = true; 
    obstacle.points = obstacle_cloud->points;
    follow_position_->obstacles.push_back( obstacle );
    if ( is_nearby_obstacle ) follow_position_->is_nearby_obstacle = is_nearby_obstacle;
}
void FollowPositionHandle::addObstacle ( const PointCloud::Ptr obstacle_cloud, const std::vector<pcl::PointIndices>& cluster_indices, const int lleg_idx, const int rleg_idx ) {
    person_tracking3::FollowPosition tmp_fp = *follow_position_;
    double dist_nearby_obs = dist_nearby_obs_;
    bool is_nearby_obstacle = false;
    int idx = -1;
    for ( auto &cluster : cluster_indices ) {
        idx++;
        if ( idx == lleg_idx || idx == rleg_idx ) continue;
        person_tracking3::Obstacle obstacle;
        Eigen::Vector4f centroid;  
        pcl::compute3DCentroid( *obstacle_cloud, cluster, centroid );
        geometry_msgs::Point cnt;
        cnt.x = centroid.x();
        cnt.y = centroid.y();
        obstacle.center = cnt;
        double dist = std::hypotf( cnt.x, cnt.y );
        if( dist < dist_nearby_obs ) is_nearby_obstacle = true; 
        for ( auto& i : cluster.indices ) obstacle.points.push_back( obstacle_cloud->points[i] ); 
        tmp_fp.obstacles.push_back( obstacle );
    }
    if ( is_nearby_obstacle ) tmp_fp.is_nearby_obstacle = is_nearby_obstacle;
    *follow_position_ = tmp_fp;
}
