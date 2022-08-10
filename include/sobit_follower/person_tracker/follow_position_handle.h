#ifndef FOLLOW_POSITION
#define FOLLOW_POSITION

#include <sobit_follower/FollowPosition.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace mypcl {

class FollowPositionHandle {
    private :
        sobit_follower::FollowPositionPtr follow_position_;
        double dist_nearby_obs_;
    public :
        FollowPositionHandle();
        void initFollowPosition();
        void setDistanceNearbyObstacle ( double distance );
        void setTarget( const geometry_msgs::Point target  );
        void setExistsTarget ( bool exists );
        void addObstacle ( const PointCloud::Ptr obstacle_cloud, const Eigen::Vector4f& center );
        void addObstacle ( const PointCloud::Ptr obstacle_cloud, const std::vector<pcl::PointIndices>& cluster_indices );
        void setProcessFlag ( const int flag );
        sobit_follower::FollowPosition getFollowPosition ( );
};

inline void FollowPositionHandle::initFollowPosition (  ) { 
    follow_position_->obstacles.clear();
    follow_position_->is_nearby_obstacle = false;
}
inline void FollowPositionHandle::setDistanceNearbyObstacle ( double distance ) { dist_nearby_obs_ = distance; }
inline void FollowPositionHandle::setTarget( const geometry_msgs::Point target  ) { 
    follow_position_->target = target; 
    follow_position_->target_distance = std::hypotf( target.x, target.y);
    follow_position_->target_angle = std::atan2 ( target.y, target.x );
}
inline void FollowPositionHandle::setExistsTarget ( bool exists ) { follow_position_->exists_target = exists; }
inline void FollowPositionHandle::setProcessFlag ( const int flag ) { follow_position_->process_flag = flag; }

inline sobit_follower::FollowPosition FollowPositionHandle::getFollowPosition ( ) { return *follow_position_; }


}
#endif