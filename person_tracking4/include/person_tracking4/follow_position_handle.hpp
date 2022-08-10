#ifndef FOLLOW_POSITION
#define FOLLOW_POSITION

#include <vector>
#include <iostream>
#include <memory>
#include <geometry_msgs/Point.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <person_tracking4/FollowPosition.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace person_tracking4 {    
    class FollowPositionHandle {
        private :
            person_tracking4::FollowPositionPtr follow_position_;
            PointCloud::Ptr cloud_obstacles_;
        public :
            FollowPositionHandle();
            void initFollowPosition();
            void setTarget( const geometry_msgs::Point &target  );
            void setExistsTarget ( bool exists );
            void addObstacle ( const PointCloud::Ptr cloud_obstaclesS );
            void setProcessFlag ( const int flag );
            person_tracking4::FollowPosition getFollowPosition ( );
    };

    inline void FollowPositionHandle::initFollowPosition (  ) { 
        cloud_obstacles_->points.clear();
    }
    inline void FollowPositionHandle::setTarget( const geometry_msgs::Point &target  ) { 
        follow_position_->target = target; 
        follow_position_->target_distance = std::hypotf( target.x, target.y);
        follow_position_->target_angle = std::atan2 ( target.y, target.x );
    }
    inline void FollowPositionHandle::setExistsTarget ( bool exists ) { follow_position_->exists_target = exists; }
    inline void FollowPositionHandle::addObstacle ( const PointCloud::Ptr cloud_obstacles) { cloud_obstacles_->points.insert( cloud_obstacles_->points.end(), cloud_obstacles->points.begin(), cloud_obstacles->points.end() ); }
    inline void FollowPositionHandle::setProcessFlag ( const int flag ) { follow_position_->process_flag = flag; }

    inline person_tracking4::FollowPosition FollowPositionHandle::getFollowPosition ( ) { 
        pcl::toROSMsg<PointT>( *cloud_obstacles_, follow_position_->obstacles );
        return *follow_position_; 
    }
}
#endif