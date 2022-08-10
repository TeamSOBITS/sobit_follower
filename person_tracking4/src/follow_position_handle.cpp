#include "person_tracking4/follow_position_handle.hpp"

using namespace person_tracking4;
FollowPositionHandle::FollowPositionHandle ( ) { 
    follow_position_.reset ( new person_tracking4::FollowPosition() );
    cloud_obstacles_.reset ( new PointCloud() );
    initFollowPosition ( ); 
}
