#include <person_tracking3/particle_person_tracker.hpp>

using namespace person_tracking3;

void ParticlePersonTracker::setParameters ( ) {
    target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
    search_radius_ = pnh_.param<double>( "search_radius", 0.2 );
    double laser_tolerance = pnh_.param<double>( "laser_clustering_tolerance", 0.05 );
    int laser_min_size = pnh_.param<int>( "laser_clustering_points_min_size", 1 );
    int laser_max_size = pnh_.param<int>( "laser_clustering_points_max_size", 1000 );      
    double large_laser_tolerance = pnh_.param<double>( "large_laser_clustering_tolerance", 0.1 );
    int large_laser_min_size = pnh_.param<int>( "large_laser_clustering_points_min_size", 10 );
    int large_laser_max_size = pnh_.param<int>( "large_laser_clustering_points_max_size", 1000 );        
    cluster_size_large_obs_ = pnh_.param<std::vector<double>>( "cluster_size_large_obstacle", {0.45, 0.45} );
    pch_.setClusteringParameters ( laser_tolerance, laser_min_size, laser_max_size );
    pch_.setLargeClusteringParameters ( large_laser_tolerance, large_laser_min_size, large_laser_max_size );
    double radius  = pnh_.param<double>( "neighborhood_search_radius", 0.3 );
    int min_pts = pnh_.param<int>( "number_of_nearest_neighbors_radius", 2 );
    bool keep_organized = pnh_.param<bool>( "keep_organized", false );
    pch_.setRadiusOutlierRemovalParameters ( radius, min_pts, keep_organized );
    detect_allowable_dist_ = pnh_.param<double>( "detect_allowable_distance", 1.5 );
    tracking_allowable_dist_ = pnh_.param<double>( "tracking_allowable_distance", 0.2 );
    prediction_allowable_dist_ = pnh_.param<double>( "prediction_allowable_distance", 0.4 );
    double nearby_obs_dist = pnh_.param<double>( "nearby_obstacle_distance", 0.3 );
    fph_.setDistanceNearbyObstacle( nearby_obs_dist );
    predicted_exec_time_ = pnh_.param<double>( "predicted_execution_time", 2.0 );
    should_output_marker_ = pnh_.param<bool>( "should_output_marker", false );
    std::cout << "\n========================================\n[ ParticlePersonTracker parameters ]"
        << "\n\n< ParticlePersonTracker : target frame >"
        << "\n  * target_frame                           : " << target_frame_ 
        << "\n\n< ParticlePersonTracker : radius search >"
        << "\n  * search_radius                          : " << search_radius_  
        << "\n\n< ParticlePersonTracker : allowable_distance >"   
        << "\n  * detect_allowable_distance              : " << detect_allowable_dist_         
        << "\n  * tracking_allowable_distance            : " << tracking_allowable_dist_
        << "\n  * prediction_allowable_distance          : " << prediction_allowable_dist_
        << "\n\n< ParticlePersonTracker : cluster_size >"
        << "\n  * cluster_size_large_obstacle            : " << cluster_size_large_obs_[0] << ", " << cluster_size_large_obs_[1] 
        << "\n\n< ParticlePersonTracker : Time >" 
        << "\n  * predicted_execution_time               : " << predicted_exec_time_
        << "\n\n< ParticlePersonTracker : flag >" 
        << "\n  * should_output_marker                   : " << (should_output_marker_ ? "True" : "False" )
        << "\n\n< PCLHandle             : clustering >"
        << "\n  * laser_clustering_tolerance             : " << laser_tolerance
        << "\n  * laser_clustering_points_min_size       : " << laser_min_size
        << "\n  * laser_clustering_points_max_size       : " << laser_max_size
        << "\n  * large_laser_clustering_tolerance       : " << large_laser_tolerance
        << "\n  * large_laser_clustering_points_min_size : " << large_laser_min_size
        << "\n  * large_laser_clustering_points_max_size : " << large_laser_max_size
        << "\n\n< PCLHandle             : RadiusOutlierRemoval >"
        << "\n  * neighborhood_search_radius             : " << radius
        << "\n  * number_of_nearest_neighbors_radius     : " << min_pts
        << "\n  * keep_organized                         : " << ( keep_organized ? "True" : "False" )
        << "\n========================================\n"
    << std::endl;
}    

visualization_msgs::Marker ParticlePersonTracker::makeMarker ( 
    const geometry_msgs::Point &target_pt, 
    const int flag, 
    const ros::Time& stamp ) 
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame_;
    marker.header.stamp = stamp;
    marker.ns = "target_pose";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position = target_pt;

    double motion_dist, motion_orien;
    pct_.getTargetTrajectoryMotion2D( &motion_dist, &motion_orien );
    tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, motion_orien);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    marker.pose.orientation = geometry_quat;

    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = ( flag == DETECTION ? 1.0f : 0.0f );
    marker.color.g = ( flag == TRACKING ? 1.0f : 0.0f );
    marker.color.b = ( flag == PREDICTION ? 1.0f : 0.0f );
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration(1.0);
    return marker;
}

void ParticlePersonTracker::sortLaserCloud ( PointCloud::Ptr input_laser ) {
    if ( process_flag_ != TRACKING ) return;
    PointCloud::Ptr tmp_laser (new PointCloud());
    PointCloud::Ptr tmp_cloud ( new PointCloud() );
    PointCloud::Ptr around_tgt_cloud ( new PointCloud() );
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::PointIndices::Ptr tmp_cluster(new pcl::PointIndices);
    pcl::PointIndices::Ptr cluster_large_obs(new pcl::PointIndices);
    pcl::PointIndices::Ptr around_tgt (new pcl::PointIndices);
    std::vector<int> mapping;
    std::vector<double> cluster_size_large_obs = cluster_size_large_obs_;
    pcl::removeNaNFromPointCloud( *input_laser, *tmp_laser, mapping );
    pch_.radiusOutlierRemoval( tmp_laser, tmp_laser );
    if( pch_.radiusSearch( tmp_laser, around_tgt, pre_tgt_pt_, 0.5, false) ){
        pch_.extractIndices( tmp_laser, around_tgt_cloud, around_tgt, false );
        pch_.extractIndices( tmp_laser, tmp_laser, around_tgt, true );
    }
    pch_.largeClusterExtraction ( tmp_laser, &cluster_indices );
    for ( auto &cluster : cluster_indices ) {
        Eigen::Vector4f min_pt, max_pt, centroid, cluster_size;
        pcl::compute3DCentroid( *tmp_laser, cluster, centroid );
        pcl::getMinMax3D( *tmp_laser , cluster, min_pt, max_pt);
        cluster_size = max_pt - min_pt;
        if ( cluster_size.x() < cluster_size_large_obs[0] && cluster_size.y() < cluster_size_large_obs[1] ) continue;
        cluster_large_obs->indices.insert(cluster_large_obs->indices.end(), cluster.indices.begin(), cluster.indices.end());
        // addObstacle ( Large ) :
        *tmp_cluster = cluster;         
        pch_.extractIndices( tmp_laser, tmp_cloud, tmp_cluster, false );
        fph_.addObstacle( tmp_cloud, centroid );
    }
    tmp_laser->header.frame_id = target_frame_;
    pch_.extractIndices( tmp_laser, tmp_laser, cluster_large_obs, true );
    tmp_laser->points.insert( tmp_laser->points.end(), around_tgt_cloud->points.begin(), around_tgt_cloud->points.end() );
    *input_laser = *tmp_laser;
    return;
}

geometry_msgs::Point ParticlePersonTracker::transformPoint ( std::string org_frame, std::string target_frame, geometry_msgs::Point point ) {
    geometry_msgs::PointStamped pt_transformed;
    geometry_msgs::PointStamped pt;
    pt.header.frame_id = org_frame;
    pt.header.stamp = ros::Time(0);
    pt.point = point;
    if ( tf_listener_.frameExists( target_frame ) ) {
        try {
            tf_listener_.transformPoint( target_frame, pt, pt_transformed );
        } catch ( tf::TransformException ex ) {
            ROS_ERROR( "%s",ex.what( ) );
        }
    } else {
        ROS_ERROR("target_frame is not Exists");
    }
    return pt_transformed.point;
}

bool ParticlePersonTracker::detectPersonPosition( 
    const PointCloud::Ptr laser, 
    const std::vector<geometry_msgs::Point>& person_pts,
    PointCloud::Ptr cloud_obs, 
    PointCloud::Ptr cloud_tgt, 
    geometry_msgs::Point* tgt_pt ) 
{
    geometry_msgs::Point search_pt, tmp_tgt_pt;
    if ( !decideTargetPerson( person_pts, search_pt, &tmp_tgt_pt, process_flag_ ) ) {
        ROS_ERROR ( "detectPersonPosition() : Failed to decide Tracking target." );
        return false;                
    }
    // Find Matching Point to Laser :
    pcl::PointIndices::Ptr target_indices (new pcl::PointIndices);
    if ( !pch_.radiusSearch( laser, target_indices, tmp_tgt_pt, search_radius_, false) ) {
        ROS_ERROR ( "detectPersonPosition() : Failed to find matching point to Laser." );
        return false;                
    }
    pch_.extractIndices( laser, cloud_tgt, target_indices, false );
    pch_.extractIndices( laser, cloud_obs, target_indices, true );
    Eigen::Vector4f centroid;          
    pcl::compute3DCentroid( *cloud_tgt, centroid );
    tmp_tgt_pt.x = centroid.x();
    tmp_tgt_pt.y = centroid.y();
    tmp_tgt_pt.z = centroid.z();
    // setTrackTarget :
    pct_.setTrackTarget( cloud_tgt, tmp_tgt_pt );
    *tgt_pt = tmp_tgt_pt;
    pre_tgt_pt_ = tmp_tgt_pt; 
    return true;
}

int ParticlePersonTracker::trackPersonPosition(
    const PointCloud::Ptr laser,  
    const std::vector<geometry_msgs::Point>& person_pts,
    PointCloud::Ptr cloud_obs, 
    PointCloud::Ptr cloud_tgt, 
    geometry_msgs::Point* tgt_pt )
{
    bool result = false;
    bool is_accept_add_point;
    geometry_msgs::Point search_pt, tmp_tgt_pt;
    if( std::fabs( std::atan2( pre_tgt_pt_.y, pre_tgt_pt_.x) ) <= M_PI*5/12 ) {
        result = pct_.getTrackResult( laser, cloud_tgt, &tmp_tgt_pt );
        if ( !result ) {
            ROS_ERROR("trackPersonPosition() : Couldn't get any results from the particle filter."); 
            predicted_start_time_ = ros::Time::now().toSec();
            *tgt_pt = pre_tgt_pt_;
            return result; 
        }
        geometry_msgs::Point tgt_pt_xtion = transformPoint ( target_frame_, "xtion_pan_link", tmp_tgt_pt );
        double tgt_ang = std::atan2( tgt_pt_xtion.y, tgt_pt_xtion.x);
        double tgt_dist = std::hypotf( tmp_tgt_pt.x, tmp_tgt_pt.y );
        if (  fabs( tgt_ang ) < 0.43 && tgt_dist > 0.35 && person_pts.size() != 0 ) {
            search_pt = tmp_tgt_pt;
            if ( !decideTargetPerson( person_pts, search_pt, &tmp_tgt_pt, process_flag_ ) ) {
                cnt_lost_tgt_++;
                if ( cnt_lost_tgt_ > 0 ) {
                    ROS_ERROR ( "trackPersonPosition() : Failed to decide Tracking target." );
                    predicted_start_time_ = ros::Time::now().toSec();
                    *tgt_pt = search_pt;
                    return false; 
                }
            } else cnt_lost_tgt_ = 0;
        }
        is_accept_add_point = false;
    } else {
        result = decideTargetPerson( person_pts, pre_tgt_pt_, &tmp_tgt_pt, process_flag_ );
        if ( !result ) {
            ROS_ERROR ( "trackPersonPosition() : Target is out of tracking range." );
            predicted_start_time_ = ros::Time::now().toSec();
            *tgt_pt = pre_tgt_pt_;
            return result;                
        }
        is_accept_add_point = true;
    }
    // Find Matching Point to Laser :
    pcl::PointIndices::Ptr target_indices (new pcl::PointIndices);
    if ( !pch_.radiusSearch( laser, target_indices, tmp_tgt_pt, search_radius_, is_accept_add_point ) ) {
        ROS_ERROR ( "trackPersonPosition() : Failed to find matching point to Laser." );
        predicted_start_time_ = ros::Time::now().toSec();
        *tgt_pt = tmp_tgt_pt;
        return false;                
    }
    // Set Target cloud and Obstacle cloud :
    pch_.extractIndices( laser, cloud_tgt, target_indices, false );
    pch_.extractIndices( laser, cloud_obs, target_indices, true );
    Eigen::Vector4f centroid;          
    pcl::compute3DCentroid( *cloud_tgt, centroid );
    tmp_tgt_pt.x = centroid.x();
    tmp_tgt_pt.y = centroid.y();
    tmp_tgt_pt.z = centroid.z();
    pct_.changeTrackTarget( cloud_tgt, tmp_tgt_pt );
    *tgt_pt = tmp_tgt_pt;
    pre_tgt_pt_ = tmp_tgt_pt; 
    return result; 
}

int ParticlePersonTracker::predictPersonPosition(
    const PointCloud::Ptr laser,  
    const std::vector<geometry_msgs::Point>& person_pts,
    PointCloud::Ptr cloud_obs, 
    PointCloud::Ptr cloud_tgt, 
    geometry_msgs::Point* tgt_pt )
{
    int result = PREDICTION;
    pcl::PointIndices::Ptr target_indices (new pcl::PointIndices);
    geometry_msgs::Point search_pt, tgt_pt_xtion, predicted_pt, tmp_tgt_pt;
    // Check execution time :
    double elapsed_time = curt_time_ - predicted_start_time_;
    
    if ( elapsed_time > predicted_exec_time_ ) {
        ROS_ERROR( "predictPersonPosition() : Prediction time has expired." );
        return DETECTION;
    }
    // Updata predicted point(the largest particles) from motion :
    predicted_pt = transformPoint ( "odom", target_frame_, predicted_pt_odom_ );
    double motion_dist, motion_orien;
    pct_.getTargetMotion2D( &motion_dist, &motion_orien );
    if( motion_dist > 0.06 ) {
        predicted_pt.x += cos ( motion_orien ) * ( motion_dist * 0.05 );
        predicted_pt.y += sin ( motion_orien ) * ( motion_dist * 0.05 );
        predicted_pt_odom_ = transformPoint ( target_frame_, "odom", predicted_pt );
    }
    // Compare tracking_point with SSD_NODE results : 
    search_pt = predicted_pt; 
    if ( !decideTargetPerson( person_pts, search_pt, &tmp_tgt_pt, process_flag_ ) ) {
        ROS_ERROR ( "predictPersonPosition() : There is no person at the prediction point." );  
        *tgt_pt = predicted_pt;            
    } else {
        // Find Matching Point to Laser :
        pch_.radiusSearch( laser, target_indices, tmp_tgt_pt, search_radius_, false );
        if ( target_indices->indices.size() == 0 ) {
            ROS_ERROR ( "predictPersonPosition() : Failed to find matching point to Laser." );
            *tgt_pt = tmp_tgt_pt;
            return result;                
        }
        pch_.extractIndices( laser, cloud_tgt, target_indices, false );
        pch_.extractIndices( laser, cloud_obs, target_indices, true );
        Eigen::Vector4f centroid;          
        pcl::compute3DCentroid( *cloud_tgt, centroid );
        tmp_tgt_pt.x = centroid.x();
        tmp_tgt_pt.y = centroid.y();
        tmp_tgt_pt.z = centroid.z();
        // setTrackTarget :
        ROS_INFO("predictPersonPosition() : Detect lost targets");
        pct_.setTrackTarget( cloud_tgt, tmp_tgt_pt );
        pre_tgt_pt_ = tmp_tgt_pt; 
        result = TRACKING;
        *tgt_pt = tmp_tgt_pt;
    }
    return result;
}

// Decide Target Person :
bool ParticlePersonTracker::decideTargetPerson (   
    const std::vector<geometry_msgs::Point>& person_pts, 
    geometry_msgs::Point& search_pt, 
    geometry_msgs::Point* target_pt, 
    const int process_flag ) 
{
    bool is_decide = false;
    double target_distance = DBL_MAX;
    geometry_msgs::Point tmp_pt; 

    if ( process_flag ) { search_pt.x = 0.0; search_pt.y = 0.0; search_pt.z = 0.0; }
    double allowable_dist = (process_flag ) ? detect_allowable_dist_ : (process_flag == TRACKING) ? tracking_allowable_dist_ : prediction_allowable_dist_;
    
    for ( auto &person : person_pts ) {
        double distance = std::hypotf( search_pt.x - person.x, search_pt.y - person.y ); 
        if ( distance > allowable_dist ) continue;
        if ( distance < target_distance ) {
            tmp_pt.x = person.x;
            tmp_pt.y = person.y;
            tmp_pt.z = person.z;
            target_distance =  distance;
            is_decide = true;
        }
    }
    *target_pt = tmp_pt;
    return is_decide;
}

ParticlePersonTracker::ParticlePersonTracker() : nh_(), pnh_("~")  {
    setParameters();
    fph_.initFollowPosition();
    cnt_lost_tgt_ = 0;
    process_flag_ = DETECTION;
    curt_time_ = ros::Time::now().toSec();
    // Publisher :
    pub_tgt_ = nh_.advertise<PointCloud>("target", 1);
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("marker", 1);
    pub_laser_ = nh_.advertise<PointCloud>("urg", 1);
}

int ParticlePersonTracker::computeParticle( 
    PointCloud::Ptr laser,  
    const std::vector<geometry_msgs::Point>& person_pts, 
    const ros::Time& stamp,
    person_tracking3::FollowPosition* follow_position ) 
{
    geometry_msgs::Point tgt_pt;
    bool exists_tgt = false;
    int cnt_person = person_pts.size();
    person_tracking3::FollowPosition tmp_fp;
    PointCloud::Ptr cloud_tgt (new PointCloud());
    PointCloud::Ptr cloud_obs (new PointCloud());

    sortLaserCloud( laser );
    pcl_conversions::toPCL(stamp, laser->header.stamp);

    // Detect, Track :
    if ( process_flag_ == DETECTION && cnt_person != 0 ) {
        ROS_INFO("DETECTION");
        exists_tgt = detectPersonPosition ( laser, person_pts, cloud_obs, cloud_tgt, &tgt_pt );
        process_flag_ = ( exists_tgt ) ?  TRACKING : DETECTION;
    } else if ( process_flag_ == DETECTION && cnt_person == 0 ) {
        ROS_INFO("NO PERSON");
    } else if ( process_flag_ == TRACKING ) {
        ROS_INFO("TRACKING");
        exists_tgt = trackPersonPosition ( laser, person_pts, cloud_obs, cloud_tgt, &tgt_pt );
        process_flag_ = ( exists_tgt ) ?  TRACKING : PREDICTION;
        if ( !exists_tgt ) predicted_pt_odom_ = transformPoint ( target_frame_, "odom", pre_tgt_pt_ );
    } 
    // Predict :
    if ( process_flag_ == PREDICTION ) {
        ROS_INFO("PREDICTION");
        process_flag_ = predictPersonPosition ( laser, person_pts, cloud_obs, cloud_tgt, &tgt_pt );
        exists_tgt = ( process_flag_ == TRACKING ) ? true : false;
    } 
    // set Trajectory :
    if( process_flag_ == DETECTION ) pct_.resetTargetTrajectory(); 
    else pct_.addTargetTrajectory( tgt_pt );
    // Set FollowPosition :
    fph_.setExistsTarget( exists_tgt );
    fph_.setProcessFlag( process_flag_ );
    fph_.setTarget ( tgt_pt );
    if( exists_tgt ) {
        std::vector<pcl::PointIndices> cluster_indices;
        pch_.euclideanClusterExtraction( cloud_obs, &cluster_indices );
        fph_.addObstacle( cloud_obs, cluster_indices );
        cloud_obs->header.frame_id = target_frame_;
        //pub_obs_.publish(cloud_obs);
    }
    *follow_position = fph_.getFollowPosition ( );
    if ( should_output_marker_ ) {
        pub_marker_.publish( makeMarker(tgt_pt, process_flag_, stamp ) );
        pcl_conversions::toPCL(stamp, cloud_tgt->header.stamp);
        pub_tgt_.publish( cloud_tgt ); 
        pub_laser_.publish( laser );
    }
    sensor_rotator_.rotateSensor( follow_position->process_flag, follow_position->target_distance, follow_position->target_angle );
    // Update Time :
    curt_time_ = ros::Time::now().toSec();
    //  Allocate memory :
    fph_.initFollowPosition();
    return process_flag_;
}