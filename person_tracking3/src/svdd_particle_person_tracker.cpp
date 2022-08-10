#include <person_tracking3/svdd_particle_person_tracker.hpp>

using namespace person_tracking3;

int SVDDParticlePersonTracker::detectLegsSVDD ( 
    const PointCloud::Ptr laser, 
    std::vector<svdd_leg_tracker_library::PersonInfo>* persons ) 
{
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<svdd_leg_tracker_library::LegInfo> legs;
    std::vector<svdd_leg_tracker_library::PersonInfo> tmp_persons;
    visualization_msgs::MarkerArray leg_marker;
    ros::Time stamp;
    pcl_conversions::fromPCL( laser->header.stamp, stamp );
    pch_.radiusOutlierRemoval( laser, laser );
    if ( laser->points.empty() ) return 0;
    pch_.euclideanClusterExtraction( laser , &cluster_indices );
    if ( leg_tracker_.detectLegs( laser, cluster_indices, &legs ) == 0 ) return 0;
    int person_num =  leg_tracker_.detectPersons( legs, &tmp_persons );
    leg_tracker_.makePersonsMaker ( laser, cluster_indices, tmp_persons, stamp, &leg_marker );
    pub_leg_marker_.publish(leg_marker);
    *persons = tmp_persons;
    return person_num;
}

bool SVDDParticlePersonTracker::detectPersonPositionSVDD ( const PointCloud::Ptr laser )
{
    try {        
        // Rotate the sensor in the direction of the person detected by SVDD, and detect the person again by SSD.
        std::vector<svdd_leg_tracker_library::PersonInfo> persons;
        if ( detectLegsSVDD ( laser, &persons ) == 0 ) {
            svdd_results_.clear();
            return false;
        }
        if ( svdd_results_.empty() ) {
            std::cout << "[ Set SVDD Results ]" << std::endl;
            int cnt = 1;
            //double allowable_dist = detect_allowable_dist_;
            for ( const auto person : persons ) {
                double dist = std::hypotf( person.centroid_.x(), person.centroid_.y() );
                if ( dist > 0.6 ) continue;
                svdd_results_.push_back( PersonList ( std::hypotf( person.centroid_.x(), person.centroid_.y() ), std::atan2( person.centroid_.y(), person.centroid_.x() ) ) );
                std::cout << "[ " << cnt << " ] : Distance = " << std::hypotf( person.centroid_.x(), person.centroid_.y() ) << ", angle = " <<  std::atan2( person.centroid_.y(), person.centroid_.x() ) << std::endl;
            }
            std::sort( svdd_results_.begin(), svdd_results_.end(), PersonList::compareDistance );
        } 
        std::cout << "list size   : " << svdd_results_.size() << std::endl;
        if ( svdd_results_.size() == 0 ) return true; 
        PersonList svdd_person = svdd_results_.back();
        std::cout << "target info : Distance = " << svdd_person.distance_ << ", angle = " <<  svdd_person.angle_ << std::endl;
        svdd_results_.pop_back();
        sensor_rotator_.rotateSensor( svdd_person.distance_, svdd_person.angle_ );
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

int SVDDParticlePersonTracker::predictPersonPositionSVDD ( const PointCloud::Ptr laser )
{
    try {      
        double elapsed_time = curt_time_ - predicted_start_time_;
        if ( elapsed_time > predicted_exec_time_ ) {
            ROS_ERROR( "predictPersonPositionSVDD() : Prediction time has expired." );
            return DETECTION;
        }  
        // Rotate the sensor in the direction of the person detected by SVDD, and detect the person again by SSD.
        std::vector<svdd_leg_tracker_library::PersonInfo> persons;
        if ( detectLegsSVDD ( laser, &persons ) == 0 ) return PREDICTION;
        double nearest_dist = DBL_MAX, target_dist = -1.0, target_ang = -1.0;
        geometry_msgs::Point predicted_pt = transformPoint ( "odom", target_frame_, predicted_pt_odom_ );
        for ( const auto person : persons ) {
            double prediction_point_dist = std::hypotf( predicted_pt.x - person.centroid_.x(), predicted_pt.y - person.centroid_.y() );
            if ( prediction_point_dist < nearest_dist ) {
                nearest_dist = prediction_point_dist;
                target_dist = std::hypotf( person.centroid_.x(), person.centroid_.y() );
                target_ang = std::atan2( person.centroid_.y(), person.centroid_.x() );
            }
        }
        sensor_rotator_.rotateSensor( target_dist, target_ang );
        return PREDICTION;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return DETECTION;
    }
}

SVDDParticlePersonTracker::SVDDParticlePersonTracker() {
    svdd_results_.clear();
    pub_leg_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("leg_marker", 1);
}

int SVDDParticlePersonTracker::computeSVDDParticle( 
    PointCloud::Ptr laser,  
    const std::vector<geometry_msgs::Point>& person_pts, 
    const ros::Time& stamp,
    person_tracking3::FollowPosition* follow_position ) 
{
    geometry_msgs::Point tgt_pt;
    bool exists_tgt = false, is_rotate = true;
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
        svdd_results_.clear();
    } else if ( process_flag_ == DETECTION && cnt_person == 0 ) {
        ROS_INFO("NO PERSON : DETECTION");
        detectPersonPositionSVDD( laser );
        //is_rotate = false;
    } else if ( process_flag_ == TRACKING ) {
        ROS_INFO("TRACKING");
        exists_tgt = trackPersonPosition ( laser, person_pts, cloud_obs, cloud_tgt, &tgt_pt );
        process_flag_ = ( exists_tgt ) ?  TRACKING : PREDICTION;
        if ( !exists_tgt ) predicted_pt_odom_ = transformPoint ( target_frame_, "odom", pre_tgt_pt_ );
    } 
    // Predict :
    if ( process_flag_ == PREDICTION && cnt_person != 0 ) {
        ROS_INFO("PREDICTION");
        process_flag_ = predictPersonPosition ( laser, person_pts, cloud_obs, cloud_tgt, &tgt_pt );
        exists_tgt = ( process_flag_ == TRACKING ) ? true : false;
    } else if ( process_flag_ == PREDICTION && cnt_person == 0 ) {
        ROS_INFO("NO PERSON : PREDICTION");
        process_flag_ = predictPersonPositionSVDD( laser );
        //is_rotate = false;
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
    if ( is_rotate ) sensor_rotator_.rotateSensor( follow_position->process_flag, follow_position->target_distance, follow_position->target_angle );
    // Update Time :
    curt_time_ = ros::Time::now().toSec();
    //  Allocate memory :
    fph_.initFollowPosition();
    return process_flag_;
}