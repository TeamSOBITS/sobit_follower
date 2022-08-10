#include "sobit_follower/sub_functions/point_cloud_tracker.h"

using namespace mypcl;
PointCloudTracker::PointCloudTracker ( ) {
    // ParticleFilterTracker :
    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;
    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);
    pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointT, ParticleT> *tracker
    (new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointT, ParticleT> (8));
    //Set the size of one particle 
    ParticleT bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.0f;
    bin_size.roll = 0.0f;
    bin_size.pitch = 0.0f;
    bin_size.yaw = 0.1f;
    // Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
    tracker->setMaximumParticleNum(1000);
    tracker->setDelta(0.99);
    tracker->setEpsilon(0.2);
    tracker->setBinSize(bin_size);
    // Set all parameters for  ParticleFilter
    tracker_ = tracker;
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);
    tracker_->setParticleNum (600);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);
    //Setup coherence object for tracking
    pcl::tracking::ApproxNearestPairPointCloudCoherence<PointT>::Ptr coherence 
    (new pcl::tracking::ApproxNearestPairPointCloudCoherence<PointT>);
    pcl::tracking::DistanceCoherence<PointT>::Ptr distance_coherence 
    (new pcl::tracking::DistanceCoherence<PointT>);
    coherence->addPointCoherence (distance_coherence);
    pcl::search::Octree<PointT>::Ptr search (new pcl::search::Octree<PointT> (0.01));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);
    tracker_->setCloudCoherence (coherence);
    // allocate Memory :
    cloud_tracked_target_.reset ( new PointCloud() );
    // target_trajectory_ clear : 
    target_trajectory_.clear();
}
// Set Particle Filter :
bool PointCloudTracker::setTrackTarget ( const PointCloud::Ptr ref_cloud, const geometry_msgs::Point& ref_pt ) {
    try {
        Eigen::Affine3f trans = Eigen::Affine3f::Identity();
        trans.translation ().matrix () = Eigen::Vector3f ( ref_pt.x, ref_pt.y, ref_pt.z );
        pcl::transformPointCloud<PointT> ( *ref_cloud, *cloud_tracked_target_, trans.inverse() );
        tracker_->resetTracking();
        tracker_->setReferenceCloud ( cloud_tracked_target_ );
        tracker_->setTrans ( trans );
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
// change track target :
bool PointCloudTracker::changeTrackTarget ( const PointCloud::Ptr ref_cloud, const geometry_msgs::Point& ref_pt ) {
    try {
        Eigen::Affine3f trans = Eigen::Affine3f::Identity();
        trans.translation ().matrix () = Eigen::Vector3f ( ref_pt.x, ref_pt.y, ref_pt.z );
        pcl::transformPointCloud<PointT> ( *ref_cloud, *cloud_tracked_target_, trans.inverse() );
        tracker_->setReferenceCloud ( cloud_tracked_target_ );
        tracker_->setTrans ( trans );
        return true;
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
// Get Particle Filter Tracking Result :
bool PointCloudTracker::getTrackResult ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, geometry_msgs::Pose* output_pose ) {
    try {
        tracker_->setInputCloud ( input_cloud );
        tracker_->compute ( );
        ParticleT pf_result = tracker_->getResult ();
        if (!pcl_isfinite (pf_result.x) || 
            !pcl_isfinite (pf_result.y) || 
            !pcl_isfinite (pf_result.z)) {
            ROS_ERROR( "Tracking failed : TrackResult is Nan data." );
            output_pose->position.x = 0.0;
            output_pose->position.y = 0.0;
            output_pose->position.z = 0.0;
            output_pose->orientation.x = 0.0;
            output_pose->orientation.y = 0.0;
            output_pose->orientation.z = 0.0;
            return false;
        } else {
            // Get Target PointCloud :
            Eigen::Affine3f transformation = tracker_->toEigenMatrix ( pf_result );
            pcl::transformPointCloud<PointT> ( *(tracker_->getReferenceCloud ()), *output_cloud, transformation );
            // Get Target Pose :
            output_pose->position.x = pf_result.x;
            output_pose->position.y = pf_result.y;
            output_pose->position.z = pf_result.z;
            tf::Quaternion quat = tf::createQuaternionFromRPY( pf_result.roll, pf_result.pitch, pf_result.yaw );
            quaternionTFToMsg( quat, output_pose->orientation ); 
            // add trajectory :
            target_trajectory_.push_back( output_pose->position );
            if ( target_trajectory_.size () > 10 ) target_trajectory_.erase ( target_trajectory_.begin( ) );
            return true;
        }
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
// Get Particle Filter Tracking Result :
bool PointCloudTracker::getTrackResult ( const PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, geometry_msgs::Point* output_pt ) {
    try {
        tracker_->setInputCloud ( input_cloud );
        tracker_->compute ( );
        ParticleT pf_result = tracker_->getResult ();
        if (!pcl_isfinite (pf_result.x) || 
            !pcl_isfinite (pf_result.y) || 
            !pcl_isfinite (pf_result.z)) {
            ROS_ERROR( "Tracking failed : TrackResult is Nan data." );
            output_pt->x = 0.0;
            output_pt->y = 0.0;
            output_pt->z = 0.0;
            return false;
        } else {
            // Get Target PointCloud :
            Eigen::Affine3f transformation = tracker_->toEigenMatrix ( pf_result );
            pcl::transformPointCloud<PointT> ( *(tracker_->getReferenceCloud ()), *output_cloud, transformation );
            // Get Target Pose :
            output_pt->x = pf_result.x;
            output_pt->y = pf_result.y;
            output_pt->z = pf_result.z; 
            return true;
        }
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}
// Get the current particles :
bool PointCloudTracker::getParticles ( PointCloud::Ptr output_cloud ) {
    try {
        ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
        PointCloud::Ptr tmp ( new PointCloud() );   
        //Set pointCloud with particle's points
        for( auto &particle : particles->points ) {
            PointT point;
            point.x = particle.x;
            point.y = particle.y;
            point.z = particle.z;
            tmp->points.push_back (point);
        }
        *output_cloud = *tmp;
        return true;        
    } catch ( std::exception& ex ) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

void PointCloudTracker::addTargetTrajectory ( const geometry_msgs::Point& current_measured_pt ) {
    if ( moving_distance_ == -1.0 ) {
        current_predicted_pt_ = current_measured_pt;
        previous_measured_pt_ = current_measured_pt;
        moving_distance_ = 0.0;
    } else {
        double dist = hypotf ( current_measured_pt.x - previous_measured_pt_.x, current_measured_pt.y - previous_measured_pt_.y ); 
        moving_distance_ = ( moving_distance_ == 0.0 ) ?  dist : 0.95 * moving_distance_ + 0.05 * dist;

        geometry_msgs::Point tmp;
        geometry_msgs::Point previous_predicted_pt = current_predicted_pt_;
        tmp.x = 0.95 * previous_predicted_pt.x + 0.05 * previous_measured_pt_.x;
        tmp.y = 0.95 * previous_predicted_pt.y + 0.05 * previous_measured_pt_.y;
        tmp.z = 0.95 * previous_predicted_pt.z + 0.05 * previous_measured_pt_.z;
        current_predicted_pt_ = tmp;
        previous_measured_pt_ = current_measured_pt;
        moving_angle_ = atan2 ( current_predicted_pt_.y - previous_predicted_pt.y, current_predicted_pt_.x - previous_predicted_pt.x );
    }
}
