// 2D-LiDARセンサとパンチルト回転機構上のRGB-Dセンサを組み合わせた人物追跡
#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sobit_common_msg/StringArray.h>
#include <sobit_common_msg/BoundingBoxes.h>
#include <sobit_common_msg/ObjectPoseArray.h>
#include <multiple_sensor_person_tracking/LegPoseArray.h>
#include <multiple_sensor_person_tracking/FollowingPosition.h>

#include <tf/transform_listener.h>

#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <ssd_nodelet/single_shot_multibox_detector.hpp>
#include <multiple_observation_kalman_filter/multiple_observation_kalman_filter.hpp>
#include <multiple_sensor_person_tracking/TrackerParameterConfig.h>
#include <dynamic_reconfigure/server.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<multiple_sensor_person_tracking::LegPoseArray, sobit_common_msg::ObjectPoseArray> MySyncPolicy;

namespace multiple_sensor_person_tracking {
    enum Status {
        NO_EXISTS = 0, EXISTS_LEG, EXISTS_BODY, EXISTS_LEG_AND_BODY
    };

    class SobitEduPersonTracker : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_following_position_;
            ros::Publisher pub_marker_;
            ros::Publisher pub_obstacles_;
            ros::Publisher pub_target_odom_;

            std::unique_ptr<message_filters::Subscriber<multiple_sensor_person_tracking::LegPoseArray>> sub_dr_spaam_;
            std::unique_ptr<message_filters::Subscriber<sobit_common_msg::ObjectPoseArray>> sub_ssd_;
            std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

            dynamic_reconfigure::Server<multiple_sensor_person_tracking::TrackerParameterConfig>* server_;
            dynamic_reconfigure::Server<multiple_sensor_person_tracking::TrackerParameterConfig>::CallbackType f_;

            std::unique_ptr<multiple_observation_kalman_filter::KalmanFilter> kf_;
            laser_geometry::LaserProjection projector_;
            cv_bridge::CvImagePtr cv_ptr_;
            pcl::KdTreeFLANN<PointT> flann_;
            pcl::ExtractIndices<PointT> extract_;
            pcl::RadiusOutlierRemoval<PointT> outrem_;
            pcl::VoxelGrid<PointT> voxel_;
            PointCloud::Ptr cloud_scan_;
            visualization_msgs::MarkerArrayPtr marker_array_;
            multiple_sensor_person_tracking::FollowingPositionPtr following_position_;

            tf::TransformListener tf_listener_;
            std::string target_frame_;

            geometry_msgs::Point previous_target_;
            double previous_time_;
            bool exists_target_;
            double leg_tracking_range_;
            double body_tracking_range_;
            double target_range_;
            double target_cloud_radius_;
            bool display_marker_;
            double no_exists_time_;
            double target_change_tolerance_;
            double attention_leg_time_;
            unsigned int attention_leg_idx_;


            visualization_msgs::Marker makeLegPoseMarker( const std::vector<geometry_msgs::Pose>& leg_poses );
            visualization_msgs::Marker makeLegAreaMarker( const std::vector<geometry_msgs::Pose>& leg_poses );
            visualization_msgs::Marker makeBodyPoseMarker( const std::vector<sobit_common_msg::ObjectPose>& body_poses );
            visualization_msgs::Marker makeTargetPoseMarker( const Eigen::Vector4f& target_pose );

            void callbackDynamicReconfigure( multiple_sensor_person_tracking::TrackerParameterConfig& config, uint32_t level );

            int findTwoObservationValue(
                const std::vector<geometry_msgs::Pose>& leg_poses,
                const std::vector<sobit_common_msg::ObjectPose>& body_poses,
                Eigen::Vector2f* leg_observed_value,
                Eigen::Vector2f* body_observed_value );

            void searchObstacles(
                const geometry_msgs::Point& search_pt,
                const PointCloud::Ptr input_cloud,
                sensor_msgs::PointCloud2* obstacles );

            geometry_msgs::PointStamped transformPoint(
                const std::string& org_frame,
                const std::string& target_frame,
                const geometry_msgs::Point& point );

            void callbackPoseArray (
                const multiple_sensor_person_tracking::LegPoseArrayConstPtr &dr_spaam_msg,
                const sobit_common_msg::ObjectPoseArrayConstPtr &ssd_msg );

        public:
            virtual void onInit();
    };
}

visualization_msgs::Marker multiple_sensor_person_tracking::SobitEduPersonTracker::makeLegPoseMarker( const std::vector<geometry_msgs::Pose>& leg_poses ) {
    visualization_msgs::Marker leg_marker;
    leg_marker.header.frame_id = target_frame_;
    leg_marker.header.stamp = ros::Time::now();
    leg_marker.ns = "leg_marker";
    leg_marker.id =  1;
    leg_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    leg_marker.action = visualization_msgs::Marker::ADD;
    leg_marker.scale.x = 0.15;leg_marker.scale.y = 0.15;leg_marker.scale.z = 0.15;
    leg_marker.color.r = 1.0; leg_marker.color.g = 0.0; leg_marker.color.b = 0.0; leg_marker.color.a = 1.0;
    leg_marker.pose.orientation.w = 1.0;
    leg_marker.lifetime = ros::Duration(0.3);
    for ( const auto& pose : leg_poses ) leg_marker.points.push_back( pose.position );
    return leg_marker;
}

visualization_msgs::Marker multiple_sensor_person_tracking::SobitEduPersonTracker::makeLegAreaMarker( const std::vector<geometry_msgs::Pose>& leg_poses ) {
    visualization_msgs::Marker leg_marker;
    std::vector<double> offset_x, offset_y;
    double tolerance = 2.0*M_PI / 20.0;
    double radius = 0.4;
    for ( unsigned int i = 0; i < 20; i++ ) {
        offset_x.push_back( radius * std::cos(i * tolerance) );
        offset_y.push_back( radius * std::sin(i * tolerance) );
    }
    leg_marker.header.frame_id = target_frame_;
    leg_marker.header.stamp = ros::Time::now();
    leg_marker.ns = "leg_area_marker";
    leg_marker.id =  1;
    leg_marker.type = visualization_msgs::Marker::LINE_LIST;
    leg_marker.action = visualization_msgs::Marker::ADD;
    leg_marker.scale.x = 0.03;
    leg_marker.color.r = 1.0; leg_marker.color.g = 0.0; leg_marker.color.b = 0.0; leg_marker.color.a = 1.0;
    leg_marker.pose.orientation.w = 1.0;
    leg_marker.lifetime = ros::Duration(0.3);
    for ( const auto& pose : leg_poses ) {
        for ( unsigned int i = 0; i < 20; i++ ) {
            geometry_msgs::Point p0, p1;
            p0.x = pose.position.x + offset_x[i];
            p0.y = pose.position.y + offset_y[i];
            p0.z = -0.3;
            p1.x = ( i+1 < 20 ) ? pose.position.x + offset_x[i+1] : pose.position.x + offset_x[0];
            p1.y = ( i+1 < 20 ) ? pose.position.y + offset_y[i+1] : pose.position.y + offset_y[0];;
            p1.z = -0.3;
            leg_marker.points.push_back( p0 );
            leg_marker.points.push_back( p1 );
        }
    }
    return leg_marker;
}

visualization_msgs::Marker multiple_sensor_person_tracking::SobitEduPersonTracker::makeBodyPoseMarker( const std::vector<sobit_common_msg::ObjectPose>& body_poses ) {
    visualization_msgs::Marker body_marker;
    body_marker.header.frame_id = target_frame_;
    body_marker.header.stamp = ros::Time::now();
    body_marker.ns = "body_marker";
    body_marker.id =  1;
    body_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    body_marker.action = visualization_msgs::Marker::ADD;
    body_marker.scale.x = 0.15;body_marker.scale.y = 0.15;body_marker.scale.z = 0.15;
    body_marker.color.r = 0.0; body_marker.color.g = 1.0; body_marker.color.b = 0.0; body_marker.color.a = 1.0;
    body_marker.pose.orientation.w = 1.0;
    body_marker.lifetime = ros::Duration(0.3);
    for ( const auto& pose : body_poses ) body_marker.points.push_back( pose.pose.position );
    return body_marker;
}

visualization_msgs::Marker multiple_sensor_person_tracking::SobitEduPersonTracker::makeTargetPoseMarker( const Eigen::Vector4f& target_pose ) {
    visualization_msgs::Marker target_marker;
    target_marker.header.frame_id = target_frame_;
    target_marker.header.stamp = ros::Time::now();
    target_marker.ns = "target_marker";
    target_marker.id =  1;
    target_marker.type = visualization_msgs::Marker::ARROW;
    target_marker.action = visualization_msgs::Marker::ADD;
    target_marker.scale.x = 0.5;target_marker.scale.y = 0.15;target_marker.scale.z = 0.15;
    target_marker.color.r = 1.0; target_marker.color.g = 1.0; target_marker.color.b = 0.0; target_marker.color.a = 1.0;
    target_marker.pose.position.x = target_pose[0];
    target_marker.pose.position.y = target_pose[1];
    target_marker.pose.position.z = 0.5;
    tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, std::atan2(target_pose[3], target_pose[2]));
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    target_marker.pose.orientation = geometry_quat;
    target_marker.lifetime = ros::Duration(0.3);
    return target_marker;
}

void multiple_sensor_person_tracking::SobitEduPersonTracker::callbackDynamicReconfigure( multiple_sensor_person_tracking::TrackerParameterConfig& config, uint32_t level) {
    kf_->changeParameter( config.process_noise, config.system_noise );
    leg_tracking_range_ = config.leg_tracking_range;
    body_tracking_range_ = config.body_tracking_range;
    target_range_ = config.target_range;
    display_marker_ = config.display_marker;
    target_change_tolerance_ = config.target_change_tolerance;

    outrem_.setRadiusSearch( config.outlier_radius );
    outrem_.setMinNeighborsInRadius ( config.outlier_min_pts );
    outrem_.setKeepOrganized( false );
    double leaf_size = config.leaf_size;
    voxel_.setLeafSize( leaf_size, leaf_size, 0.0 );
    target_cloud_radius_ = config.target_cloud_radius;

}

int multiple_sensor_person_tracking::SobitEduPersonTracker::findTwoObservationValue(
    const std::vector<geometry_msgs::Pose>& leg_poses,
    const std::vector<sobit_common_msg::ObjectPose>& body_poses,
    Eigen::Vector2f* leg_observed_value,
    Eigen::Vector2f* body_observed_value )
{
    geometry_msgs::Point search_pt, leg_pt, body_pt;
    double min_distance = ( exists_target_ ) ? leg_tracking_range_ : target_range_;
    bool exists_leg_pt = false, exists_body_pt = false;
    int result;
    if ( !exists_target_ ) {
        // Determine targets to track (search for the person closest to the robot)
        search_pt.x = 0.0; search_pt.y = 0.0;
    } else {
        // Searching for the observed value of the tracking target (search for the person closest to the previous tracking position)
        search_pt = previous_target_;
    }
    for ( const auto& pose : leg_poses ) {
        double distance = std::hypotf( pose.position.x - search_pt.x, pose.position.y - search_pt.y );
        if ( min_distance > distance ) {
            min_distance = distance;
            leg_pt = pose.position;
            exists_leg_pt = true;
        }
    }
    min_distance = ( exists_target_ ) ? body_tracking_range_ : target_range_;
    for ( const auto& pose : body_poses ) {
        double distance = std::hypotf( pose.pose.position.x - search_pt.x, pose.pose.position.y - search_pt.y );
        if ( min_distance > distance ) {
            min_distance = distance;
            body_pt = pose.pose.position;
            exists_body_pt = true;
        }
    }
    Eigen::Vector2f leg_observed( leg_pt.x, leg_pt.y );
    Eigen::Vector2f body_observed( body_pt.x, body_pt.y );
    if ( !exists_leg_pt && !exists_body_pt ) result = Status::NO_EXISTS;
    else if ( exists_leg_pt && !exists_body_pt ) result = Status::EXISTS_LEG;
    else if ( !exists_leg_pt && exists_body_pt ) result = Status::EXISTS_BODY;
    else result = Status::EXISTS_LEG_AND_BODY;
    *leg_observed_value = leg_observed;
    *body_observed_value = body_observed;
    return result;
}

void multiple_sensor_person_tracking::SobitEduPersonTracker::searchObstacles( const geometry_msgs::Point& search_pt,  const PointCloud::Ptr input_cloud, sensor_msgs::PointCloud2* obstacles ) {
    PointCloud::Ptr cloud_obstacles ( new PointCloud() );
    pcl::PointIndices::Ptr target_indices ( new pcl::PointIndices );
    PointT p_q;
    p_q.x = search_pt.x;
    p_q.y = search_pt.y;
    p_q.z = input_cloud ->points [0].z;
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    flann_.setInputCloud ( input_cloud );
    int num_match = flann_.radiusSearch (p_q, target_cloud_radius_, k_indices, k_sqr_distances, 0);

    if ( num_match == 0 ) {
        *cloud_obstacles = *input_cloud;
    } else {
        for ( const auto& index : k_indices ) { target_indices->indices.push_back ( index ); }
        extract_.setInputCloud( input_cloud );
        extract_.setIndices( target_indices );
        extract_.setNegative( true );
        extract_.filter( *cloud_obstacles );
    }
    pcl::toROSMsg(*cloud_obstacles, *obstacles );
    return;
}

geometry_msgs::PointStamped multiple_sensor_person_tracking::SobitEduPersonTracker::transformPoint (
    const std::string& org_frame,
    const std::string& target_frame,
    const geometry_msgs::Point& point)
{
    geometry_msgs::PointStamped pt_transformed;
    geometry_msgs::PointStamped pt;
    pt.header.frame_id = org_frame;
    pt.header.stamp = ros::Time(0);
    pt.point = point;
    try {
        tf_listener_.transformPoint( target_frame, pt, pt_transformed );
    } catch ( const tf::TransformException& ex ) {
        NODELET_ERROR( "%s",ex.what( ) );
    }
    return pt_transformed;
}

void multiple_sensor_person_tracking::SobitEduPersonTracker::callbackPoseArray ( const multiple_sensor_person_tracking::LegPoseArrayConstPtr &dr_spaam_msg, const sobit_common_msg::ObjectPoseArrayConstPtr &ssd_msg ) {
    std::cout << "\n====================================" << std::endl;
    // variable initialization
    std::string target_frame = target_frame_;
    sensor_msgs::PointCloud2 cloud_scan_msg;
    Eigen::Vector4f estimated_value( 0.0, 0.0, 0.0, 0.0 );

    double dt = ( dr_spaam_msg->header.stamp.toSec()  - previous_time_ );	//dt - expressed in seconds
    previous_time_ = dr_spaam_msg->header.stamp.toSec();

    // Sensor data to TF conversion
    try {
        tf_listener_.waitForTransform( target_frame, dr_spaam_msg->header.frame_id, dr_spaam_msg->header.stamp, ros::Duration(5.0) );
        projector_.transformLaserScanToPointCloud( target_frame, dr_spaam_msg->scan, cloud_scan_msg, tf_listener_ );
        pcl::fromROSMsg<PointT>( cloud_scan_msg, *cloud_scan_);
        cloud_scan_->header.frame_id = target_frame;
    } catch ( const tf::TransformException& ex ) {
        ROS_ERROR("%s", ex.what());
        following_position_->pose.position.x = 0.0;
        following_position_->pose.position.y = 0.0;
        following_position_->rotation_position = following_position_->pose.position;
        following_position_->status = Status::NO_EXISTS;
        pub_following_position_.publish( following_position_ );
        return;
    }

    if ( !exists_target_ && ssd_msg->object_poses.size() == 0) {
        if ( dr_spaam_msg->poses.size() == 0 ) {
            NODELET_ERROR("Result :          NO_EXISTS (DR-SPAAM)" );
            exists_target_ = false;
            following_position_->pose.position.x = 0.0;
            following_position_->pose.position.y = 0.0;
            following_position_->rotation_position = following_position_->pose.position;
            following_position_->status = Status::NO_EXISTS;
            pub_following_position_.publish( following_position_ );
            return;
        }
        if( attention_leg_time_ == -1.0 ) attention_leg_time_ = ros::Time::now().toSec();
        exists_target_ = false;
        std::vector<geometry_msgs::Pose> leg_poses = dr_spaam_msg->poses;
        // Rotate the RGB-D sensor in the direction in which the leg_poses
        // 近い順にソート
        std::sort(
            leg_poses.begin(),
            leg_poses.end(),
            []( const auto & a, const auto & b)
            { return std::hypotf(a.position.x, a.position.y) > std::hypotf(b.position.x, b.position.y); } );
        // attention_leg_idx_ の位置を設定 ※attention_leg_idx_が配列より大きい場合は修正
        // 2秒でattention_leg_idx_を変える
        if ( ros::Time::now().toSec() - attention_leg_time_ >= 2.0 ) {
            attention_leg_idx_ = ( attention_leg_idx_ <= leg_poses.size() ) ? attention_leg_idx_ + 1 : 0;
            attention_leg_time_ = ros::Time::now().toSec();
        } else attention_leg_idx_ = ( attention_leg_idx_ <= leg_poses.size() ) ? attention_leg_idx_ : leg_poses.size()-1;
        following_position_->rotation_position.x = leg_poses[attention_leg_idx_].position.x;
        following_position_->rotation_position.y = leg_poses[attention_leg_idx_].position.y;
        following_position_->pose.position.x = 0.0;
        following_position_->pose.position.y = 0.0;
        following_position_->status = Status::NO_EXISTS;
        pub_following_position_.publish( following_position_ );
        NODELET_ERROR("Result :          NO_EXISTS (SSD) attention_leg_idx = %d",attention_leg_idx_ );
        return;
    } else {
        attention_leg_time_ = -1.0;
        attention_leg_idx_ = 0;
    }
    // Searching for observables to input to the Kalman filter
    Eigen::Vector2f leg_observed_value, body_observed_value;
    int result = findTwoObservationValue( dr_spaam_msg->poses, ssd_msg->object_poses, &leg_observed_value, &body_observed_value );
    if ( result == Status::NO_EXISTS ) {
        if ( no_exists_time_ == -1.0 ) no_exists_time_ = ros::Time::now().toSec();
        else if ( ros::Time::now().toSec() - no_exists_time_ >= target_change_tolerance_ ){
            NODELET_ERROR("Result :          NO_EXISTS (findTwoObservationValue)" );
            exists_target_ = false;
            following_position_->pose.position.x = 0.0;
            following_position_->pose.position.y = 0.0;
            following_position_->status = Status::NO_EXISTS;
            pub_following_position_.publish( following_position_ );
            return;
        }
    } else no_exists_time_ = -1.0;

    // Tracking by Kalman Filter
    if ( ( !exists_target_ && result == Status::EXISTS_LEG_AND_BODY) || (!exists_target_ && result == Status::EXISTS_BODY) ) {
        kf_->init( body_observed_value );
        estimated_value[0] = body_observed_value[0];
        estimated_value[1] = body_observed_value[1];
        exists_target_ = true;
    } else if ( !exists_target_ && result != EXISTS_LEG_AND_BODY ) {
        NODELET_ERROR("Result :          NO_EXISTS" );
        exists_target_ = false;
        following_position_->pose.position.x = 0.0;
        following_position_->pose.position.y = 0.0;
        following_position_->status = Status::NO_EXISTS;
        pub_following_position_.publish( following_position_ );
        return;
    }else {
        if( result == Status::EXISTS_LEG ) {
            kf_->compute( dt, leg_observed_value, &estimated_value );
            following_position_->rotation_position.x = estimated_value[0];
            following_position_->rotation_position.y = estimated_value[1];
        } else if( result == Status::EXISTS_BODY ) {
            kf_->compute( dt, body_observed_value, &estimated_value );
            following_position_->rotation_position.x = body_observed_value[0];
            following_position_->rotation_position.y = body_observed_value[1];
        } else if ( result == Status::EXISTS_LEG_AND_BODY ) {
            kf_->compute( dt, leg_observed_value, body_observed_value, &estimated_value );
            following_position_->rotation_position.x = body_observed_value[0];
            following_position_->rotation_position.y = body_observed_value[1];
        } else {
            kf_->compute( dt, &estimated_value );
            following_position_->rotation_position.x = estimated_value[0];
            following_position_->rotation_position.y = estimated_value[1];
        }
    }

    // following_position_ : pose :
    following_position_->pose.position.x = estimated_value[0];
    following_position_->pose.position.y = estimated_value[1];
    tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, std::atan2(estimated_value[3], estimated_value[2]));
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    following_position_->pose.orientation = geometry_quat;
    following_position_->velocity = std::hypotf(estimated_value[2], estimated_value[3]);
    following_position_->status = result;

    // following_position_ : obstacles :
    outrem_.setInputCloud( cloud_scan_ );
    outrem_.filter ( *cloud_scan_ );
    voxel_.setInputCloud( cloud_scan_ );
    voxel_.filter ( *cloud_scan_ );
    searchObstacles( following_position_->pose.position, cloud_scan_, &following_position_->obstacles );

    // following_position_ : header :
    following_position_->header.stamp = ros::Time::now();
    pub_following_position_.publish( following_position_ );
    pub_target_odom_.publish( transformPoint( target_frame_, "odom", following_position_->pose.position ) );
    if ( display_marker_ ) {
        pub_obstacles_.publish( following_position_->obstacles );
        marker_array_->markers.push_back( makeLegPoseMarker(dr_spaam_msg->poses) );
        marker_array_->markers.push_back( makeLegAreaMarker(dr_spaam_msg->poses) );
        marker_array_->markers.push_back( makeBodyPoseMarker(ssd_msg->object_poses) );
        marker_array_->markers.push_back( makeTargetPoseMarker(estimated_value) );
        pub_marker_.publish ( marker_array_ );
    }
    previous_target_ = following_position_->pose.position;

    NODELET_INFO("\033[1mResult\033[m = %s",
        ( following_position_->status == Status::EXISTS_LEG ? "\033[1;36m EXISTS_LEG \033[m" :
        ( following_position_->status == Status::EXISTS_BODY ? "\033[1;33m EXISTS_BODY \033[m" :
        ( following_position_->status == Status::EXISTS_LEG_AND_BODY ? "\033[1;32m EXISTS_LEG_AND_BODY \033[m" : "\033[1;31m NO_EXISTS \033[m")) ));
    NODELET_INFO("\033[1mTarget\033[m = %5.3f [m]\t%5.3f [m]", following_position_->pose.position.x, following_position_->pose.position.y );

    return;
}

void multiple_sensor_person_tracking::SobitEduPersonTracker::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();
    cloud_scan_.reset(new PointCloud());
    marker_array_.reset(new visualization_msgs::MarkerArray);
    following_position_.reset( new multiple_sensor_person_tracking::FollowingPosition );

    target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
    // message_filters :
    sub_dr_spaam_ .reset ( new message_filters::Subscriber<multiple_sensor_person_tracking::LegPoseArray> ( nh_, pnh_.param<std::string>( "dr_spaam_topic_name", "/dr_spaam_detections" ), 1 ) );
    sub_ssd_ .reset ( new message_filters::Subscriber<sobit_common_msg::ObjectPoseArray> ( nh_, pnh_.param<std::string>( "ssd_topic_name", "/ssd_object_detect/object_pose" ), 1 ) );

    sync_ .reset ( new message_filters::Synchronizer<MySyncPolicy> ( MySyncPolicy(10), *sub_dr_spaam_, *sub_ssd_ ) );
    sync_ ->registerCallback ( boost::bind( &SobitEduPersonTracker::callbackPoseArray, this, _1, _2 ) );

    pub_following_position_ = nh_.advertise< multiple_sensor_person_tracking::FollowingPosition >( "following_position", 1 );
    pub_marker_ = nh_.advertise< visualization_msgs::MarkerArray >( "tracker_marker", 1 );
    pub_obstacles_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacles", 1);
    pub_target_odom_ = nh_.advertise<geometry_msgs::PointStamped>("target_postion_odom", 1);

    target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );

    kf_.reset( new multiple_observation_kalman_filter::KalmanFilter( 0.033, 1000, 1.0 ) );

    previous_time_ = 0.0;
    exists_target_ = false;
    leg_tracking_range_ = pnh_.param<double>("leg_tracking_range", 0.5);
    body_tracking_range_ = pnh_.param<double>("body_tracking_range", 0.5);

    outrem_.setRadiusSearch( pnh_.param<double>("outlier_radius", 0.1) );
    outrem_.setMinNeighborsInRadius ( pnh_.param<int>("outlier_min_pts", 2) );
    outrem_.setKeepOrganized( false );
    double leaf_size = pnh_.param<double>("leaf_size", 0.1 );
    voxel_.setLeafSize( leaf_size, leaf_size, 0.0 );
    target_cloud_radius_ = pnh_.param<double>("target_cloud_radius", 0.4 );

    server_ = new dynamic_reconfigure::Server<multiple_sensor_person_tracking::TrackerParameterConfig>(pnh_);
    f_ = boost::bind(&multiple_sensor_person_tracking::SobitEduPersonTracker::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);
    no_exists_time_ = -1.0;
    attention_leg_time_ = -1.0;
    attention_leg_idx_ = 0;
}

PLUGINLIB_EXPORT_CLASS( multiple_sensor_person_tracking::SobitEduPersonTracker, nodelet::Nodelet );