#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sobit_common_msg/StringArray.h>
#include <sobit_common_msg/BoundingBoxes.h>
#include <sobit_common_msg/ObjectPoseArray.h>
#include <multiple_sensor_person_tracking/LegDetection.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <person_following_control/FollowingPosition.h>

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
#include <multiple_sensor_person_tracking/multiple_observation_kalman_filter.hpp>
#include <multiple_sensor_person_tracking/TrackerParameterConfig.h>
#include <dynamic_reconfigure/server.h>

#define NO_EXISTS 0
#define EXISTS_LEG 1
#define EXISTS_BODY 2
#define EXISTS_LEG_AND_BODY 3

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

namespace multiple_sensor_person_tracking {
    class PersonTracker : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_following_position_;
            ros::Publisher pub_result_img_;
            ros::Publisher pub_marker_;
            ros::Publisher pub_obstacles_;
            ros::ServiceClient client_legs_detection_;

            std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub_laser_;
            std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_img_;
            std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_cloud_;
            std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

            dynamic_reconfigure::Server<multiple_sensor_person_tracking::TrackerParameterConfig>* server_;
            dynamic_reconfigure::Server<multiple_sensor_person_tracking::TrackerParameterConfig>::CallbackType f_;

            std::unique_ptr<ssd_nodelet::SingleShotMultiboxDetector> ssd_;
            std::unique_ptr<multiple_sensor_person_tracking::KalmanFilter> kf_;
            laser_geometry::LaserProjection projector_;
            cv_bridge::CvImagePtr cv_ptr_;
            pcl::KdTreeFLANN<PointT> flann_;
            pcl::ExtractIndices<PointT> extract_;
            pcl::RadiusOutlierRemoval<PointT> outrem_;
            pcl::VoxelGrid<PointT> voxel_;

            tf::TransformListener tf_listener_;
            std::string target_frame_;

            geometry_msgs::Point previous_target_;
            double previous_time_;
            bool exists_target_;
            double leg_tracking_range_;
            double body_tracking_range_;
            double target_cloud_radius_;

            visualization_msgs::Marker makeLegPoseMarker( const std::vector<geometry_msgs::Pose>& leg_poses );
            visualization_msgs::Marker makeLegAreaMarker( const std::vector<geometry_msgs::Pose>& leg_poses );
            visualization_msgs::Marker makeBodyPoseMarker( const std::vector<sobit_common_msg::ObjectPose>& body_poses );
            visualization_msgs::Marker makeTargetPoseMarker( const Eigen::Vector4f& target_pose );

            void callbackDynamicReconfigure(multiple_sensor_person_tracking::TrackerParameterConfig& config, uint32_t level);

            int findTwoObservationValue(
                const std::vector<geometry_msgs::Pose>& leg_poses,
                const std::vector<sobit_common_msg::ObjectPose>& body_poses,
                Eigen::Vector2f* leg_observed_value,
                Eigen::Vector2f* body_observed_value );

            void searchObstacles( const geometry_msgs::Point& search_pt,  const PointCloud::Ptr input_cloud, sensor_msgs::PointCloud2* obstacles );

            void callbackSenserData (
                const sensor_msgs::LaserScanConstPtr &scan_msg,
                const sensor_msgs::ImageConstPtr &img_msg,
                const sensor_msgs::PointCloud2ConstPtr &cloud_msg
            );

        public:
            virtual void onInit();
    };
}

visualization_msgs::Marker multiple_sensor_person_tracking::PersonTracker::makeLegPoseMarker( const std::vector<geometry_msgs::Pose>& leg_poses ) {
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
    leg_marker.lifetime = ros::Duration(1.0);
    for ( const auto& pose : leg_poses ) leg_marker.points.push_back( pose.position );
    return leg_marker;
}

visualization_msgs::Marker multiple_sensor_person_tracking::PersonTracker::makeLegAreaMarker( const std::vector<geometry_msgs::Pose>& leg_poses ) {
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
    leg_marker.lifetime = ros::Duration(1.0);
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

visualization_msgs::Marker multiple_sensor_person_tracking::PersonTracker::makeBodyPoseMarker( const std::vector<sobit_common_msg::ObjectPose>& body_poses ) {
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
    body_marker.lifetime = ros::Duration(1.0);
    for ( const auto& pose : body_poses ) body_marker.points.push_back( pose.pose.position );
    return body_marker;
}

visualization_msgs::Marker multiple_sensor_person_tracking::PersonTracker::makeTargetPoseMarker( const Eigen::Vector4f& target_pose ) {
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
    target_marker.lifetime = ros::Duration(1.0);
    return target_marker;
}

void multiple_sensor_person_tracking::PersonTracker::callbackDynamicReconfigure(multiple_sensor_person_tracking::TrackerParameterConfig& config, uint32_t level) {
    kf_->changeParameter( config.process_noise, config.system_noise );
    leg_tracking_range_ = config.leg_tracking_range;
    body_tracking_range_ = config.body_tracking_range;

    outrem_.setRadiusSearch( config.outlier_radius );
    outrem_.setMinNeighborsInRadius ( config.outlier_min_pts );
    outrem_.setKeepOrganized( false );
    double leaf_size = config.leaf_size;
    voxel_.setLeafSize( leaf_size, leaf_size, 0.0 );
    target_cloud_radius_ = config.target_cloud_radius;

}

int multiple_sensor_person_tracking::PersonTracker::findTwoObservationValue(
    const std::vector<geometry_msgs::Pose>& leg_poses,
    const std::vector<sobit_common_msg::ObjectPose>& body_poses,
    Eigen::Vector2f* leg_observed_value,
    Eigen::Vector2f* body_observed_value )
{
    geometry_msgs::Point search_pt, leg_pt, body_pt;
    double min_distance = ( exists_target_ ) ? leg_tracking_range_ : DBL_MAX;
    bool exists_leg_pt = false, exists_body_pt = false;
    int result;
    if ( !exists_target_ ) {
        // Determine targets to track (search for the person closest to the robot)
        search_pt.x = 0.0; search_pt.y = 0.0;
        if ( body_poses.size() == 0 ) {
            // Rotate the RGB-D sensor in the direction in which the leg_poses
        }
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
    min_distance = ( exists_target_ ) ? body_tracking_range_ : DBL_MAX;
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
    if ( !exists_leg_pt && !exists_body_pt ) result = NO_EXISTS;
    else if ( exists_leg_pt && !exists_body_pt ) result = EXISTS_LEG;
    else if ( !exists_leg_pt && exists_body_pt ) result = EXISTS_BODY;
    else result = EXISTS_LEG_AND_BODY;
    *leg_observed_value = leg_observed;
    *body_observed_value = body_observed;
    return result;
}

void multiple_sensor_person_tracking::PersonTracker::searchObstacles( const geometry_msgs::Point& search_pt,  const PointCloud::Ptr input_cloud, sensor_msgs::PointCloud2* obstacles ) {
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

void multiple_sensor_person_tracking::PersonTracker::callbackSenserData (
    const sensor_msgs::LaserScanConstPtr &scan_msg,
    const sensor_msgs::ImageConstPtr &img_msg,
    const sensor_msgs::PointCloud2ConstPtr &cloud_msg ) {
    // variable initialization
    std::string target_frame = target_frame_;
    sensor_msgs::PointCloud2Ptr cloud_scan_msg ( new sensor_msgs::PointCloud2 );
    PointCloud::Ptr cloud_scan (new PointCloud());
    PointCloud::Ptr cloud (new PointCloud());
    cv::Mat img_raw;
    sobit_common_msg::StringArrayPtr detect_object_name(new sobit_common_msg::StringArray);
    sobit_common_msg::BoundingBoxesPtr object_bbox_array(new sobit_common_msg::BoundingBoxes);
    sobit_common_msg::ObjectPoseArrayPtr body_pose_array(new sobit_common_msg::ObjectPoseArray);
    sensor_msgs::ImagePtr result_img_msg(new sensor_msgs::Image);
    visualization_msgs::MarkerArrayPtr marker_array(new visualization_msgs::MarkerArray);
    Eigen::Vector4f estimated_value( 0.0, 0.0, 0.0, 0.0 );
    person_following_control::FollowingPositionPtr following_position( new person_following_control::FollowingPosition );
    double dt = ( scan_msg->header.stamp.toSec()  - previous_time_ );	//dt - expressed in seconds
    previous_time_ = scan_msg->header.stamp.toSec();
    // Sensor data to TF conversion
    try {
        tf_listener_.waitForTransform( target_frame, scan_msg->header.frame_id, scan_msg->header.stamp, ros::Duration(5.0) );
        projector_.transformLaserScanToPointCloud( target_frame, *scan_msg, *cloud_scan_msg, tf_listener_ );
        pcl::fromROSMsg<PointT>( *cloud_scan_msg, *cloud_scan);
        pcl::fromROSMsg<PointT>( *cloud_msg, *cloud );
        tf_listener_.waitForTransform( target_frame, cloud->header.frame_id, ros::Time(0), ros::Duration(1.0));
        pcl_ros::transformPointCloud( target_frame, ros::Time(0), *cloud, cloud->header.frame_id,  *cloud, tf_listener_ );
        cloud_scan->header.frame_id = target_frame;
        cloud->header.frame_id = target_frame;
    } catch ( const tf::TransformException& ex ) {
        ROS_ERROR("%s", ex.what());
        return;
    }
    try {
        cv_ptr_ = cv_bridge::toCvCopy( img_msg, sensor_msgs::image_encodings::BGR8 );
        img_raw = cv_ptr_->image.clone();
    } catch ( const cv_bridge::Exception& ex ) {
        NODELET_ERROR("cv_bridge exception: %s", ex.what());
        return;
    }
    // Person detection by 2D-LiDAR（DR-SPAAM（Distance Robust SPatial Attention and Auto-regressive Model））
    multiple_sensor_person_tracking::LegDetection srv;
    srv.request.scan = *scan_msg;
    if (client_legs_detection_.call(srv)) {
        NODELET_INFO("DR-SPAAM Result : %zu", srv.response.leg_pose_array.poses.size());

    } else {
        NODELET_ERROR("Failed to call service /dr_spaam_ros_server\n");
        return;
    }

    // Person detection by RGB-D sensor（SSD（Single Shot Multibox Detector））
    ssd_->conpute( img_raw, cloud, img_msg->header, cloud_msg->header, detect_object_name, object_bbox_array, body_pose_array, result_img_msg);
    NODELET_INFO("SSD Result :      %zu", body_pose_array->object_poses.size());

    // Searching for observables to input to the Kalman filter
    Eigen::Vector2f leg_observed_value, body_observed_value;
    int result = findTwoObservationValue( srv.response.leg_pose_array.poses, body_pose_array->object_poses, &leg_observed_value, &body_observed_value );
    if ( result == NO_EXISTS ) {
        exists_target_ = false;
        return;
    }
    // Tracking by Kalman Filter
    if ( !exists_target_ ) {
        kf_->init( body_observed_value );
        exists_target_ = true;
    } else {
        if( result == EXISTS_LEG ) kf_->compute( dt, leg_observed_value, &estimated_value );
        else if( result == EXISTS_BODY ) kf_->compute( dt, body_observed_value, &estimated_value );
        else if ( result == EXISTS_LEG_AND_BODY ) kf_->compute( dt, leg_observed_value, body_observed_value, &estimated_value );
    }

    // following_position : pose :
    following_position->pose.position.x = estimated_value[0];
    following_position->pose.position.y = estimated_value[1];
    tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, std::atan2(estimated_value[3], estimated_value[2]));
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    following_position->pose.orientation = geometry_quat;
    // following_position : obstacles :
    outrem_.setInputCloud( cloud_scan );
    outrem_.filter ( *cloud_scan );
    voxel_.setInputCloud( cloud_scan );
    voxel_.filter ( *cloud_scan );
    searchObstacles( following_position->pose.position, cloud_scan, &following_position->obstacles );
    // following_position : header :
    following_position->header.stamp = ros::Time::now();
    pub_following_position_.publish( following_position );

    NODELET_INFO("Result :          %s", ( result == EXISTS_LEG ? "EXISTS_LEG" : ( result == EXISTS_BODY ? "EXISTS_BODY" : "EXISTS_LEG_AND_BODY")) );
    NODELET_INFO("Tracker: x = %8.3f [m],\ty = %8.3f [m]", following_position->pose.position.x, following_position->pose.position.y);

    pub_result_img_.publish( result_img_msg );
    pub_obstacles_.publish( following_position->obstacles );
    marker_array->markers.push_back( makeLegPoseMarker(srv.response.leg_pose_array.poses) );
    marker_array->markers.push_back( makeLegAreaMarker(srv.response.leg_pose_array.poses) );
    marker_array->markers.push_back( makeBodyPoseMarker(body_pose_array->object_poses) );
    marker_array->markers.push_back( makeTargetPoseMarker(estimated_value) );
    pub_marker_.publish ( marker_array );
    previous_target_ = following_position->pose.position;

    NODELET_INFO("dt     : %.4f [sec] ( %.4f [Hz] )\n", dt, 1.0/dt );
    return;
}

void multiple_sensor_person_tracking::PersonTracker::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );
    // message_filters :
    sub_laser_ .reset ( new message_filters::Subscriber<sensor_msgs::LaserScan> ( nh_, pnh_.param<std::string>( "laser_topic_name", "/scan" ), 1 ) );
    sub_img_ .reset ( new message_filters::Subscriber<sensor_msgs::Image> ( nh_, pnh_.param<std::string>( "image_topic_name", "/camera/rgb/image_raw" ), 1 ) );
    sub_cloud_ .reset ( new message_filters::Subscriber<sensor_msgs::PointCloud2> ( nh_, pnh_.param<std::string>( "cloud_topic_name", "/camera/depth/points" ), 1 ) );

    sync_ .reset ( new message_filters::Synchronizer<MySyncPolicy> ( MySyncPolicy(10), *sub_laser_, *sub_img_, *sub_cloud_ ) );
    sync_ ->registerCallback ( boost::bind( &PersonTracker::callbackSenserData, this, _1, _2, _3 ) );

    pub_following_position_ = nh_.advertise< person_following_control::FollowingPosition >( "following_position", 1 );
    pub_marker_ = nh_.advertise< visualization_msgs::MarkerArray >( "tracker_marker", 1 );
    pub_result_img_ = nh_.advertise<sensor_msgs::Image>("detect_result", 1);
    pub_obstacles_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacles", 1);
    // ros::service::waitForService("/dr_spaam_ros_server", ros::Duration(5.0));
    client_legs_detection_ = nh_.serviceClient<multiple_sensor_person_tracking::LegDetection>("/dr_spaam_ros_server");

    std::string model_configuration_path = ros::package::getPath("ssd_nodelet") + "/models/" + pnh_.param<std::string>("ssd_prototxt_name", "voc_object.prototxt");
    std::string model_binary_path = ros::package::getPath("ssd_nodelet") + "/models/" + pnh_.param<std::string>("ssd_caffemodel_name", "voc_object.caffemodel");
    std::string class_names_file_path = ros::package::getPath("ssd_nodelet") + "/models/" + pnh_.param<std::string>("ssd_class_names_file", "voc_object_names.txt");
    target_frame_ = pnh_.param<std::string>( "target_frame", "base_footprint" );

    ssd_.reset( new ssd_nodelet::SingleShotMultiboxDetector( model_configuration_path, model_binary_path, class_names_file_path ) );
    ssd_->setDNNParametr( pnh_.param<double>("ssd_in_scale_factor", 0.007843), pnh_.param<double>("ssd_confidence_threshold", 0.5) );
    ssd_->setImgShowFlag( pnh_.param<bool>("ssd_img_show_flag", true) );
    ssd_->setUseTF( pnh_.param<bool>("use_tf", true), target_frame_ );
    ssd_->specifyDetectionObject( pnh_.param<bool>("object_specified_enabled", false), pnh_.param<std::string>("specified_object_name", "None") );

    kf_.reset( new multiple_sensor_person_tracking::KalmanFilter( 0.033, 1000, 1.0 ) );

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
    f_ = boost::bind(&multiple_sensor_person_tracking::PersonTracker::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);
}

PLUGINLIB_EXPORT_CLASS( multiple_sensor_person_tracking::PersonTracker, nodelet::Nodelet );