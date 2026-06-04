#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <cv_bridge/cv_bridge.hpp>

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"

#include "multiple_sensor_person_tracking/msg/following_position.hpp"

#include "multiple_observation_kalman_filter/multiple_observation_kalman_filter.hpp"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace multiple_sensor_person_tracking {
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    enum Status {
        NO_EXISTS = 0, EXISTS_LEG, EXISTS_BODY, EXISTS_LEG_AND_BODY
    };
    enum class DetectionMode {
        LEG,
        BODY,
        BODY_LEG
    };

    class PersonTracker : public rclcpp_lifecycle::LifecycleNode {
        private:
            rclcpp_lifecycle::LifecyclePublisher<multiple_sensor_person_tracking::msg::FollowingPosition>::SharedPtr pub_following_position_;
            rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacles_;
            rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_target_odom_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_nontravelable_region_;
            rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_dr_spaam_;
            rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr sub_image_;
            PointCloud::Ptr cloud_nontravelable_region_;

            std::unique_ptr<multiple_observation_kalman_filter::KalmanFilter> kf_;
            laser_geometry::LaserProjection projector_;
            cv_bridge::CvImagePtr cv_ptr_;
            pcl::KdTreeFLANN<PointT> flann_;
            pcl::ExtractIndices<PointT> extract_;
            pcl::RadiusOutlierRemoval<PointT> outrem_;
            pcl::VoxelGrid<PointT> voxel_;
            PointCloud::Ptr cloud_scan_;
            visualization_msgs::msg::MarkerArray::SharedPtr marker_array_;
            multiple_sensor_person_tracking::msg::FollowingPosition::SharedPtr following_position_;
            sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg_;
            geometry_msgs::msg::PoseArray::ConstSharedPtr dr_spaam_msg_;

            tf2_ros::Buffer tfBuffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_sub_;
            std::string target_frame_;
            std::string odom_frame_name_;
            std::string scan_frame_name_;

            geometry_msgs::msg::Point previous_target_;
            rclcpp::Time previous_time_;
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
            bool merge_nontravelable_region_;
            DetectionMode detection_mode_;
            bool active_;

            visualization_msgs::msg::Marker makeLegPoseMarker( const std::vector<geometry_msgs::msg::Pose>& leg_poses );
            visualization_msgs::msg::Marker makeLegAreaMarker( const std::vector<geometry_msgs::msg::Pose>& leg_poses );
            visualization_msgs::msg::Marker makeBodyPoseMarker( const std::vector<vision_msgs::msg::Detection3D>& body_poses );
            visualization_msgs::msg::Marker makeTargetPoseMarker( const Eigen::Vector4f& target_pose );

            int findTwoObservationValue(
                const std::vector<geometry_msgs::msg::Pose>& leg_poses,
                const std::vector<vision_msgs::msg::Detection3D>& body_poses,
                Eigen::Vector2f* leg_observed_value,
                Eigen::Vector2f* body_observed_value );

            bool searchObstacles(
                const geometry_msgs::msg::Point& search_pt,
                const PointCloud::Ptr input_cloud,
                sensor_msgs::msg::PointCloud2* obstacles );

            geometry_msgs::msg::PointStamped transformPoint(
                const std::string& org_frame,
                const std::string& target_frame,
                const geometry_msgs::msg::Point& point );

            void scan_callback (
                const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg );

            void nontravelableRegionCallback(
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& nontravelable_region_msg );

            void dr_spaam_callback (
                const geometry_msgs::msg::PoseArray::ConstSharedPtr &dr_spaam_msg
            );

            void callbackPoseArray (
                const vision_msgs::msg::Detection3DArray::ConstSharedPtr &body_msg );
        public:
            explicit PersonTracker(const rclcpp::NodeOptions & options)
            : rclcpp_lifecycle::LifecycleNode("person_tracker", options),
            tfBuffer_(this->get_clock()),
            active_(false)
            {}

            CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
            CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
            CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
            CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
            CallbackReturn on_error(const rclcpp_lifecycle::State & state);
            void resetInterfaces();
    };
}

visualization_msgs::msg::Marker multiple_sensor_person_tracking::PersonTracker::makeLegPoseMarker( const std::vector<geometry_msgs::msg::Pose>& leg_poses ) {
    visualization_msgs::msg::Marker leg_marker;
    leg_marker.header.frame_id = target_frame_;
    leg_marker.header.stamp = this->get_clock()->now();
    leg_marker.ns = "leg_marker";
    leg_marker.id =  1;
    leg_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    leg_marker.action = visualization_msgs::msg::Marker::ADD;
    leg_marker.scale.x = 0.15;leg_marker.scale.y = 0.15;leg_marker.scale.z = 0.15;
    leg_marker.color.r = 1.0; leg_marker.color.g = 0.0; leg_marker.color.b = 0.0; leg_marker.color.a = 1.0;
    leg_marker.pose.orientation.w = 1.0;
    leg_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    for ( const auto& pose : leg_poses ) leg_marker.points.push_back( pose.position );
    return leg_marker;
}

visualization_msgs::msg::Marker multiple_sensor_person_tracking::PersonTracker::makeLegAreaMarker( const std::vector<geometry_msgs::msg::Pose>& leg_poses ) {
    visualization_msgs::msg::Marker leg_marker;
    std::vector<double> offset_x, offset_y;
    double tolerance = 2.0*M_PI / 20.0;
    double radius = 0.4;
    for ( unsigned int i = 0; i < 20; i++ ) {
        offset_x.push_back( radius * std::cos(i * tolerance) );
        offset_y.push_back( radius * std::sin(i * tolerance) );
    }
    leg_marker.header.frame_id = target_frame_;
    leg_marker.header.stamp = this->get_clock()->now();
    leg_marker.ns = "leg_area_marker";
    leg_marker.id =  1;
    leg_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    leg_marker.action = visualization_msgs::msg::Marker::ADD;
    leg_marker.scale.x = 0.03;
    leg_marker.color.r = 1.0; leg_marker.color.g = 0.0; leg_marker.color.b = 0.0; leg_marker.color.a = 1.0;
    leg_marker.pose.orientation.w = 1.0;
    leg_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    for ( const auto& pose : leg_poses ) {
        for ( unsigned int i = 0; i < 20; i++ ) {
            geometry_msgs::msg::Point p0, p1;
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

visualization_msgs::msg::Marker multiple_sensor_person_tracking::PersonTracker::makeBodyPoseMarker( const std::vector<vision_msgs::msg::Detection3D>& body_poses ) {
    visualization_msgs::msg::Marker body_marker;
    body_marker.header.frame_id = target_frame_;
    body_marker.header.stamp = this->get_clock()->now();
    body_marker.ns = "body_marker";
    body_marker.id =  1;
    body_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    body_marker.action = visualization_msgs::msg::Marker::ADD;
    body_marker.scale.x = 0.15;body_marker.scale.y = 0.15;body_marker.scale.z = 0.15;
    body_marker.color.r = 0.0; body_marker.color.g = 1.0; body_marker.color.b = 0.0; body_marker.color.a = 1.0;
    body_marker.pose.orientation.w = 1.0;
    body_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    for ( const auto& detection : body_poses ) body_marker.points.push_back(detection.bbox.center.position);
    return body_marker;
}

visualization_msgs::msg::Marker multiple_sensor_person_tracking::PersonTracker::makeTargetPoseMarker( const Eigen::Vector4f& target_pose ) {
    visualization_msgs::msg::Marker target_marker;
    target_marker.header.frame_id = target_frame_;
    target_marker.header.stamp = this->get_clock()->now();
    target_marker.ns = "target_marker";
    target_marker.id =  1;
    target_marker.type = visualization_msgs::msg::Marker::ARROW;
    target_marker.action = visualization_msgs::msg::Marker::ADD;
    target_marker.scale.x = 0.5;target_marker.scale.y = 0.15;target_marker.scale.z = 0.15;
    target_marker.color.r = 1.0; target_marker.color.g = 1.0; target_marker.color.b = 0.0; target_marker.color.a = 1.0;
    target_marker.pose.position.x = target_pose[0];
    target_marker.pose.position.y = target_pose[1];
    target_marker.pose.position.z = 0.5;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, std::atan2(target_pose[3], target_pose[2]));
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(quat_tf, quat_msg);
    target_marker.pose.orientation = quat_msg;
    target_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    return target_marker;
}

int multiple_sensor_person_tracking::PersonTracker::findTwoObservationValue(
    const std::vector<geometry_msgs::msg::Pose>& leg_poses,
    const std::vector<vision_msgs::msg::Detection3D>& body_poses,
    Eigen::Vector2f* leg_observed_value,
    Eigen::Vector2f* body_observed_value )
{

    geometry_msgs::msg::Point search_pt, leg_pt, body_pt;
    
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
    for ( const auto& detection : body_poses ) {
        
        if (detection.results.empty()) continue;
        
        std::string class_id = detection.results[0].hypothesis.class_id;
        // In the MS COCO dataset, people are output as "0" or "person"
        if (class_id != "0" && class_id != "person") {
            continue; // If it is not a person, it will be ignored and the distance calculation below will not be performed.
        }

        double distance = std::hypotf( detection.bbox.center.position.x - search_pt.x, detection.bbox.center.position.y - search_pt.y );

        if ( min_distance > distance ) {
            min_distance = distance;
            body_pt = detection.bbox.center.position;
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

bool multiple_sensor_person_tracking::PersonTracker::searchObstacles( const geometry_msgs::msg::Point& search_pt,  const PointCloud::Ptr input_cloud, sensor_msgs::msg::PointCloud2* obstacles ) {
    
    // Merge input_cloud with the non-travelable region cloud
    bool can_pub_obstacles = false;
    PointCloud::Ptr merged_cloud(new PointCloud(*input_cloud));

    if (merge_nontravelable_region_) {
        if (cloud_nontravelable_region_ && !cloud_nontravelable_region_->empty())
        {
            RCLCPP_INFO( this->get_logger(), "[searchObstacles] Merging input cloud with non-travelable region cloud");
            // Merge the two clouds:
            *merged_cloud += *cloud_nontravelable_region_;
            can_pub_obstacles = true;
        } else {
            RCLCPP_INFO( this->get_logger(), "[searchObstacles] No non-travelable region cloud available. Using input cloud only.");
            return can_pub_obstacles; // comment this line if you want to use only input_cloud
        }
    } else {
        can_pub_obstacles = true;
        RCLCPP_INFO( this->get_logger(), "[searchObstacles] Merge disabled. Using input cloud only.");
    }

    // Radius search: remove points near 'search_pt'
    PointCloud::Ptr cloud_obstacles ( new PointCloud() );
    pcl::PointIndices::Ptr target_indices ( new pcl::PointIndices );
    PointT p_q;
    p_q.x = search_pt.x;
    p_q.y = search_pt.y;
    p_q.z = merged_cloud->points[0].z;
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;

    if (!std::isfinite(p_q.x) || !std::isfinite(p_q.y) || !std::isfinite(p_q.z)) {
        RCLCPP_ERROR(this->get_logger(),
            "[searchObstacles] Invalid query point (NaN/Inf), skipping");
        return false;
    }

    flann_.setInputCloud ( merged_cloud );
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
    pcl::toROSMsg( *cloud_obstacles, *obstacles );
    obstacles->header.frame_id = merged_cloud->header.frame_id;
    obstacles->header.stamp = this->get_clock()->now();

    return can_pub_obstacles;
}

geometry_msgs::msg::PointStamped multiple_sensor_person_tracking::PersonTracker::transformPoint (
    const std::string& org_frame,
    const std::string& target_frame,
    const geometry_msgs::msg::Point& point)
{
    geometry_msgs::msg::PointStamped pt_transformed;
    geometry_msgs::msg::PointStamped pt;
    pt.header.frame_id = org_frame;
    // Use latest available TF to avoid tiny "future extrapolation" races
    // between incoming sensor timestamps and tf publication timing.
    pt.header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    pt.point = point;
    try{
        tfBuffer_.transform(pt, pt_transformed, target_frame);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
    return pt_transformed;
}

void multiple_sensor_person_tracking::PersonTracker::scan_callback (const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg)
{
    if (!active_) {
        return;
    }
    scan_msg_ = scan_msg;
}

void multiple_sensor_person_tracking::PersonTracker::nontravelableRegionCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& nontravelable_region_msg)
{
    if (!active_) {
        return;
    }
    if (nontravelable_region_msg->header.frame_id.empty()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Skipping non-travelable region cloud with empty frame_id.");
        return;
    }

    PointCloud temp_cloud;
    try {
        pcl::fromROSMsg(*nontravelable_region_msg, temp_cloud);
    } catch (std::runtime_error &ex) {
        RCLCPP_ERROR(this->get_logger(), "nontravelableRegionCallback() conversion failed: %s", ex.what());
        return;
    }

    // Transform to the same frame as /obstacles
    sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tfBuffer_.lookupTransform(
            target_frame_,
            nontravelable_region_msg->header.frame_id,
            tf2::TimePointZero);

        tf2::doTransform(*nontravelable_region_msg, transformed_cloud_msg, transform);

        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::fromROSMsg(transformed_cloud_msg, transformed_cloud);

        *cloud_nontravelable_region_ = transformed_cloud;
        cloud_nontravelable_region_->header.frame_id = target_frame_; 
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform non-travelable region cloud: %s", ex.what());
        return;
    }
}

void multiple_sensor_person_tracking::PersonTracker::dr_spaam_callback(const geometry_msgs::msg::PoseArray::ConstSharedPtr &dr_spaam_msg) 
{
    if (!active_) {
        return;
    }
    dr_spaam_msg_ = dr_spaam_msg;
    if (detection_mode_ == DetectionMode::LEG) {
        auto empty_body_msg = std::make_shared<vision_msgs::msg::Detection3DArray>();
        empty_body_msg->header.stamp = dr_spaam_msg->header.stamp;
        empty_body_msg->header.frame_id = target_frame_;
        callbackPoseArray(empty_body_msg);
    }
}

void multiple_sensor_person_tracking::PersonTracker::callbackPoseArray ( const vision_msgs::msg::Detection3DArray::ConstSharedPtr &body_msg ) {
    if (!active_) {
        return;
    }
    
    std::cout << "\n====================================" << std::endl;
    // variable initialization
    std::string target_frame = target_frame_;
    sensor_msgs::msg::PointCloud2 cloud_scan_msg;
    Eigen::Vector4f estimated_value( 0.0, 0.0, 0.0, 0.0 );

    rclcpp::Time current_time = this->get_clock()->now();
    double dt = (current_time - previous_time_).seconds();
    previous_time_ = current_time;
    const bool use_leg_detection = detection_mode_ != DetectionMode::BODY;
    const bool use_body_detection = detection_mode_ != DetectionMode::LEG;

    // Wait until a valid scan frame is available.
    if (!scan_msg_ || scan_msg_->header.frame_id.empty()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Waiting for a valid LaserScan frame on scan_topic_name.");
        return;
    }

    // Sensor data to TF2 conversion
    try {
        sensor_msgs::msg::LaserScan scan_msg = *scan_msg_;
        if (!scan_frame_name_.empty()) {
            scan_msg.header.frame_id = scan_frame_name_;
        }
        if (!tfBuffer_.canTransform(
                target_frame,
                scan_msg.header.frame_id,
                scan_msg.header.stamp,
                tf2::durationFromSec(0.05))) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for transform from '%s' to '%s'.",
                scan_msg.header.frame_id.c_str(),
                target_frame.c_str());
            return;
        }
        projector_.transformLaserScanToPointCloud( target_frame, scan_msg, cloud_scan_msg, tfBuffer_ );
        pcl::fromROSMsg<PointT>( cloud_scan_msg, *cloud_scan_);
        cloud_scan_->header.frame_id = target_frame;
    } catch ( const tf2::TransformException& ex ) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        following_position_->pose.position.x = 0.0;
        following_position_->pose.position.y = 0.0;
        following_position_->rotation_position = following_position_->pose.position;
        following_position_->status = Status::NO_EXISTS;
        pub_following_position_->publish( *following_position_ );
        return;
    }

    std::vector<geometry_msgs::msg::Pose> leg_detections_in_target = dr_spaam_msg_->poses;
    std::string leg_array_frame = dr_spaam_msg_->header.frame_id;
    if (!scan_frame_name_.empty() && scan_msg_ && leg_array_frame == scan_msg_->header.frame_id) {
        leg_array_frame = scan_frame_name_;
    }
    if (use_leg_detection && !leg_array_frame.empty() && leg_array_frame != target_frame_) {
        for (auto & pose : leg_detections_in_target) {
            const auto transformed = transformPoint(leg_array_frame, target_frame_, pose.position);
            pose.position = transformed.point;
        }
    }

    if ( !exists_target_ && use_body_detection && body_msg->detections.size() == 0) {
        if ( !use_leg_detection || leg_detections_in_target.size() == 0 ) {
            RCLCPP_ERROR(this->get_logger(), "Result :          NO_EXISTS (DR-SPAAM)" );
            exists_target_ = false;
            following_position_->pose.position.x = 0.0;
            following_position_->pose.position.y = 0.0;
            following_position_->rotation_position = following_position_->pose.position;
            following_position_->status = Status::NO_EXISTS;
            pub_following_position_->publish( *following_position_ );
            return;
        }
        if( attention_leg_time_ == -1.0 ) attention_leg_time_ = this->get_clock()->now().seconds();
        exists_target_ = false;
        std::vector<geometry_msgs::msg::Pose> leg_poses = leg_detections_in_target;
        // Rotate the RGB-D sensor in the direction in which the leg_poses
        // Sort by proximity
        std::sort(
            leg_poses.begin(),
            leg_poses.end(),
            []( const auto & a, const auto & b)
            { return std::hypotf(a.position.x, a.position.y) < std::hypotf(b.position.x, b.position.y); } );
        // set the position of attention_leg_idx_ -> (if attention_leg_idx_ is larger than the array, modify)
        // change attention_leg_idx_ in 2 seconds
        if ( this->get_clock()->now().seconds() - attention_leg_time_ >= 2.0 ) {
            attention_leg_idx_ = (attention_leg_idx_ + 1) % leg_poses.size();
            attention_leg_time_ = this->get_clock()->now().seconds();
        } else if (attention_leg_idx_ >= leg_poses.size()) {
            attention_leg_idx_ = leg_poses.size() - 1;
        }
        following_position_->rotation_position.x = leg_poses[attention_leg_idx_].position.x;
        following_position_->rotation_position.y = leg_poses[attention_leg_idx_].position.y;
        following_position_->pose.position.x = 0.0;
        following_position_->pose.position.y = 0.0;
        following_position_->status = Status::NO_EXISTS;
        pub_following_position_->publish( *following_position_ );
        RCLCPP_ERROR(this->get_logger(), "Result :          NO_EXISTS (Object) attention_leg_idx = %d",attention_leg_idx_ );
        return;
    } else {
        attention_leg_time_ = -1.0;
        attention_leg_idx_ = 0;
    }
    // Transform body detections into tracker target frame so body/leg fusion
    // is computed in one coordinate system.
    std::vector<vision_msgs::msg::Detection3D> body_detections_in_target = body_msg->detections;
    const std::string array_frame = body_msg->header.frame_id;
    for (auto & detection : body_detections_in_target) {
        std::string src_frame = detection.header.frame_id;
        if (src_frame.empty()) {
            src_frame = array_frame;
        }
        if (!src_frame.empty() && src_frame != target_frame_) {
            const auto transformed = transformPoint(src_frame, target_frame_, detection.bbox.center.position);
            detection.bbox.center.position = transformed.point;
            detection.header.frame_id = target_frame_;
        }
    }

    // Searching for observables to input to the Kalman filter
    Eigen::Vector2f leg_observed_value, body_observed_value;
    const std::vector<geometry_msgs::msg::Pose> leg_observations =
        use_leg_detection ? leg_detections_in_target : std::vector<geometry_msgs::msg::Pose>{};
    const std::vector<vision_msgs::msg::Detection3D> body_observations =
        use_body_detection ? body_detections_in_target : std::vector<vision_msgs::msg::Detection3D>{};
    int result = findTwoObservationValue( leg_observations, body_observations, &leg_observed_value, &body_observed_value );
    if ( result == Status::NO_EXISTS ) {
        if ( no_exists_time_ == -1.0 ) no_exists_time_ = this->get_clock()->now().seconds();
        else if ( this->get_clock()->now().seconds() - no_exists_time_ >= target_change_tolerance_ ){
            exists_target_ = false;
            following_position_->pose.position.x = 0.0;
            following_position_->pose.position.y = 0.0;
            following_position_->status = Status::NO_EXISTS;
            pub_following_position_->publish( *following_position_ );
            return;
        }
    } else no_exists_time_ = -1.0;

    // Tracking by Kalman Filter
    if ( !exists_target_ ) {
        if ( result == Status::EXISTS_BODY || result == Status::EXISTS_LEG_AND_BODY ) {
            kf_->init( body_observed_value );
            estimated_value[0] = body_observed_value[0];
            estimated_value[1] = body_observed_value[1];
            following_position_->rotation_position.x = body_observed_value[0];
            following_position_->rotation_position.y = body_observed_value[1];
            exists_target_ = true;
        } else if ( result == Status::EXISTS_LEG ) {
            kf_->init( leg_observed_value );
            estimated_value[0] = leg_observed_value[0];
            estimated_value[1] = leg_observed_value[1];
            following_position_->rotation_position.x = leg_observed_value[0];
            following_position_->rotation_position.y = leg_observed_value[1];
            exists_target_ = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Result :          NO_EXISTS" );
            exists_target_ = false;
            following_position_->pose.position.x = 0.0;
            following_position_->pose.position.y = 0.0;
            following_position_->status = Status::NO_EXISTS;
            pub_following_position_->publish( *following_position_ );
            return;
        }
    } else {
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
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, std::atan2(estimated_value[3], estimated_value[2]));
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(quat_tf, quat_msg);

    following_position_->pose.orientation = quat_msg;
    following_position_->velocity = std::hypotf(estimated_value[2], estimated_value[3]);
    following_position_->status = result;

    // search obstacles :
    sensor_msgs::msg::PointCloud2 obstacles;
    outrem_.setInputCloud( cloud_scan_ );
    outrem_.filter ( *cloud_scan_ );
    voxel_.setInputCloud( cloud_scan_ );
    voxel_.filter ( *cloud_scan_ );
    bool can_pub_obstacles = searchObstacles( following_position_->pose.position, cloud_scan_, &obstacles );

    // following_position_ : header :
    if ( can_pub_obstacles ){
        pub_obstacles_->publish( obstacles );
        following_position_->header.stamp = this->get_clock()->now();
        pub_following_position_->publish( *following_position_ );
        pub_target_odom_->publish( transformPoint( target_frame_, odom_frame_name_, following_position_->pose.position ) );
    } 

    if ( display_marker_ ) {
        marker_array_->markers.clear();
        marker_array_->markers.push_back( makeLegPoseMarker(leg_detections_in_target) );
        marker_array_->markers.push_back( makeLegAreaMarker(leg_detections_in_target) );
        marker_array_->markers.push_back( makeBodyPoseMarker(body_detections_in_target) );
        marker_array_->markers.push_back( makeTargetPoseMarker(estimated_value) );
        pub_marker_->publish ( *marker_array_ );
    }
    previous_target_ = following_position_->pose.position;

    RCLCPP_INFO( this->get_logger(), "\033[1mResult\033[m = %s",
        ( following_position_->status == Status::EXISTS_LEG ? "\033[1;36m EXISTS_LEG \033[m" :
        ( following_position_->status == Status::EXISTS_BODY ? "\033[1;33m EXISTS_BODY \033[m" :
        ( following_position_->status == Status::EXISTS_LEG_AND_BODY ? "\033[1;32m EXISTS_LEG_AND_BODY \033[m" : "\033[1;31m NO_EXISTS \033[m")) ));
    RCLCPP_INFO( this->get_logger(), "\033[1mTarget\033[m = %5.3f [m]\t%5.3f [m]", following_position_->pose.position.x, following_position_->pose.position.y );

    return;
}

multiple_sensor_person_tracking::CallbackReturn multiple_sensor_person_tracking::PersonTracker::on_configure(const rclcpp_lifecycle::State &) {

    // Declare parameters
    try {
        this->declare_parameter<std::string>("scan_topic_name", "/scan");
        this->declare_parameter<std::string>("pointcloud_nontravelable_region_topic_name", "sobits_follower/multiple_sensor_person_tracking/pointcloud_nontravelable_region");
        this->declare_parameter<std::string>("dr_spaam_topic_name", "/dr_spaam_detections");
        this->declare_parameter<std::string>("body_detection_topic_name", "/sobits_follower/object_3d_poses");
        this->declare_parameter<std::string>("target_frame", "base_footprint");
        this->declare_parameter<std::string>("odom_frame_name", "odom");
        this->declare_parameter<std::string>("scan_frame_name", "");
        this->declare_parameter<std::string>("detection_mode", "body_leg");
        this->declare_parameter<bool>("merge_nontravelable_region", false);
        this->declare_parameter<double>("leg_tracking_range", 2.5);
        this->declare_parameter<double>("body_tracking_range", 2.5);
        this->declare_parameter<double>("outlier_radius", 0.1);
        this->declare_parameter<int>("outlier_min_pts", 2);
        this->declare_parameter<double>("leaf_size", 0.1);
        this->declare_parameter<double>("target_cloud_radius", 0.4);
        this->declare_parameter<double>("target_change_tolerance", 2.0);
        this->declare_parameter<bool>("display_marker", true);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    }

    // Retrieve parameter values
    auto scan_topic_name = this->get_parameter("scan_topic_name").as_string();
    auto pointcloud_nontravelable_region_topic_name = this->get_parameter("pointcloud_nontravelable_region_topic_name").as_string();
    auto dr_spaam_topic_name = this->get_parameter("dr_spaam_topic_name").as_string();
    auto body_detection_topic_name = this->get_parameter("body_detection_topic_name").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    odom_frame_name_ = this->get_parameter("odom_frame_name").as_string();
    scan_frame_name_ = this->get_parameter("scan_frame_name").as_string();
    auto detection_mode = this->get_parameter("detection_mode").as_string();
    merge_nontravelable_region_ = this->get_parameter("merge_nontravelable_region").as_bool();
    leg_tracking_range_ = this->get_parameter("leg_tracking_range").as_double();
    body_tracking_range_ = this->get_parameter("body_tracking_range").as_double();
    target_change_tolerance_ = this->get_parameter("target_change_tolerance").as_double();
    display_marker_ = this->get_parameter("display_marker").as_bool();
    if (detection_mode == "leg") {
        detection_mode_ = DetectionMode::LEG;
    } else if (detection_mode == "body") {
        detection_mode_ = DetectionMode::BODY;
    } else if (detection_mode == "body_leg") {
        detection_mode_ = DetectionMode::BODY_LEG;
    } else {
        RCLCPP_WARN(
            this->get_logger(),
            "Unknown detection_mode '%s'. Falling back to 'body_leg'.",
            detection_mode.c_str());
        detection_mode_ = DetectionMode::BODY_LEG;
        detection_mode = "body_leg";
    }
    RCLCPP_INFO(this->get_logger(), "detection_mode: %s", detection_mode.c_str());

    // Initialize class members
    tf_sub_.reset(new tf2_ros::TransformListener(tfBuffer_));
    cloud_scan_.reset(new PointCloud());
    marker_array_.reset(new visualization_msgs::msg::MarkerArray);
    following_position_.reset( new multiple_sensor_person_tracking::msg::FollowingPosition );
    scan_msg_.reset( new sensor_msgs::msg::LaserScan );
    cloud_nontravelable_region_.reset( new PointCloud() );
    dr_spaam_msg_.reset( new geometry_msgs::msg::PoseArray() );

    // Create subscribers with sensor QoS to interoperate with Gazebo/bridge topics.
    auto sensor_qos = rclcpp::SensorDataQoS();
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_name, sensor_qos, std::bind(&PersonTracker::scan_callback, this, std::placeholders::_1));

    sub_nontravelable_region_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_nontravelable_region_topic_name, sensor_qos, std::bind(&PersonTracker::nontravelableRegionCallback, this, std::placeholders::_1));

    sub_dr_spaam_ = create_subscription<geometry_msgs::msg::PoseArray>(
        dr_spaam_topic_name, sensor_qos, std::bind(&PersonTracker::dr_spaam_callback, this, std::placeholders::_1));
    
    sub_image_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        body_detection_topic_name, sensor_qos, std::bind(&PersonTracker::callbackPoseArray, this, std::placeholders::_1));
    
    // Create publishers
    pub_following_position_ = create_publisher< multiple_sensor_person_tracking::msg::FollowingPosition >( "sobits_follower/multiple_sensor_person_tracking/following_position", 1 );
    pub_marker_ = create_publisher< visualization_msgs::msg::MarkerArray >( "sobits_follower/multiple_sensor_person_tracking/tracker_marker", 1 );
    pub_obstacles_ = create_publisher< sensor_msgs::msg::PointCloud2 >( "sobits_follower/multiple_sensor_person_tracking/obstacles", 1 );
    pub_target_odom_ = create_publisher< geometry_msgs::msg::PointStamped >( "sobits_follower/multiple_sensor_person_tracking/target_postion_odom", 1 );

    // Initialize Kalman filter
    kf_ = std::make_unique<multiple_observation_kalman_filter::KalmanFilter>(0.033, 1000, 1.0);

    // Configure pcl filters
    outrem_.setRadiusSearch(this->get_parameter("outlier_radius").as_double());
    outrem_.setMinNeighborsInRadius(this->get_parameter("outlier_min_pts").as_int());
    outrem_.setKeepOrganized(false);

    double leaf_size = this->get_parameter("leaf_size").as_double();
    voxel_.setLeafSize(leaf_size, leaf_size, 0.0);
    target_cloud_radius_ = this->get_parameter("target_cloud_radius").as_double();

    // Initialize timers and flags
    previous_time_ = this->now();;
    exists_target_ = false;
    no_exists_time_ = -1.0;
    attention_leg_time_ = -1.0;
    attention_leg_idx_ = 0;
    target_range_ = 3.0;
    active_ = false;

    return CallbackReturn::SUCCESS;
}

multiple_sensor_person_tracking::CallbackReturn multiple_sensor_person_tracking::PersonTracker::on_activate(const rclcpp_lifecycle::State &) {
    active_ = true;
    pub_following_position_->on_activate();
    pub_marker_->on_activate();
    pub_obstacles_->on_activate();
    pub_target_odom_->on_activate();
    previous_time_ = this->now();
    return CallbackReturn::SUCCESS;
}

multiple_sensor_person_tracking::CallbackReturn multiple_sensor_person_tracking::PersonTracker::on_deactivate(const rclcpp_lifecycle::State &) {
    active_ = false;
    if (pub_following_position_) pub_following_position_->on_deactivate();
    if (pub_marker_) pub_marker_->on_deactivate();
    if (pub_obstacles_) pub_obstacles_->on_deactivate();
    if (pub_target_odom_) pub_target_odom_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

void multiple_sensor_person_tracking::PersonTracker::resetInterfaces() {
    sub_scan_.reset();
    sub_nontravelable_region_.reset();
    sub_dr_spaam_.reset();
    sub_image_.reset();
    pub_following_position_.reset();
    pub_marker_.reset();
    pub_obstacles_.reset();
    pub_target_odom_.reset();
    tf_sub_.reset();
    cloud_nontravelable_region_.reset();
    kf_.reset();
    cloud_scan_.reset();
    marker_array_.reset();
    following_position_.reset();
    scan_msg_.reset();
    dr_spaam_msg_.reset();
}

multiple_sensor_person_tracking::CallbackReturn multiple_sensor_person_tracking::PersonTracker::on_cleanup(const rclcpp_lifecycle::State &) {
    active_ = false;
    resetInterfaces();
    return CallbackReturn::SUCCESS;
}

multiple_sensor_person_tracking::CallbackReturn multiple_sensor_person_tracking::PersonTracker::on_shutdown(const rclcpp_lifecycle::State &) {
    active_ = false;
    resetInterfaces();
    return CallbackReturn::SUCCESS;
}

multiple_sensor_person_tracking::CallbackReturn multiple_sensor_person_tracking::PersonTracker::on_error(const rclcpp_lifecycle::State &) {
    active_ = false;
    resetInterfaces();
    return CallbackReturn::SUCCESS;
}

RCLCPP_COMPONENTS_REGISTER_NODE(multiple_sensor_person_tracking::PersonTracker)
