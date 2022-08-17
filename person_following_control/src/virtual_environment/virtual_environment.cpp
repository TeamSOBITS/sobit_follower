#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include<bits/stdc++.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <person_following_control/FollowingPosition.h>

#include <dynamic_reconfigure/server.h>
#include <person_following_control/VirtualEnvironmentParameterConfig.h>
#include <multiple_observation_kalman_filter/multiple_observation_kalman_filter.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace person_following_control{
    class VirtualEnvironment {
        private :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_mrk_tgt_;
            ros::Publisher pub_mrk_sensor_;
            ros::Subscriber sub_tgt_pose_;
            tf::TransformListener tf_listener_;
            geometry_msgs::Point tgt_pt_;
            PointCloud::Ptr cloud_sensor_map_;
            double tgt_theta_;
            std::unique_ptr<std::normal_distribution<double>> dist_;
            std::unique_ptr<multiple_observation_kalman_filter::KalmanFilter> kf_;

            dynamic_reconfigure::Server<person_following_control::VirtualEnvironmentParameterConfig>* server_;
            dynamic_reconfigure::Server<person_following_control::VirtualEnvironmentParameterConfig>::CallbackType f_;

            void callbackDynamicReconfigure(person_following_control::VirtualEnvironmentParameterConfig& config, uint32_t level);
            void callbackTarget ( const geometry_msgs::TwistConstPtr &msg );
            void makeVirtualEnvironment( int obstacle_number );
            void displaySensorMarker ( );

        public :
            VirtualEnvironment ( );
            void pubData (  );
    };
}

void person_following_control::VirtualEnvironment::callbackDynamicReconfigure(person_following_control::VirtualEnvironmentParameterConfig& config, uint32_t level) {
    dist_.reset( new std::normal_distribution<double>( 0.0, config.observation_noise ) );
    makeVirtualEnvironment( config.obstacle_number );
    ROS_INFO("callbackDynamicReconfigure");
}

void person_following_control::VirtualEnvironment::callbackTarget ( const geometry_msgs::TwistConstPtr &msg ) {
    tgt_theta_ += 0.05 * msg->angular.z ;
    if ( tgt_theta_ > M_PI )    tgt_theta_ =tgt_theta_ - 2 * M_PI;
    if ( tgt_theta_ < - M_PI )  tgt_theta_ =tgt_theta_ + 2 * M_PI;

    if ( msg->linear.y == 0.0 ) {
        tgt_pt_.x += 0.05 * msg->linear.x * std::cos( tgt_theta_ );
        tgt_pt_.y += 0.05 * msg->linear.x * std::sin( tgt_theta_ );
    } else {
        double ang = std::atan2( msg->linear.y, msg->linear.x );
        double dist = std::hypotf( msg->linear.x, msg->linear.y );
        tgt_pt_.x += 0.05 * dist * std::cos( tgt_theta_ + ang );
        tgt_pt_.y += 0.05 * dist * std::sin( tgt_theta_ + ang );
    }
    return;
}

void  person_following_control::VirtualEnvironment::makeVirtualEnvironment( int obstacle_number ) {
    cloud_sensor_map_ .reset ( new PointCloud() );
    cloud_sensor_map_->header.frame_id = "map";
    cloud_sensor_map_->width  = obstacle_number;
    cloud_sensor_map_->height = 1;
    cloud_sensor_map_->points.resize (cloud_sensor_map_->width * cloud_sensor_map_->height);
    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_real_distribution<> rand5( -5.0, 5.0);
    for ( auto& obs : cloud_sensor_map_->points ) {
        obs.x = rand5(mt);
        obs.y = rand5(mt);
        obs.z = 0.0;
        obs.x = ( std::fabs(obs.x) < 0.4 ) ? ( obs.x > 0 ) ? obs.x + 0.4 : obs.x - 0.4 : obs.x;
        obs.y = ( std::fabs(obs.y) < 0.4 ) ? ( obs.y > 0 ) ? obs.y + 0.4 : obs.y - 0.4 : obs.y;
    }
    ROS_INFO("Make Virtual Environment... : sensor size = %d", obstacle_number);
}
void  person_following_control::VirtualEnvironment::displaySensorMarker ( ) {
    ROS_INFO("displaySensorMarker : %zu", cloud_sensor_map_->points.size() );
    int marker_num = 0;
    visualization_msgs::MarkerArray marker_array;
    for ( auto& pt : cloud_sensor_map_->points ) {
        visualization_msgs::Marker obs;
        obs.header.frame_id = "map";
        obs.header.stamp = ros::Time::now();
        obs.type = visualization_msgs::Marker::SPHERE;
        obs.action = visualization_msgs::Marker::ADD;
        obs.ns = "obstacles";
        obs.id = marker_num;
        obs.pose.position.x = pt.x;
        obs.pose.position.y = pt.y;
        obs.pose.position.z = pt.z;
        obs.pose.orientation.x = 0.0;
        obs.pose.orientation.y = 0.0;
        obs.pose.orientation.z = 0.0;
        obs.pose.orientation.w = 1.0;
        obs.scale.x = 0.1;
        obs.scale.y = 0.1;
        obs.scale.z = 0.05;
        obs.color.r = 0.0f;
        obs.color.g = 1.0f;
        obs.color.b = 0.0f;
        obs.color.a = 1.0f;
        obs.lifetime = ros::Duration(0.3);
        marker_num++;
        marker_array.markers.push_back( obs );

        visualization_msgs::Marker cost;
        cost.header.frame_id = "map";
        cost.header.stamp = ros::Time::now();
        cost.type = visualization_msgs::Marker::SPHERE;
        cost.action = visualization_msgs::Marker::ADD;
        cost.ns = "cost";
        cost.id = marker_num;
        cost.pose.position.x = pt.x;
        cost.pose.position.y = pt.y;
        cost.pose.position.z = pt.z;
        cost.pose.orientation.x = 0.0;
        cost.pose.orientation.y = 0.0;
        cost.pose.orientation.z = 0.0;
        cost.pose.orientation.w = 1.0;
        cost.scale.x = 0.35;
        cost.scale.y = 0.35;
        cost.scale.z = 0.01;
        cost.color.r = 0.86f;
        cost.color.g = 0.44f;
        cost.color.b = 0.57f;
        cost.color.a = 1.0f;
        cost.lifetime = ros::Duration(0.3);
        marker_num++;
        marker_array.markers.push_back( cost );
    }
    pub_mrk_sensor_.publish( marker_array );
}

person_following_control::VirtualEnvironment::VirtualEnvironment ( ) : nh_(), pnh_("~") {
    tgt_pt_.x = 0.0;
    tgt_pt_.y = 0.0;
    tgt_theta_ = 0.0;
    pub_mrk_tgt_ = nh_.advertise< visualization_msgs::Marker >( "/target_marker", 1 );
    pub_mrk_sensor_ = nh_.advertise< visualization_msgs::MarkerArray >( "/sensor_marker", 1 );
    sub_tgt_pose_ = nh_.subscribe( "/virtual_environment/cmd_vel", 10, &VirtualEnvironment::callbackTarget, this );
    kf_.reset( new multiple_observation_kalman_filter::KalmanFilter( 0.033, 1000, 1.0 ) );

    server_ = new dynamic_reconfigure::Server<person_following_control::VirtualEnvironmentParameterConfig>(pnh_);
    f_ = boost::bind(&person_following_control::VirtualEnvironment::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);
}

void person_following_control::VirtualEnvironment::pubData (  ) {
    ros::Rate rate(30);
    static tf::TransformBroadcaster br;
    ros::Publisher pub_trajectory = nh_.advertise< visualization_msgs::Marker >( "/target_trajectory", 1 );
    ros::Publisher pub_following_position = nh_.advertise< person_following_control::FollowingPosition >( "/following_position", 1 );
    ros::Publisher pub_following_position_marker_ = nh_.advertise< visualization_msgs::Marker >( "/following_position_marker", 1 );


    visualization_msgs::Marker trajectory, target_pose;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = ros::Time::now();
    trajectory.ns = "trajectory";
    trajectory.id = 1;
    trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory.action = visualization_msgs::Marker::ADD;
    trajectory.lifetime = ros::Duration(1.0);
    trajectory.scale.x = 0.05;
    trajectory.color.a = 1.0;
    trajectory.color.r = 0.0;
    trajectory.color.g = 0.0;
    trajectory.color.b = 1.0;
    trajectory.pose.orientation.x = 0.0;
    trajectory.pose.orientation.y = 0.0;
    trajectory.pose.orientation.z = 0.0;
    trajectory.pose.orientation.w = 1.0;

    target_pose.header.frame_id = "base_footprint";
    target_pose.header.stamp = ros::Time::now();
    target_pose.ns = "target_pose";
    target_pose.id = 1;
    target_pose.type = visualization_msgs::Marker::ARROW;
    target_pose.action = visualization_msgs::Marker::ADD;
    target_pose.lifetime = ros::Duration(1.0);
    target_pose.scale.x = 0.7;target_pose.scale.y = 0.15;target_pose.scale.z = 0.15;
    target_pose.color.a = 1.0;
    target_pose.color.r = 0.0;
    target_pose.color.g = 1.0;
    target_pose.color.b = 1.0;

    person_following_control::FollowingPositionPtr following_position ( new person_following_control::FollowingPosition );
    std::random_device seed;
    std::mt19937 engine(seed());            // メルセンヌ・ツイスター法
    // geometry_msgs::Point pre_pt;

    while(ros::ok()){
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(tgt_pt_.x, tgt_pt_.y, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, tgt_theta_);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "target" ));
        ROS_INFO("[ Target ] x = %.3f , y = %.3f", tgt_pt_.x, tgt_pt_.y );

        geometry_msgs::Point temp;
        temp.x = tgt_pt_.x;
        temp.y = tgt_pt_.y;
        temp.z = 0.1;

        trajectory.points.push_back( temp );
        if ( trajectory.points.size() > 200 ) trajectory.points.erase(trajectory.points.begin());
        trajectory.header.stamp = ros::Time::now();
        pub_trajectory.publish ( trajectory );

        PointCloud::Ptr cloud_sensor_base (new PointCloud() );
        sensor_msgs::PointCloud2Ptr sensor_msg ( new sensor_msgs::PointCloud2 );
        try {
            tf_listener_.waitForTransform( "base_footprint", "map", ros::Time(0), ros::Duration(1.0));
            pcl_ros::transformPointCloud( "base_footprint", ros::Time(0), *cloud_sensor_map_, "map",  *cloud_sensor_base, tf_listener_);
        } catch (const tf::TransformException& ex) {ROS_ERROR("%s", ex.what());}
        pcl::toROSMsg(*cloud_sensor_base, *sensor_msg );

        displaySensorMarker ( );

        tf::StampedTransform transform_stamped;
        geometry_msgs::PoseStamped target_pose_base_robot;
        try {
            // tf_listener_.waitForTransform("/base_footprint", "/target", ros::Time(0), ros::Duration(10.0) );
            tf_listener_.lookupTransform("/base_footprint", "/target",  ros::Time(0), transform_stamped);
        } catch ( const tf::TransformException& ex){
            ROS_ERROR("%s",ex.what());
            continue;
        }
        tf::poseTFToMsg(transform_stamped, target_pose_base_robot.pose);

        target_pose_base_robot.header.stamp = ros::Time::now();
        target_pose_base_robot.header.frame_id = "/base_footprint";

        Eigen::Vector2f observed_value( 0.0, 0.0 );
        observed_value[0] = target_pose_base_robot.pose.position.x + (*dist_)(engine);
        observed_value[1] = target_pose_base_robot.pose.position.y + (*dist_)(engine);
        Eigen::Vector4f estimated_value( 0.0, 0.0, 0.0, 0.0 );
        kf_->compute( 0.033, observed_value, &estimated_value );
        // following_position : pose :
        following_position->pose.position.x = estimated_value[0];
        following_position->pose.position.y = estimated_value[1];
        following_position->velocity = std::hypotf(estimated_value[2], estimated_value[3]);
        following_position->pose.orientation.w = 1.0;
        // pre_pt = following_position->pose.position;
        // double yaw = ( following_position->velocity > 0.1 ) ? std::atan2(estimated_value[3], estimated_value[2]) : 0.0;
        // double yaw = std::atan2( pre_pt.y - following_position->pose.position.y, pre_pt.x - following_position->pose.position.x );
        // tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, yaw);
        // geometry_msgs::Quaternion geometry_quat;
        // quaternionTFToMsg(quat, geometry_quat);
        // following_position->pose.orientation = geometry_quat;

        following_position->header.stamp = ros::Time::now();
        following_position->obstacles = *sensor_msg;
        pub_following_position.publish( following_position );

        target_pose.header.stamp = ros::Time::now();
        target_pose.pose = following_position->pose;
        pub_following_position_marker_.publish( target_pose );

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "virtual_environment");
    person_following_control::VirtualEnvironment ve;
    ve.pubData();
    ros::spin();
}