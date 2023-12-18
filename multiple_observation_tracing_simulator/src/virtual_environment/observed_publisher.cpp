#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <bits/stdc++.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include "multiple_observation_tracing_simulator/ObservationParameterConfig.h"

namespace multiple_observation_tracing_simulator {
    class ObservedPublisher {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_true_value_;
            ros::Publisher pub_observed_value_;
            ros::Publisher pub_observed_value_add_;
            ros::Publisher pub_marker_;
            tf2_ros::Buffer               tfBuffer_;
            tf2_ros::TransformListener    tfListener_;
            std::unique_ptr<ros::Rate> rate_;
            std::unique_ptr<std::normal_distribution<double>> dist_;
            std::unique_ptr<std::normal_distribution<double>> dist_add_;
            dynamic_reconfigure::Server<multiple_observation_tracing_simulator::ObservationParameterConfig>* server_;
            dynamic_reconfigure::Server<multiple_observation_tracing_simulator::ObservationParameterConfig>::CallbackType f_;

            void callbackDynamicReconfigure(multiple_observation_tracing_simulator::ObservationParameterConfig& config, uint32_t level);

        public:
            ObservedPublisher( );
            void loopPublish();
    };
}

void multiple_observation_tracing_simulator::ObservedPublisher::callbackDynamicReconfigure(multiple_observation_tracing_simulator::ObservationParameterConfig& config, uint32_t level) {
    rate_.reset( new ros::Rate(config.fps) );
    dist_.reset( new std::normal_distribution<double>( 0.0, config.observation_noise ) );
    dist_add_.reset( new std::normal_distribution<double>( 0.0, config.observation_noise_add ) );
}

multiple_observation_tracing_simulator::ObservedPublisher::ObservedPublisher( ) : nh_(), pnh_("~"), tfBuffer_(), tfListener_(tfBuffer_)  {
    rate_.reset( new ros::Rate(30) );
    dist_.reset( new std::normal_distribution<double>( 0.0, 0.1 ) );
    dist_add_.reset( new std::normal_distribution<double>( 0.0, 0.1 ) );

    pub_true_value_ = nh_.advertise< geometry_msgs::PointStamped >( "/true_value", 1 );
    pub_observed_value_ = nh_.advertise< geometry_msgs::PointStamped >( "/observed_value", 1 );
    pub_observed_value_add_ = nh_.advertise< geometry_msgs::PointStamped >( "/observed_value_add", 1 );
    pub_marker_ = nh_.advertise< visualization_msgs::MarkerArray >( "/observed_marker", 1 );

    server_ = new dynamic_reconfigure::Server<multiple_observation_tracing_simulator::ObservationParameterConfig>(pnh_);
    f_ = boost::bind(&multiple_observation_tracing_simulator::ObservedPublisher::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);
}

void multiple_observation_tracing_simulator::ObservedPublisher::loopPublish( ){
    visualization_msgs::Marker observed, observed_add;
    observed.header.frame_id = observed_add.header.frame_id ="map";
    observed.header.stamp = observed_add.header.stamp = ros::Time::now();
    observed.ns = "observed"; observed_add.ns = "observed_add";
    observed.id =  observed_add.id =  1;
    observed.type = observed_add.type = visualization_msgs::Marker::SPHERE;
    observed.action = observed_add.action =  visualization_msgs::Marker::ADD;
    observed.scale.x = 0.2; observed.scale.y = 0.2; observed.scale.z = 0.2;
    observed_add.scale.x = 0.2; observed_add.scale.y = 0.2; observed_add.scale.z = 0.2;
    observed.color.r = 0.0; observed.color.g = 0.0; observed.color.b = 1.0; observed.color.a = 1.0;
    observed_add.color.r = 0.61; observed_add.color.g = 0.8; observed_add.color.b = 0.88; observed_add.color.a = 1.0;
    observed.pose.orientation.w = 1.0; observed_add.pose.orientation.w = 1.0;

    std::random_device seed;
    std::mt19937 engine(seed());

    while(ros::ok()){
        geometry_msgs::Pose true_value_pose;
        geometry_msgs::PoseStamped true_value_pose_stamped;
        geometry_msgs::PointStamped true_value, observed_value, observed_value_add;
        geometry_msgs::TransformStamped transform_stamped;
        tf2::Stamped<tf2::Transform> tf2_stamped_transform;
        visualization_msgs::MarkerArray marker_array;
        try {
            transform_stamped = tfBuffer_.lookupTransform("robot", "target", ros::Time(0), ros::Duration(10.0));
        } catch ( const tf2::TransformException& ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        tf2::fromMsg(transform_stamped, tf2_stamped_transform);
        tf2::toMsg (tf2_stamped_transform, true_value_pose_stamped);
        true_value.point = true_value_pose_stamped.pose.position;

        observed_value.point.x = true_value.point.x + (*dist_)(engine);
        observed_value.point.y = true_value.point.y + (*dist_)(engine);
        observed_value.point.z = -0.05;
        observed.pose.position = observed_value.point;
        observed_value_add.point.x = true_value.point.x + (*dist_add_)(engine);
        observed_value_add.point.y = true_value.point.y + (*dist_add_)(engine);
        observed_value_add.point.z = -0.05;
        observed_add.pose.position = observed_value_add.point;
        marker_array.markers.push_back( observed );
        marker_array.markers.push_back( observed_add );

        true_value.header.stamp = ros::Time::now(); true_value.header.frame_id = "robot";
        observed_value.header.stamp = true_value.header.stamp; observed_value.header.frame_id = true_value.header.frame_id;
        observed_value_add.header.stamp = true_value.header.stamp; observed_value_add.header.frame_id = true_value.header.frame_id;
        observed.header.stamp = true_value.header.stamp; observed.header.frame_id = true_value.header.frame_id;
        observed_add.header.stamp = true_value.header.stamp; observed_add.header.frame_id = true_value.header.frame_id;
        pub_true_value_.publish ( true_value );
        pub_observed_value_.publish ( observed_value );
        pub_observed_value_add_.publish ( observed_value_add );
        pub_marker_.publish ( marker_array );
        ROS_INFO("[ true_value ]            x = %.3f , y = %.3f", true_value.point.x, true_value.point.y );
        ROS_INFO("[ observed_value ]        x = %.3f , y = %.3f", observed_value.point.x, observed_value.point.y );
        ROS_INFO("[ observed_value_add ]    x = %.3f , y = %.3f\n", observed_value_add.point.x, observed_value_add.point.y );
        ros::spinOnce();
        rate_->sleep();
    }
}

int main(int argc, char *argv[])  {
    ros::init(argc, argv, "observed_publisher");
    multiple_observation_tracing_simulator::ObservedPublisher observed_publisher;
    observed_publisher.loopPublish();
    ros::spin();
    return 0;
}