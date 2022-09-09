#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include<bits/stdc++.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>

namespace multiple_observation_tracing_simulator{
    class VirtualEnvironment {
        private :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_marker_;

            ros::Subscriber sub_teleop_;
            geometry_msgs::Point tgt_pt_;
            double tgt_theta_;

            void callbackTarget ( const geometry_msgs::TwistConstPtr &msg );
            void displaySensorMarker ( );

        public :
            VirtualEnvironment ( );
            void pubData (  );
    };
}

void multiple_observation_tracing_simulator::VirtualEnvironment::callbackTarget ( const geometry_msgs::TwistConstPtr &msg ) {
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

multiple_observation_tracing_simulator::VirtualEnvironment::VirtualEnvironment ( ) : nh_(), pnh_("~") {
    tgt_pt_.x = 0.0;
    tgt_pt_.y = 0.0;
    tgt_theta_ = 0.0;
    pub_marker_ = nh_.advertise< visualization_msgs::MarkerArray >( "/target_marker", 1 );
    sub_teleop_ = nh_.subscribe( "/target/teleop", 10, &VirtualEnvironment::callbackTarget, this );
}

void multiple_observation_tracing_simulator::VirtualEnvironment::pubData (  ) {
    static tf::TransformBroadcaster br;
    visualization_msgs::Marker trajectory;
    trajectory.header.frame_id ="map";
    trajectory.header.stamp = ros::Time::now();
    trajectory.ns = "trajectory";
    trajectory.id =  1;
    trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory.action = visualization_msgs::Marker::ADD;
    trajectory.scale.x = 0.2;
    trajectory.color.r = 1.0; trajectory.color.g = 0.0; trajectory.color.b = 0.0; trajectory.color.a = 1.0;
    trajectory.pose.orientation.w = 1.0;
    //marker.lifetime = ros::Duration(0.3);

    ros::Rate rate(30);

    while(ros::ok()){
        visualization_msgs::MarkerArray marker_array;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(tgt_pt_.x, tgt_pt_.y, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, tgt_theta_);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "target" ));

        geometry_msgs::PointStamped position, observed_value, observed_value_add;
        position.point.x = tgt_pt_.x;
        position.point.y = tgt_pt_.y;
        position.point.z = -0.3;

        trajectory.points.push_back( position.point );
        if ( trajectory.points.size() > 200 ) trajectory.points.erase(trajectory.points.begin());
        trajectory.header.stamp = ros::Time::now();

        marker_array.markers.push_back( trajectory );
        pub_marker_.publish ( marker_array );

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "virtual_environment");
    multiple_observation_tracing_simulator::VirtualEnvironment ve;
    ve.pubData();
    ros::spin();
}