#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

typedef struct {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
}RobotPose;

RobotPose g_robot;
nav_msgs::Odometry g_odom;

void callbackTwist ( const geometry_msgs::TwistConstPtr &msg ) {
    g_robot.theta += 0.05 * msg->angular.z ;
    if ( g_robot.theta > M_PI )     g_robot.theta = g_robot.theta - 2 * M_PI;
    if ( g_robot.theta < - M_PI )   g_robot.theta = g_robot.theta + 2 * M_PI;

    g_robot.x += 0.05 * msg->linear.x * std::cos( g_robot.theta );
    g_robot.y += 0.05 * msg->linear.x * std::sin( g_robot.theta );

    g_odom.twist.twist.linear.x += 0.1 * ( msg->linear.x - g_odom.twist.twist.linear.x );
    g_odom.twist.twist.angular.z += 0.1 * ( msg->angular.z - g_odom.twist.twist.angular.z );
    g_odom.pose.pose.position.x = g_robot.x;
    g_odom.pose.pose.position.y = g_robot.y;
    tf::Quaternion q;
    q.setRPY(0, 0, g_robot.theta);
    g_odom.pose.pose.orientation.w = q.getW();
    g_odom.pose.pose.orientation.x = q.getX();
    g_odom.pose.pose.orientation.y = q.getY();
    g_odom.pose.pose.orientation.z = q.getZ();
    return;
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "robot_pose_broadcaster_node");

	ros::NodeHandle nh;
    ros::Subscriber sub_robot_pose = nh.subscribe( "/cmd_vel_mux/input/teleop", 10, &callbackTwist );
    ros::Publisher pub_odom = nh.advertise< nav_msgs::Odometry >( "/odom", 1 );
    ros::Publisher pub_marker = nh.advertise< visualization_msgs::Marker >( "robot_trajectory", 1 );
    static tf::TransformBroadcaster br;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory_potential";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    //marker.lifetime = ros::Duration(1.0);
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    g_robot.x = -1.0;
    g_robot.y = 0.0;
    g_robot.theta = 0.0;
    g_odom.pose.pose.orientation.w = 1.0;

    ros::Rate rate(50);
    while(ros::ok()){
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(g_robot.x, g_robot.y, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, g_robot.theta);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_footprint" ));
        g_odom.header.stamp = ros::Time::now();
        pub_odom.publish( g_odom );
        ROS_INFO("[ Robot ]  x = %.3f , y = %.3f", g_robot.x, g_robot.y );

        geometry_msgs::Point temp;
        temp.x = g_robot.x;
        temp.y = g_robot.y;
        temp.z = 0.1;

        marker.points.push_back( temp );
        if ( marker.points.size() > 200 ) marker.points.erase(marker.points.begin());
        marker.header.stamp = ros::Time::now();
        pub_marker.publish ( marker );

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
}