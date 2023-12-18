#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
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

visualization_msgs::Marker makePathMarker ( float vel, float ang_vel ) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = ros::Time::now();
    marker.ns = "path";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration(0.1);

    int predict_step = 20;
    float theta = 0.0;
    float sampling_time = 0.1;
    geometry_msgs::Point pt, pre_pt;
    pt.z = 0.1;
    for ( int step = 0; step < predict_step; ++step) {
        pre_pt = pt;
        pt.x = vel * cos(theta) * sampling_time + pre_pt.x;
        pt.y = vel * sin(theta) * sampling_time + pre_pt.y;
        pt.z = 0.4;
        theta = ang_vel * sampling_time + theta;
        // if ( std::hypotf( pt.x, pt.y ) > 1.5 ) break;
        marker.points.push_back( pt );
    }
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    return marker;
}

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
    tf2::Quaternion q;
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
    ros::Subscriber sub_robot_pose = nh.subscribe( "/mobile_base/commands/velocity", 10, &callbackTwist );
    ros::Publisher pub_odom = nh.advertise< nav_msgs::Odometry >( "/odom", 1 );
    ros::Publisher pub_vel = nh.advertise< geometry_msgs::Twist >( "/mobile_base/commands/velocity", 1 );
    ros::Publisher pub_marker = nh.advertise< visualization_msgs::Marker >( "robot_trajectory", 1 );
    ros::Publisher pub_mrk_path = nh.advertise< visualization_msgs::Marker >( "path", 1 );
    static tf2_ros::TransformBroadcaster br;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory_potential";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    //marker.lifetime = ros::Duration(0.3);
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
        tf2::Transform transform;
        transform.setOrigin( tf2::Vector3(g_robot.x, g_robot.y, 0.0) );
        tf2::Quaternion q;
        q.setRPY(0, 0, g_robot.theta);
        transform.setRotation(q);

        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "base_footprint";
        tf2::convert(transform, transform_stamped.transform);
        br.sendTransform(transform_stamped);
        g_odom.header.stamp = ros::Time::now();
        pub_odom.publish( g_odom );
        pub_vel.publish( g_odom.twist.twist );
        ROS_INFO("[ Robot ]  x = %.3f , y = %.3f", g_robot.x, g_robot.y );

        geometry_msgs::Point temp;
        temp.x = g_robot.x;
        temp.y = g_robot.y;
        temp.z = 0.1;

        marker.points.push_back( temp );
        if ( marker.points.size() > 200 ) marker.points.erase(marker.points.begin());
        marker.header.stamp = ros::Time::now();
        pub_marker.publish ( marker );
        pub_mrk_path.publish( makePathMarker( g_odom.twist.twist.linear.x, g_odom.twist.twist.angular.z ) );

        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();
}