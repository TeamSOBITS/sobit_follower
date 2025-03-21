#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

struct RobotPose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

class RobotPoseBroadcaster : public rclcpp::Node {
public:
    RobotPoseBroadcaster() : Node("robot_pose_broadcaster_node") {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        sub_robot_pose_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/mobile_base/commands/velocity", 10, std::bind(&RobotPoseBroadcaster::callbackTwist, this, std::placeholders::_1));

        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/mobile_base/commands/velocity", 1);
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("robot_trajectory", 1);
        pub_mrk_path_ = this->create_publisher<visualization_msgs::msg::Marker>("path", 1);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&RobotPoseBroadcaster::updatePose, this));
    }

private:
    RobotPose g_robot_;
    nav_msgs::msg::Odometry g_odom_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_robot_pose_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_mrk_path_;
    rclcpp::TimerBase::SharedPtr timer_;

    void callbackTwist(const geometry_msgs::msg::Twist::SharedPtr msg) {
        g_robot_.theta += 0.05 * msg->angular.z;
        if (g_robot_.theta > M_PI) g_robot_.theta -= 2 * M_PI;
        if (g_robot_.theta < -M_PI) g_robot_.theta += 2 * M_PI;

        g_robot_.x += 0.05 * msg->linear.x * std::cos(g_robot_.theta);
        g_robot_.y += 0.05 * msg->linear.x * std::sin(g_robot_.theta);

        g_odom_.twist.twist.linear.x += 0.1 * (msg->linear.x - g_odom_.twist.twist.linear.x);
        g_odom_.twist.twist.angular.z += 0.1 * (msg->angular.z - g_odom_.twist.twist.angular.z);
        g_odom_.pose.pose.position.x = g_robot_.x;
        g_odom_.pose.pose.position.y = g_robot_.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, g_robot_.theta);
        g_odom_.pose.pose.orientation = tf2::toMsg(q);
    }

    void updatePose() {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "base_footprint";

        transform_stamped.transform.translation.x = g_robot_.x;
        transform_stamped.transform.translation.y = g_robot_.y;
        transform_stamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, g_robot_.theta);
        transform_stamped.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(transform_stamped);

        g_odom_.header.stamp = this->get_clock()->now();
        pub_odom_->publish(g_odom_);
        pub_vel_->publish(g_odom_.twist.twist);

        RCLCPP_INFO(this->get_logger(), "[ Robot ]  x = %.3f , y = %.3f", g_robot_.x, g_robot_.y);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "trajectory_potential";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.pose.orientation.w = 1.0;

        geometry_msgs::msg::Point temp;
        temp.x = g_robot_.x;
        temp.y = g_robot_.y;
        temp.z = 0.1;
        marker.points.push_back(temp);

        if (marker.points.size() > 200) marker.points.erase(marker.points.begin());

        pub_marker_->publish(marker);
        pub_mrk_path_->publish(makePathMarker(g_odom_.twist.twist.linear.x, g_odom_.twist.twist.angular.z));
    }

    visualization_msgs::msg::Marker makePathMarker(float vel, float ang_vel) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_footprint";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "path";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.pose.orientation.w = 1.0;

        int predict_step = 20;
        double theta = 0.0;
        double sampling_time = 0.1;
        geometry_msgs::msg::Point pt, pre_pt;
        pt.z = 0.1;

        for (int step = 0; step < predict_step; ++step) {
            pre_pt = pt;
            pt.x = vel * cos(theta) * sampling_time + pre_pt.x;
            pt.y = vel * sin(theta) * sampling_time + pre_pt.y;
            pt.z = 0.4;
            theta = ang_vel * sampling_time + theta;
            marker.points.push_back(pt);
        }

        return marker;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotPoseBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
