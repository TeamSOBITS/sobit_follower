#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

struct RobotPose {
    double x = -1.0;
    double y = 0.0;
    double theta = 0.0;
};

namespace multiple_observation_tracing_simulator {
    class RobotPoseBroadcaster : public rclcpp::Node {
    private:
        RobotPose robot_;
        nav_msgs::msg::Odometry odom_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        visualization_msgs::msg::Marker marker_;

        rclcpp::TimerBase::SharedPtr timer_;

        void callbackTwist(const geometry_msgs::msg::Twist::SharedPtr msg);
        void publishData();

    public:
        RobotPoseBroadcaster();
    };

    RobotPoseBroadcaster::RobotPoseBroadcaster() : Node("robot_pose_broadcaster_node") {
        sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_mux/input/teleop", 10,
            std::bind(&RobotPoseBroadcaster::callbackTwist, this, std::placeholders::_1));

        pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
        pub_marker_ = create_publisher<visualization_msgs::msg::Marker>("/robot_trajectory", 1);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        marker_.header.frame_id = "map";
        marker_.ns = "trajectory_potential";
        marker_.id = 1;
        marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_.action = visualization_msgs::msg::Marker::ADD;
        marker_.scale.x = 0.05;
        marker_.color.a = 1.0;
        marker_.color.r  = 0.0;
        marker_.color.g = 1.0;
        marker_.color.b  = 0.0;
        marker_.pose.orientation.x = 0.0;
        marker_.pose.orientation.y = 0.0;
        marker_.pose.orientation.z = 0.0;
        marker_.pose.orientation.w = 1.0;

        timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                std::bind(&RobotPoseBroadcaster::publishData, this));
    }

    void RobotPoseBroadcaster::callbackTwist(const geometry_msgs::msg::Twist::SharedPtr msg) {
        robot_.theta += 0.05 * msg->angular.z;
        robot_.theta = std::atan2(std::sin(robot_.theta), std::cos(robot_.theta));

        robot_.x += 0.05 * msg->linear.x * std::cos(robot_.theta);
        robot_.y += 0.05 * msg->linear.x * std::sin(robot_.theta);

        odom_.twist.twist.linear.x += 0.1 * (msg->linear.x - odom_.twist.twist.linear.x);
        odom_.twist.twist.angular.z += 0.1 * (msg->angular.z - odom_.twist.twist.angular.z);

        odom_.pose.pose.position.x = robot_.x;
        odom_.pose.pose.position.y = robot_.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, robot_.theta);

        odom_.pose.pose.orientation = tf2::toMsg(q);
    }

    void RobotPoseBroadcaster::publishData() {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "robot";
        transform_stamped.transform.translation.x = robot_.x;
        transform_stamped.transform.translation.y = robot_.y;
        transform_stamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, robot_.theta);
        transform_stamped.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(transform_stamped);

        odom_.header.stamp = this->now();
        odom_.header.frame_id = "map";
        pub_odom_->publish(odom_);

        geometry_msgs::msg::Point point;
        point.x = robot_.x;
        point.y = robot_.y;
        point.z = -0.3;

        marker_.points.push_back(point);
        if (marker_.points.size() > 200)
            marker_.points.erase(marker_.points.begin());

        marker_.header.stamp = this->now();
        pub_marker_->publish(marker_);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<multiple_observation_tracing_simulator::RobotPoseBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
