#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <memory>
#include <vector>
// #include "multiple_observation_kalman_filter/multiple_observation_kalman_filter.hpp"
#include "/home/sobits/colcon_ws/src/sobit_follower/multiple_observation_kalman_filter/include/multiple_observation_kalman_filter/multiple_observation_kalman_filter.hpp"

using namespace std::chrono_literals;
using MySyncPolicy = message_filters::sync_policies::ApproximateTime<
    geometry_msgs::msg::PointStamped,
    geometry_msgs::msg::PointStamped,
    geometry_msgs::msg::PointStamped>;

namespace multiple_observation_tracing_simulator {
    class Tracker : public rclcpp::Node {
        public:
            Tracker();
        
        private:
            void callbackMessage(
                const geometry_msgs::msg::PointStamped::ConstSharedPtr true_value_msg,
                const geometry_msgs::msg::PointStamped::ConstSharedPtr observed_value_msg,
                const geometry_msgs::msg::PointStamped::ConstSharedPtr observed_value_add_msg);
            
            geometry_msgs::msg::Point transformPoint(
                const std::string &org_frame, 
                const std::string &target_frame, 
                const geometry_msgs::msg::Point &point);

            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_;

            std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PointStamped>> sub_true_value_;
            std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PointStamped>> sub_observed_value_;
            std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PointStamped>> sub_observed_value_add_;

            std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

            std::unique_ptr<multiple_observation_kalman_filter::KalmanFilter> kf_;
            std::unique_ptr<geometry_msgs::msg::Point> target_smooth_;

            tf2_ros::Buffer tfBuffer_;
            tf2_ros::TransformListener tfListener_;

            visualization_msgs::msg::Marker trajectory_;
            visualization_msgs::msg::Marker trajectory_smooth_;

            bool exists_target_;
            rclcpp::Time previous_time_;
            bool use_smoothing_;
            double smoothing_weight_;
    };

    Tracker::Tracker()
        : Node("tracker"), tfBuffer_(this->get_clock()), tfListener_(tfBuffer_), exists_target_(false), previous_time_(this->now();),
        use_smoothing_(false), smoothing_weight_(0.75)
    {
        pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/track_marker", 1);
        pub_target_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 1);

        sub_true_value_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PointStamped>>(this, "/true_value");
        sub_observed_value_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PointStamped>>(this, "/observed_value");
        sub_observed_value_add_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PointStamped>>(this, "/observed_value_add");

        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
            MySyncPolicy(10), *sub_true_value_, *sub_observed_value_, *sub_observed_value_add_);

        sync_->registerCallback(std::bind(&Tracker::callbackMessage, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        kf_ = std::make_unique<multiple_observation_kalman_filter::KalmanFilter>(0.033, 1000, 1.0);
        target_smooth_ = std::make_unique<geometry_msgs::msg::Point>();

        trajectory_.header.frame_id = "map";
        trajectory_.ns = "trajectory";
        trajectory_.id = 1;
        trajectory_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        trajectory_.scale.x = 0.08;
        trajectory_.color.r = 0.0;
        trajectory_.color.g = 1.0;
        trajectory_.color.b = 0.0;
        trajectory_.color.a = 1.0;

        trajectory_smooth_.header.frame_id = "map";
        trajectory_smooth_.ns = "trajectory_smooth_";
        trajectory_smooth_.id = 1;
        trajectory_smooth_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        trajectory_smooth_.scale.x = 0.08;
        trajectory_smooth_.color.r = 1.0;
        trajectory_smooth_.color.g = 0.84;
        trajectory_smooth_.color.b = 0.0;
        trajectory_smooth_.color.a = 1.0;
    }

    void Tracker::callbackMessage(
        const geometry_msgs::msg::PointStamped::ConstSharedPtr true_value_msg,
        const geometry_msgs::msg::PointStamped::ConstSharedPtr observed_value_msg,
        const geometry_msgs::msg::PointStamped::ConstSharedPtr observed_value_add_msg)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        Eigen::Vector2f observed_value1(observed_value_msg->point.x, observed_value_msg->point.y);
        Eigen::Vector2f observed_value2(observed_value_add_msg->point.x, observed_value_add_msg->point.y);
        Eigen::Vector4f estimated_value(0.0, 0.0, 0.0, 0.0);
        // double dt = this->now().seconds() - previous_time_;
        // previous_time_ = this->now().seconds();
        rclcpp::Time current_time = this->now();
        double dt = (current_time - previous_time_).seconds();
        previous_time_ = current_time;

        geometry_msgs::msg::PoseStamped target;
        if (!exists_target_) {
            kf_->init(observed_value1);
            exists_target_ = true;
            target.pose.position.x = observed_value1[0];
            target.pose.position.y = observed_value1[1];
            *target_smooth_ = target.pose.position;
        } else {
            kf_->compute(dt, observed_value1, observed_value2, &estimated_value);
            target.pose.position.x = estimated_value[0];
            target.pose.position.y = estimated_value[1];
        }
        target.pose.position.z = -0.2;
        target.pose.orientation.w = 1.0;

        trajectory_.points.push_back(target.pose.position);
        if (trajectory_.points.size() > 200)
            trajectory_.points.erase(trajectory_.points.begin());

        marker_array.markers.push_back(trajectory_);
        pub_marker_->publish(marker_array);
        pub_target_->publish(target);
    }

    geometry_msgs::msg::Point Tracker::transformPoint(
        const std::string &org_frame, const std::string &target_frame, const geometry_msgs::msg::Point &point)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        geometry_msgs::msg::PointStamped pt_transformed;
        geometry_msgs::msg::PointStamped pt;
        pt.header.frame_id = org_frame;
        pt.header.stamp = this->now();
        pt.point = point;

        try {
            transform_stamped = tfBuffer_.lookupTransform(target_frame, org_frame, tf2::TimePointZero);
            tf2::doTransform(pt, pt_transformed, transform_stamped);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform failure: %s", ex.what());
        }

        return pt_transformed.point;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<multiple_observation_tracing_simulator::Tracker>());
    rclcpp::shutdown();
    return 0;
}
