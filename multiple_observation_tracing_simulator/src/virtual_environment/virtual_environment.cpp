#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace multiple_observation_tracing_simulator{
    class VirtualEnvironment : public rclcpp::Node {
        private :
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

            geometry_msgs::msg::Point tgt_pt_;
            double tgt_theta_;

            void callbackTarget ( const geometry_msgs::msg::Twist::SharedPtr msg );

        public :
            VirtualEnvironment ( );
            void publishData (  );
    };

    VirtualEnvironment::VirtualEnvironment ( ) : Node( "virtual_environment" ), tgt_theta_( 0.0 ) {
        tgt_pt_.x = 0.0;
        tgt_pt_.y = 0.0;

        pub_marker_ = this->create_publisher< visualization_msgs::msg::MarkerArray >( "/target_marker", 1 );
        sub_teleop_ = this->create_subscription< geometry_msgs::msg::Twist >("/target/teleop", 10, std::bind(&VirtualEnvironment::callbackTarget, this, std::placeholders::_1));
        
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    void VirtualEnvironment::callbackTarget ( const geometry_msgs::msg::Twist::SharedPtr msg ) {
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

    void VirtualEnvironment::publishData (  ) {
        
        visualization_msgs::msg::Marker trajectory;
        trajectory.header.frame_id = "map";
        trajectory.header.stamp = this->now();
        trajectory.ns = "trajectory";
        trajectory.id = 1;
        trajectory.type = visualization_msgs::msg::Marker::LINE_STRIP;
        trajectory.action = visualization_msgs::msg::Marker::ADD;
        trajectory.scale.x = 0.2;
        trajectory.color.r = 1.0; trajectory.color.g = 0.0; trajectory.color.b = 0.0; trajectory.color.a = 1.0;
        trajectory.pose.orientation.w = 1.0;

        rclcpp::Rate rate(30);

        while (rclcpp::ok()) {
            // Broadcast target transform
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = this->now();
            transform_stamped.header.frame_id = "map";
            transform_stamped.child_frame_id = "target";
            transform_stamped.transform.translation.x = tgt_pt_.x;
            transform_stamped.transform.translation.y = tgt_pt_.y;
            transform_stamped.transform.translation.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, tgt_theta_);
            transform_stamped.transform.rotation = tf2::toMsg(q);
            tf_broadcaster_->sendTransform(transform_stamped);
    
            // Update trajectory marker
            geometry_msgs::msg::Point point;
            point.x = tgt_pt_.x;
            point.y = tgt_pt_.y;
            point.z = -0.3;
            trajectory.points.push_back(point);
            
            trajectory.header.stamp = this->now();
            trajectory.id = 1;
            if (trajectory.points.size() > 200) trajectory.points.erase(trajectory.points.begin());
    
            visualization_msgs::msg::MarkerArray marker_array;
            marker_array.markers.push_back(trajectory);
            pub_marker_->publish(marker_array);
    
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto virtual_environment_node = std::make_shared<multiple_observation_tracing_simulator::VirtualEnvironment>();
    virtual_environment_node->publishData();
    rclcpp::shutdown();
    return 0;
}