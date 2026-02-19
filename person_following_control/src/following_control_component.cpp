#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "multiple_sensor_person_tracking/msg/following_position.hpp"
#include "person_following_control/virtual_spring_model.hpp"
#include "person_following_control/dynamic_window_approach.hpp"
#include "person_following_control/pid_controller.hpp"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
// typedef message_filters::sync_policies::ApproximateTime<multiple_sensor_person_tracking::msg::FollowingPosition, nav_msgs::msg::Odometry> MySyncPolicy;

namespace person_following_control {
    enum FollowingMethod {
        VSM_DWA = 0, VSM, DWA, PID
    };

    class PersonFollowing : public rclcpp::Node {
        private:

            rclcpp::TimerBase::SharedPtr init_timer_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_obstacles_;
            rclcpp::Subscription<multiple_sensor_person_tracking::msg::FollowingPosition>::SharedPtr sub_following_position_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

            std::unique_ptr<person_following_control::VirtualSpringModel> vsm_;
            std::unique_ptr<person_following_control::DynamicWindowApproach> dwa_;
            person_following_control::PIDController pid_;

            geometry_msgs::msg::Twist velocity_;
            PointCloud::Ptr cloud_obstacles_;
            sensor_msgs::msg::PointCloud2::ConstSharedPtr obstacles_msg_;
            multiple_sensor_person_tracking::msg::FollowingPosition::ConstSharedPtr following_position_msg_;
            nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_;


            int following_method_;
            double following_distance_;
            std::string  command_velocity_topic_name_;
            std::string obstacles_topic_name_;
            std::string following_position_topic_name_;
            std::string odom_topic_name_;
            rclcpp::Time pre_time_;
            bool use_pid_;

            void loadParametersFromServer( );
            void callbackData (
                const multiple_sensor_person_tracking::msg::FollowingPosition::ConstSharedPtr &following_position_msg
            );
            void virtualSpringModelDynamicWindowApproach ();
            void virtualSpringModel ();
            void dynamicWindowApproach ();
            void rotatePID ();
            void obstacles_callback(
                const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &obstacles_msg 
            );
            void odom_callback(
                const std::shared_ptr<const nav_msgs::msg::Odometry> &odom_msg
            );

        public:
            explicit PersonFollowing(const rclcpp::NodeOptions & options)
            : Node("person_following_control", options)
            {
                auto timer_callback = [this]() -> void {
                    this->onInit();
                    init_timer_->cancel();
                };
            
                init_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
            }

            void onInit();
    };
}

void person_following_control::PersonFollowing::loadParametersFromServer() {

    following_method_ = this->get_parameter("following_method").as_int();
    following_distance_ = this->get_parameter("following_distance").as_double();
    command_velocity_topic_name_ = this->get_parameter("command_velocity_topic_name").as_string();
    obstacles_topic_name_ = this->get_parameter("obstacles_topic_name").as_string();
    following_position_topic_name_ = this->get_parameter("following_position_topic_name").as_string();
    odom_topic_name_ = this->get_parameter("odom_topic_name").as_string();

    vsm_->setFollowParamater(
        this->get_parameter("following_angle_deg").as_double(),
        following_distance_ - 0.4
    );
    vsm_->setSpringParamater(
        this->get_parameter("spring_constant_linear").as_double(),
        this->get_parameter("spring_constant_angular").as_double()
    );
    vsm_->setFrictionParamater(
        this->get_parameter("viscous_friction_linear").as_double(),
        this->get_parameter("viscous_friction_angular").as_double()
    );
    vsm_->setRobotParamater(
        this->get_parameter("weight_robot").as_double(),
        this->get_parameter("radius_robot").as_double()
    );
    vsm_->setMomentParamater(
        this->get_parameter("moment_inertia").as_double()
    );
    vsm_->setDisplayFlag(
        this->get_parameter("display_vsm_path").as_bool(),
        this->get_parameter("display_vsm_target").as_bool()
    );

    dwa_->setTargetFrame(this->get_parameter("base_footprint_name").as_string());
    dwa_->setStepValue(
        this->get_parameter("predict_step").as_int(),
        this->get_parameter("sampling_time").as_double()
    );
    dwa_->setVelocityLimit(
        this->get_parameter("min_linear").as_double(),
        this->get_parameter("max_linear").as_double(),
        this->get_parameter("min_angular_deg").as_double() * M_PI / 180.0,
        this->get_parameter("max_angular_deg").as_double() * M_PI / 180.0,
        this->get_parameter("velocity_step").as_double(),
        this->get_parameter("angle_velocity_step").as_double()
    );
    if (following_method_ == FollowingMethod::VSM_DWA) {
        dwa_->setWeight(
            this->get_parameter("weight_vsm_heading").as_double(),
            this->get_parameter("weight_vsm_obstacle").as_double(),
            this->get_parameter("weight_velocity").as_double(),
            this->get_parameter("weight_vsm_linear").as_double(),
            this->get_parameter("weight_vsm_angular").as_double()
        );
    } else if (following_method_ == FollowingMethod::VSM) {
        dwa_->setWeight(
            this->get_parameter("weight_heading").as_double(),
            this->get_parameter("weight_obstacle").as_double(),
            this->get_parameter("weight_velocity").as_double(),
            this->get_parameter("weight_vsm_linear").as_double(),
            this->get_parameter("weight_vsm_angular").as_double()
        );
    }
    dwa_->setCostDistance(this->get_parameter("obstacle_cost_radius").as_double());
    dwa_->setDisplayFlag(
        this->get_parameter("display_optimal_path").as_bool(),
        this->get_parameter("display_all_path").as_bool()
    );

    pid_.setGain(
        this->get_parameter("p_gain").as_double(),
        this->get_parameter("i_gain").as_double(),
        this->get_parameter("d_gain").as_double()
    );
    pid_.setMaxAngular(
        this->get_parameter("max_pid_angular_deg").as_double() * M_PI / 180.0
    );
}


void person_following_control::PersonFollowing::callbackData (
    const multiple_sensor_person_tracking::msg::FollowingPosition::ConstSharedPtr &following_position_msg
) {

    following_position_msg_ = following_position_msg;

    // Guard against async startup ordering: odom/obstacles may not be received yet.
    if (!odom_msg_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Waiting for odometry message before computing following control.");
        return;
    }
    if (!obstacles_msg_ &&
        (following_method_ == FollowingMethod::VSM_DWA || following_method_ == FollowingMethod::DWA)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Waiting for obstacles message before computing DWA-based control.");
        return;
    }

    if ( following_position_msg_->pose.position.x == 0.0 && following_position_msg_->pose.position.y == 0.0 ) {
        velocity_.linear.x = 0.0;
        velocity_.angular.z = 0.0;
        pub_vel_->publish(velocity_);
        return;
    }
    RCLCPP_INFO( this->get_logger(), "\033[1mOdom\033[m   = %5.3f [m/s]\t%5.3f [deg/s]", odom_msg_->twist.twist.linear.x, odom_msg_->twist.twist.angular.z*180/M_PI );

    if ( following_method_ == FollowingMethod::VSM_DWA ) virtualSpringModelDynamicWindowApproach( );
    else if ( following_method_ == FollowingMethod::VSM ) virtualSpringModel( );
    else if ( following_method_ == FollowingMethod::DWA ) dynamicWindowApproach( );
    else if ( following_method_ == FollowingMethod::PID ) rotatePID( );

    std::cout << "\n" << std::endl;
    pub_vel_->publish(velocity_);
    pre_time_ = this->get_clock()->now();
    return;
}

void person_following_control::PersonFollowing::virtualSpringModelDynamicWindowApproach()
{
    double target_angle = std::atan2(  following_position_msg_->pose.position.y,  following_position_msg_->pose.position.x );
    double target_distance = std::hypotf( following_position_msg_->pose.position.x, following_position_msg_->pose.position.y );

    pcl::fromROSMsg<PointT>( *obstacles_msg_, *cloud_obstacles_ );

    vsm_->compute( following_position_msg_->pose, odom_msg_->twist.twist.linear.x, odom_msg_->twist.twist.angular.z, velocity_ );
    RCLCPP_INFO( this->get_logger(), "\033[1;33mVSM\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", velocity_.linear.x, velocity_.angular.z*180/M_PI );

    if ( velocity_.linear.x <= 0.0 || target_distance < following_distance_  ) {
        if ( std::fabs( target_angle ) < 0.174533 ) {
            velocity_.linear.x = 0.0;
            velocity_.angular.z = 0.0;
            RCLCPP_INFO( this->get_logger(), "\033[1;34mSTOP\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", velocity_.linear.x, velocity_.angular.z*180/M_PI );
        } else {
            pid_.generatePIRotate( pre_time_ - this->get_clock()->now(), odom_msg_->twist.twist.angular.z, target_angle, velocity_ );
            RCLCPP_INFO( this->get_logger(), "\033[1;32mPID\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", velocity_.linear.x, velocity_.angular.z*180/M_PI );
        }
    }

    if ( dwa_->generatePath2TargetVSMDWA( following_position_msg_->pose.position, cloud_obstacles_, velocity_ ) ) {
        RCLCPP_INFO( this->get_logger(), "\033[1;36mDWA\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", velocity_.linear.x, velocity_.angular.z*180/M_PI );
    } else {
        if ( std::fabs( target_angle ) < 0.174533 ) {
            velocity_.linear.x = 0.0;
            velocity_.angular.z = 0.0;
            RCLCPP_INFO( this->get_logger(), "\033[1;34mSTOP\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", velocity_.linear.x, velocity_.angular.z*180/M_PI );
        } else {
            pid_.generatePIRotate( pre_time_ - this->get_clock()->now(), odom_msg_->twist.twist.angular.z, target_angle, velocity_ );
            RCLCPP_INFO( this->get_logger(), "\033[1;32mPID\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", velocity_.linear.x, velocity_.angular.z*180/M_PI );
        }
    }
    return;
}

void person_following_control::PersonFollowing::virtualSpringModel ()
{
    vsm_->compute( following_position_msg_->pose, odom_msg_->twist.twist.linear.x, odom_msg_->twist.twist.angular.z, velocity_ );
    RCLCPP_INFO( this->get_logger(), "\033[1;33mVSM\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", velocity_.linear.x, velocity_.angular.z*180/M_PI );
    return;
}

void person_following_control::PersonFollowing::dynamicWindowApproach ()
{
    double target_angle = std::atan2(  following_position_msg_->pose.position.y,  following_position_msg_->pose.position.x );
    double target_distance = std::hypotf( following_position_msg_->pose.position.x, following_position_msg_->pose.position.y );
    
    pcl::fromROSMsg<PointT>( *obstacles_msg_, *cloud_obstacles_ );

    if ( target_distance < following_distance_  ) use_pid_ = true;
    if ( use_pid_ ) {
        if ( odom_msg_->twist.twist.linear.x > 0.1 ) {
            velocity_.angular.z = 0.0;
            velocity_.linear.x = odom_msg_->twist.twist.linear.x * 0.5;
            RCLCPP_INFO( this->get_logger(), "\033[1;34mSTOP\033[m   = %5.3f [m/s]\t%5.3f [deg/s]", velocity_.linear.x, velocity_.angular.z*180/M_PI );
        } else {
            pid_.generatePIRotate( pre_time_ - this->get_clock()->now(), odom_msg_->twist.twist.angular.z, target_angle, velocity_ );
            RCLCPP_INFO( this->get_logger(), "\033[1;32mPID\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", velocity_.linear.x, velocity_.angular.z*180/M_PI );
            if ( std::fabs( target_angle ) < 0.174533 ) use_pid_ = false;
        }
        return;
    }

    if( dwa_->generatePath2TargetDWA( following_position_msg_->pose.position, cloud_obstacles_, velocity_ ) ) {
        RCLCPP_INFO( this->get_logger(), "\033[1;36mDWA\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", velocity_.linear.x, velocity_.angular.z*180/M_PI );
    } else {
        pid_.generatePIRotate( pre_time_ - this->get_clock()->now(), odom_msg_->twist.twist.angular.z, target_angle, velocity_ );
        RCLCPP_INFO( this->get_logger(), "\033[1;32mPID\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", velocity_.linear.x, velocity_.angular.z*180/M_PI );
    }
    return;
}

void person_following_control::PersonFollowing::rotatePID ()
{
    double target_angle = std::atan2(  following_position_msg_->pose.position.y,  following_position_msg_->pose.position.x );
    pid_.generatePIRotate( pre_time_ - this->get_clock()->now(), odom_msg_->twist.twist.angular.z, target_angle, velocity_ );
    RCLCPP_INFO( this->get_logger(), "\033[1;32mPID\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", velocity_.linear.x, velocity_.angular.z*180/M_PI );
    return;
}

void person_following_control::PersonFollowing::obstacles_callback (const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &obstacles_msg)
{
    obstacles_msg_ = obstacles_msg;
}

void person_following_control::PersonFollowing::odom_callback (const std::shared_ptr<const nav_msgs::msg::Odometry> &odom_msg)
{
    odom_msg_ = odom_msg;
    if (following_position_msg_) {
        callbackData(following_position_msg_);
    }
}

void person_following_control::PersonFollowing::onInit() {

    // Declare parameters
    this->declare_parameter<std::string>("command_velocity_topic_name", "/commands/velocity");
    this->declare_parameter<std::string>("obstacles_topic_name", "obstacles");
    this->declare_parameter<std::string>("following_position_topic_name", "following_position");
    this->declare_parameter<std::string>("odom_topic_name", "/odom");
    this->declare_parameter<int>("following_method", FollowingMethod::VSM_DWA);
    this->declare_parameter<double>("following_distance", 1.0);

    // VSM
    this->declare_parameter<std::string>("base_footprint_name", "base_footprint");
    this->declare_parameter<double>("following_angle_deg", 0.0);
    this->declare_parameter<double>("spring_constant_linear", 30.0);
    this->declare_parameter<double>("spring_constant_angular", 1.0);
    this->declare_parameter<double>("weight_robot", 30.0);
    this->declare_parameter<double>("moment_inertia", 15.0);
    this->declare_parameter<double>("viscous_friction_linear", 1.0);
    this->declare_parameter<double>("viscous_friction_angular", 1.0);
    this->declare_parameter<double>("radius_robot", 0.3);
    this->declare_parameter<bool>("display_vsm_path", true);
    this->declare_parameter<bool>("display_vsm_target", false);

    // DWA
    this->declare_parameter<double>("min_linear", 0.10);
    this->declare_parameter<double>("max_linear", 0.80);
    this->declare_parameter<double>("min_angular_deg", -150.0);
    this->declare_parameter<double>("max_angular_deg", 150.0);
    this->declare_parameter<int>("predict_step", 10);
    this->declare_parameter<double>("sampling_time", 0.2);
    this->declare_parameter<double>("velocity_step", 7.0);
    this->declare_parameter<double>("angle_velocity_step", 30.0);
    this->declare_parameter<double>("weight_heading", 3.0);
    this->declare_parameter<double>("weight_obstacle", 1.0);
    this->declare_parameter<double>("weight_velocity", 1.5);
    this->declare_parameter<double>("weight_vsm_heading", 3.0);
    this->declare_parameter<double>("weight_vsm_obstacle", 1.2);
    this->declare_parameter<double>("weight_vsm_linear", 2.0);
    this->declare_parameter<double>("weight_vsm_angular", 0.5);
    this->declare_parameter<double>("obstacle_cost_radius", 0.35);
    this->declare_parameter<bool>("display_optimal_path", true);
    this->declare_parameter<bool>("display_all_path", true);

    // PID
    this->declare_parameter<double>("p_gain", 1.2);
    this->declare_parameter<double>("i_gain", 0.6);
    this->declare_parameter<double>("d_gain", 0.0);
    this->declare_parameter<double>("max_pid_angular_deg", 70.0);

    // Instantiate class members
    vsm_ = std::make_unique<person_following_control::VirtualSpringModel>(this);
    dwa_ = std::make_unique<person_following_control::DynamicWindowApproach>(this);
    // pid_ = std::make_unique<person_following_control::PIDController>();
    // velocity_ = std::make_shared<geometry_msgs::msg::Twist>();
    cloud_obstacles_ = std::make_shared<PointCloud>();

    // Load parameters
    loadParametersFromServer();

    // Publisher initialization
    pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(command_velocity_topic_name_, 10);

    // Subscriber initialization
    sub_obstacles_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        obstacles_topic_name_, 10,
        std::bind(&PersonFollowing::obstacles_callback, this, std::placeholders::_1)
    );
    sub_following_position_ = this->create_subscription<multiple_sensor_person_tracking::msg::FollowingPosition>(
        following_position_topic_name_, 10,
        std::bind(&PersonFollowing::callbackData, this, std::placeholders::_1)
    );
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name_, 10,
        std::bind(&PersonFollowing::odom_callback, this, std::placeholders::_1)
    );

    // Initialize PID usage flag and previous time
    use_pid_ = false;
    pre_time_ = this->get_clock()->now();
}

RCLCPP_COMPONENTS_REGISTER_NODE(person_following_control::PersonFollowing)