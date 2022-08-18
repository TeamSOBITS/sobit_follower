#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
#include <person_following_control/PersonFollowingParameterConfig.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

#include <person_following_control/FollowingPosition.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <person_following_control/virtual_spring_model.hpp>
#include <person_following_control/dynamic_window_approach.hpp>
#include <person_following_control/pid_controller.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<person_following_control::FollowingPosition, nav_msgs::Odometry> MySyncPolicy;

namespace person_following_control {
    enum FollowingMethod {
        VSM_DWA = 0, VSM, DWA
    };

    class PersonFollowing : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_vel_;
            std::unique_ptr<message_filters::Subscriber<person_following_control::FollowingPosition>> sub_following_position_;
            std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> sub_odom_;
            std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
            person_following_control::FollowingPositionPtr following_position_;
            geometry_msgs::TwistPtr current_velocity_;

            std::unique_ptr<person_following_control::VirtualSpringModel> vsm_;
            std::unique_ptr<person_following_control::DynamicWindowApproach> dwa_;
            std::unique_ptr<person_following_control::PIDController> pid_;

            dynamic_reconfigure::Server<person_following_control::PersonFollowingParameterConfig>* server_;
            dynamic_reconfigure::Server<person_following_control::PersonFollowingParameterConfig>::CallbackType f_;

            int following_method_;
            double following_distance_;
            double pre_time_;

            void callbackDynamicReconfigure(person_following_control::PersonFollowingParameterConfig& config, uint32_t level);
            void callbackData (
                const person_following_control::FollowingPositionConstPtr &following_position_msg,
                const nav_msgs::OdometryConstPtr &odom_msg
            );

        public:
            virtual void onInit();

    };
}

void person_following_control::PersonFollowing::callbackDynamicReconfigure(person_following_control::PersonFollowingParameterConfig& config, uint32_t level) {
    vsm_->setFollowParamater( config.following_angle_deg, config.following_distance );
    vsm_->setSpringParamater( config.spring_constant_linear, config.spring_constant_angular );
    vsm_->setFrictionParamater( config.viscous_friction_linear, config.viscous_friction_angular );
    vsm_->setRobotParamater( config.weight_robot, config.radius_robot );
    vsm_->setMomentParamater( config.moment_inertia );
    vsm_->setDisplayFlag( config.display_vsm_path, config.display_vsm_target );

    dwa_->setTargetFrame( config.base_footprint_name );
	dwa_->setStepValue( config.predict_step, config.sampling_time );
	dwa_->setVelocityLimit( config.min_velocity, config.max_velocity, config.min_angle_velocity_deg * M_PI / 180.0, config.max_angle_velocity_deg * M_PI / 180.0, config.velocity_step, config.angle_velocity_step );
	dwa_->setWeight( config.weight_goal, config.weight_obstacle, config.weight_angle, config.weight_velocity, config.weight_vsm_angular, config.weight_vsm_linear );
	dwa_->setCostDistance ( config.obstacle_cost_radius );
	dwa_->setDisplayFlag( config.display_optimal_path, config.display_all_path );
    following_method_ = config.following_method;
    following_distance_ = config.following_distance;
    return;
}

void person_following_control::PersonFollowing::callbackData (
    const person_following_control::FollowingPositionConstPtr &following_position_msg,
    const nav_msgs::OdometryConstPtr &odom_msg) {
    if ( following_position_msg->pose.position.x == 0.0 && following_position_msg->pose.position.y == 0.0 ) return;
    double target_angle = std::atan2(  following_position_msg->pose.position.y,  following_position_msg->pose.position.x );
    double target_distance = std::hypotf( following_position_msg->pose.position.x, following_position_msg->pose.position.y );

    NODELET_INFO("Target                =\t%5.3f [m]\t%5.3f [m]", following_position_msg->pose.position.x, following_position_msg->pose.position.y );

    geometry_msgs::TwistPtr vel ( new  geometry_msgs::Twist );
    if ( following_method_ == FollowingMethod::VSM_DWA ) {
        geometry_msgs::TwistPtr vsm_vel ( new  geometry_msgs::Twist );
        PointCloud::Ptr cloud_obstacles ( new PointCloud() );
        pcl::fromROSMsg<PointT>( following_position_msg->obstacles, *cloud_obstacles );
        vsm_->compute( following_position_msg->pose, odom_msg->twist.twist.linear.x, odom_msg->twist.twist.angular.z, vsm_vel );
        NODELET_INFO("VirtualSpringModel    =\t%5.3f [m/s]\t%5.3f [deg/s]", vsm_vel->linear.x, vsm_vel->angular.z*180/M_PI );
        if ( vsm_vel->linear.x <= 0.0 /*|| std::fabs( target_angle ) > M_PI/2*/ ) {
            pid_->generatePIRotate( pre_time_, odom_msg->twist.twist.angular.z, target_angle, vel );
            NODELET_INFO("PIDController         =\t%5.3f [m/s]\t%5.3f [deg/s]\n", vel->linear.x, vel->angular.z*180/M_PI );
        } else {
            if( target_distance > following_distance_ ) {
                if ( dwa_->generatePath2Target( following_position_msg->pose.position, cloud_obstacles, vsm_vel, vel ) ) {
                    NODELET_INFO("DynamicWindowApproach =\t%5.3f [m/s]\t%5.3f [deg/s]\n", vel->linear.x, vel->angular.z*180/M_PI );
                } else {
                    pid_->generatePIRotate( pre_time_, odom_msg->twist.twist.angular.z, target_angle, vel );
                    NODELET_INFO("PIDController         =\t%5.3f [m/s]\t%5.3f [deg/s]\n", vel->linear.x, vel->angular.z*180/M_PI );
                }
            } else {
                pid_->generatePIRotate( pre_time_, odom_msg->twist.twist.angular.z, target_angle, vel );
                NODELET_INFO("PIDController         =\t%5.3f [m/s]\t%5.3f [deg/s]\n", vel->linear.x, vel->angular.z*180/M_PI );
            }
        }
    } else if ( following_method_ == FollowingMethod::VSM ) {
        vsm_->compute( following_position_msg->pose, odom_msg->twist.twist.linear.x, odom_msg->twist.twist.angular.z, vel );
        NODELET_INFO("VirtualSpringModel    =\t%5.3f [m/s]\t%5.3f [deg/s]\n", vel->linear.x, vel->angular.z*180/M_PI );
    } else if ( following_method_ == FollowingMethod::DWA ) {
        PointCloud::Ptr cloud_obstacles ( new PointCloud() );
        pcl::fromROSMsg<PointT>( following_position_msg->obstacles, *cloud_obstacles );
        if( target_distance > following_distance_ ) {
            dwa_->generatePath2Target( following_position_msg->pose.position, cloud_obstacles, vel );
            NODELET_INFO("DynamicWindowApproach =\t%5.3f [m/s]\t%5.3f [deg/s]\n", vel->linear.x, vel->angular.z*180/M_PI );
        } else {
            pid_->generatePIRotate( pre_time_, odom_msg->twist.twist.angular.z, target_angle, vel );
            NODELET_INFO("PIDController         =\t%5.3f [m/s]\t%5.3f [deg/s]\n", vel->linear.x, vel->angular.z*180/M_PI );
        }
    }
    pub_vel_.publish(vel);
    pre_time_ = ros::Time::now().toSec();
    return;
}

void person_following_control::PersonFollowing::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();
    following_position_.reset( new person_following_control::FollowingPosition );
    current_velocity_.reset( new geometry_msgs::Twist );
    vsm_.reset( new person_following_control::VirtualSpringModel );
    dwa_.reset( new person_following_control::DynamicWindowApproach );
    pid_.reset( new person_following_control::PIDController );

    pub_vel_ = nh_.advertise< geometry_msgs::Twist >( "/cmd_vel_mux/input/teleop", 1 );
    // message_filters :
    sub_following_position_.reset ( new message_filters::Subscriber<person_following_control::FollowingPosition> ( nh_, pnh_.param<std::string>( "following_position_topic_name", "/following_position" ), 1 ) );
    sub_odom_.reset ( new message_filters::Subscriber<nav_msgs::Odometry> ( nh_, pnh_.param<std::string>( "odom_topic_name", "/odom" ), 1 ) );
    sync_.reset ( new message_filters::Synchronizer<MySyncPolicy> ( MySyncPolicy(10), *sub_following_position_, *sub_odom_ ) );
    sync_->registerCallback ( boost::bind( &PersonFollowing::callbackData, this, _1, _2 ) );
    // dynamic_reconfigure :
    server_ = new dynamic_reconfigure::Server<person_following_control::PersonFollowingParameterConfig>(pnh_);
    f_ = boost::bind(&PersonFollowing::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);
    pre_time_ = ros::Time::now().toSec();
}

PLUGINLIB_EXPORT_CLASS(person_following_control::PersonFollowing, nodelet::Nodelet);
