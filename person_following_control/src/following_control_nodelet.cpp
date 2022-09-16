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

#include <multiple_sensor_person_tracking/FollowingPosition.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <person_following_control/virtual_spring_model.hpp>
#include <person_following_control/dynamic_window_approach.hpp>
#include <person_following_control/pid_controller.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<multiple_sensor_person_tracking::FollowingPosition, nav_msgs::Odometry> MySyncPolicy;

namespace person_following_control {
    enum FollowingMethod {
        VSM_DWA = 0, VSM, DWA, PID
    };

    class PersonFollowing : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_vel_;
            std::unique_ptr<message_filters::Subscriber<multiple_sensor_person_tracking::FollowingPosition>> sub_following_position_;
            std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> sub_odom_;
            std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

            std::unique_ptr<person_following_control::VirtualSpringModel> vsm_;
            std::unique_ptr<person_following_control::DynamicWindowApproach> dwa_;
            std::unique_ptr<person_following_control::PIDController> pid_;

            dynamic_reconfigure::Server<person_following_control::PersonFollowingParameterConfig>* server_;
            dynamic_reconfigure::Server<person_following_control::PersonFollowingParameterConfig>::CallbackType f_;

            geometry_msgs::TwistPtr velocity_;
            PointCloud::Ptr cloud_obstacles_;

            int following_method_;
            double following_distance_;
            double pre_time_;
            bool use_pid_;

            void callbackDynamicReconfigure(person_following_control::PersonFollowingParameterConfig& config, uint32_t level);
            void callbackData (
                const multiple_sensor_person_tracking::FollowingPositionConstPtr &following_position_msg,
                const nav_msgs::OdometryConstPtr &odom_msg
            );
            void virtualSpringModelDynamicWindowApproach (
                const multiple_sensor_person_tracking::FollowingPositionConstPtr &following_position_msg,
                const nav_msgs::OdometryConstPtr &odom_msg,
                geometry_msgs::TwistPtr output_path
            );
            void virtualSpringModel (
                const multiple_sensor_person_tracking::FollowingPositionConstPtr &following_position_msg,
                const nav_msgs::OdometryConstPtr &odom_msg,
                geometry_msgs::TwistPtr output_path
            );
            void dynamicWindowApproach (
                const multiple_sensor_person_tracking::FollowingPositionConstPtr &following_position_msg,
                const nav_msgs::OdometryConstPtr &odom_msg,
                geometry_msgs::TwistPtr output_path
            );
            void rotatePID (
                const multiple_sensor_person_tracking::FollowingPositionConstPtr &following_position_msg,
                const nav_msgs::OdometryConstPtr &odom_msg,
                geometry_msgs::TwistPtr output_path
            );

        public:
            virtual void onInit();

    };
}

void person_following_control::PersonFollowing::callbackDynamicReconfigure(person_following_control::PersonFollowingParameterConfig& config, uint32_t level) {
    following_method_ = config.following_method;
    following_distance_ = config.following_distance;

    vsm_->setFollowParamater( config.following_angle_deg, config.following_distance );
    vsm_->setSpringParamater( config.spring_constant_linear, config.spring_constant_angular );
    vsm_->setFrictionParamater( config.viscous_friction_linear, config.viscous_friction_angular );
    vsm_->setRobotParamater( config.weight_robot, config.radius_robot );
    vsm_->setMomentParamater( config.moment_inertia );
    vsm_->setDisplayFlag( config.display_vsm_path, config.display_vsm_target );

    dwa_->setTargetFrame( config.base_footprint_name );
	dwa_->setStepValue( config.predict_step, config.sampling_time );
	dwa_->setVelocityLimit( config.min_linear, config.max_linear, config.min_angular_deg * M_PI / 180.0, config.max_angular_deg * M_PI / 180.0, config.velocity_step, config.angle_velocity_step );
	if ( following_method_ == FollowingMethod::VSM_DWA ) {
        dwa_->setWeight( config.weight_vsm_heading, config.weight_vsm_obstacle, config.weight_velocity, config.weight_vsm_linear, config.weight_vsm_angular );
    } else if ( following_method_ == FollowingMethod::VSM ) {
        dwa_->setWeight( config.weight_heading, config.weight_obstacle, config.weight_velocity, config.weight_vsm_linear, config.weight_vsm_angular );
    }
	dwa_->setCostDistance ( config.obstacle_cost_radius );
	dwa_->setDisplayFlag( config.display_optimal_path, config.display_all_path );

    pid_->setGain( config.p_gain, config.i_gain, config.d_gain );
    pid_->setMaxAngular( config.max_pid_angular_deg * M_PI / 180.0 );

    return;
}

void person_following_control::PersonFollowing::callbackData (
    const multiple_sensor_person_tracking::FollowingPositionConstPtr &following_position_msg,
    const nav_msgs::OdometryConstPtr &odom_msg) {
    //std::cout << "\n====================================" << std::endl;
    if ( following_position_msg->pose.position.x == 0.0 && following_position_msg->pose.position.y == 0.0 ) {
        velocity_->linear.x = 0.0;
        velocity_->angular.z = 0.0;
        pub_vel_.publish(velocity_);
    }
    NODELET_INFO("\033[1mOdom\033[m   = %5.3f [m/s]\t%5.3f [deg/s]", odom_msg->twist.twist.linear.x, odom_msg->twist.twist.angular.z*180/M_PI );

    if ( following_method_ == FollowingMethod::VSM_DWA ) virtualSpringModelDynamicWindowApproach( following_position_msg, odom_msg, velocity_ );
    else if ( following_method_ == FollowingMethod::VSM ) virtualSpringModel( following_position_msg, odom_msg, velocity_ );
    else if ( following_method_ == FollowingMethod::DWA ) dynamicWindowApproach( following_position_msg, odom_msg, velocity_ );
    else if ( following_method_ == FollowingMethod::PID ) rotatePID( following_position_msg, odom_msg, velocity_ );

    // if ( velocity_->linear.x == 0.0 && odom_msg->twist.twist.linear.x >= 0.2 ) {
    //     NODELET_ERROR("Velocity Error");
    // }
    std::cout << "\n" << std::endl;
    pub_vel_.publish(velocity_);
    pre_time_ = ros::Time::now().toSec();
    return;
}

void person_following_control::PersonFollowing::virtualSpringModelDynamicWindowApproach(
    const multiple_sensor_person_tracking::FollowingPositionConstPtr &following_position_msg,
    const nav_msgs::OdometryConstPtr &odom_msg,
    geometry_msgs::TwistPtr output_path)
{
    double target_angle = std::atan2(  following_position_msg->pose.position.y,  following_position_msg->pose.position.x );
    double target_distance = std::hypotf( following_position_msg->pose.position.x, following_position_msg->pose.position.y );

    pcl::fromROSMsg<PointT>( following_position_msg->obstacles, *cloud_obstacles_ );

    vsm_->compute( following_position_msg->pose, odom_msg->twist.twist.linear.x, odom_msg->twist.twist.angular.z, output_path );
    NODELET_INFO("\033[1;33mVSM\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", output_path->linear.x, output_path->angular.z*180/M_PI );

    if ( output_path->linear.x <= 0.0 || target_distance < following_distance_  ) use_pid_ = true;
    if ( use_pid_ ) {
        // if ( odom_msg->twist.twist.linear.x > 0.1 ) {
        //     output_path->angular.z = 0.0;
        //     output_path->linear.x = odom_msg->twist.twist.linear.x * 0.8;
        //     NODELET_INFO("\033[1;34mSTOP\033[m   = %5.3f [m/s]\t%5.3f [deg/s]", output_path->linear.x, output_path->angular.z*180/M_PI );
        // } else {
        //     pid_->generatePIRotate( pre_time_, odom_msg->twist.twist.angular.z, target_angle, output_path );
        //     NODELET_INFO("\033[1;32mPID\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", output_path->linear.x, output_path->angular.z*180/M_PI );
        //     if ( std::fabs( target_angle ) < 0.523599 ) use_pid_ = false;
        // }
        pid_->generatePIRotate( pre_time_, odom_msg->twist.twist.angular.z, target_angle, output_path );
        NODELET_INFO("\033[1;32mPID\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", output_path->linear.x, output_path->angular.z*180/M_PI );
        if ( std::fabs( target_angle ) < 0.523599 ) use_pid_ = false;
        return;
    }

    if ( dwa_->generatePath2TargetVSMDWA( following_position_msg->pose.position, cloud_obstacles_, output_path, output_path ) ) {
    // if ( dwa_->generatePath2Target( following_position_msg->pose.position, cloud_obstacles_, output_path, output_path ) ) {
        NODELET_INFO("\033[1;36mDWA\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", output_path->linear.x, output_path->angular.z*180/M_PI );
    } else {
        pid_->generatePIRotate( pre_time_, odom_msg->twist.twist.angular.z, target_angle, output_path );
        NODELET_INFO("\033[1;32mPID\033[m    = %5.3f [m/s]\t%5.3f [deg/s] : No Path", output_path->linear.x, output_path->angular.z*180/M_PI );
    }

    return;
}

void person_following_control::PersonFollowing::virtualSpringModel (
    const multiple_sensor_person_tracking::FollowingPositionConstPtr &following_position_msg,
    const nav_msgs::OdometryConstPtr &odom_msg,
    geometry_msgs::TwistPtr output_path)
{
    vsm_->compute( following_position_msg->pose, odom_msg->twist.twist.linear.x, odom_msg->twist.twist.angular.z, output_path );
    NODELET_INFO("\033[1;33mVSM\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", output_path->linear.x, output_path->angular.z*180/M_PI );
    return;
}

void person_following_control::PersonFollowing::dynamicWindowApproach (
    const multiple_sensor_person_tracking::FollowingPositionConstPtr &following_position_msg,
    const nav_msgs::OdometryConstPtr &odom_msg,
    geometry_msgs::TwistPtr output_path)
{
    double target_angle = std::atan2(  following_position_msg->pose.position.y,  following_position_msg->pose.position.x );
    double target_distance = std::hypotf( following_position_msg->pose.position.x, following_position_msg->pose.position.y );
    pcl::fromROSMsg<PointT>( following_position_msg->obstacles, *cloud_obstacles_ );

    if ( target_distance < following_distance_  ) use_pid_ = true;
    if ( use_pid_ ) {
        if ( odom_msg->twist.twist.linear.x > 0.1 ) {
            output_path->angular.z = 0.0;
            output_path->linear.x = odom_msg->twist.twist.linear.x * 0.5;
            NODELET_INFO("\033[1;34mSTOP\033[m   = %5.3f [m/s]\t%5.3f [deg/s]", output_path->linear.x, output_path->angular.z*180/M_PI );
        } else {
            pid_->generatePIRotate( pre_time_, odom_msg->twist.twist.angular.z, target_angle, output_path );
            NODELET_INFO("\033[1;32mPID\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", output_path->linear.x, output_path->angular.z*180/M_PI );
            if ( std::fabs( target_angle ) < 0.523599 ) use_pid_ = false;
        }
        return;
    }

    if( dwa_->generatePath2TargetDWA( following_position_msg->pose.position, cloud_obstacles_, output_path ) ) {
        NODELET_INFO("\033[1;36mDWA\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", output_path->linear.x, output_path->angular.z*180/M_PI );
    } else {
        pid_->generatePIRotate( pre_time_, odom_msg->twist.twist.angular.z, target_angle, output_path );
        NODELET_INFO("\033[1;32mPID\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", output_path->linear.x, output_path->angular.z*180/M_PI );
    }
    return;
}

void person_following_control::PersonFollowing::rotatePID (
    const multiple_sensor_person_tracking::FollowingPositionConstPtr &following_position_msg,
    const nav_msgs::OdometryConstPtr &odom_msg,
    geometry_msgs::TwistPtr output_path)
{
    double target_angle = std::atan2(  following_position_msg->pose.position.y,  following_position_msg->pose.position.x );
    pid_->generatePIRotate( pre_time_, odom_msg->twist.twist.angular.z, target_angle, output_path );
    NODELET_INFO("\033[1;32mPID\033[m    = %5.3f [m/s]\t%5.3f [deg/s]", output_path->linear.x, output_path->angular.z*180/M_PI );
    return;
}

void person_following_control::PersonFollowing::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();
    vsm_.reset( new person_following_control::VirtualSpringModel );
    dwa_.reset( new person_following_control::DynamicWindowApproach );
    pid_.reset( new person_following_control::PIDController );
    velocity_.reset( new geometry_msgs::Twist );
    cloud_obstacles_.reset( new PointCloud() );

    pub_vel_ = nh_.advertise< geometry_msgs::Twist >( "cmd_vel", 1 );
    // message_filters :
    sub_following_position_.reset ( new message_filters::Subscriber<multiple_sensor_person_tracking::FollowingPosition> ( nh_, pnh_.param<std::string>( "following_position_topic_name", "/following_position" ), 1 ) );
    sub_odom_.reset ( new message_filters::Subscriber<nav_msgs::Odometry> ( nh_, pnh_.param<std::string>( "odom_topic_name", "/odom" ), 1 ) );
    sync_.reset ( new message_filters::Synchronizer<MySyncPolicy> ( MySyncPolicy(10), *sub_following_position_, *sub_odom_ ) );
    sync_->registerCallback ( boost::bind( &PersonFollowing::callbackData, this, _1, _2 ) );
    // dynamic_reconfigure :
    server_ = new dynamic_reconfigure::Server<person_following_control::PersonFollowingParameterConfig>(pnh_);
    f_ = boost::bind(&PersonFollowing::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);
    use_pid_ = false;
    pre_time_ = ros::Time::now().toSec();
}

PLUGINLIB_EXPORT_CLASS(person_following_control::PersonFollowing, nodelet::Nodelet);
