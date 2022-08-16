#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <multiple_observation_kalman_filter/multiple_observation_kalman_filter.hpp>
#include <dynamic_reconfigure/server.h>
#include <multiple_observation_tracing_simulator/TrackerParameterConfig.h>

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, geometry_msgs::PointStamped, geometry_msgs::PointStamped> MySyncPolicy;

namespace multiple_observation_tracing_simulator {
    class Tracker {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_marker_;
            ros::Publisher pub_target_;
            std::unique_ptr<message_filters::Subscriber<geometry_msgs::PointStamped>> sub_true_value_;
            std::unique_ptr<message_filters::Subscriber<geometry_msgs::PointStamped>> sub_observed_value_;
            std::unique_ptr<message_filters::Subscriber<geometry_msgs::PointStamped>> sub_observed_value_add_;
            std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
            dynamic_reconfigure::Server<multiple_observation_tracing_simulator::TrackerParameterConfig>* server_;
            dynamic_reconfigure::Server<multiple_observation_tracing_simulator::TrackerParameterConfig>::CallbackType f_;
            geometry_msgs::PointPtr target_smooth_;
            tf::TransformListener tf_listener_;
            std::unique_ptr<multiple_observation_kalman_filter::KalmanFilter> kf_;
            visualization_msgs::Marker trajectory_;
            visualization_msgs::Marker trajectory_smooth_;
            bool exists_target_;
            double previous_time_;
            bool use_smoothing_;
            double smoothing_weight_;

            double error_min_;
            double error_max_;
            std::vector<double> errors_;

            double error_smooth_min_;
            double error_smooth_max_;
            std::vector<double> errors_smooth_;

            void callbackDynamicReconfigure(multiple_observation_tracing_simulator::TrackerParameterConfig& config, uint32_t level);
            void callbackDynamicReconfigure2(multiple_observation_tracing_simulator::TrackerParameterConfig& config, uint32_t level);
            geometry_msgs::Point transformPoint ( std::string org_frame, std::string target_frame, geometry_msgs::Point point );
            void callbackMessage(
                const geometry_msgs::PointStampedConstPtr &true_value_msg,
                const geometry_msgs::PointStampedConstPtr &observed_value_msg,
                const geometry_msgs::PointStampedConstPtr &observed_value_add_msg
            );

        public:
            Tracker( );
    };
}

void multiple_observation_tracing_simulator::Tracker::callbackDynamicReconfigure(multiple_observation_tracing_simulator::TrackerParameterConfig& config, uint32_t level) {
    kf_->changeParameter( config.process_noise, config.system_noise );
    use_smoothing_ = config.use_smoothing;
    smoothing_weight_ = config.smoothing_weight;
    error_min_ = 10.0;
    error_max_ = 0.0;
    errors_.clear();
    error_smooth_min_ = 10.0;
    error_smooth_max_ = 0.0;
    errors_smooth_.clear();
}

geometry_msgs::Point multiple_observation_tracing_simulator::Tracker::transformPoint ( std::string org_frame, std::string target_frame, geometry_msgs::Point point ) {
    geometry_msgs::PointStamped pt_transformed;
    geometry_msgs::PointStamped pt;
    pt.header.frame_id = org_frame;
    pt.header.stamp = ros::Time(0);
    pt.point = point;
    if ( tf_listener_.frameExists( target_frame ) ) {
        try {
            tf_listener_.transformPoint( target_frame, pt, pt_transformed );
        } catch ( const tf::TransformException& ex ) {
            ROS_ERROR( "%s",ex.what( ) );
        }
    } else {
        ROS_ERROR("target_frame is not Exists");
    }
    return pt_transformed.point;
}


void multiple_observation_tracing_simulator::Tracker::callbackMessage(
        const geometry_msgs::PointStampedConstPtr &true_value_msg,
        const geometry_msgs::PointStampedConstPtr &observed_value_msg,
        const geometry_msgs::PointStampedConstPtr &observed_value_add_msg )
{
    visualization_msgs::MarkerArray marker_array;
    Eigen::Vector2f observed_value1( observed_value_msg->point.x, observed_value_msg->point.y );
    Eigen::Vector2f observed_value2( observed_value_add_msg->point.x, observed_value_add_msg->point.y );
    Eigen::Vector4f estimated_value( 0.0, 0.0, 0.0, 0.0 );
    double dt = ( observed_value_msg->header.stamp.toSec()  - previous_time_ );	//dt - expressed in seconds
    previous_time_ = observed_value_msg->header.stamp.toSec();
    geometry_msgs::PoseStamped target;
    geometry_msgs::Point target_odom, target_odom_smooth;
    if ( !exists_target_ ) {
        kf_->init( observed_value1 );
        exists_target_ = true;
        target.pose.position.x = observed_value1[0];
        target.pose.position.y = observed_value1[1];
        *target_smooth_ = target.pose.position;
    } else {
        kf_->compute( dt, observed_value1, observed_value2, &estimated_value );
        target.pose.position.x = estimated_value[0];
        target.pose.position.y = estimated_value[1];
    }
    target.pose.position.z = -0.2;
    target.pose.orientation.w = 1.0;
    target_odom = transformPoint ( "robot", "map", target.pose.position );
    trajectory_.points.push_back( target_odom );
    if ( trajectory_.points.size() > 200 ) trajectory_.points.erase(trajectory_.points.begin());

    double error = std::hypotf( target.pose.position.x - true_value_msg->point.x, target.pose.position.y - true_value_msg->point.y );
    double error_ave = 0.0;
    errors_.push_back( error );
    if ( errors_.size() > 200 ) errors_.erase(errors_.begin());
    error_max_ = ( error_max_ < error ) ? error : error_max_;
    error_min_ = ( error_min_ > error ) ? error : error_min_;
    for ( const auto& er : errors_ ) error_ave += er;
    error_ave /= 1.0 * errors_.size();
    double error_smooth_ave = 0.0;
    if ( use_smoothing_ ) {
        target_smooth_->x = smoothing_weight_ * target_smooth_->x + ( 1.0 - smoothing_weight_ ) * target.pose.position.x;
        target_smooth_->y = smoothing_weight_ * target_smooth_->y + ( 1.0 - smoothing_weight_ ) * target.pose.position.y;
        if ( errors_smooth_.size() == 0 ) *target_smooth_ = target.pose.position;
        target_smooth_->z = -0.1;

        target_odom_smooth = transformPoint ( "robot", "map", *target_smooth_ );
        trajectory_smooth_.points.push_back( target_odom_smooth );
        if ( trajectory_smooth_.points.size() > 200 ) trajectory_smooth_.points.erase(trajectory_smooth_.points.begin());

        double error_smooth = std::hypotf( target_smooth_->x - true_value_msg->point.x, target_smooth_->y - true_value_msg->point.y );
        errors_smooth_.push_back( error );
        if ( errors_smooth_.size() > 200 ) errors_smooth_.erase(errors_smooth_.begin());
        error_smooth_max_ = ( error_smooth_max_ < error_smooth ) ? error_smooth : error_smooth_max_;
        error_smooth_min_ = ( error_smooth_min_ > error_smooth ) ? error_smooth : error_smooth_min_;
        for ( const auto& er : errors_smooth_ ) error_smooth_ave += er;
        error_smooth_ave /= 1.0 * errors_.size();
    }

    ROS_INFO("[ dt ]                        %8.3f [sec]", dt );
    ROS_INFO("[ true_value ]                x = %8.3f [m],\ty = %8.3f [m]", true_value_msg->point.x, true_value_msg->point.y );
    ROS_INFO("[ observed_value ]            x = %8.3f [m],\ty = %8.3f [m]", observed_value_msg->point.x, observed_value_msg->point.y );
    ROS_INFO("[ observed_value_add ]        x = %8.3f [m],\ty = %8.3f [m]", observed_value_add_msg->point.x, observed_value_add_msg->point.y );
    ROS_INFO("[ estimated_value]            x = %8.3f [m],\ty = %8.3f [m], \tv_x = %8.3f [m/s],\tv_y = %8.3f [m/s]", estimated_value[0], estimated_value[1], estimated_value[2], estimated_value[3] );
    if ( use_smoothing_ ) ROS_INFO("[ estimated_value(smooth)]    x = %8.3f [m],\ty = %8.3f [m], \t( k = %8.3f )", target_smooth_->x, target_smooth_->y, smoothing_weight_ );
    if ( use_smoothing_ )  {
        ROS_INFO("[ error ]                     ave = %8.3f [m],\tmin = %8.3f [m],\tmax = %8.3f [m]", error_ave, error_min_, error_max_ );
        ROS_INFO("[ error(smooth) ]             ave = %8.3f [m],\tmin = %8.3f [m],\tmax = %8.3f [m]\n", error_smooth_ave, error_smooth_min_, error_smooth_max_ );
    } else {
        ROS_INFO("[ error ]                     ave = %8.3f [m],\tmin = %8.3f [m],\tmax = %8.3f [m]\n", error_ave, error_min_, error_max_ );
    }


    trajectory_.header.stamp = trajectory_smooth_.header.stamp = ros::Time::now();
    target.header.stamp = ros::Time::now(); target.header.frame_id = "robot";
    marker_array.markers.push_back( trajectory_ );
    if ( use_smoothing_ ) marker_array.markers.push_back( trajectory_smooth_ );
    pub_marker_.publish ( marker_array );

    pub_target_.publish ( target );
    return;
}

multiple_observation_tracing_simulator::Tracker::Tracker( ) : nh_(), pnh_("~") {
    pub_marker_ = nh_.advertise< visualization_msgs::MarkerArray >( "/track_marker", 1 );
    pub_target_ = nh_.advertise< geometry_msgs::PoseStamped >( "/target_pose", 1 ); ;
    sub_true_value_ .reset ( new message_filters::Subscriber<geometry_msgs::PointStamped> ( nh_, "/true_value", 1 ) );
    sub_observed_value_ .reset ( new message_filters::Subscriber<geometry_msgs::PointStamped> ( nh_, "/observed_value", 1 ) );
    sub_observed_value_add_ .reset ( new message_filters::Subscriber<geometry_msgs::PointStamped> ( nh_, "/observed_value_add", 1 ) );
    sync_ .reset ( new message_filters::Synchronizer<MySyncPolicy> ( MySyncPolicy(10), *sub_true_value_, *sub_observed_value_, *sub_observed_value_add_ ) );
    sync_ ->registerCallback ( boost::bind( &Tracker::callbackMessage, this, _1, _2, _3 ) );
    kf_.reset( new multiple_observation_kalman_filter::KalmanFilter( 0.033, 1000, 1.0 ) );
    target_smooth_.reset( new geometry_msgs::Point );
    exists_target_ = false;

    server_ = new dynamic_reconfigure::Server<multiple_observation_tracing_simulator::TrackerParameterConfig>(pnh_);
    f_ = boost::bind(&multiple_observation_tracing_simulator::Tracker::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);

    trajectory_.header.frame_id = "map";
    trajectory_.header.stamp = ros::Time::now();
    trajectory_.ns = "trajectory";
    trajectory_.id =  1;
    trajectory_.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_.action = visualization_msgs::Marker::ADD;
    trajectory_.scale.x = 0.08;
    trajectory_.color.r = 0.0; trajectory_.color.g = 1.0; trajectory_.color.b = 0.0; trajectory_.color.a = 1.0;
    trajectory_.pose.orientation.w = 1.0;
    trajectory_.lifetime = ros::Duration(1.0);

    trajectory_smooth_.header.frame_id = "map";
    trajectory_smooth_.header.stamp = ros::Time::now();
    trajectory_smooth_.ns = "trajectory_smooth_";
    trajectory_smooth_.id =  1;
    trajectory_smooth_.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_smooth_.action = visualization_msgs::Marker::ADD;
    trajectory_smooth_.scale.x = 0.08;
    trajectory_smooth_.color.r = 1.0; trajectory_smooth_.color.g = 0.84; trajectory_smooth_.color.b = 0.0; trajectory_smooth_.color.a = 1.0;
    trajectory_smooth_.pose.orientation.w = 1.0;
    trajectory_smooth_.lifetime = ros::Duration(1.0);

    previous_time_ = 0.0;
    smoothing_weight_ = 0.95;
    error_min_ = 10.0;
    error_max_ = 0.0;
    errors_.clear();
    error_smooth_min_ = 10.0;
    error_smooth_max_ = 0.0;
    errors_smooth_.clear();
}

int main(int argc, char *argv[])  {
    ros::init(argc, argv, "tracker");
    multiple_observation_tracing_simulator::Tracker tracker;
    ros::spin();
    return 0;
}
