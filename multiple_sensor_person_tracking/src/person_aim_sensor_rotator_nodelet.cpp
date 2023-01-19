#include <ros/ros.h>

// パンチルト回転機構上のRGB-Dセンサの制御
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <ros/time.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <sobit_edu_library/sobit_edu_controller.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
#include <multiple_sensor_person_tracking/SensorRotatorParameterConfig.h>
#include <multiple_sensor_person_tracking/FollowingPosition.h>

typedef message_filters::sync_policies::ApproximateTime<multiple_sensor_person_tracking::FollowingPosition, nav_msgs::Odometry> MySyncPolicy;

namespace multiple_sensor_person_tracking {
    class PersonAimSensorRotator : public nodelet::Nodelet {
        private:
			ros::NodeHandle nh_;
			ros::NodeHandle pnh_;
			ros::Publisher pub_marker_;
            ros::Subscriber sub_following_position_;
			tf::TransformListener tf_listener_;

            // std::unique_ptr<message_filters::Subscriber<multiple_sensor_person_tracking::FollowingPosition>> sub_following_position_;
            // std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> sub_odom_;
            // std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

			dynamic_reconfigure::Server<multiple_sensor_person_tracking::SensorRotatorParameterConfig>* server_;
            dynamic_reconfigure::Server<multiple_sensor_person_tracking::SensorRotatorParameterConfig>::CallbackType f_;

			std::unique_ptr<sobit_edu::SobitEduController> sobit_edu_ctr_;

			geometry_msgs::PointPtr tracking_position_;
			double pre_tilt_;
			double pre_pan_;
			double tilt_angle_min_;
			double tilt_angle_max_;
			double person_height_;
			double smoothing_gain_;
			bool use_rotate_;
			bool use_smoothing_;
			bool display_marker_;

			void makeMarker( const double pan_angle, const double tilt_angle, const double distance );
			void callbackDynamicReconfigure(multiple_sensor_person_tracking::SensorRotatorParameterConfig& config, uint32_t level);
            void callbackData (
                const multiple_sensor_person_tracking::FollowingPositionConstPtr &following_position_msg
                /*const nav_msgs::OdometryConstPtr &odom_msg*/
            );

        public:
            virtual void onInit();
    };
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::makeMarker( const double pan_angle, const double tilt_angle, const double distance ) {
	visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = ros::Time::now();
    marker.ns = "sensor_direction";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = distance; marker.scale.y = 0.05; marker.scale.z = 0.05;
	marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
    marker.lifetime = ros::Duration(0.1);
	marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.8;
    tf::Quaternion quat = tf::createQuaternionFromRPY(0, -tilt_angle, pan_angle);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    marker.pose.orientation = geometry_quat;
    pub_marker_.publish ( marker );
}
void multiple_sensor_person_tracking::PersonAimSensorRotator::callbackDynamicReconfigure(multiple_sensor_person_tracking::SensorRotatorParameterConfig& config, uint32_t level) {
	use_rotate_ = config.use_rotate;
	tilt_angle_min_ = config.tilt_angle_min_deg * M_PI / 180.0;
	tilt_angle_max_ = config.tilt_angle_max_deg * M_PI / 180.0;
	person_height_ = config.camera2person_height;
	use_smoothing_ = config.use_smoothing;
	smoothing_gain_ = config.smoothing_gain;
	display_marker_ = config.display_marker;
	return;
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::callbackData (
    const multiple_sensor_person_tracking::FollowingPositionConstPtr &following_position_msg
    /*const nav_msgs::OdometryConstPtr &odom_msg*/) {
	geometry_msgs::Point pt;
    //if ( ( odom_msg->twist.twist.linear.x >= 0.1 && odom_msg->twist.twist.linear.x != 0.0 ) || odom_msg->twist.twist.angular.z == 0.0 ) {
    if ( use_smoothing_ ) {
        tracking_position_->x = smoothing_gain_ * tracking_position_->x + ( 1.0 - smoothing_gain_ ) * following_position_msg->rotation_position.x;
        tracking_position_->y = smoothing_gain_ * tracking_position_->y + ( 1.0 - smoothing_gain_ ) * following_position_msg->rotation_position.y;
    } else *tracking_position_ = following_position_msg->rotation_position;
    //}
    pt = *tracking_position_;
	double distance = std::hypotf( pt.x, pt.y );
	double angle = std::atan2( pt.y, pt.x );
	double pan_angle, tilt_angle;
	double sec = ( distance == 0.0 ) ? 0.5 : 0.05;

	tilt_angle = std::atan2( person_height_, distance );
	if ( distance == 0.0 ) tilt_angle = 0.2;
	else if ( tilt_angle < tilt_angle_min_ ) tilt_angle = tilt_angle_min_;
	else if ( tilt_angle > tilt_angle_max_ ) tilt_angle = tilt_angle_max_;
    pan_angle = angle;

	NODELET_INFO("\033[1mRotator\033[m               :\tpan = %8.3f[deg],\ttilt = %8.3f [deg]", pan_angle*180/M_PI, tilt_angle*180/M_PI);

	if ( use_rotate_ ) sobit_edu_ctr_->moveHeadPanTilt ( pan_angle, tilt_angle, sec, false );
	if ( display_marker_ ) makeMarker( pan_angle, tilt_angle, distance );

	return;
}

void multiple_sensor_person_tracking::PersonAimSensorRotator::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

	pub_marker_ = nh_.advertise< visualization_msgs::Marker >( "rotator_marker", 1 );

	sobit_edu_ctr_.reset( new sobit_edu::SobitEduController );
	tracking_position_.reset( new geometry_msgs::Point );

    server_ = new dynamic_reconfigure::Server<multiple_sensor_person_tracking::SensorRotatorParameterConfig>(pnh_);
    f_ = boost::bind(&multiple_sensor_person_tracking::PersonAimSensorRotator::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);

    // message_filters :
    // sub_following_position_.reset ( new message_filters::Subscriber<multiple_sensor_person_tracking::FollowingPosition> ( nh_, pnh_.param<std::string>( "following_position_topic_name", "/following_position" ), 1 ) );
    // sub_odom_.reset ( new message_filters::Subscriber<nav_msgs::Odometry> ( nh_, pnh_.param<std::string>( "odom_topic_name", "/odom" ), 1 ) );
    // sync_.reset ( new message_filters::Synchronizer<MySyncPolicy> ( MySyncPolicy(10), *sub_following_position_, *sub_odom_ ) );
    // sync_->registerCallback ( boost::bind( &PersonAimSensorRotator::callbackData, this, _1, _2 ) );

    sub_following_position_ = nh_.subscribe(pnh_.param<std::string>( "following_position_topic_name", "/following_position" ), 1, &PersonAimSensorRotator::callbackData, this);


	if ( !use_rotate_ ) return;
	sobit_edu_ctr_->moveToPose( "initial_pose" );
	sobit_edu_ctr_->moveHeadPanTilt ( 0.0, 0.2, 0.3, false );
}

PLUGINLIB_EXPORT_CLASS(multiple_sensor_person_tracking::PersonAimSensorRotator, nodelet::Nodelet);
