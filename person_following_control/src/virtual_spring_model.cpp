#include <person_following_control/virtual_spring_model.hpp>

using namespace person_following_control;

void VirtualSpringModel::displayVirtualSpringPathMarker ( double vel, double ang_vel ) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = ros::Time::now();
    marker.ns = "virtual_spring_path";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.08;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0.1);

    int predict_step = 20;
    double theta = 0.0;
    double sampling_time = 0.1;
    geometry_msgs::Point pt, pre_pt;
    pt.z = 0.1;
    for ( int step = 0; step < predict_step; ++step) {
        pre_pt = pt;
        pt.x = vel * cos(theta) * sampling_time + pre_pt.x;
        pt.y = vel * sin(theta) * sampling_time + pre_pt.y;
        pt.z = 0.3;
        theta = ang_vel * sampling_time + theta;
        // if ( std::hypotf( pt.x, pt.y ) > 1.5 ) break;
        marker.points.push_back( pt );
    }
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(0.1);
    pub_mrk_path_.publish ( marker );
}

void VirtualSpringModel::displayTargetMarker ( ) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "target";
    marker.header.stamp = ros::Time::now();
    marker.ns = "target_pose";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(1.0);

    marker.pose.position.x -= dist_follow_;
    marker.pose.position.z = 0.2;
    double x1 = marker.pose.position.x;
    double y1 = marker.pose.position.y;
    double alpha = ang_follow_;
    marker.pose.position.x = x1 * std::cos(alpha) - y1 * std::sin(alpha);
    marker.pose.position.y = x1 * std::sin(alpha) + y1 * std::cos(alpha);

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.35;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    return;
}

VirtualSpringModel::VirtualSpringModel ( ) : nh_(), pnh_("~") {
    pub_mrk_tgt_ = nh_.advertise< visualization_msgs::Marker >( "/vsm_target_marker", 1 );
    pub_mrk_path_ = nh_.advertise< visualization_msgs::Marker >( "/vsm_path_marker", 1 );

    setFollowParamater( 0.0, 0.7 );
    setSpringParamater( 3.0 , 0.001 );
    setFrictionParamater( 30.0, 20.0 );
    setRobotParamater( 30.0, 0.3 );
    setMomentParamater( 15.0 );
	setDisplayFlag( false, false );
}

void VirtualSpringModel::compute ( const geometry_msgs::Pose &pose_msg, const double curt_vel_linear, const double curt_vel_angular, geometry_msgs::TwistPtr &output_vel ) {
    tf::Quaternion quat;
    double roll, pitch, yaw;
	quaternionMsgToTF(pose_msg.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference

    // 移動ロボットが人を追従するときのロボットの位置を求める(座標変換)
    double ang_follow = ang_follow_;
    Eigen::Vector3f robot(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f human(pose_msg.position.x, pose_msg.position.y, yaw);
    Eigen::Vector3f ang_follow_v(0.0f, 0.0f, ang_follow);
    Eigen::Matrix3f F;
    F << std::cos( yaw + ang_follow ), std::sin( yaw + ang_follow ), 0.0,
        -std::sin( yaw + ang_follow ), std::cos( yaw + ang_follow ), 0.0,
        0.0, 0.0, 1.0;
    Eigen::Vector3f robot_transformed;
    robot_transformed = F * ( robot - human - ang_follow_v );

    // 人間,ロボットの位置関係から,仮想ばねの長さl,ロボットに対する角度φといった,仮想ばねの状態を求める
    double length_spring = std::hypotf( robot_transformed[0], robot_transformed[1] );
    double angle_spring = std::atan2( robot_transformed[1], robot_transformed[0] );

    // 仮想ばねから移動ロボットに作用する弾性力を求める
    // 伸びの方向には(length_spring - dist_follow_)に比例した弾性力F1
    double elastic_force_linear = spring_constant_linear_ * ( length_spring - dist_follow_ );
    if ( std::isnan( elastic_force_linear ) ) elastic_force_linear = 0.0;
    // 曲げの方向にはangle_springに比例した弾性力F2
    double elastic_force_angular = (spring_constant_angular_ * angle_spring ) / length_spring;
    if ( std::isnan( elastic_force_angular ) ) elastic_force_angular = 0.0;

    // 仮想的な弾性力F1,F2を,移動ロボットの推進力として,以下のように運動方程式を導く
    // 左辺が質量と移動ロボットの並進速度の微分の積
    double linear = -elastic_force_linear * std::cos ( robot_transformed[2] - angle_spring )    // 第1項 : 移動ロボットにかかる伸びの弾性力F1の並進方向成分
                    -elastic_force_angular * std::sin ( robot_transformed[2] - angle_spring )   // 第2項 : 曲げの方向の弾性力F2の並進方向成分
                    -viscous_friction_linear_ * curt_vel_linear;                               // 第3項 : κ3と移動ロボットの並進速度の積によって粘性摩擦力
    linear = linear / weight_robot_;
    // 左辺が回転モーメントと移動ロボットの回転角速度の微分の積
    double angular = ( -elastic_force_linear * std::sin ( robot_transformed[2] - angle_spring )     // 第1項 : 移動ロボットにかかる伸びの弾性力F1の回転方向成分
                    -elastic_force_angular * std::cos ( robot_transformed[2] - angle_spring )       // 第2項 : 曲げの方向の弾性力F2の回転方向成分
                    -viscous_friction_angular_ * curt_vel_angular ) * radius_robot_;                // 第3項 : κ3と移動ロボットの回転速度の積によって粘性摩擦力
    angular = angular / moment_inertia_;

    geometry_msgs::Twist vel;
    vel.linear.x = linear;
    vel.angular.z = angular;
    *output_vel = vel;
    if ( display_vsm_path_ ) displayVirtualSpringPathMarker ( linear, angular );
    if ( display_target_ ) displayTargetMarker( );
    return;
}