#include "multiple_observation_tracing_simulator/virtual_spring_model.hpp"

using namespace multiple_observation_tracing_simulator;

void VirtualSpringModel::displayTargetMarker ( const geometry_msgs::Twist& vel, const double curt_vel_angular ) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "true_value";
    marker.header.stamp = ros::Time::now();
    marker.ns = "target_pose";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.3);

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

    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "robot";
    path_marker.header.stamp = marker.header.stamp;
    path_marker.ns = "optimal_path";
    path_marker.id = 1;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.lifetime = ros::Duration(0.3);
    path_marker.scale.x = 0.05f;

    path_marker.color.r = 1.0f;
    path_marker.color.g = 0.5f;
    path_marker.color.b = 0.5f;
    path_marker.color.a = 1.0f;

    double x = 0.0, y = 0.0, theta = curt_vel_angular;
    double pre_x = 0.0, pre_y = 0.0, pre_theta = 0.0;
    double dt = 0.1;
    for ( int step = 0; step < 30; ++step) {
        pre_x = x, pre_y = y, pre_theta = theta;
        x = vel.linear.x * cos(pre_theta) * dt + pre_x;
        y = vel.linear.x * sin(pre_theta) * dt + pre_y;
        theta = vel.angular.z * dt + pre_theta;
        geometry_msgs::Point temp;
        temp.x = x;
        temp.y = y;
        temp.z = 0.1;
        path_marker.points.push_back( temp );
    }

    pub_mrk_tgt_.publish( marker );
    pub_mrk_path_.publish( path_marker );

    return;
}

VirtualSpringModel::VirtualSpringModel ( ) : nh_(), pnh_("~") {
    pub_mrk_tgt_ = nh_.advertise< visualization_msgs::Marker >( "/following_marker", 1 );
    pub_mrk_path_ = nh_.advertise< visualization_msgs::Marker >( "/path_marker", 1 );

    setFollowParamater( pnh_.param<double>( "following_angle_deg", 0.0 ), pnh_.param<double>( "following_distance", 0.7 ) );
    setSpringParamater( pnh_.param<double>( "spring_constant_linear", 3.0 ), pnh_.param<double>( "spring_constant_angular", 0.001 ) );
    setFrictionParamater( pnh_.param<double>( "viscous_friction_linear", 30.0 ), pnh_.param<double>( "viscous_friction_angular", 20.0 ) );
    setRobotParamater( pnh_.param<double>( "weight_robot", 30.0 ), pnh_.param<double>( "radius_robot", 0.3 ) );
    setMomentParamater( pnh_.param<double>( "moment_inertia", 15.0 ) );
    std::cout  << "\n============================================================"
        << "\n[ Virtual Spring Model Paramater]"
        << "\n* following_angle_deg      [deg]      : " << pnh_.param<double>( "following_angle_deg", 0.0 )
        << "\n* following_distance       [m]        : " << pnh_.param<double>( "following_distance", 0.7 )
        << "\n* spring_constant_linear   [N/m]      : " << pnh_.param<double>( "spring_constant_linear", 3.0 )
        << "\n* spring_constant_angular  [N・m/rad] : " << pnh_.param<double>( "spring_constant_angular", 0.001 )
        << "\n* weight_robot             [Kg]       : " << pnh_.param<double>( "weight_robot", 30.0 )
        << "\n* moment_inertia           [Kg・m^2]  : " << pnh_.param<double>( "moment_inertia", 15.0 )
        << "\n* viscous_friction_linear  [N・s/m]   : " << pnh_.param<double>( "viscous_friction_linear", 30.0 )
        << "\n* viscous_friction_angular [N・s/rad] : " << pnh_.param<double>( "viscous_friction_angular", 20.0 )
        << "\n* radius_robot             [m]        : " << pnh_.param<double>( "radius_robot", 0.3 )
    << std::endl;
}

void VirtualSpringModel::compute ( const geometry_msgs::PoseStampedConstPtr &pose_msg, const double curt_vel_linear, const double curt_vel_angular, geometry_msgs::TwistPtr &output_vel ) {
    tf2::Quaternion quat_tf;
    double roll = 0., pitch = 0., yaw = 0.;
    tf2::fromMsg(pose_msg->pose.orientation, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

    std::cout  << "\n============================================================"
            << "\n[Position]"
            << "\n* x     : " << pose_msg->pose.position.x
            << "\n* y     : " << pose_msg->pose.position.y
            << "\n* z     : " << pose_msg->pose.position.z
            << "\n[Orientation]"
            << "\n* roll  : " << roll
            << "\n* pitch : " << pitch
            << "\n* yaw   : " << yaw
    << std::endl;

    // Find the position of the robot when the mobile robot follows a person (coordinate transformation)
    double ang_follow = ang_follow_;
    Eigen::Vector3f robot(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f human(pose_msg->pose.position.x, pose_msg->pose.position.y, yaw);
    Eigen::Vector3f ang_follow_v(0.0f, 0.0f, ang_follow);
    Eigen::Matrix3f F;
    F << std::cos( yaw + ang_follow ), std::sin( yaw + ang_follow ), 0.0,
        -std::sin( yaw + ang_follow ), std::cos( yaw + ang_follow ), 0.0,
        0.0, 0.0, 1.0;
    Eigen::Vector3f robot_transformed;
    robot_transformed = F * ( robot - human - ang_follow_v );
    std::cout << "\n[Robot Transformed]\n"  << robot_transformed << std::endl;

    // Calculate the state of the virtual spring, such as the length l of the virtual spring and the angle φ with respect to the robot, based on the positional relationship between the human and the robot
    double length_spring = std::hypotf( robot_transformed[0], robot_transformed[1] );
    double angle_spring = std::atan2( robot_transformed[1], robot_transformed[0] );
    std::cout << "\n[Virtual Spring]"
        << "\n* length[m]  : " <<  length_spring
        << "\n* angle[deg] : " <<  angle_spring * 180.0 / M_PI << std::endl;

    // Calculate the elastic force acting on the mobile robot from the virtual spring
    // Elastic force F1 : proportional to (length_spring - dist_follow_) in the direction of elongation
    double elastic_force_linear = spring_constant_linear_ * ( length_spring - dist_follow_ );
    if ( std::isnan( elastic_force_linear ) ) elastic_force_linear = 0.0;
    // Elastic force F2 : proportional to angle_spring in the direction of bending
    double elastic_force_angular = (spring_constant_angular_ * angle_spring ) / length_spring;
    if ( std::isnan( elastic_force_angular ) ) elastic_force_angular = 0.0;
    std::cout << "\n[Elastic Force]"
        << "\n* elastic_force_linear  : " <<  elastic_force_linear 
        << "\n* elastic_force_angular : " <<  elastic_force_angular << std::endl;

    // The equations of motion are derived as follows, assuming the hypothetical elastic forces F1 and F2 as the propulsive forces of the mobile robot
    // The left-hand side is the product of the mass and the derivative of the translational velocity of the mobile robot
    double linear = -elastic_force_linear * std::cos ( robot_transformed[2] - angle_spring )    // Term 1 : Translational component of the elastic force of elongation F1 applied to the mobile robot
                    -elastic_force_angular * std::sin ( robot_transformed[2] - angle_spring )   // Term 2 : Translational component of elastic force F2 in the direction of bending
                    -viscous_friction_linear_ * curt_vel_linear;                                // Term 3 : Viscous frictional force by the product of κ3 and the translational velocity of the mobile robot
    linear = linear / weight_robot_;
    // The left-hand side is the product of the rotational moment and the derivative of the rotational angular velocity of the mobile robot
    double angular = ( -elastic_force_linear * std::sin ( robot_transformed[2] - angle_spring )     // Term 1 : Rotational component of the elastic force F1 of elongation applied to the mobile robot
                    -elastic_force_angular * std::cos ( robot_transformed[2] - angle_spring )       // Term 2 : Rotational component of elastic force F2 in the direction of bending
                    -viscous_friction_angular_ * curt_vel_angular ) * radius_robot_;                // Term 3 : Viscous frictional force by the product of K3 and the rotational speed of the mobile robot
    std::cout << "\n[Velocity]"
        << "\n* linear  [m/s]   : " <<  linear 
        << "\n* angular [rad/s] : " <<  angular << std::endl;

    geometry_msgs::Twist vel;
    vel.linear.x = linear;
    vel.angular.z = angular;
    *output_vel = vel;
    displayTargetMarker( vel, curt_vel_angular );
    return;
}