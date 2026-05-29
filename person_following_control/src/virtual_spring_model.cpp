#include "person_following_control/virtual_spring_model.hpp"

using namespace person_following_control;

void VirtualSpringModel::displayVirtualSpringPathMarker ( float vel, float ang_vel ) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = node_->get_clock()->now();
    marker.ns = "virtual_spring_path";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.08;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);

    int predict_step = 20;
    float theta = 0.0;
    float sampling_time = 0.1;
    geometry_msgs::msg::Point pt, pre_pt;
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
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    
    // Ensure correct publish type
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    pub_mrk_path_->publish(marker_array);
}

visualization_msgs::msg::Marker VirtualSpringModel::displayTargetMarker ( const Eigen::Vector3f& pt, const std::string& name, const float r, const float g, const float b ) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = ( name != "robot_transformed" ) ? "base_footprint" : "target";
    marker.header.stamp = node_->get_clock()->now();
    marker.ns = name;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);

    marker.pose.position.x = pt[0];
    marker.pose.position.y = pt[1];
    marker.pose.position.z = 0.3;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, pt[2]);
    geometry_msgs::msg::Quaternion quat_msg;
    tf2::convert(quat_tf, quat_msg);
    marker.pose.orientation = quat_msg;

    marker.scale.x = 0.7;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0f;
    return marker;
}

VirtualSpringModel::VirtualSpringModel ( rclcpp_lifecycle::LifecycleNode* node ) : node_( node ) {
    pub_mrk_tgt_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("sobits_follower/person_following_control/vsm_target_marker", 1);
    pub_mrk_path_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("sobits_follower/person_following_control/vsm_path_marker", 1);

    ang_follow_ = 0.0;
    dist_follow_ = 0.7;
    spring_constant_linear_ = 3.0;
    spring_constant_angular_ = 0.001;
    weight_robot_ = 30.0;
    moment_inertia_ = 15.0;
    viscous_friction_linear_ = 30.0;
    viscous_friction_angular_ = 20.0;
    radius_robot_ = 0.3;
    display_vsm_path_ = false;
    display_target_ = false;

    setFollowParamater( 0.0, 0.7 );
    setSpringParamater( 3.0 , 0.001 );
    setFrictionParamater( 30.0, 20.0 );
    setRobotParamater( 30.0, 0.3 );
    setMomentParamater( 15.0 );
	setDisplayFlag( false, false );
}

void VirtualSpringModel::activatePublishers() {
    pub_mrk_tgt_->on_activate();
    pub_mrk_path_->on_activate();
}

void VirtualSpringModel::deactivatePublishers() {
    pub_mrk_tgt_->on_deactivate();
    pub_mrk_path_->on_deactivate();
}

void VirtualSpringModel::compute ( const geometry_msgs::msg::Pose &pose_msg, const float curt_vel_linear, const float curt_vel_angular, geometry_msgs::msg::Twist& output_vel ) {
    float yaw = std::atan2( pose_msg.position.y, pose_msg.position.x );

    // Find the position of the robot when the mobile robot follows a person (coordinate transformation)
    float ang_follow = ang_follow_;
    Eigen::Vector3f robot(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f human(pose_msg.position.x, pose_msg.position.y, yaw);
    Eigen::Vector3f ang_follow_v(0.0f, 0.0f, ang_follow);
    Eigen::Matrix3f F;
    F << std::cos( yaw + ang_follow ), std::sin( yaw + ang_follow ), 0.0,
        -std::sin( yaw + ang_follow ), std::cos( yaw + ang_follow ), 0.0,
        0.0, 0.0, 1.0;
    Eigen::Vector3f robot_transformed;
    robot_transformed = F * ( robot - human - ang_follow_v );
    // ROS_INFO("Robot                 =\t%5.3f [m]\t%5.3f [m]", robot_transformed[0], robot_transformed[1] );

    // Calculate the state of the virtual spring, such as the length l of the virtual spring and the angle φ with respect to the robot, based on the positional relationship between the human and the robot.
    float length_spring = std::hypotf( robot_transformed[0], robot_transformed[1] );
    float angle_spring = std::atan2( robot_transformed[1], robot_transformed[0] );

    // Calculate the elastic force acting on the mobile robot from the virtual spring
    // Elastic force F1 : proportional to (length_spring - dist_follow_) in the direction of elongation
    float elastic_force_linear = spring_constant_linear_ * ( length_spring - dist_follow_ );
    if ( std::isnan( elastic_force_linear ) ) elastic_force_linear = 0.0;
    // Elastic force F2 : proportional to angle_spring in the direction of bending
    float elastic_force_angular = (spring_constant_angular_ * angle_spring ) / length_spring;
    if ( std::isnan( elastic_force_angular ) ) elastic_force_angular = 0.0;

    // The equations of motion are derived as follows, assuming the hypothetical elastic forces F1 and F2 as the propulsive forces of the mobile robot
    // The left-hand side is the product of the mass and the derivative of the translational velocity of the mobile robot
    float linear = -elastic_force_linear * std::cos ( robot_transformed[2] - angle_spring )     // Term 1 : Translational component of the elastic force of elongation F1 applied to the mobile robot
                    -elastic_force_angular * std::sin ( robot_transformed[2] - angle_spring )   // Term 2 : Translational component of elastic force F2 in the direction of bending
                    -viscous_friction_linear_ * curt_vel_linear;                                // Term 3 : Viscous frictional force by the product of κ3 and the translational velocity of the mobile robot
    linear = linear / weight_robot_;
    // left hand side is the product of the rotational moment and the derivative of the rotational angular velocity of the mobile robot
    float angular = ( -elastic_force_linear * std::sin ( robot_transformed[2] - angle_spring )  // Term 1: The rotational component of the elastic force of elongation F1 applied to the mobile robot
                    -elastic_force_angular * std::cos ( robot_transformed[2] - angle_spring )   // Term 2 : Rotational component of elastic force F2 in the direction of bending
                    -viscous_friction_angular_ * curt_vel_angular ) * radius_robot_;            // Term 3 : Viscous frictional force by the product of κ3 and the rotational speed of the mobile robot
    angular = angular / moment_inertia_;

    if (linear < 0.0f) {
        std::cout << "minus linear output" << std::endl;
        linear = 0.0f; // Add haku
    }

    auto vel = geometry_msgs::msg::Twist();
    vel.linear.x = linear;
    vel.angular.z = angular;
    output_vel = vel;
    if ( display_vsm_path_ ) displayVirtualSpringPathMarker ( linear, angular );
    if ( display_target_ ) {
        auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();
        marker_array->markers.push_back( displayTargetMarker(robot, "robot", 1.0, 0.0, 0.0) );
        marker_array->markers.push_back( displayTargetMarker(human, "human", 0.0, 1.0, 0.0) );
        // marker_array->markers.push_back( displayTargetMarker(robot_transformed, "robot_transformed", 0.0, 0.0, 1.0) );
        pub_mrk_tgt_->publish(*marker_array);
    }
    return;
}
