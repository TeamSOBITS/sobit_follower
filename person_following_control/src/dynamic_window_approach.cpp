#include <person_following_control/dynamic_window_approach.hpp>

using namespace person_following_control;

void DynamicWindowApproach::displayOptimalPathMarker ( const double optimal_linear, const double optimal_angular ) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = dwap_->target_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "optimal_path";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.08;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(0.1);

    int predict_step = dwap_->predict_step;
    double theta = 0.0;
    double sampling_time = dwap_->sampling_time;
    geometry_msgs::Point pt, pre_pt;
    pt.z = 0.1;
    for ( int step = 0; step < predict_step; ++step) {
        pre_pt = pt;
        pt.x = optimal_linear * cos(theta) * sampling_time + pre_pt.x;
        pt.y = optimal_linear * sin(theta) * sampling_time + pre_pt.y;
        pt.z = 0.5;
        theta = optimal_angular * sampling_time + theta;
        // if ( std::hypotf( pt.x, pt.y ) > 1.5 ) break;
        marker.points.push_back( pt );
    }
    pub_path_marker_.publish ( marker );
}

void DynamicWindowApproach::displayAllPathMarker ( const std::vector< EvaluatedPath >& path_list ) {
    int marker_num = 0;
    visualization_msgs::MarkerArray marker_all;
    std::string target_frame = dwap_->target_frame;
    int predict_step = dwap_->predict_step;
    double sampling_time = dwap_->sampling_time;

    for( auto path : path_list ) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = target_frame;
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.ns = "all_path";
        marker.id = marker_num;
        marker.scale.x = 0.03;
        marker.color.a = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(0.1);
        if ( path.is_collision ) {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
        }
        double theta = 0.0;
        geometry_msgs::Point pt, pre_pt;
        pt.z = 0.1;
        for ( int step = 0; step < predict_step; ++step) {
            pre_pt = pt;
            pt.x = path.linear * cos(theta) * sampling_time + pre_pt.x;
            pt.y = path.linear * sin(theta) * sampling_time + pre_pt.y;
            theta = path.angular * sampling_time + theta;
            //if ( std::hypotf( pt.x, pt.y ) > 1.0 ) break;
            marker.points.push_back( pt );
        }
        marker_num++;
        marker_all.markers.push_back( marker );
    }
    pub_path_marker_all_.publish( marker_all );
}

DynamicWindowApproach::DynamicWindowApproach ( ) : nh_(), pnh_("~") {
    pub_path_marker_ = nh_.advertise< visualization_msgs::Marker >( "/dwa_path_marker", 1 );
    pub_path_marker_all_ = nh_.advertise< visualization_msgs::MarkerArray >( "/dwa_path_marker_all", 1 );
    dwap_.reset ( new DWAParameters );
    // DWA Parameters :
	setTargetFrame( "base_footprint" );
	setStepValue( 30, 0.1 );
	setVelocityLimit( 0.2, 1.2, -60.0, 60.0, 7.0, 15.0 );
	setWeight( 1.0, 1.0, 2.0, 1.0, 1.0 );
	setCostDistance ( 0.35 );
	setDisplayFlag( false, false );
}

bool DynamicWindowApproach::generatePath2Target (
    const geometry_msgs::Point& target,
    const PointCloud::Ptr obstacles,
    geometry_msgs::TwistPtr output_path )
{
    // 経路予測して、経路リストを作成
    std::vector< EvaluatedPath > path_list;
    MinMaxValue mmv;

    std::vector<double> linear_list = dwap_->linear_list;
    std::vector<double> angular_list = dwap_->angular_list;

    int predict_step = dwap_->predict_step;
    double sampling_time = dwap_->sampling_time;

    std::vector<int> k_indices;
    std::vector<float> k_distances;
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (obstacles);
    double dist_nearest_obstacle = 0.0;
    double obstacle_cost_radius = dwap_->obstacle_cost_radius;

    double h = dwap_->weight_heading;
    double o = dwap_->weight_obstacle;
    double v = dwap_->weight_velocity;
    double max_score = 0.0;
    double optimal_linear = 0.0;
    double optimal_angular = 0.0;
    bool exists_path = false;

    // 経路予測
    for ( auto& linear : linear_list ) {
        for ( auto& angular : angular_list ) {
            EvaluatedPath eval;
            eval.linear = linear;
            eval.angular = angular;

            Path path, pre_path;
            bool is_collision = false;
            double theta = 0.0;
            // 衝突チェック
            for ( int step = 0; step < predict_step; ++step) {
                pre_path = path;
                path.point.x = linear * cos(theta) * sampling_time + pre_path.point.x;
                path.point.y = linear * sin(theta) * sampling_time + pre_path.point.y;
                path.theta = angular * sampling_time + pre_path.theta;
                if ( kdtree.nearestKSearch ( path.point, 1, k_indices, k_distances ) > 0 ) {
                    dist_nearest_obstacle = std::sqrt(k_distances[0]);
                    if ( dist_nearest_obstacle <= obstacle_cost_radius ) {
                        is_collision = true;
                        break;
                    }
                } else {
                    ROS_ERROR("No Obstacle");
                }
                theta = path.theta;
            }
            eval.is_collision = is_collision;
            if ( !is_collision ) {
                // [ heading(v,ω) ] Evaluate the angle of difference between predicted point and Target point :
                eval.heading = M_PI - std::fabs( std::atan2(target.y - path.point.y, target.x - path.point.x) - path.theta );
                // [ dist(v,ω) ]  Evaluate the distance between the predicted point and the nearest obstacle :
                eval.obstacle = dist_nearest_obstacle;
                // [ velocity(v,ω) ] Evaluate Velocity :
                eval.velocity = std::fabs( linear );
                // Update the max and min evaluation values ​​of all paths :
                if( mmv.heading.min_score > eval.heading ) mmv.heading.min_score = eval.heading;
                if( mmv.heading.max_score < eval.heading ) mmv.heading.max_score = eval.heading;
                if( mmv.obstacle.min_score > eval.obstacle ) mmv.obstacle.min_score = eval.obstacle;
                if( mmv.obstacle.max_score < eval.obstacle ) mmv.obstacle.max_score = eval.obstacle;
                if( mmv.velocity.min_score > eval.velocity ) mmv.velocity.min_score = eval.velocity;
                if( mmv.velocity.max_score < eval.velocity ) mmv.velocity.max_score = eval.velocity;
            }
            path_list.push_back( eval );
        }
    }

    // 各スコアを正規化して、評価関数に入力
    for ( auto& path : path_list ) {
        if ( path.is_collision || ( path.linear == 0.0 && path.angular == 0.0 ) ) continue;
        if ( mmv.heading.max_score - mmv.heading.min_score != 0.0 ) {
            path.heading = ( path.heading - mmv.heading.min_score ) / ( mmv.heading.max_score - mmv.heading.min_score );
        } else path.heading = 0.0;
        if ( mmv.obstacle.max_score - mmv.obstacle.min_score != 0.0 ) {
            path.obstacle = ( path.obstacle - mmv.obstacle.min_score ) / ( mmv.obstacle.max_score - mmv.obstacle.min_score );
        } else path.obstacle = 0.0;
        if ( mmv.velocity.max_score - mmv.velocity.min_score != 0.0 ) {
            path.velocity = ( path.velocity - mmv.velocity.min_score ) / ( mmv.velocity.max_score - mmv.velocity.min_score );
        } else path.velocity = 0.0;

        double sum_score = 	  h * path.heading
                            + o * path.obstacle
                            + v * path.velocity;
        if ( sum_score > max_score ) {
            max_score = sum_score;
            optimal_linear = path.linear;
            optimal_angular = path.angular;
            exists_path = true;
        }
    }
    // 最適な経路の速度を出力
    if ( exists_path ) {
        output_path->linear.x = optimal_linear;
        output_path->angular.z = optimal_angular;
    } else {
        ROS_ERROR("No Optimal Path");
    }

    if ( display_optimal_path_ ) displayOptimalPathMarker ( optimal_linear, optimal_angular );
    if ( display_all_path_ ) displayAllPathMarker ( path_list );
    return exists_path;
}

bool DynamicWindowApproach::generatePath2Target (
    const geometry_msgs::Point& target,
    const PointCloud::Ptr obstacles,
    const geometry_msgs::TwistPtr base_path,
    geometry_msgs::TwistPtr output_path ) {

    // 経路予測して、経路リストを作成
    std::vector< EvaluatedPath > path_list;
    MinMaxValue mmv;
    double base_linear = ( base_path->linear.x < dwap_->max_vel ) ? base_path->linear.x : dwap_->max_vel;
    double base_angular = base_path->angular.z;
    std::vector<double> linear_list;
    double delta_lin = ( base_linear - dwap_->min_vel) / dwap_->vel_step;
    for ( double vel = dwap_->min_vel; vel < base_linear; vel += delta_lin ) linear_list.push_back(vel);
    // linear_list.push_back(base_linear);
    std::vector<double> angular_list;
    double delta_ang = ( dwap_->max_ang_vel - dwap_->min_ang_vel ) / dwap_->ang_vel_step;
    for ( double vel = dwap_->min_ang_vel; vel < dwap_->max_ang_vel; vel += delta_ang ) angular_list.push_back(vel);
    angular_list.push_back(base_angular);

    // std::vector<double> linear_list = dwap_->linear_list;
    // linear_list.push_back(base_linear);
    // std::vector<double> angular_list = dwap_->angular_list;
    // angular_list.push_back(base_angular);

    int predict_step = dwap_->predict_step;
    double sampling_time = dwap_->sampling_time;

    std::vector<int> k_indices;
    std::vector<float> k_distances;
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (obstacles);
    double dist_nearest_obstacle = 0.0;
    double obstacle_cost_radius = dwap_->obstacle_cost_radius;

    double h = dwap_->weight_heading;
    double o = dwap_->weight_obstacle;
    double l = dwap_->weight_vsm_linear;
    double a = dwap_->weight_vsm_angular;
    double max_score = 0.0;
    double optimal_linear = 0.0;
    double optimal_angular = 0.0;
    bool exists_path = false;

    // 経路予測
    for ( auto& linear : linear_list ) {
        for ( auto& angular : angular_list ) {
            EvaluatedPath eval;
            eval.linear = linear;
            eval.angular = angular;

            Path path, pre_path;
            bool is_collision = false;
            double theta = 0.0;
            // 衝突チェック
            for ( int step = 0; step < predict_step; ++step) {
                pre_path = path;
                path.point.x = linear * cos(theta) * sampling_time + pre_path.point.x;
                path.point.y = linear * sin(theta) * sampling_time + pre_path.point.y;
                path.theta = angular * sampling_time + pre_path.theta;
                if ( kdtree.nearestKSearch ( path.point, 1, k_indices, k_distances ) > 0 ) {
                    dist_nearest_obstacle = std::sqrt(k_distances[0]);
                    if ( dist_nearest_obstacle <= obstacle_cost_radius ) {
                        is_collision = true;
                        break;
                    }
                } else {
                    ROS_ERROR("No Obstacle");
                }
                theta = path.theta;
            }
            eval.is_collision = is_collision;
            if ( !is_collision ) {
                // [ heading(v,ω) ] Evaluate the angle of difference between predicted point and Target point :
                eval.heading = M_PI - std::fabs( std::atan2(target.y - path.point.y, target.x - path.point.x) - path.theta );
                // [ obstacle(v,ω) ]  Evaluate the distance between the predicted point and the nearest obstacle :
                eval.obstacle = dist_nearest_obstacle;
                // [ linear(v,ω) ] Evaluate Velocity :
                eval.velocity = 1 / ( 1.0 + std::fabs( linear - base_linear ) );
                // [ angular(v,ω) ] Evaluate the angle of difference between predicted point and Target point :
                eval.velocity_angle =  1 / ( 1.0 + std::fabs( angular - base_angular) );
                // Update the max and min evaluation values ​​of all paths :
                if( mmv.obstacle.min_score > eval.obstacle ) mmv.obstacle.min_score = eval.obstacle;
                if( mmv.obstacle.max_score < eval.obstacle ) mmv.obstacle.max_score = eval.obstacle;
                if( mmv.heading.min_score > eval.heading ) mmv.heading.min_score = eval.heading;
                if( mmv.heading.max_score < eval.heading ) mmv.heading.max_score = eval.heading;
                if( mmv.velocity.min_score > eval.velocity ) mmv.velocity.min_score = eval.velocity;
                if( mmv.velocity.max_score < eval.velocity ) mmv.velocity.max_score = eval.velocity;
                if( mmv.velocity_angle.min_score > eval.velocity_angle ) mmv.velocity_angle.min_score = eval.velocity_angle;
                if( mmv.velocity_angle.max_score < eval.velocity_angle ) mmv.velocity_angle.max_score = eval.velocity_angle;
            }
            path_list.push_back( eval );
        }
    }

    // 各スコアを正規化して、評価関数に入力
    for ( auto& path : path_list ) {
        if ( path.is_collision || ( path.linear == 0.0 && path.angular == 0.0 ) ) continue;
        if ( mmv.heading.max_score - mmv.heading.min_score != 0.0 ) {
            path.heading = ( path.heading - mmv.heading.min_score ) / ( mmv.heading.max_score - mmv.heading.min_score );
        } else path.heading = 0.0;
        if ( mmv.obstacle.max_score - mmv.obstacle.min_score != 0.0 ) {
            path.obstacle = ( path.obstacle - mmv.obstacle.min_score ) / ( mmv.obstacle.max_score - mmv.obstacle.min_score );
        } else path.obstacle = 0.0;
        if ( mmv.velocity.max_score - mmv.velocity.min_score != 0.0 ) {
            path.velocity = ( path.velocity - mmv.velocity.min_score ) / ( mmv.velocity.max_score - mmv.velocity.min_score );
        } else path.velocity = 0.0;
        if ( mmv.velocity_angle.max_score - mmv.velocity_angle.min_score != 0.0 ) {
            path.velocity_angle = ( path.velocity_angle - mmv.velocity_angle.min_score ) / ( mmv.velocity_angle.max_score - mmv.velocity_angle.min_score );
        } else path.velocity_angle = 0.0;

        double sum_score = 	  h * path.heading
                            + o * path.obstacle
                            + l * path.velocity
                            + a * path.velocity_angle;
        if ( sum_score > max_score ) {
            max_score = sum_score;
            optimal_linear = path.linear;
            optimal_angular = path.angular;
            exists_path = true;
        }
    }
    // 最適な経路の速度を出力
    if ( exists_path ) {
        output_path->linear.x = optimal_linear;
        output_path->angular.z = optimal_angular;
    } else {
        ROS_ERROR("No Optimal Path");
    }

    if ( display_optimal_path_ ) displayOptimalPathMarker ( optimal_linear, optimal_angular );
    if ( display_all_path_ ) displayAllPathMarker ( path_list );

    return exists_path;
}