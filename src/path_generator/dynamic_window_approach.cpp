#include "sobit_follower/path_generator/dynamic_window_approach.h"

namespace PathPlan {

    bool DynamicWindowApproach::setDWAParameters ( const geometry_msgs::Twist& curt_vel, const double tgt_dist, DWAParameters* dwa_param ) { 
        DWAParameters tmp;
        double range_vel = sampling_time_ * max_acc_ * ( curt_vel.linear.x + acc_gain_ ); // 0.15 * 
        double limit_max_vel =  1.3 / ( 1 + std::exp( -1.3 * 1.3 * ( tgt_dist - 2 ) ) ); // Logistic function
        tmp.max_vel = ( curt_vel.linear.x + range_vel < limit_max_vel ) ? curt_vel.linear.x + range_vel : limit_max_vel;
        tmp.min_vel = ( curt_vel.linear.x - range_vel > tmp.max_vel ) ? curt_vel.linear.x - range_vel : 0.0;
        tmp.delta_vel = ( tmp.max_vel - tmp.min_vel ) / vel_step_;
        tmp.curt_vel = curt_vel.linear.x;
        tmp.curt_ang_vel = curt_vel.angular.z;
        tmp.pre_step = predict_step_;
        tmp.dt = sampling_time_;
        tmp.cost_obs_cnt_rad = obstacle_center_radius_;
        tmp.cost_obs_pt_rad = obstacle_points_radius_;
        *dwa_param = tmp;
        return ( tmp.max_vel - tmp.min_vel > 0.0 ) ? true : false;
    }
    void DynamicWindowApproach::makePathsList( const DWAParameters& dwa_param, std::vector< EvaluatedPath >* paths_list ) {
        std::vector< EvaluatedPath > tmp_plist;
        EvaluatedPath paths;
        MinMaxValue mmv;
        bool is_predict;
        double tgt_ang = fp_.target_angle;
        for ( double lin_vel = dwa_param.min_vel; lin_vel < dwa_param.max_vel; lin_vel += dwa_param.delta_vel ) {
            for ( auto& ang_vel : ang_vels_ ) {
                is_predict = predictPath ( lin_vel, ang_vel, dwa_param, &paths, &mmv );
                if( is_predict ) tmp_plist.push_back( paths );
            }
        }
        // Normalize evaluation values :
        normalizeEvaluationValues( &tmp_plist, mmv );
        *paths_list = tmp_plist;
    }
    bool DynamicWindowApproach::predictPath( const double vel, const double ang_vel, const DWAParameters& dwa_param, EvaluatedPath* paths, MinMaxValue* mmv ) {
        MinMaxValue tmp_mmv = *mmv;
        EvaluatedPath tmp_path;
        Path path, pre_path; 
        tmp_path.vel = vel; tmp_path.ang_vel = ang_vel;
        double theta = dwa_param.curt_ang_vel;
        int step = 0;
        bool is_collision = false;
        double min_obs_dist_score = 0.0;
        for ( step = 0; step < dwa_param.pre_step; ++step) {
            pre_path = path;
            // Calculate prediction points :
            path.next_x = vel * cos(theta) * dwa_param.dt + pre_path.next_x;
            path.next_y = vel * sin(theta) * dwa_param.dt + pre_path.next_y;
            path.next_theta = ang_vel * dwa_param.dt + pre_path.next_theta;
            // Predicted point collides with obstacle :
            min_obs_dist_score = 0.0;
            for ( auto& obs : fp_.obstacles ) {
                for ( auto& obs_pt : obs.points ) {
                    double dist_obs_predict = hypotf( path.next_x - obs_pt.x , path.next_y - obs_pt.y );
                    if ( dist_obs_predict <= dwa_param.cost_obs_pt_rad ) is_collision = true;
                }
                double dist_cnt_predict = std::hypotf( path.next_x - obs.center.x , path.next_y - obs.center.x );
                double diff_obs_cnt = std::fabs ( dwa_param.cost_obs_cnt_rad - dist_cnt_predict );
                if ( diff_obs_cnt < min_obs_dist_score ) min_obs_dist_score = diff_obs_cnt;
            }	
            if( is_collision ) break;					
            tmp_path.paths.push_back( path );
            theta = path.next_theta;
        }
        // Determine the path used to calculate the score :
        Path base_path = ( is_collision ) ? pre_path : path; 
        evaluatePath ( min_obs_dist_score, vel, base_path, &tmp_path.score, &tmp_mmv );
        tmp_path.score.is_collision = is_collision;
        *mmv = tmp_mmv;
        *paths = tmp_path;
        return step != 0 ? true : false; 
    }
    void DynamicWindowApproach::evaluatePath ( const double dist_score, const double vel, const Path& base_path, EvaluatedValue* score, MinMaxValue* mmv ) {
        EvaluatedValue tmp_score;
        MinMaxValue tmp_mmv = *mmv;
        // [ dist(v,ω) ]  Evaluate the distance between the predicted point and the nearest obstacle :
        tmp_score.obs.score = dist_score;
        // [ goal(v,ω) ]  Evaluate the distance between the predicted point and Taget point  :
        double vec_tgt2next_x = fp_.target.x - base_path.next_x;
        double vec_tgt2next_y = fp_.target.y - base_path.next_y;
        tmp_score.goal.score = 10.0 - std::hypotf(  vec_tgt2next_x, vec_tgt2next_y );	
        // [ heading(v,ω) ] Evaluate the angle of difference between predicted point and Target point :
        double ang_target_predicted = std::fabs( std::atan2 ( vec_tgt2next_y, vec_tgt2next_x ) );
        double diff_ang = std::fabs( fp_.target_angle - base_path.next_theta );
        tmp_score.angle.score = 2 * M_PI - ( ang_target_predicted + diff_ang); 
        // [ velocity(v,ω) ] Evaluate Velocity :
        tmp_score.vel.score = std::fabs( vel );
        // Update the max and min evaluation values ​​of all paths :
        if( tmp_mmv.obs.min_score > tmp_score.obs.score ) tmp_mmv.obs.min_score = tmp_score.obs.score;
        if( tmp_mmv.obs.max_score < tmp_score.obs.score ) tmp_mmv.obs.max_score = tmp_score.obs.score;
        if( tmp_mmv.goal.min_score > tmp_score.goal.score ) tmp_mmv.goal.min_score = tmp_score.goal.score;
        if( tmp_mmv.goal.max_score < tmp_score.goal.score ) tmp_mmv.goal.max_score = tmp_score.goal.score;
        if( tmp_mmv.angle.min_score > tmp_score.angle.score ) tmp_mmv.angle.min_score = tmp_score.angle.score;
        if( tmp_mmv.angle.max_score < tmp_score.angle.score ) tmp_mmv.angle.max_score = tmp_score.angle.score;
        if( tmp_mmv.vel.min_score > tmp_score.vel.score ) tmp_mmv.vel.min_score = tmp_score.vel.score;
        if( tmp_mmv.vel.max_score < tmp_score.vel.score ) tmp_mmv.vel.max_score = tmp_score.vel.score;
        //set score :
        *score = tmp_score;
        *mmv = tmp_mmv;
        return;
    }
    void DynamicWindowApproach::normalizeEvaluationValues( std::vector< EvaluatedPath > *paths_list, const MinMaxValue& mmv ) {
        for ( auto& pl : *paths_list ) {
            if ( mmv.goal.max_score - mmv.goal.min_score != 0.0 ) {
                pl.score.goal.norm_score = ( pl.score.goal.score - mmv.goal.min_score ) / ( mmv.goal.max_score - mmv.goal.min_score );
            } else pl.score.goal.norm_score = 0.0;
            if ( mmv.obs.max_score - mmv.obs.min_score != 0.0 ) {
                pl.score.obs.norm_score = ( pl.score.obs.score - mmv.obs.min_score ) / ( mmv.obs.max_score - mmv.obs.min_score );
            } else pl.score.obs.norm_score = 0.0;
            if ( mmv.angle.max_score - mmv.angle.min_score != 0.0 ) {
                pl.score.angle.norm_score = ( pl.score.angle.score - mmv.angle.min_score ) / ( mmv.angle.max_score - mmv.angle.min_score );
            } else pl.score.angle.norm_score = 0.0;
            if ( mmv.vel.max_score - mmv.vel.min_score != 0.0 ) {
                pl.score.vel.norm_score = ( pl.score.vel.score - mmv.vel.min_score ) / ( mmv.vel.max_score - mmv.vel.min_score );
            } else pl.score.vel.norm_score = 0.0; 
        }
    }
    // Display Optimal Path Marker :
    void DynamicWindowApproach::displayOptimalPathMarker ( const std::vector< Path >& paths ) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = target_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "optimal_path";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(1.0);
        marker.scale.x = 0.03;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        for ( auto& path : paths ) {
            geometry_msgs::Point temp;
            temp.x = path.next_x;
            temp.y = path.next_y;
            temp.z = 0.1;
            marker.points.push_back( temp );
        }
        pub_path_marker_.publish ( marker );
    }
    // Display ALL Path Marker :
    void DynamicWindowApproach::displayAllPathMarker ( std::vector< EvaluatedPath >& paths_list ) {
        int marker_num = 0;
        visualization_msgs::MarkerArray marker_all;
        for( auto pl : paths_list ) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = target_frame_;
            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.lifetime = ros::Duration(1.0);
            marker.ns = "all_path";
            marker.id = marker_num;
            marker.scale.x = 0.03;
            marker.color.a = 0.5;
            if ( pl.score.is_collision ) {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            for ( auto& path : pl.paths ) {
                geometry_msgs::Point pt;
                pt.x = path.next_x;
                pt.y = path.next_y;
                pt.z = 0.0;
                marker.points.push_back(pt);
            }
            marker_num++;
            marker_all.markers.push_back( marker );
        }
        pub_path_marker_all_.publish( marker_all );
    }

    DynamicWindowApproach::DynamicWindowApproach ( ) : nh_() {
        pub_path_marker_ = nh_.advertise< visualization_msgs::Marker >( "path_marker", 1 );
        pub_path_marker_all_ = nh_.advertise< visualization_msgs::MarkerArray >( "path_marker_all", 1 );
        // DWA Parameters :
        vel_step_ = 7.0;
        ang_vel_step_ = 15.0;
        std::vector<double> ang_range{ -1.0, 1.0 };
        setTargetFrame( "base_footprint" );
        setVelocityLimit ( ang_range, 1.5, 1.5, 1.1 );
        setWeight( 1.0, 1.0, 2.0, 1.0 );
        setCostDistance( 0.35, 0.35 );
        setStepValue( 30, 0.10, 7.0, 15.0 );
        setDisplayFlag ( false, false );
    }
    void DynamicWindowApproach::setTargetFrame ( const std::string& target_frame ) { target_frame_ = target_frame; }

    void DynamicWindowApproach::setVelocityLimit ( const std::vector<double>& ang_range, const double max_acc, const double max_ang_acc, const double acc_gain ) {
        ang_range_.resize(2);
        ang_vels_.clear();
        ang_range_[0] = ang_range[0];
        ang_range_[1] = ang_range[1];
        double delta_ang = ( ang_range_[1] - ang_range_[0] ) / ang_vel_step_;
        for ( double ang_vel = ang_range_[0]; ang_vel < ang_range_[1]; ang_vel += delta_ang ) ang_vels_.push_back(ang_vel);
        max_acc_ = max_acc;
        max_ang_acc_ = max_ang_acc;
        acc_gain_ = acc_gain;
    }
    void DynamicWindowApproach::setWeight ( const double goal, const double obs, const double ang, const double vel ) {
        weight_goal_ = goal;
        weight_obs_ = obs;
        weight_ang_ = ang;
        weight_vel_ = vel;
    }
    void DynamicWindowApproach::setCostDistance ( const double obstacle_center_radius, const double obstacle_points_radius ) {
        obstacle_center_radius_ = obstacle_center_radius;
        obstacle_points_radius_ = obstacle_points_radius;
    }
    void DynamicWindowApproach::setStepValue ( const int predict_step, const double sampling_time, const double vel_step, const double ang_vel_step ) {
        predict_step_ = predict_step;
        sampling_time_ = sampling_time;
        vel_step_ = vel_step;
        ang_vel_step_ = ang_vel_step;
    }
    void DynamicWindowApproach::setDisplayFlag ( const bool is_display_path, const bool is_display_all_path ) {
        is_disp_path_ = is_display_path;
        is_disp_all_path_ = is_display_all_path;
    }
    bool DynamicWindowApproach::generatePath2Target ( const geometry_msgs::Twist& curt_vel, const sobit_follower::FollowPositionConstPtr &fp, geometry_msgs::Twist* output_path ) {
        DWAParameters dwa_param;
        if( !setDWAParameters( curt_vel, fp->target_distance,  &dwa_param )) return false;
        
        fp_ = *fp;
        // make paths :
        std::vector< EvaluatedPath > paths_list;
        makePathsList( dwa_param, &paths_list );
        // Search the optimal path :
        double g = weight_goal_;
        double o = weight_obs_;
        double a = weight_ang_;
        double v = weight_vel_;
        double max_score = 0.0;
        double optimal_vel;
        double optimal_ang_vel;
        bool exists_path = false;
        std::vector< Path > optimal_paths;
        for ( auto& pl : paths_list ) {
            if ( pl.score.is_collision || ( pl.vel == 0.0 && pl.ang_vel == 0.0 ) ) continue;
            double sum_score = 	  g * pl.score.goal.norm_score 
                                + o * pl.score.obs.norm_score
                                + a * pl.score.angle.norm_score
                                + v * pl.score.vel.norm_score;
            if ( sum_score > max_score ) {
                max_score = sum_score;
                optimal_vel = pl.vel;
                optimal_ang_vel = pl.ang_vel;
                optimal_paths = pl.paths;
                exists_path = true;
            }		
        }
        // Update robot velocity :
        if ( exists_path ) {
            output_path->linear.x = optimal_vel;
            output_path->angular.z = optimal_ang_vel;
        }
        
        if ( is_disp_path_ ) displayOptimalPathMarker ( optimal_paths );
        if ( is_disp_all_path_ ) displayAllPathMarker ( paths_list );
        return exists_path;
    }
    
}