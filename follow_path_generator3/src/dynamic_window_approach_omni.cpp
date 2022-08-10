#include <follow_path_generator3/dynamic_window_approach_omni.hpp>

namespace follow_path_generator3 {
    void DynamicWindowApproachOmni::makePathsListOmni( const DWAParameters& dwa_param, std::vector< EvaluatedPath >* paths_list ) {
        std::vector< EvaluatedPath > tmp_plist;
        EvaluatedPath paths;
        MinMaxValue mmv;
        bool is_predict;
        //double tgt_ang = fp_->target_angle;
        for ( double x_vel = dwa_param.min_vel; x_vel < dwa_param.max_vel; x_vel += dwa_param.delta_vel ) {
            for ( double y_vel = - dwa_param.max_vel; y_vel < dwa_param.max_vel; y_vel += dwa_param.delta_vel ) {
                is_predict = predictPathOmni ( x_vel, y_vel, dwa_param, &paths, &mmv );
                if( is_predict ) tmp_plist.push_back( paths );
            }
        }
        // Normalize evaluation values :
        normalizeEvaluationValues( &tmp_plist, mmv );
        *paths_list = tmp_plist;
    }
    bool DynamicWindowApproachOmni::predictPathOmni( const double x_vel, const double y_vel, const DWAParameters& dwa_param, EvaluatedPath* paths, MinMaxValue* mmv ) {
        MinMaxValue tmp_mmv = *mmv;
        EvaluatedPath tmp_path;
        Path path, pre_path; 
        tmp_path.x_vel = x_vel; tmp_path.y_vel = y_vel;
        int step = 0;
        bool is_collision = false;
        double min_obs_dist_score = 0.0;
        for ( step = 0; step < dwa_param.pre_step; ++step) {
            pre_path = path;
            // Calculate prediction points :
            path.next_x = x_vel * dwa_param.dt + pre_path.next_x;
            path.next_y = y_vel * dwa_param.dt + pre_path.next_y;
            path.next_theta = std::atan2( y_vel, x_vel );
            // Predicted point collides with obstacle :
            min_obs_dist_score = 0.0;
            for ( auto& obs : fp_->obstacles ) {
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
        }
        // Determine the path used to calculate the score :
        Path base_path = ( is_collision ) ? pre_path : path; 
        evaluatePath ( min_obs_dist_score, x_vel, base_path, &tmp_path.score, &tmp_mmv );
        tmp_path.score.is_collision = is_collision;
        *mmv = tmp_mmv;
        *paths = tmp_path;
        return step != 0 ? true : false; 
    }

    DynamicWindowApproachOmni::DynamicWindowApproachOmni ( ) {

    }

    bool DynamicWindowApproachOmni::generatePath2TargetOmni ( const geometry_msgs::Twist& curt_vel, const person_tracking3::FollowPosition& fp, geometry_msgs::Twist* output_path ) {
        DWAParameters dwa_param;
        if( !setDWAParameters( curt_vel, fp.target_distance,  &dwa_param )) return false;
        
        *fp_ = fp;
        // make paths :
        std::vector< EvaluatedPath > paths_list;
        makePathsListOmni( dwa_param, &paths_list );
        // Search the optimal path :
        double g = weight_goal_, o = weight_obs_, a = weight_ang_, v = weight_vel_;
        double max_score = 0.0, optimal_x_vel, optimal_y_vel, optimal_ang_vel;
        bool exists_path = false;
        std::vector< Path > optimal_paths;
        for ( auto& pl : paths_list ) {
            if ( pl.score.is_collision || ( pl.x_vel == 0.0 && pl.y_vel == 0.0 ) ) continue;
            double sum_score = 	  g * pl.score.goal.norm_score 
                                + o * pl.score.obs.norm_score
                                + a * pl.score.angle.norm_score
                                + v * pl.score.vel.norm_score;
            if ( sum_score > max_score ) {
                max_score = sum_score;
                optimal_x_vel = pl.x_vel;
                optimal_y_vel = pl.y_vel;
                optimal_ang_vel = pl.ang_vel;
                optimal_paths = pl.paths;
                exists_path = true;
            }		
        }
        // Update robot velocity :
        if ( exists_path ) {
            output_path->linear.x = optimal_x_vel;
            output_path->linear.y = optimal_y_vel;
            output_path->angular.z = optimal_ang_vel;
        }
        
        if ( is_disp_path_ ) displayOptimalPathMarker ( optimal_paths );
        if ( is_disp_all_path_ ) displayAllPathMarker ( paths_list );
        return exists_path;
    }
}