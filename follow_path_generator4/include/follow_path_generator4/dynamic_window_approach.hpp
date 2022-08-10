#ifndef DYNAMIC_WINDOW_APPROACH
#define DYNAMIC_WINDOW_APPROACH

#include <ros/ros.h>
#include <cmath>
#include <cstring>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <person_tracking4/follow_position_handle.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <geometry_msgs/Point.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class Score {
	public :        
		double score;
		double norm_score;
};
class MinMaxScore {
	public :        
		double min_score = DBL_MAX;
		double max_score = DBL_MIN;
};
class MinMaxValue {
	public :      
		MinMaxScore goal;
		MinMaxScore obs;
		MinMaxScore angle; 
		MinMaxScore vel;
};

class Path {
	public :        
		pcl::PointXYZ point;
		double theta = 0.0;
};
class EvaluatedPath {
	public :
		double vel;
		double ang_vel;

		Score goal;
		Score obs;
		Score angle; 
		Score vel_s;
		bool is_collision; 
};

class DWAParameters {
	public :
		std::string target_frame;

		std::vector<double> lin_vels;
		std::vector<double> ang_vels;
		double max_acc;
		double max_ang_acc;
		double acc_gain;

		int predict_step;		// pre_step
		double sampling_time;	// dt
		double vel_step;
		double ang_vel_step;

		double obstacle_cost_radius;	// cost_obs_pt_rad

		double weight_goal;
		double weight_obs;
		double weight_ang;
		double weight_vel;

		// Set by setDWAParameters
		double min_vel;
		double max_vel;
		double delta_vel;
		double curt_vel;
		double curt_ang_vel;
};

namespace follow_path_generator4 {
    class DynamicWindowApproach {
		protected :
			ros::NodeHandle nh_;
			ros::NodeHandle pnh_;
			ros::Publisher pub_path_marker_;
			ros::Publisher pub_path_marker_all_;

			bool is_disp_path_;
			bool is_disp_all_path_;

			std::shared_ptr<DWAParameters> dwap_;

			void displayOptimalPathMarker ( double optimal_vel, double optimal_ang_vel );
			void displayAllPathMarker ( std::vector< EvaluatedPath >& path_list );
		public :
			DynamicWindowApproach();
			
			void setTargetFrame ( const std::string& target_frame );
			void setVelocityLimit ( const std::vector<double>& lin_range, const std::vector<double>& ang_range, const double max_acc, const double max_ang_acc, const double acc_gain );
			void setWeight ( const double goal, const double obs, const double ang, const double vel );
			void setCostDistance ( const double obstacle_cost_radius );
			void setStepValue ( const int predict_step, const double sampling_time, const double vel_step, const double ang_vel_step );
			void setDisplayFlag ( const bool is_display_path, const bool is_display_all_path );
			
			bool generatePath2Target ( 
				const geometry_msgs::Point& target,
                const PointCloud::Ptr obstacles,
				const geometry_msgs::TwistConstPtr curt_vel,
				geometry_msgs::TwistPtr output_path );
	};
}

inline void follow_path_generator4::DynamicWindowApproach::setTargetFrame ( const std::string& target_frame ) { dwap_->target_frame = target_frame; }

inline void follow_path_generator4::DynamicWindowApproach::setVelocityLimit ( const std::vector<double>& lin_range, const std::vector<double>& ang_range, const double max_acc, const double max_ang_acc, const double acc_gain ) {
	dwap_->lin_vels.clear();
	dwap_->ang_vels.clear();

	double delta_lin = ( lin_range[1] - lin_range[0] ) / dwap_->vel_step;
	for ( double vel = lin_range[0]; vel < lin_range[1]; vel += delta_lin ) dwap_->lin_vels.push_back(vel);
	double delta_ang = ( ang_range[1] - ang_range[0] ) / dwap_->ang_vel_step;
	for ( double ang_vel = ang_range[0]; ang_vel < ang_range[1]; ang_vel += delta_ang ) dwap_->ang_vels.push_back(ang_vel);

	dwap_->max_acc = max_acc;
	dwap_->max_ang_acc = max_ang_acc;
	dwap_->acc_gain = acc_gain;
}
inline void follow_path_generator4::DynamicWindowApproach::setWeight ( const double goal, const double obs, const double ang, const double vel ) {
	dwap_->weight_goal = goal;
	dwap_->weight_obs = obs;
	dwap_->weight_ang = ang;
	dwap_->weight_vel = vel;
}
inline void follow_path_generator4::DynamicWindowApproach::setCostDistance ( const double obstacle_cost_radius ) {
	dwap_->obstacle_cost_radius = obstacle_cost_radius;
}
inline void follow_path_generator4::DynamicWindowApproach::setStepValue ( const int predict_step, const double sampling_time, const double vel_step, const double ang_vel_step ) {
	dwap_->predict_step = predict_step;
	dwap_->sampling_time = sampling_time;
	dwap_->vel_step = vel_step;
	dwap_->ang_vel_step = ang_vel_step;
}
inline void follow_path_generator4::DynamicWindowApproach::setDisplayFlag ( const bool is_display_path, const bool is_display_all_path ) {
	is_disp_path_ = is_display_path;
	is_disp_all_path_ = is_display_all_path;
}

#endif