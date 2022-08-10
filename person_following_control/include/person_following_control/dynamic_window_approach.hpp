#ifndef DYNAMIC_WINDOW_APPROACH
#define DYNAMIC_WINDOW_APPROACH

#include <ros/ros.h>
#include <cmath>
#include <cstring>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
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

		double min_vel;
		double max_vel;
        double min_ang_vel;
        double max_ang_vel;

		double vel_step;
        double ang_vel_step;

		std::vector<double> lin_vels;
		std::vector<double> ang_vels;

		int predict_step;		// pre_step
		double sampling_time;	// dt

		double obstacle_cost_radius;	// cost_obs_pt_rad

		double weight_goal;
		double weight_obs;
		double weight_ang;
		double weight_vel;

		double weight_vsm_ang;
		double weight_vsm_lin;
};

namespace person_following_control {
    class DynamicWindowApproach {
		protected :
			ros::NodeHandle nh_;
			ros::NodeHandle pnh_;
			ros::Publisher pub_path_marker_;
			ros::Publisher pub_path_marker_all_;

			bool display_optimal_path_;
			bool display_all_path_;

			std::shared_ptr<DWAParameters> dwap_;

			void displayOptimalPathMarker ( double optimal_vel, double optimal_ang_vel );
			void displayAllPathMarker ( std::vector< EvaluatedPath >& path_list );
		public :
			DynamicWindowApproach();

			void setTargetFrame ( const std::string& target_frame );
			void setVelocityLimit ( const double min_vel, const double max_vel, const double min_ang_vel, const double max_ang_vel, const double vel_step, const double ang_vel_step );
			void setWeight ( const double goal, const double obs, const double ang, const double vel, const double vsm_a, const double vsm_l );
			void setCostDistance ( const double obstacle_cost_radius );
			void setStepValue ( const int predict_step, const double sampling_time );
			void setDisplayFlag ( const bool display_optimal_path, const bool display_all_path );

			bool generatePath2Target (
				const geometry_msgs::Point& target,
                const PointCloud::Ptr obstacles,
				geometry_msgs::TwistPtr output_path );

			bool generatePath2Target (
				const geometry_msgs::Point& target,
                const PointCloud::Ptr obstacles,
				const geometry_msgs::TwistPtr base_path,
				geometry_msgs::TwistPtr output_path );
	};
}

inline void person_following_control::DynamicWindowApproach::setTargetFrame ( const std::string& target_frame ) { dwap_->target_frame = target_frame; }

inline void person_following_control::DynamicWindowApproach::setVelocityLimit ( const double min_vel, const double max_vel, const double min_ang_vel, const double max_ang_vel, const double vel_step, const double ang_vel_step  ) {
	dwap_->lin_vels.clear();
	dwap_->ang_vels.clear();
	std::vector<double> lin_vels;
	std::vector<double> ang_vels;
    double delta_lin = ( max_vel - min_vel) / vel_step;
    double delta_ang = ( max_ang_vel - min_ang_vel ) / ang_vel_step;
    for ( double vel = min_vel; vel < max_vel; vel += delta_lin ) lin_vels.push_back(vel);
	for ( double vel = min_ang_vel; vel < max_ang_vel; vel += delta_ang ) ang_vels.push_back(vel);
	dwap_->lin_vels = lin_vels;
	dwap_->ang_vels = ang_vels;
	dwap_->min_vel = min_vel;
	dwap_->max_vel = max_vel;
    dwap_->min_ang_vel = min_ang_vel;
    dwap_->max_ang_vel = max_ang_vel;
    dwap_->vel_step = vel_step;
    dwap_->ang_vel_step = ang_vel_step;
}
inline void person_following_control::DynamicWindowApproach::setWeight ( const double goal, const double obs, const double ang, const double vel, const double vsm_a, const double vsm_l ) {
	dwap_->weight_goal = goal;
	dwap_->weight_obs = obs;
	dwap_->weight_ang = ang;
	dwap_->weight_vel = vel;
	dwap_->weight_vsm_ang = vsm_a;
	dwap_->weight_vsm_lin = vsm_l;
}
inline void person_following_control::DynamicWindowApproach::setCostDistance ( const double obstacle_cost_radius ) {
	dwap_->obstacle_cost_radius = obstacle_cost_radius;
}
inline void person_following_control::DynamicWindowApproach::setStepValue ( const int predict_step, const double sampling_time ) {
	dwap_->predict_step = predict_step;
	dwap_->sampling_time = sampling_time;
}
inline void person_following_control::DynamicWindowApproach::setDisplayFlag ( const bool display_optimal_path, const bool display_all_path) {
	display_optimal_path_ = display_optimal_path;
	display_all_path_ = display_all_path;
}

#endif