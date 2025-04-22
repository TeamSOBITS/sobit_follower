#ifndef DYNAMIC_WINDOW_APPROACH
#define DYNAMIC_WINDOW_APPROACH

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstring>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class MinMaxScore {
	public :
		double min_score = DBL_MAX;
		double max_score = DBL_MIN;
};
class MinMaxValue {
	public :
		MinMaxScore heading;
		MinMaxScore obstacle;
		MinMaxScore velocity;
		MinMaxScore velocity_angle;  // only vsm dwa
};

class Path {
	public :
		pcl::PointXYZ point;
		double theta = 0.0;
};
class EvaluatedPath {
	public :
		double linear;
		double angular;

        double heading;
		double obstacle;
		double velocity;
        double velocity_angle;    // only vsm dwa
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

		std::vector<double> linear_list;
		std::vector<double> angular_list;

		int predict_step;		// pre_step
		double sampling_time;	// dt

		double obstacle_cost_radius;	// cost_obstacle_pt_rad

        double weight_heading;
		double weight_obstacle;
		double weight_velocity;

        double weight_vsm_linear;   // only vsm dwa
		double weight_vsm_angular;  // only vsm dwa

};

namespace person_following_control {
    class DynamicWindowApproach {
		protected :
			std::shared_ptr<rclcpp::Node> node_;
			rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_path_marker_;
            		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_path_marker_all_;

			bool display_optimal_path_;
			bool display_all_path_;

			std::shared_ptr<DWAParameters> dwap_;

			void displayOptimalPathMarker ( const EvaluatedPath& optimal_path );
			void displayAllPathMarker ( const std::vector< EvaluatedPath >& path_list );
		public :
			DynamicWindowApproach( rclcpp::Node* node );

			void setTargetFrame ( const std::string& target_frame );

			void setVelocityLimit (
                const double min_vel,
                const double max_vel,
                const double min_ang_vel,
                const double max_ang_vel,
                const double vel_step,
                const double ang_vel_step );

			void setWeight (
                const double heading,
                const double obstacle,
                const double velocity,
                const double linear,
                const double angular );

			void setCostDistance ( const double obstacle_cost_radius );
			void setStepValue ( const int predict_step, const double sampling_time );
			void setDisplayFlag ( const bool display_optimal_path, const bool display_all_path );

			bool generatePath2TargetDWA (
				const geometry_msgs::msg::Point& target,
                const PointCloud::Ptr obstacles,
				geometry_msgs::msg::Twist& output_path );

			bool generatePath2TargetVSMDWA (
				const geometry_msgs::msg::Point& target,
                const PointCloud::Ptr obstacles,
				geometry_msgs::msg::Twist& output_path );

			bool generatePath2Target (
				const geometry_msgs::msg::Point& target,
                const PointCloud::Ptr obstacles,
				geometry_msgs::msg::Twist& output_path );
	};
}

inline void person_following_control::DynamicWindowApproach::setTargetFrame ( const std::string& target_frame ) { dwap_->target_frame = target_frame; }

inline void person_following_control::DynamicWindowApproach::setVelocityLimit (
    const double min_vel,
    const double max_vel,
    const double min_ang_vel,
    const double max_ang_vel,
    const double vel_step,
    const double ang_vel_step )
{
	dwap_->linear_list.clear();
	dwap_->angular_list.clear();
	std::vector<double> linear_list;
	std::vector<double> angular_list;
    double delta_lin = ( max_vel - min_vel) / vel_step;
    double delta_ang = ( max_ang_vel - min_ang_vel ) / ang_vel_step;
    for ( double vel = min_vel; vel < max_vel; vel += delta_lin ) linear_list.push_back(vel);
	for ( double vel = min_ang_vel; vel < max_ang_vel; vel += delta_ang ) angular_list.push_back(vel);
	dwap_->linear_list = linear_list;
	dwap_->angular_list = angular_list;
	dwap_->min_vel = min_vel;
	dwap_->max_vel = max_vel;
    dwap_->min_ang_vel = min_ang_vel;
    dwap_->max_ang_vel = max_ang_vel;
    dwap_->vel_step = vel_step;
    dwap_->ang_vel_step = ang_vel_step;
}
inline void person_following_control::DynamicWindowApproach::setWeight (
    const double heading,
    const double obstacle,
    const double velocity,
    const double linear,
    const double angular )
{
	dwap_->weight_heading = heading;
	dwap_->weight_obstacle = obstacle;
	dwap_->weight_velocity = velocity;
	dwap_->weight_vsm_linear = linear;
	dwap_->weight_vsm_angular = angular;
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

#endif // DYNAMIC_WINDOW_APPROACH_HPP