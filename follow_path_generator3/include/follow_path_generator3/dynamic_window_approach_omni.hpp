#ifndef DYNAMIC_WINDOW_APPROACH_OMNI
#define DYNAMIC_WINDOW_APPROACH_OMNI

#include <follow_path_generator3/dynamic_window_approach.hpp>

namespace follow_path_generator3 {
    class DynamicWindowApproachOmni : public DynamicWindowApproach {
		private :

			void makePathsListOmni( const DWAParameters& dwa_param, std::vector< EvaluatedPath >* paths_list );
			bool predictPathOmni( const double vel_x, const double vel_y, const DWAParameters& dwa_param, EvaluatedPath* paths, MinMaxValue* mmv );

		public :
			DynamicWindowApproachOmni();
			bool generatePath2TargetOmni ( const geometry_msgs::Twist& curt_vel, const person_tracking3::FollowPosition& fp, geometry_msgs::Twist* output_path );
	};
}

#endif