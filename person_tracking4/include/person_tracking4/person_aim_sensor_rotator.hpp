#ifndef PERSON_AIM_SENSOR_ROTATOR_HPP
#define PERSON_AIM_SENSOR_ROTATOR_HPP

#include <iostream>
#include <ros/ros.h> 
#include <ros/time.h>
#include <tf/transform_listener.h>  
#include <sobit_education_library/sobit_education_controller.hpp>
// #include <sobit_mini_library/sobit_mini_controller.hpp>
// #include <sobit_pro_library/sobit_pro_joint_controller.h>

namespace person_tracking4 {
	class PersonAimSensorRotator {
		private:
			ros::NodeHandle nh_;
			ros::NodeHandle pnh_;
			tf::TransformListener tf_listener_;
			sobit_education::SobitEducationController sobit_edu_ctr_;
			// sobit::SobitMiniController sobit_mini_ctr_;
			// sobit::SobitProJointController sobit_pro_ctr_;

			std::vector<double> tilt_angle_range_;
			double person_height_;
			
			double pre_tilt_;
			double pre_pan_;
			int pre_flag_;

			bool use_sobit_mini_;
			bool use_rotate_;
			
			double getAngleFrameDifference( std::string org_frame, std::string target_frame );
		
		public:
			PersonAimSensorRotator();
			void rotateSensor ( const int process_flag, const double target_distance, const double target_angle );
			void rotateSensor ( const double target_distance, const double target_angle );
	};
}

#endif