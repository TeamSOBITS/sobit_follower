cmake_minimum_required(VERSION 3.0.2)
project(multiple_sensor_person_tracking)

add_compile_options(-std=c++14 -O2 -g -Wall)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  laser_geometry
  dr_spaam_ros
  sobit_edu_library
  ssd_nodelet
  pluginlib
  nodelet
  dynamic_reconfigure
  multiple_observation_kalman_filter
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LegPoseArray.msg
  FollowingPosition.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  LegDetectionService.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs geometry_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  config/dynamic_reconfigure/TrackerParameter.cfg
  config/dynamic_reconfigure/SensorRotatorParameter.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS dr_spaam_ros sobit_edu_library ssd_nodelet
  DEPENDS OpenCV
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/multiple_sensor_person_tracker_nodelet.cpp
                            src/person_aim_sensor_rotator_nodelet.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})