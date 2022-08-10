cmake_minimum_required(VERSION 3.0.2)
project(person_tracking4)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O2 -g -Wall)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_filters
  point_cloud_tracker
  sobit_education_library
  sobit_mini_library
  # sobit_pro_library
  sobit_common_msg
  svdd_leg_tracker_library
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  FollowPosition.msg
  Obstacle.msg
  RotatePosition.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs   
  sensor_msgs   
  std_msgs

)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_filters point_cloud_tracker sobit_education_library sobit_mini_library sobit_common_msg svdd_leg_tracker_library # sobit_pro_library
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}   src/particle_person_tracker.cpp
                              src/svdd_particle_person_tracker.cpp
                              src/person_detector.cpp
                              src/pcl_handle.cpp
                              src/follow_position_handle.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)