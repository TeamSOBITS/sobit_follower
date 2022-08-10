cmake_minimum_required(VERSION 3.0.2)
project(person_follower4)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O2 -g -Wall)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  follow_path_generator4
  person_tracking4
  pluginlib
  nodelet
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES person_follower4
  CATKIN_DEPENDS follow_path_generator4 person_tracking4
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/person_tracker_nodelet.cpp
                            src/path_generator_nodelet.cpp
                            src/person_aim_sensor_rotator_nodelet.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
