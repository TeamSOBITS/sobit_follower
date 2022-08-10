cmake_minimum_required(VERSION 3.0.2)
project(person_tracking3)

## Compile as C++11, supported in ROS Kinetic and newer
#魔法の1行
add_compile_options(-std=c++11 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  sobit_education_library
  sobit_mini_library
  sobit_pro_library
  ssd_node
  svdd_leg_tracker_library
  point_cloud_tracker
  message_filters
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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sobit_education_library sobit_mini_library sobit_pro_library ssd_node svdd_leg_tracker_library point_cloud_tracker message_filters
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}   src/person_detector.cpp
                              src/particle_person_tracker.cpp
                              src/svdd_particle_person_tracker.cpp
                              src/pcl_handle.cpp
                              src/follow_position_handle.cpp
                              src/person_aim_sensor_rotator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

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