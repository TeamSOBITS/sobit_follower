cmake_minimum_required(VERSION 2.8.3)
project(sobit_follower)

add_compile_options(-std=c++14 -O2 -g -Wall)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  trajectory_msgs
  message_filters
  pcl_ros
  laser_geometry
  ssd_node
  sobit_common_msg
  message_generation
  cv_bridge
  sobit_education_library
  sobit_mini_library
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  FollowPosition.msg
  Obstacle.msg
)
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sobit_follower
  CATKIN_DEPENDS geometry_msgs message_filters pcl_ros roscpp rospy sensor_msgs ssd_node std_msgs visualization_msgs sobit_education_library sobit_mini_library
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(sf_person_tracker src/person_tracker/person_tracker.cpp
                              src/person_tracker/person_detector.cpp
                              src/person_tracker/follow_position_handle.cpp
                              src/sub_functions/standard_point_cloud_handle.cpp
                              src/sub_functions/point_cloud_tracker.cpp
                              src/sub_functions/laser_scan_handle.cpp)
target_link_libraries(sf_person_tracker  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(sf_person_tracker ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sf_person_aim_xtion_rotator src/sub_functions/person_aim_xtion_rotator.cpp)
target_link_libraries(sf_person_aim_xtion_rotator ${catkin_LIBRARIES})
add_dependencies(sf_person_aim_xtion_rotator ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sf_path_generator src/path_generator/path_generator.cpp
                              src/path_generator/dynamic_window_approach.cpp
                              src/path_generator/pid_controller.cpp)
target_link_libraries(sf_path_generator  ${catkin_LIBRARIES})
add_dependencies(sf_path_generator ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sf_follow_simulator src/follow_simulator/follow_simulator.cpp)
target_link_libraries(sf_follow_simulator  ${catkin_LIBRARIES})
add_dependencies(sf_follow_simulator ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sf_robot_pose_broadcaster src/follow_simulator/robot_pose_broadcaster.cpp)
target_link_libraries(sf_robot_pose_broadcaster  ${catkin_LIBRARIES})
add_dependencies(sf_robot_pose_broadcaster ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sf_sensor_data_saver src/follow_simulator/sensor_data_saver.cpp
                                  src/sub_functions/standard_point_cloud_handle.cpp)
target_link_libraries(sf_sensor_data_saver ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(sf_sensor_data_saver ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sf_sensor_data_publisher src/follow_simulator/sensor_data_publisher.cpp
                                  src/sub_functions/standard_point_cloud_handle.cpp)
target_link_libraries(sf_sensor_data_publisher ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(sf_sensor_data_publisher ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sf_follow_position_recoder src/follow_simulator/follow_position_recoder.cpp)
target_link_libraries(sf_follow_position_recoder  ${catkin_LIBRARIES})
add_dependencies(sf_follow_position_recoder ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sf_follow_me_handle src/sub_functions/follow_me_handle.cpp)
target_link_libraries(sf_follow_me_handle  ${catkin_LIBRARIES})
add_dependencies(sf_follow_me_handle ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sf_follow_me_talker src/sub_functions/follow_me_talker.cpp)
target_link_libraries(sf_follow_me_talker  ${catkin_LIBRARIES})
add_dependencies(sf_follow_me_talker ${PROJECT_NAME}_generate_messages_cpp)