cmake_minimum_required(VERSION 3.10.2)
project(multiple_observation_tracing_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
add_compile_options(-O2 -g -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(multiple_observation_kalman_filter REQUIRED)

# generate_dynamic_reconfigure_options( TODO:set as params
#   config/dynamic_reconfigure/TrackerParameter.cfg
#   config/dynamic_reconfigure/ObservationParameter.cfg
#   config/dynamic_reconfigure/TargetControllerParameter.cfg
#   config/dynamic_reconfigure/VirtualSpringModelParameter.cfg
# )

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(virtual_environment src/virtual_environment/virtual_environment.cpp)
ament_target_dependencies(virtual_environment rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs message_filters multiple_observation_kalman_filter)

# add_executable(observed_publisher src/virtual_environment/observed_publisher.cpp)
# ament_target_dependencies(observed_publisher rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros visualization_msgs message_filters multiple_observation_kalman_filter)

add_executable(robot_pose_broadcaster src/virtual_environment/robot_pose_broadcaster.cpp)
ament_target_dependencies(robot_pose_broadcaster rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs message_filters multiple_observation_kalman_filter)

add_executable(target_controller src/virtual_environment/target_controller.cpp)
ament_target_dependencies(target_controller rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros visualization_msgs message_filters multiple_observation_kalman_filter)

# add_executable(tracker src/tracker.cpp)
# ament_target_dependencies(tracker rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs message_filters multiple_observation_kalman_filter)
# target_link_libraries(tracker multiple_observation_kalman_filter)

add_executable(following_control  src/following_control/following_control.cpp 
                                  src/following_control/virtual_spring_model.cpp)
ament_target_dependencies(following_control rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs message_filters multiple_observation_kalman_filter)

install(TARGETS
  virtual_environment
  # observed_publisher
  robot_pose_broadcaster
  target_controller
  # tracker
  following_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch param config/rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()