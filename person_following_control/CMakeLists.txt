cmake_minimum_required(VERSION 3.0.2)
project(person_following_control)

add_compile_options(-std=c++14 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
  sensor_msgs
  message_generation
  message_filters
  rviz
  multiple_observation_kalman_filter
  multiple_sensor_person_tracking
)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

# # Generate messages in the 'msg' folder
# add_message_files(
#   FILES
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

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs   nav_msgs   std_msgs   visualization_msgs sensor_msgs
)

# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  config/dynamic_reconfigure/PersonFollowingParameter.cfg
  config/dynamic_reconfigure/VirtualEnvironmentParameter.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs nav_msgs nodelet pcl_conversions pcl_ros roscpp std_msgs tf visualization_msgs rviz
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(person_following_control_robot_pose_broadcaster src/virtual_environment/robot_pose_broadcaster.cpp)
target_link_libraries(person_following_control_robot_pose_broadcaster  ${catkin_LIBRARIES})
# add_dependencies(person_following_control_robot_pose_broadcaster ${PROJECT_NAME}_generate_messages_cpp)

add_executable(person_following_control_virtual_environment src/virtual_environment/virtual_environment.cpp)
target_link_libraries(person_following_control_virtual_environment  ${catkin_LIBRARIES})
# add_dependencies(person_following_control_virtual_environment ${PROJECT_NAME}_generate_messages_cpp)

add_library(${PROJECT_NAME} src/sobit_edu_following_control_nodelet.cpp
                            src/sobit_pro_following_control_nodelet.cpp
                            src/virtual_spring_model.cpp
                            src/dynamic_window_approach.cpp
                            src/pid_controller.cpp
                            src/rviz_panel_plugin/qt_twist_panel.cpp
                            src/rviz_panel_plugin/qt_touch.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})