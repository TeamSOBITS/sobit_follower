cmake_minimum_required(VERSION 3.10.2)
project(person_following_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(multiple_observation_kalman_filter REQUIRED)
find_package(multiple_sensor_person_tracking REQUIRED)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Add Executables
add_executable(robot_pose_broadcaster src/virtual_environment/robot_pose_broadcaster.cpp)
ament_target_dependencies(robot_pose_broadcaster rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs visualization_msgs)

# TODO
# add_executable(virtual_environment src/virtual_environment/virtual_environment.cpp)
# ament_target_dependencies(virtual_environment rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions visualization_msgs multiple_observation_kalman_filter multiple_sensor_person_tracking)

# Add Library
add_library(${PROJECT_NAME} SHARED
            src/following_control_component.cpp
            # src/hsrb_following_control_nodelet.cpp
            src/virtual_spring_model.cpp
            src/dynamic_window_approach.cpp
            src/pid_controller.cpp
            src/rviz_panel_plugin/qt_twist_panel.cpp
            src/rviz_panel_plugin/qt_touch.cpp
            )

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  rclcpp_components
  geometry_msgs 
  nav_msgs 
  pcl_conversions 
  pcl_ros 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  visualization_msgs 
  sensor_msgs 
  message_filters 
  rviz_common
  multiple_sensor_person_tracking)

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

# Register nodes as components explicitly
rclcpp_components_register_nodes(${PROJECT_NAME}
  "person_following_control::PersonFollowing"
)

# Install Targets
install(TARGETS 
  # robot_pose_broadcaster 
  # virtual_environment
  ${PROJECT_NAME}
  DESTINATION lib)

# Install Headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Install RViz Plugins
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param)

install(FILES
  component_description.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rclcpp_components component_description.xml)

# Testing & Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
