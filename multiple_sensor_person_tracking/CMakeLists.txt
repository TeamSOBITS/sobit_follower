cmake_minimum_required(VERSION 3.10)
project(multiple_sensor_person_tracking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sobits_interfaces REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
# find_package(sobit_edu_library REQUIRED) 
# find_package(sobit_pro_library REQUIRED)
# find_package(hsrb_library REQUIRED)
find_package(multiple_observation_kalman_filter REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FollowingPosition.msg"
  "srv/LegDetectionService.srv"
  DEPENDENCIES sensor_msgs geometry_msgs std_msgs
)

include_directories(include ${sobit_edu_library_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_component SHARED
  src/multiple_sensor_person_tracker_component.cpp 
  # src/sobit_edu/sobit_edu_person_aim_sensor_rotator_component.cpp
  # src/sobit_pro/sobit_pro_person_aim_sensor_rotator_component.cpp
  # src/hsrb/hsrb_person_aim_sensor_rotator_component.cpp
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  sobits_interfaces
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  laser_geometry
  pluginlib
  # sobit_edu_library
  # sobit_pro_library
  # hsrb_library
  multiple_observation_kalman_filter
  OpenCV
  cv_bridge
)

target_link_libraries(${PROJECT_NAME}_component ${OpenCV_LIBRARIES})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_component "${cpp_typesupport_target}")


install(TARGETS ${PROJECT_NAME}_component
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
