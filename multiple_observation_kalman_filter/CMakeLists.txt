cmake_minimum_required(VERSION 3.10.2)
project(multiple_observation_kalman_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
add_compile_options(-O2 -g -Wall)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Add library
add_library(${PROJECT_NAME} src/multiple_observation_kalman_filter.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp Eigen3)

# Install targets
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp Eigen3)
ament_export_targets(${PROJECT_NAME}Targets)

ament_package()