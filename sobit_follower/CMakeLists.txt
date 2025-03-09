cmake_minimum_required(VERSION 3.10.2)
project(sobit_follower)

# Find dependencies
find_package(ament_cmake REQUIRED)
# find_package(multiple_sensor_person_tracking REQUIRED)
# find_package(person_following_control REQUIRED)

# Include directories
include_directories(
  # include
  # ${multiple_sensor_person_tracking_INCLUDE_DIRS}
  # ${person_following_control_INCLUDE_DIRS}
)

ament_package()