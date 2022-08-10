cmake_minimum_required(VERSION 3.0.2)
project(sobit_follower)

find_package(catkin REQUIRED COMPONENTS
  multiple_sensor_person_tracking
  person_following_control
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sobit_follower
#  CATKIN_DEPENDS multiple_sensor_person_tracking person_following_control
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)