cmake_minimum_required(VERSION 3.10.2)
project(sobit_follower)

# Find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

ament_package()