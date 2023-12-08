cmake_minimum_required(VERSION 3.10.2)
project(multiple_observation_kalman_filter)

add_compile_options(-std=c++14 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    #  DEPENDS other non-ROS libs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
include_directories("/usr/include/eigen3")

add_library(${PROJECT_NAME}
    src/multiple_observation_kalman_filter.cpp
)
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