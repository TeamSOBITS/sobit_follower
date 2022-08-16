cmake_minimum_required(VERSION 3.0.2)
project(multiple_observation_tracing_simulator)

#魔法の1行
add_compile_options(-std=c++14 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
  multiple_observation_kalman_filter
  message_filters
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   std_msgs#   visualization_msgs
# )

generate_dynamic_reconfigure_options(
  config/dynamic_reconfigure/TrackerParameter.cfg
  config/dynamic_reconfigure/ObservationParameter.cfg
  config/dynamic_reconfigure/TargetControllerParameter.cfg
  config/dynamic_reconfigure/VirtualSpringModelParameter.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES multiple_observation_tracing_simulator
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs nav_msgs roscpp std_msgs tf visualization_msgs message_filters
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)
include_directories("/usr/include/eigen3")

add_executable(multiple_observation_tracing_simulator_virtual_environment src/virtual_environment/virtual_environment.cpp)
target_link_libraries(multiple_observation_tracing_simulator_virtual_environment  ${catkin_LIBRARIES})
# add_dependencies(multiple_observation_tracing_simulator_virtual_environment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(multiple_observation_tracing_simulator_observed_publisher src/virtual_environment/observed_publisher.cpp)
target_link_libraries(multiple_observation_tracing_simulator_observed_publisher  ${catkin_LIBRARIES})
add_dependencies(multiple_observation_tracing_simulator_observed_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(multiple_observation_tracing_simulator_robot_pose_broadcaster src/virtual_environment/robot_pose_broadcaster.cpp)
target_link_libraries(multiple_observation_tracing_simulator_robot_pose_broadcaster  ${catkin_LIBRARIES})
# add_dependencies(multiple_observation_tracing_simulator_robot_pose_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(multiple_observation_tracing_simulator_target_controller src/virtual_environment/target_controller.cpp)
target_link_libraries(multiple_observation_tracing_simulator_target_controller  ${catkin_LIBRARIES})
# add_dependencies(multiple_observation_tracing_simulator_target_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(multiple_observation_tracing_simulator_tracker src/tracker.cpp)
target_link_libraries(multiple_observation_tracing_simulator_tracker  ${catkin_LIBRARIES})
# add_dependencies(multiple_observation_tracing_simulator_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(multiple_observation_tracing_simulator_following_control src/following_control/following_control.cpp
                                                                        src/following_control/virtual_spring_model.cpp)
target_link_libraries(multiple_observation_tracing_simulator_following_control  ${catkin_LIBRARIES})
# add_dependencies(multiple_observation_tracing_simulator_following_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})