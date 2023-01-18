cmake_minimum_required(VERSION 2.8.3)
project(torso)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  hardware_tools
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(torso_node_pololu src/torso_node_pololu.cpp)
add_executable(torso_node_fake   src/torso_fake.cpp)

add_dependencies(torso_node_pololu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(torso_node_fake   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(torso_node_pololu JrkManager
	${catkin_LIBRARIES}
)
target_link_libraries(torso_node_fake
	${catkin_LIBRARIES}
)
