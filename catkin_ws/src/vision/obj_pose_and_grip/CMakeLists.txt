cmake_minimum_required(VERSION 3.0.2)
project(obj_pose_and_grip)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  vision_msgs
  tf
  visualization_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(obj_pose_and_grip_node
  src/obj_pose_and_grip_node.cpp
)

add_dependencies(obj_pose_and_grip_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(obj_pose_and_grip_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

