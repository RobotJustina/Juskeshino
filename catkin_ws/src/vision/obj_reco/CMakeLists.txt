cmake_minimum_required(VERSION 3.0.2)
project(obj_reco)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  vision_msgs
  pcl_ros
  visualization_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(obj_reco_node
  src/obj_reco_node.cpp
  src/Utils.cpp
  src/GeometricFeatures.cpp
  src/ObjectRecognizer.cpp
  src/ObjectPattern.cpp
)

add_dependencies(obj_reco_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(obj_reco_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
