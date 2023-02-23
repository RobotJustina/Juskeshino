cmake_minimum_required(VERSION 3.0.2)
project(human_pose_estimation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  message_generation
  vision_msgs
  )

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PCL REQUIRED)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  vision_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

add_executable(pose_estimator_srv src/pose_estimator_srv.cpp)
add_dependencies(pose_estimator_srv ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_estimator_srv
	${PCL_LIBRARIES}
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
	)

add_executable(streaming_keypoints src/streaming_keypoints.cpp)
add_dependencies(streaming_keypoints ${catkin_EXPORTED_TARGETS})
target_link_libraries(streaming_keypoints 
	${PCL_LIBRARIES}
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
	)
