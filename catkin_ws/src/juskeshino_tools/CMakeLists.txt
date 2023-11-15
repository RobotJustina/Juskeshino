cmake_minimum_required(VERSION 3.0.2)
project(juskeshino_tools)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  hardware_msgs
  hri_msgs
  manip_msgs
  nav_msgs
  navig_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  vision_msgs
  visualization_msgs
  message_generation
  sound_play
)
catkin_python_setup()
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

generate_messages(
  DEPENDENCIES
  geometry_msgs   sensor_msgs   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES juskeshino_tools
  CATKIN_DEPENDS cv_bridge geometry_msgs hardware_msgs hri_msgs manip_msgs nav_msgs navig_msgs roscpp rospy sensor_msgs std_msgs tf vision_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(juskeshino_tools
  src/JuskeshinoNavigation.cpp
  src/JuskeshinoVision.cpp
)

add_dependencies(juskeshino_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(juskeshino_tools
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
