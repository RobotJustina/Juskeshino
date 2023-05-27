cmake_minimum_required(VERSION 3.0.2)
project(vq_cuda)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  vision_msgs
  pcl_ros
  cv_bridge
  tf
  tf_conversions
)

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
)

include_directories(
 include
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)

set(CUDA_NVCC_FLAGS "-arch=compute_50" CACHE STRING "nvcc flags" FORCE)
set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE) 
set(LIB_TYPE STATIC)
cuda_add_library(CUDAVqLib ${LIB_TYPE} src/vq.cu)

add_executable(vq_cuda_node src/vq_cuda_node.cpp src/Utils.cpp)
add_dependencies(vq_cuda_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(vq_cuda_node CUDAVqLib)
target_link_libraries(vq_cuda_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  CUDAVqLib
 )

