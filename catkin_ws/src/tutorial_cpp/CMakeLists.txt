cmake_minimum_required(VERSION 3.0.2)
project(tutorial_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tutorial_cpp
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(publicador_ejemplo      #simple src/simple.cpp
                      src/publicador_ejemplo.cpp)

add_dependencies(publicador_ejemplo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(publicador_ejemplo
${catkin_LIBRARIES}
)

