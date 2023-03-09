cmake_minimum_required(VERSION 2.8.3)
project(arms)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  trajectory_msgs
  dynamixel_sdk
  message_generation
  )

# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )
      
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	)

catkin_package(
	)

include_directories(
	${catkin_INCLUDE_DIRS}
	)

add_executable(servo_test src/servo_test.cpp)
add_executable(arm_node src/arm_node.cpp)
#add_executable(arm_torque_node src/arm_torque_node.cpp)
#add_executable(left_arm_real_simul_node src/left_arm_real_simul.cpp)
#add_executable(right_arm_real_simul_node src/right_arm_real_simul.cpp)

add_dependencies(servo_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(arm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(arm_torque_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_dependencies(left_arm_real_simul_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(right_arm_real_simul_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(servo_test
	${catkin_LIBRARIES}
)
target_link_libraries(arm_node
	${catkin_LIBRARIES}
)
#target_link_libraries(arm_torque_node
#	${catkin_LIBRARIES}
#)
# target_link_libraries(left_arm_real_simul_node
# 	${catkin_LIBRARIES}
# )
# target_link_libraries(right_arm_real_simul_node
# 	${catkin_LIBRARIES}	
# )
