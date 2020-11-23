cmake_minimum_required(VERSION 2.8.3)
project(kitt3_drive)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  serial
  message_generation
  
)

add_message_files(
  FILES
  Position.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kitt5_dirve
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include/kitt3_drive
  ${catkin_INCLUDE_DIRS}
)

LIST(APPEND SOURCEFILES
  src/motion.cpp
  src/controller.cpp
  src/can_bus
  )

add_library(kittMotion
  src/motion.cpp
)
add_library(kittController
  src/controller.cpp
)
add_library(imu
  src/imu.cpp
)
add_library(can
  src/can_bus.cpp
)

#add_executable(kitt3_drive_node src/robot_drive_node.cpp src/motion.cpp src/imu.cpp)
#add_executable(kitt3_controller_node src/robot_control_node.cpp src/controller.cpp)
add_executable(kitt3_drive_node src/robot_drive_node.cpp )
add_executable(kitt3_controller_node src/robot_control_node.cpp )
add_executable(kitt3_cmd_node src/robot_control_node.cpp )


target_link_libraries(kitt3_drive_node
  ${PROJECT_SOURCE_DIR}/lib/libimu.so
  ${PROJECT_SOURCE_DIR}/lib/libkittMotion.so
  ${PROJECT_SOURCE_DIR}/lib/libcan.so
  ${catkin_LIBRARIES}
#  ${PROJECT_SOURCE_DIR}/include/robot_drive/libhello.so
)

target_link_libraries(kitt3_controller_node
 ${catkin_LIBRARIES}
 ${PROJECT_SOURCE_DIR}/lib/libkittController.so
#   ${PROJECT_SOURCE_DIR}/include/robot_drive/libhello.so
)

target_link_libraries(kitt3_cmd_node
 ${catkin_LIBRARIES}
 ${PROJECT_SOURCE_DIR}/lib/libcan.so
 ${PROJECT_SOURCE_DIR}/lib/libkittController.so
#   ${PROJECT_SOURCE_DIR}/include/robot_drive/libhello.so
)
