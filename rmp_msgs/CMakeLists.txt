cmake_minimum_required(VERSION 2.8.3)
project(rmp_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation 
  roscpp 
  rospy
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  MotorStatus.msg
  Battery.msg
  BoolStamped.msg
  AudioCommand.msg
  FaultStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS ${PROJECT_NAME}_generate_messages
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
