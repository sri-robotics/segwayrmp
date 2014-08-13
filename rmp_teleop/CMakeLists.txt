cmake_minimum_required(VERSION 2.8.3)
project(rmp_teleop)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic")

find_package(catkin REQUIRED COMPONENTS
  rmp_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  diagnostic_updater
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rmp_msgs geometry_msgs roscpp sensor_msgs std_msgs diagnostic_updater
)

# Look for <linux/joystick.h>
include(CheckIncludeFiles)
check_include_files(linux/joystick.h HAVE_LINUX_JOYSTICK_H)

if(NOT HAVE_LINUX_JOYSTICK_H)
  message(FATAL_ERROR  "Warning: no <linux/joystick.h>; won't build joy node")
endif()

include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_node src/joy_node.cpp)

target_link_libraries(joy_node 
  ${catkin_LIBRARIES}
)

add_executable(rmp_teleop_node 
  src/rmp_teleop_node.cpp
  src/RmpJoystickConverter.cpp
  src/RmpXboxConverter.cpp
)

target_link_libraries(rmp_teleop_node
  ${catkin_LIBRARIES}
)

add_dependencies(rmp_teleop_node rmp_msgs_generate_messages_cpp)

install(TARGETS joy_node rmp_teleop_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*~" EXCLUDE
)

option(BUILD_RMP_TELEOP_DOC "Create documentation" OFF)
if(BUILD_RMP_TELEOP_DOC)
  find_package(Doxygen)
  if (NOT DOXYGEN_FOUND)
    message(FATAL_ERROR "Doxygen is needed to build the documentation. Please install it correctly")
  endif()

  configure_file(Doxyfile.in 
    ${PROJECT_BINARY_DIR}/Doxyfile  @ONLY IMMEDIATE
  )

  add_custom_target(teleop_doc ALL 
    COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
    SOURCES ${PROJECT_BINARY_DIR}/Doxyfile
  )

  install(DIRECTORY ${PROJECT_BINARY_DIR}/doc
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN ".svn" EXCLUDE
  )
endif()
