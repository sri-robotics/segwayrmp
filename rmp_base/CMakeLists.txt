cmake_minimum_required(VERSION 2.8.3)
project(rmp_base)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic")

add_subdirectory(rmp)

find_package(catkin REQUIRED COMPONENTS
  rmp_msgs
  tf
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rmp_msgs tf geometry_msgs nav_msgs roscpp sensor_msgs std_msgs
)

include_directories(include
  rmp/include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rmp_440le_node src/rmp_440le_node.cpp src/Rmp440LE.cpp)

target_link_libraries(rmp_440le_node
  ${catkin_LIBRARIES}
  rmp
)

install(TARGETS rmp_440le_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*~" EXCLUDE
  PATTERN ".svn" EXCLUDE
)

option(BUILD_RMP_BASE_DOC "Create documentation" OFF)
if(BUILD_RMP_BASE_DOC)
  find_package(Doxygen)
  if (NOT DOXYGEN_FOUND)
    message(FATAL_ERROR "Doxygen is needed to build the documentation. Please install it correctly")
  endif()

  configure_file(Doxyfile.in 
    ${PROJECT_BINARY_DIR}/Doxyfile  @ONLY IMMEDIATE
  )

  add_custom_target(base_doc ALL 
    COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
    SOURCES ${PROJECT_BINARY_DIR}/Doxyfile
  )

  install(DIRECTORY ${PROJECT_BINARY_DIR}/doc
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN ".svn" EXCLUDE
  )
endif()
