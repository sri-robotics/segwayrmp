cmake_minimum_required(VERSION 2.8.3)
project(rmp_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
#  xacro # does not work with groovy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  DEPENDS xacro
  CATKIN_DEPENDS urdf 
)

foreach(directory launch meshes robots urdf) 
  install(DIRECTORY ${directory}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*~" EXCLUDE
  )
endforeach()
