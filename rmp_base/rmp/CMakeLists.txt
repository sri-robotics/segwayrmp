#list(APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost COMPONENTS system REQUIRED)

include_directories(include)

file(GLOB LIB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB LIB_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")

add_library(rmp STATIC
  ${LIB_SOURCES}
  ${LIB_HEADERS}
) 

target_link_libraries(rmp
  ${Boost_SYSTEM_LIBRARY}
)