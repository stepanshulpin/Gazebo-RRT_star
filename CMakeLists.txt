cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (CMAKE_AUTOMOC ON)

find_package(gazebo REQUIRED)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(RRT_star SHARED launch_world.cc BoxExpanded.hh BoxExpanded.cc RRT_star.hh RRT_star.cc ModelExpanded.hh ModelExpanded.cc)
target_link_libraries(RRT_star  ${GAZEBO_LIBRARIES})
