cmake_minimum_required(VERSION 3.0.2)
project(Can2EtherCAT_Library)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  soem
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES Can2EtherCAT_Library
 CATKIN_DEPENDS roscpp soem
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/Can2EtherCAT_Library.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
