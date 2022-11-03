# Produced by yoon 2022. 03. 18
# Modified by yoon 2022. 03. 18
# Modified by Kang & Yang 2022.11.02

cmake_minimum_required(VERSION 3.0.2)
project(Can2EtherCAT)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  Can2EtherCAT_Library
)

# ---- basics for Eigen start(1) ---- #
find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})
# ---- basics for Eigen end(1) ---- #

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Can2EtherCAT
  CATKIN_DEPENDS roscpp std_msgs Can2EtherCAT_Library
  DEPENDS system_lib Eigen3 
)



# ---- basics for Xenomai start(1) ---- #
include_directories(
  ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} include
  /usr/xenomai/include 
  /usr/xenomai/include/cobalt
)

link_directories(
/usr/xenomai/lib
)

link_libraries(
    alchemy
    cobalt
    copperplate
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

execute_process(COMMAND /usr/xenomai/bin/xeno-config --posix --alchemy --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND /usr/xenomai/bin/xeno-config --posix --alchemy --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}${XENO_CFLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${XENO_LDFLAGS}")

# ---- basics for Xenomai end(1) ---- #


# ---- basics for Node start(1) ---- #
add_executable(Anybus_test src/Anybus_test.cpp)
add_dependencies(Anybus_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Anybus_test ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(MD400T src/MD400T.cpp)
add_dependencies(MD400T ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(MD400T ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

# ---- basics for Node end(1) ---- #
### Refer : Any node & executable name must not be same with others

