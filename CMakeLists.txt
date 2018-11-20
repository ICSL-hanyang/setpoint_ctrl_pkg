cmake_minimum_required(VERSION 2.8.3)
project(setpoint_ctrl_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES setpoint_ctrl_pkg
  CATKIN_DEPENDS geometry_msgs mavros roscpp std_msgs
 ## DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(swarm_algorithm_node src/swarm_algorithm.cpp)
#target_link_libraries(swarm_algorithm_node ${catkin_LIBRARIES})