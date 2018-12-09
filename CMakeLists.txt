cmake_minimum_required(VERSION 2.8.3)
project(setpoint_ctrl_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros
  mavros_msgs
  mavros_extras
  geometry_msgs
  tf2_ros
)

generate_messages(
   DEPENDENCIES
   std_msgs
   mavros_msgs
   geometry_msgs  
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES setpoint_ctrl_pkg
  CATKIN_DEPENDS geometry_msgs mavros mavros_extras roscpp std_msgs
 ## DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(setpoint_ctrl_node src/main.cpp src/swarm_control.cpp)
target_link_libraries(setpoint_ctrl_node ${catkin_LIBRARIES})