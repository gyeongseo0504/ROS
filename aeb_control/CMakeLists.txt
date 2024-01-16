cmake_minimum_required(VERSION 3.0.2)
project(aeb_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES aeb_control
 CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(aeb_controller_node src/aeb_controller_node.cpp)
add_dependencies(aeb_controller_node ${aeb_controller_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aeb_controller_node ${catkin_LIBRARIES} )
