cmake_minimum_required(VERSION 3.0.2)
project(move_waypoint)

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
  message_generation
)

add_message_files(
	FILES
	Target_waypoint_line.msg
)

generate_messages(
	DEPENDENCIES geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES move_waypoint
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs message_runtime
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

add_executable(move_waypoint_node_node src/move_waypoint_node.cpp)
add_dependencies(move_waypoint_node_node ${move_waypoint_node_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_waypoint_node_node ${catkin_LIBRARIES} )
