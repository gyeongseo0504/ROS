cmake_minimum_required(VERSION 2.8.3)
project(my_first_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
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
#  LIBRARIES my_first_pkg
  CATKIN_DEPENDS roscpp rospy std_msgs
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

add_executable(talker_node src/talker.cpp)
add_dependencies(talker_node ${talker_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(talker_node ${catkin_LIBRARIES})

add_executable(listener_node src/listener.cpp)
add_dependencies(listener_node ${listener_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(listener_node ${catkin_LIBRARIES})
