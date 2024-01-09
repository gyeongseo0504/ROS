cmake_minimum_required(VERSION 3.0.2)
project(gugudan)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
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
#  LIBRARIES gugudan
  CATKIN_DEPENDS roscpp rospy
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

add_executable(gugudan_publisher_node src/gugudan_publisher.cpp)
add_dependencies(gugudan_publisher_node ${gugudan_publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gugudan_publisher node ${catkin_LIBRARIES})

add_executable(gugudan_subscriber_node src/gugudan_subscriber.cpp)
add_dependencies(gugudan_subscriber node ${gugudan_subscriber_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gugudan_subscriber_node ${catkin_LIBRARIES})
