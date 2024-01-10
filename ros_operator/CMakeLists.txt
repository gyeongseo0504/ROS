cmake_minimum_required(VERSION 3.0.2)
project(my_second_pkg)

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
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_second_pkg
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

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/my_second_pkg.cpp
# )

add_executable(std_msgs_talker_node src/std_msgs_talker.cpp)
add_dependencies(std_msgs_talker_node ${std_msgs_talker_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(std_msgs_talker_node ${catkin_LIBRARIES} )

add_executable(std_msgs_listener_node src/std_msgs_listener.cpp)
add_dependencies(std_msgs_listener_node ${std_msgs_listener_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(std_msgs_listener_node ${catkin_LIBRARIES} )

add_executable(operator_node src/operator_node.cpp)
add_dependencies(operator_node ${operator_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(operator_node ${catkin_LIBRARIES} )
