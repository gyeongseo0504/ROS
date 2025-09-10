cmake_minimum_required(VERSION 3.0.2)
project(utm_to_odom)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msg geometry_msgs tf2_geometry_msgs sensor_msgs tf 
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(

  ${catkin_INCLUDE_DIRS}
)

add_executable(utm_odom_node src/utm_odom.cpp)
add_dependencies(utm_odom_node ${utm_odom_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(utm_odom_node ${catkin_LIBRARIES} )

