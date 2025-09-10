cmake_minimum_required(VERSION 3.0.2)
project(m_car_encoder_serial)

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
  serial
)
find_package(OpenCV REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msg geometry_msgs tf2_geometry_msgs sensor_msgs tf serial
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(

  ${catkin_INCLUDE_DIRS}
)

add_executable(henes_car_control_serial_node src/henes_car_control_serial_node.cpp)
add_dependencies(henes_car_control_serial_node ${henes_car_control_serial_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(henes_car_control_serial_node ${catkin_LIBRARIES} )

