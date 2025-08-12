cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} [OpenCV.cmake PATH])

project(ros_lane_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  cv_bridge
  image_transport
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV REQUIRED)
 

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES opencv_line_detection
  CATKIN_DEPENDS rospy roscpp std_msgs 
 # DEPENDS system_lib
  DEPENDS OpenCV
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIB_DIR})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/opencv_line_detection.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(ros_lane_control_node src/ros_lane_control.cpp)

target_link_libraries(ros_lane_control_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)


