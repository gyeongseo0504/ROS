cmake_minimum_required(VERSION 3.0.2)
project(gps_imu_link)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
#  LIBRARIES gps_imu_link
   CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
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
#   src/${PROJECT_NAME}/gps_imu_link.cpp
# )

add_executable(gps_imu_link_node src/gps_imu_link.cpp)
add_dependencies(gps_imu_link_node ${gps_imu_link_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_imu_link_node ${catkin_LIBRARIES} )


add_executable(gps_imu_link_new_node src/gps_imu_link_new.cpp)
add_dependencies(gps_imu_link_new_node ${gps_imu_link_new_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_imu_link_new_node ${catkin_LIBRARIES} )


add_executable(gps_imu_link_test_node src/gps_imu_link_test.cpp)
add_dependencies(gps_imu_link_test_node ${gps_imu_link_test_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_imu_link_test_node ${catkin_LIBRARIES} )

add_executable(gps_yaw_node src/gps_yaw_estimation.cpp)
add_dependencies(gps_yaw_node ${gps_yaw_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_yaw_node ${catkin_LIBRARIES} )
