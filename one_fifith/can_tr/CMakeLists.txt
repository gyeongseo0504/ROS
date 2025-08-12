cmake_minimum_required(VERSION 3.0.2)
project(can_tr)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(can_tr_node src/can_tr_node.cpp)
target_link_libraries(can_tr_node ${catkin_LIBRARIES})
# socketCAN 기본 시스템 라이브러리 연결
target_link_libraries(can_tr_node rt)
