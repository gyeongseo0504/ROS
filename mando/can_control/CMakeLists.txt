cmake_minimum_required(VERSION 3.0.2)
project(can_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python 스크립트 설치
catkin_install_python(PROGRAMS
  scripts/can_controller.py
  scripts/keyboard_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Launch 파일 설치
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Config 파일 설치
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.config"
)
