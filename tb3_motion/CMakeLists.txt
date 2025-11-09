cmake_minimum_required(VERSION 3.0.2)
project(tb3_motion)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  # image_transport   # uncomment if you start using it
)

## Declare the catkin package
catkin_package()

## Include dirs
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python nodes (ensure executable bit and proper shebang)
catkin_install_python(PROGRAMS
  scripts/detect_lane.py
  scripts/control_lane.py
  scripts/intersection.py
  scripts/construction.py
  scripts/motion_core.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch and config assets
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)