cmake_minimum_required(VERSION 3.0.2)
project(ir2425_group_26_asign_2)

## Find catkin and any required components
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  tf2_ros
  actionlib
)

## Declare a catkin package
catkin_package()

## Ensure scripts directory exists
install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
