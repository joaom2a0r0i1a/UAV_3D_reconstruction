cmake_minimum_required(VERSION 3.15.0)
project(motion_planning_python)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_DEPENDENCIES
  rospy
  roscpp
  mrs_msgs
  visualization_msgs
  voxblox_ros
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
