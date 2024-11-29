cmake_minimum_required(VERSION 3.15.0)
project(process_pointcloud)

# Enable parallel build
set(CMAKE_BUILD_PARALLEL_LEVEL 4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options("-O3")

## Compile as C++14, supported in ROS noetic and newer
#set(CMAKE_CXX_STANDARD 14) # required by new PCL
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

#find_package(catkin REQUIRED COMPONENTS
#  tf
#  sensor_msgs
#  pcl_conversions
#  pcl_ros
#  mrs_lib
#  geometry_msgs
#  comm_msgs
#)

set(CATKIN_DEPENDENCIES
  tf
  sensor_msgs
  pcl_conversions
  pcl_ros
  mrs_lib
  geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS include
)
add_executable(prune_node src/prune_node.cc src/prune.cc)

catkin_package(
  INCLUDE_DIRS include
)
add_executable(process_node src/process_pointcloud_node.cc src/process_pointcloud.cc)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

target_link_libraries(prune_node
  ${catkin_LIBRARIES}
)

target_link_libraries(process_node
  ${catkin_LIBRARIES}
)

