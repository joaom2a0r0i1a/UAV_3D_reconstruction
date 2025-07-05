cmake_minimum_required(VERSION 3.5)
project(pcl_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-O3)
add_definitions(-DCOMPILE_WITH_OUSTER)
# add_definitions(-g)

# disables anoying googletest warning
set(CATKIN_ENABLE_TESTING FALSE)

# disables anoying PCL warnings
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

set(CATKIN_DEPENDENCIES
  geometry_msgs
  mrs_lib
  mrs_msgs
  mrs_modules_msgs
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  )

find_package(PCL REQUIRED COMPONENTS
  common
  )

# Include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  DEPENDS Eigen
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )


## Add link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
  )

# This causes adds the -march=native gcc flag that prevents ditributed compilation
add_definitions(
  ${PCL_DEFINITIONS}
  )

## | ----------------------- Executables ---------------------- |

# FreespacePointcloudCreator

add_library(FreespacePointcloudCreator
  src/freespace_pointcloud_creator.cpp
  )

add_dependencies(FreespacePointcloudCreator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(FreespacePointcloudCreator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

target_include_directories(FreespacePointcloudCreator
  SYSTEM PRIVATE ${PCL_INCLUDE_DIRS}
  )

## --------------------------------------------------------------
## |                            Build                           |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/pcl_tools/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(TARGETS ${EXECUTABLES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
