cmake_minimum_required(VERSION 3.15.0)
project(cache_nodes)

# Enable parallel build
set(CMAKE_BUILD_PARALLEL_LEVEL 4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options("-O3")

set(CATKIN_DEPENDENCIES
  rospy
  roscpp
  mrs_msgs
  mrs_lib
  visualization_msgs
  trajectory_generation
  voxblox_ros
  minkindr_conversions
  message_generation
  geometry_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
)

# Add the installation prefix of AEPlanner to CMAKE_PREFIX_PATH
#list(APPEND CMAKE_PREFIX_PATH "~/workspace1/install")
#find_package(AEPlanner REQUIRED)

# Find Eigen3 package
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

# Define libraries
set(LIBRARIES
  auxiliar
)

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES Node.msg
)

## Generate services in the 'srv' folder
add_service_files(
   DIRECTORY srv
   FILES
   Query.srv
   BestNode.srv
   Reevaluate.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# Define Catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# Add library targets
add_library(auxiliar
  src/evaluator.cpp
  src/rrt_kd.cpp
  src/kd_tree_gain.cpp
)

# Add dependencies for library targets
add_dependencies(auxiliar 
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Link libraries against required Catkin libraries
target_link_libraries(auxiliar
  ${catkin_LIBRARIES}
)

# Add executable for cached.cpp
add_executable(cached src/cached.cpp)

# Link executable against required libraries
target_link_libraries(cached
  ${catkin_LIBRARIES}
  auxiliar
)

# Add executable for cached_multi.cpp
add_executable(cached_multi src/cached_multi.cpp)

# Link executable against required libraries
target_link_libraries(cached_multi
  ${catkin_LIBRARIES}
  auxiliar
)

#############
## Install ##
#############

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
