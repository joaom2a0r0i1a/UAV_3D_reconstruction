cmake_minimum_required(VERSION 3.15.0)
project(multidrone_motion_planning)

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
  std_srvs
  visualization_msgs
  trajectory_generation
  voxblox_ros
  comm_msgs
  minkindr_conversions
  cache_nodes
  process_pointcloud
  multiagent_collision_check
  motion_planning_python
  )
  
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

# Output the catkin_LIBRARIES
#message("catkin_LIBRARIES: ${catkin_LIBRARIES}")

 # Find Eigen3 package
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

 # Define libraries
set(LIBRARIES
  NBVMultiPlanner
  AEPMultiPlanner
)

  # Define Catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS Eigen
)

 ## --------------------------------------------------------------
 ## |                            Build                           |
 ## --------------------------------------------------------------

 # Set include directories
include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

 # Add library targets
add_library(NBVMultiPlanner 
  src/gain_evaluator.cpp
  src/RRT/rrt_star_kd.cpp
  src/NBVMultiplanner.cpp
)

add_library(AEPMultiPlanner 
 src/gain_evaluator.cpp
 src/RRT/rrt_star_kd.cpp
 src/kd_tree.cpp
 src/AEPMultiplanner.cpp
)

 # Add dependencies for library targets
add_dependencies(NBVMultiPlanner ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(AEPMultiPlanner ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

 # Link libraries against required Catkin libraries
target_link_libraries(NBVMultiPlanner ${catkin_LIBRARIES})
target_link_libraries(AEPMultiPlanner ${catkin_LIBRARIES})

 # Add executable for NBVplanner_node.cpp
add_executable(NBVMulti src/NBVMultiplanner_node.cpp)
add_executable(AEPMulti src/AEPMultiplanner_node.cpp)

 # Link executable against required libraries
target_link_libraries(NBVMulti ${catkin_LIBRARIES} NBVMultiPlanner)
target_link_libraries(AEPMulti ${catkin_LIBRARIES} AEPMultiPlanner)

 ## --------------------------------------------------------------
 ## |                           Install                          |
 ## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )