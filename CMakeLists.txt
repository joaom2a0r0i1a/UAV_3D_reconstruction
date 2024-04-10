cmake_minimum_required(VERSION 3.15.0)
project(motion_planning_python)

# Enable parallel build
set(CMAKE_BUILD_PARALLEL_LEVEL 4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## --------------------------------------------------------------
##                             C++
## --------------------------------------------------------------

set(CATKIN_DEPENDENCIES
  roscpp
  mrs_msgs
  mrs_lib
  visualization_msgs
  trajectory_generation
  voxblox_ros
  minkindr_conversions
  )
  
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

 # Find Eigen3 package
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

 # Define libraries
set(LIBRARIES
  MotionPlanner
  #CameraParameters
  Planner
  Rrt_Star
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
add_library(MotionPlanner 
  src/gain_evaluator.cpp
  )
#add_library(CameraParameters 
#  src/camera_parameters.cpp
#  )
add_library(Planner 
  src/planner.cpp
  )
add_library(Rrt_Star 
  src/rrt_star_2d_yaw.cpp
  )

 # Add dependencies for library targets
add_dependencies(MotionPlanner 
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(CameraParameters 
#  ${catkin_EXPORTED_TARGETS} 
#  ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(Planner 
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(Rrt_Star 
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS})

 # Link libraries against required Catkin libraries
target_link_libraries(MotionPlanner
  ${catkin_LIBRARIES}
  )
#target_link_libraries(CameraParameters
#  ${catkin_LIBRARIES}
#  )
target_link_libraries(Planner
  ${catkin_LIBRARIES}
  )
  target_link_libraries(Rrt_Star
  ${catkin_LIBRARIES}
  )

 # Add executable for camera_parameters.cpp
#add_executable(camera_parameters src/camera_parameters.cpp)
add_executable(planner src/planner.cpp)

 # Link executable against required libraries
#target_link_libraries(camera_parameters
#  ${catkin_LIBRARIES}
#  MotionPlanner
#  #CameraParameters
#  Planner
#  Rrt_Star
#)

 # Link executable against required libraries
target_link_libraries(planner
  ${catkin_LIBRARIES}
  MotionPlanner
  #CameraParameters
  Planner
  Rrt_Star
)

 ## --------------------------------------------------------------
 ## |                           Install                          |
 ## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )



