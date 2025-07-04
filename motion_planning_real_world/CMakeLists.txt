cmake_minimum_required(VERSION 3.15.0)
project(motion_planning_real_world)

# Enable parallel build
set(CMAKE_BUILD_PARALLEL_LEVEL 8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## --------------------------------------------------------------
##                             C++
## --------------------------------------------------------------

#add_compile_options("-g")
add_compile_options("-O3")
#add_compile_options("-O0")

set(CATKIN_DEPENDENCIES
  rospy
  roscpp
  mrs_msgs
  mrs_lib
  std_srvs
  std_msgs
  geometry_msgs
  visualization_msgs
  trajectory_generation
  voxblox_ros
  minkindr_conversions
  motion_planning
  cache_nodes
  mavros
  mavros_msgs
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
  NBVPlannerReal
  KinoNBVPlannerReal
  AEPlannerReal
  KinoAEPlannerReal
  )

  # Define Catkin package
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
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
add_library(NBVPlannerReal 
  src/NBVReal/NBVplannerReal.cpp
  )

 # Add library targets
 add_library(KinoNBVPlannerReal 
 src/KinoNBVReal/KinoNBVplannerReal.cpp
 )
  
 # Add library targets
add_library(AEPlannerReal
  src/AEPReal/AEPlannerReal.cpp
  )
  
   # Add library targets
add_library(KinoAEPlannerReal 
  src/KinoAEPReal/KinoAEPlannerReal.cpp
  )


 # Add dependencies for library targets
add_dependencies(NBVPlannerReal ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(KinoNBVPlannerReal ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(AEPlannerReal ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(KinoAEPlannerReal ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

 # Link libraries against required Catkin libraries
target_link_libraries(NBVPlannerReal ${catkin_LIBRARIES})
target_link_libraries(KinoNBVPlannerReal ${catkin_LIBRARIES})
target_link_libraries(AEPlannerReal ${catkin_LIBRARIES})
target_link_libraries(KinoAEPlannerReal ${catkin_LIBRARIES})

 # Add executable for NBVplanner_node.cpp
add_executable(NBVReal src/NBVReal/NBVplannerReal_node.cpp)
add_executable(KinoNBVReal src/KinoNBVReal/KinoNBVplannerReal_node.cpp)
add_executable(AEPReal src/AEPReal/AEPlannerReal_node.cpp)
add_executable(KinoAEPReal src/KinoAEPReal/KinoAEPlannerReal_node.cpp)

# tf broadcast
add_executable(mavros_tf_broadcaster src/mavros_tf_broadcaster.cpp)
target_link_libraries(mavros_tf_broadcaster ${catkin_LIBRARIES})

 # Link executable against required libraries
target_link_libraries(NBVReal ${catkin_LIBRARIES} NBVPlannerReal)
target_link_libraries(KinoNBVReal ${catkin_LIBRARIES} KinoNBVPlannerReal)
target_link_libraries(AEPReal ${catkin_LIBRARIES} AEPlannerReal)
target_link_libraries(KinoAEPReal ${catkin_LIBRARIES} KinoAEPlannerReal)

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



