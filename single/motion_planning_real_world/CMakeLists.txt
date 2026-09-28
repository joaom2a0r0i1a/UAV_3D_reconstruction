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
  tf2_ros
  visualization_msgs
  voxblox_ros
  minkindr_conversions
  gain_evaluation
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
  planner_helpers_rw
  RH_NBVP_rw_lib
  KRH_NBVP_rw_lib
  AEP_rw_lib
  KAEP_rw_lib
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
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  )
  
 # Shared helpers without MRS
add_library(planner_helpers_rw
  src/planner_helpers_rw.cpp
  )

   # Add library targets
add_library(RH_NBVP_rw_lib
  src/RH_NBVP/RH_NBVP_rw.cpp
  )

 # Add library targets
 add_library(KRH_NBVP_rw_lib 
 src/KRH_NBVP/KRH_NBVP_rw.cpp
 )
  
 # Add library targets
add_library(AEP_rw_lib
  src/AEP/AEP_rw.cpp
  )
  
   # Add library targets
add_library(KAEP_rw_lib 
  src/KAEP/KAEP_rw.cpp
  )


 # Add dependencies for library targets
add_dependencies(planner_helpers_rw ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(RH_NBVP_rw_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(KRH_NBVP_rw_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(AEP_rw_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(KAEP_rw_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

 # Link libraries against required Catkin libraries
target_link_libraries(planner_helpers_rw ${catkin_LIBRARIES})
target_link_libraries(RH_NBVP_rw_lib ${catkin_LIBRARIES} planner_helpers_rw)
target_link_libraries(KRH_NBVP_rw_lib ${catkin_LIBRARIES})
target_link_libraries(AEP_rw_lib ${catkin_LIBRARIES} planner_helpers_rw)
target_link_libraries(KAEP_rw_lib ${catkin_LIBRARIES})

 # Add executables
add_executable(RH_NBVP_rw src/RH_NBVP/RH_NBVP_rw_node.cpp)
add_executable(KRH_NBVP_rw src/KRH_NBVP/KRH_NBVP_rw_node.cpp)
add_executable(AEP_rw src/AEP/AEP_rw_node.cpp)
add_executable(KAEP_rw src/KAEP/KAEP_rw_node.cpp)

# TF broadcaster
add_executable(mavros_tf_broadcaster src/mavros_tf_broadcaster.cpp)
target_link_libraries(mavros_tf_broadcaster ${catkin_LIBRARIES})

 # Link executable against required libraries
target_link_libraries(RH_NBVP_rw ${catkin_LIBRARIES} RH_NBVP_rw_lib)
target_link_libraries(KRH_NBVP_rw ${catkin_LIBRARIES} KRH_NBVP_rw_lib)
target_link_libraries(AEP_rw ${catkin_LIBRARIES} AEP_rw_lib)
target_link_libraries(KAEP_rw ${catkin_LIBRARIES} KAEP_rw_lib)

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



