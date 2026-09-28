cmake_minimum_required(VERSION 3.15.0)
project(multi_motion_planning)

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
  voxblox_ros
  minkindr_conversions
  rrt_construction
  gain_evaluation
  cache_nodes
  process_pointcloud
  multiagent_collision_check
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
  RH_NBVP_fleet_lib
  KRH_NBVP_fleet_lib
  AEP_fleet_lib
  KAEP_fleet_lib
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
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

 # Add library targets
add_library(RH_NBVP_fleet_lib 
  src/RH_NBVP_fleet.cpp
)

add_library(KRH_NBVP_fleet_lib 
  src/KRH_NBVP_fleet.cpp
)

add_library(AEP_fleet_lib 
 src/AEP_fleet.cpp
)

add_library(KAEP_fleet_lib 
  src/KAEP_fleet.cpp
)

 # Add dependencies for library targets
add_dependencies(RH_NBVP_fleet_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(KRH_NBVP_fleet_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(AEP_fleet_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(KAEP_fleet_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

 # Link libraries against required Catkin libraries
target_link_libraries(RH_NBVP_fleet_lib ${catkin_LIBRARIES})
target_link_libraries(KRH_NBVP_fleet_lib ${catkin_LIBRARIES})
target_link_libraries(AEP_fleet_lib ${catkin_LIBRARIES})
target_link_libraries(KAEP_fleet_lib ${catkin_LIBRARIES})

 # Add executables
add_executable(RH_NBVP_fleet src/RH_NBVP_fleet_node.cpp)
add_executable(KRH_NBVP_fleet src/KRH_NBVP_fleet_node.cpp)
add_executable(AEP_fleet src/AEP_fleet_node.cpp)
add_executable(KAEP_fleet src/KAEP_fleet_node.cpp)

 # Link executable against required libraries
target_link_libraries(RH_NBVP_fleet ${catkin_LIBRARIES} RH_NBVP_fleet_lib)
target_link_libraries(KRH_NBVP_fleet ${catkin_LIBRARIES} KRH_NBVP_fleet_lib)
target_link_libraries(AEP_fleet ${catkin_LIBRARIES} AEP_fleet_lib)
target_link_libraries(KAEP_fleet ${catkin_LIBRARIES} KAEP_fleet_lib)

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