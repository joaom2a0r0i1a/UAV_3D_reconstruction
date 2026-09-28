cmake_minimum_required(VERSION 3.15.0)
project(motion_planning)

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
  visualization_msgs
  voxblox_ros
  minkindr_conversions
  rrt_construction
  gain_evaluation
  cache_nodes
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
  RH_NBVP_lib
  KRH_NBVP_lib
  AEP_lib
  KAEP_lib
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

 # Add library targets
add_library(RH_NBVP_lib
  src/RH_NBVP/RH_NBVP.cpp
  src/planner_helpers.cpp
)

 # Add library targets
 add_library(KRH_NBVP_lib 
  src/KRH_NBVP/KRH_NBVP.cpp
 )

 # Add library targets
add_library(AEP_lib
  src/AEP/AEP.cpp
  src/planner_helpers.cpp
  )

  # Add library targets
add_library(KAEP_lib 
  src/KAEP/KAEP.cpp
  )


 # Add dependencies for library targets
add_dependencies(RH_NBVP_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(KRH_NBVP_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(AEP_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(KAEP_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

 # Link libraries against required Catkin libraries
target_link_libraries(RH_NBVP_lib ${catkin_LIBRARIES})
target_link_libraries(KRH_NBVP_lib ${catkin_LIBRARIES})
target_link_libraries(AEP_lib ${catkin_LIBRARIES})
target_link_libraries(KAEP_lib ${catkin_LIBRARIES})

 # Add executables
add_executable(RH_NBVP src/RH_NBVP/RH_NBVP_node.cpp)
add_executable(KRH_NBVP src/KRH_NBVP/KRH_NBVP_node.cpp)
add_executable(AEP src/AEP/AEP_node.cpp)
add_executable(KAEP src/KAEP/KAEP_node.cpp)
add_executable(eval_voxblox_node src/eval_voxblox/eval_voxblox_node.cpp)

 # Link executable against required libraries
target_link_libraries(RH_NBVP ${catkin_LIBRARIES} RH_NBVP_lib)
target_link_libraries(KRH_NBVP ${catkin_LIBRARIES} KRH_NBVP_lib)
target_link_libraries(AEP ${catkin_LIBRARIES} AEP_lib)
target_link_libraries(KAEP ${catkin_LIBRARIES} KAEP_lib)
target_link_libraries(eval_voxblox_node ${catkin_LIBRARIES})

 ## --------------------------------------------------------------
 ## |                           Install                          |
 ## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/depth_to_pointcloud.py
  scripts/evaluation/nodes/eval_data_node.py
  scripts/evaluation/nodes/eval_plotting_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )



