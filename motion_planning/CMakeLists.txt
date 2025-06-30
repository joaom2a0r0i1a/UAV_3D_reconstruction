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
  std_msgs
  geometry_msgs
  visualization_msgs
  trajectory_generation
  voxblox_ros
  minkindr_conversions
  cache_nodes
  mavros
  mavros_msgs
  geographic_msgs
  #process_pointcloud
  #multiagent_collision_check
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
  NBVPlanner
  NBVPlannerReal
  KinoNBVPlanner
  KinoNBVPlannerReal
  #NBVMultiPlanner
  AEPlanner
  AEPlannerReal
  KinoAEPlanner
  KinoAEPlannerReal
  #CameraParameters
  #Planner
  #Rrt_Star
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
add_library(NBVPlanner 
  src/gain_evaluator.cpp
  #src/gain_evaluator_2D.cpp
  #src/RRT/rrt_star.cpp
  src/RRT/rrt_star_kd.cpp
  #src/RRT/rrt_star_2D.cpp
  src/NBV/NBVplanner.cpp
  #src/NBV/NBVplanner_2D.cpp
  )
  
   # Add library targets
add_library(NBVPlannerReal 
  src/gain_evaluator.cpp
  #src/gain_evaluator_2D.cpp
  #src/RRT/rrt_star.cpp
  src/RRT/rrt_star_kd.cpp
  #src/RRT/rrt_star_2D.cpp
  src/NBVReal/NBVplannerReal.cpp
  #src/NBV/NBVplanner_2D.cpp
  )

 # Add library targets
 add_library(KinoNBVPlanner 
 src/gain_evaluator.cpp
 #src/gain_evaluator_2D.cpp
 #src/RRT/rrt_star.cpp
 src/RRT/kino_rrt_star_kd.cpp
 #src/RRT/rrt_star_2D.cpp
 src/KinoNBV/KinoNBVplanner.cpp
 #src/NBV/NBVplanner_2D.cpp
 )

 # Add library targets
 add_library(KinoNBVPlannerReal 
 src/gain_evaluator.cpp
 #src/gain_evaluator_2D.cpp
 #src/RRT/rrt_star.cpp
 src/RRT/kino_rrt_star_kd.cpp
 #src/RRT/rrt_star_2D.cpp
 src/KinoNBVReal/KinoNBVplannerReal.cpp
 #src/NBV/NBVplanner_2D.cpp
 )

 # Add library targets
add_library(AEPlanner 
  src/gain_evaluator.cpp
  #src/gain_evaluator_2D.cpp
  #src/RRT/rrt_star.cpp
  src/RRT/rrt_star_kd.cpp
  #src/RRT/rrt_star_2D.cpp
  src/kd_tree.cpp
  src/AEP/AEPlanner.cpp
  #src/NBV/NBVplanner_2D.cpp
  )
  
 # Add library targets
add_library(AEPlannerReal
  src/gain_evaluator.cpp
  #src/gain_evaluator_2D.cpp
  #src/RRT/rrt_star.cpp
  src/RRT/rrt_star_kd.cpp
  #src/RRT/rrt_star_2D.cpp
  src/kd_tree.cpp
  src/AEPReal/AEPlannerReal.cpp
  #src/NBV/NBVplanner_2D.cpp
  )

   # Add library targets
add_library(KinoAEPlanner 
  src/gain_evaluator.cpp
  #src/gain_evaluator_2D.cpp
  #src/RRT/rrt_star.cpp
  src/RRT/kino_rrt_star_kd.cpp
  #src/RRT/rrt_star_2D.cpp
  src/kd_tree.cpp
  src/KinoAEP/KinoAEPlanner.cpp
  #src/NBV/NBVplanner_2D.cpp
  )
  
   # Add library targets
add_library(KinoAEPlannerReal 
  src/gain_evaluator.cpp
  #src/gain_evaluator_2D.cpp
  #src/RRT/rrt_star.cpp
  src/RRT/kino_rrt_star_kd.cpp
  #src/RRT/rrt_star_2D.cpp
  src/kd_tree.cpp
  src/KinoAEPReal/KinoAEPlannerReal.cpp
  #src/NBV/NBVplanner_2D.cpp
  )


 # Add dependencies for library targets
add_dependencies(NBVPlanner ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(NBVPlannerReal ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(KinoNBVPlanner ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(KinoNBVPlannerReal ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(AEPlanner ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(AEPlannerReal ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(KinoAEPlanner ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(KinoAEPlannerReal ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

 # Link libraries against required Catkin libraries
target_link_libraries(NBVPlanner ${catkin_LIBRARIES})
target_link_libraries(NBVPlannerReal ${catkin_LIBRARIES})
target_link_libraries(KinoNBVPlanner ${catkin_LIBRARIES})
target_link_libraries(KinoNBVPlannerReal ${catkin_LIBRARIES})
target_link_libraries(AEPlanner ${catkin_LIBRARIES})
target_link_libraries(AEPlannerReal ${catkin_LIBRARIES})
target_link_libraries(KinoAEPlanner ${catkin_LIBRARIES})
target_link_libraries(KinoAEPlannerReal ${catkin_LIBRARIES})

 # Add executable for NBVplanner_node.cpp
add_executable(NBV src/NBV/NBVplanner_node.cpp)
add_executable(NBVReal src/NBVReal/NBVplannerReal_node.cpp)
add_executable(KinoNBV src/KinoNBV/KinoNBVplanner_node.cpp)
add_executable(KinoNBVReal src/KinoNBVReal/KinoNBVplannerReal_node.cpp)
add_executable(AEP src/AEP/AEPlanner_node.cpp)
add_executable(AEPReal src/AEPReal/AEPlannerReal_node.cpp)
add_executable(KinoAEP src/KinoAEP/KinoAEPlanner_node.cpp)
add_executable(KinoAEPReal src/KinoAEPReal/KinoAEPlannerReal_node.cpp)
add_executable(eval_voxblox_node src/evaluate/eval_voxblox_node.cpp)

# tf broadcast
add_executable(mavros_tf_broadcaster src/mavros_tf_broadcaster.cpp)
target_link_libraries(mavros_tf_broadcaster ${catkin_LIBRARIES})

#add_executable(planner_2D src/planner_node_2D.cpp)

# Set compiler flags for debug mode
# set(CMAKE_C_FLAGS_DEBUG "-g")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")

#target_link_libraries(planner_2D
#  ${catkin_LIBRARIES}
#  NBVPlanner
#)
 # Link executable against required libraries
target_link_libraries(NBV ${catkin_LIBRARIES} NBVPlanner)
target_link_libraries(NBVReal ${catkin_LIBRARIES} NBVPlannerReal)
target_link_libraries(KinoNBV ${catkin_LIBRARIES} KinoNBVPlanner)
target_link_libraries(KinoNBVReal ${catkin_LIBRARIES} KinoNBVPlannerReal)
target_link_libraries(AEP ${catkin_LIBRARIES} AEPlanner)
target_link_libraries(AEPReal ${catkin_LIBRARIES} AEPlannerReal)
target_link_libraries(KinoAEP ${catkin_LIBRARIES} KinoAEPlanner)
target_link_libraries(KinoAEPReal ${catkin_LIBRARIES} KinoAEPlannerReal)
target_link_libraries(eval_voxblox_node ${catkin_LIBRARIES})

# Specify the command to run Valgrind
#set(VALGRIND_COMMAND valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes)
#set(VALGRIND_OUTPUT_FILE "${CMAKE_BINARY_DIR}/valgrind_output.txt")

# Create a custom target to run Valgrind
#add_custom_target(valgrind
    #COMMAND ${VALGRIND_COMMAND} $<TARGET_FILE:NBV> > ${VALGRIND_OUTPUT_FILE} 2>&1
    #DEPENDS NBV
    #COMMENT "Running Valgrind Memory Checker"
#)

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



