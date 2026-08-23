cmake_minimum_required(VERSION 3.15.0)
project(gain_evaluation)

enable_language(CUDA)

# Enable parallel build
set(CMAKE_BUILD_PARALLEL_LEVEL 8)

# CUDA
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# -lineinfo: Nsight source-line attribution for profiling (negligible overhead, no behavior change).
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -lineinfo")

# C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options("-O3")

set(CATKIN_DEPENDENCIES
  rospy
  roscpp
  mrs_msgs
  mrs_lib
  std_srvs
  voxblox_ros
  rrt_construction
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
  gain_evaluation
  )

# Define Catkin package
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS Eigen
  )

# Set include directories
include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  )

# Gain evaluation library (CPU + GPU/CUDA)
add_library(gain_evaluation
  src/gain_evaluator.cpp
  src/gpu_raycaster.cu
  )
add_dependencies(gain_evaluation ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(gain_evaluation ${catkin_LIBRARIES})

# GPU tester (links the gain_evaluation library)
add_executable(gpu_tester src/test_gpu_runner.cpp)
target_link_libraries(gpu_tester gain_evaluation ${catkin_LIBRARIES})

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
