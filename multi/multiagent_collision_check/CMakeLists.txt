cmake_minimum_required(VERSION 3.15.0)
project(multiagent_collision_check)

# Enable parallel build
set(CMAKE_BUILD_PARALLEL_LEVEL 4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options("-O3")

set(CATKIN_DEPENDENCIES
  geometry_msgs
  std_msgs
  message_generation
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
)

 # Find Eigen3 package
 find_package(Eigen3 REQUIRED)
 set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
 set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

add_message_files(
  FILES
  Segment.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multiagent_collision_check ${catkin_LIBRARIES}
)
add_library(multiagent_collision_check src/multiagent_collision_checker.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

target_link_libraries(multiagent_collision_check
  ${catkin_LIBRARIES}
)

 ## --------------------------------------------------------------
 ## |                           Install                          |
 ## --------------------------------------------------------------

install(TARGETS multiagent_collision_check
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
