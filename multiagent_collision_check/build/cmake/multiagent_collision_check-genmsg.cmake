# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "multiagent_collision_check: 1 messages, 0 services")

set(MSG_I_FLAGS "-Imultiagent_collision_check:/home/joaomendes/workspace1/src/multiagent_collision_check/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(multiagent_collision_check_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/joaomendes/workspace1/src/multiagent_collision_check/msg/Segment.msg" NAME_WE)
add_custom_target(_multiagent_collision_check_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "multiagent_collision_check" "/home/joaomendes/workspace1/src/multiagent_collision_check/msg/Segment.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(multiagent_collision_check
  "/home/joaomendes/workspace1/src/multiagent_collision_check/msg/Segment.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multiagent_collision_check
)

### Generating Services

### Generating Module File
_generate_module_cpp(multiagent_collision_check
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multiagent_collision_check
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(multiagent_collision_check_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(multiagent_collision_check_generate_messages multiagent_collision_check_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joaomendes/workspace1/src/multiagent_collision_check/msg/Segment.msg" NAME_WE)
add_dependencies(multiagent_collision_check_generate_messages_cpp _multiagent_collision_check_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multiagent_collision_check_gencpp)
add_dependencies(multiagent_collision_check_gencpp multiagent_collision_check_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multiagent_collision_check_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(multiagent_collision_check
  "/home/joaomendes/workspace1/src/multiagent_collision_check/msg/Segment.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multiagent_collision_check
)

### Generating Services

### Generating Module File
_generate_module_eus(multiagent_collision_check
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multiagent_collision_check
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(multiagent_collision_check_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(multiagent_collision_check_generate_messages multiagent_collision_check_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joaomendes/workspace1/src/multiagent_collision_check/msg/Segment.msg" NAME_WE)
add_dependencies(multiagent_collision_check_generate_messages_eus _multiagent_collision_check_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multiagent_collision_check_geneus)
add_dependencies(multiagent_collision_check_geneus multiagent_collision_check_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multiagent_collision_check_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(multiagent_collision_check
  "/home/joaomendes/workspace1/src/multiagent_collision_check/msg/Segment.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multiagent_collision_check
)

### Generating Services

### Generating Module File
_generate_module_lisp(multiagent_collision_check
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multiagent_collision_check
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(multiagent_collision_check_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(multiagent_collision_check_generate_messages multiagent_collision_check_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joaomendes/workspace1/src/multiagent_collision_check/msg/Segment.msg" NAME_WE)
add_dependencies(multiagent_collision_check_generate_messages_lisp _multiagent_collision_check_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multiagent_collision_check_genlisp)
add_dependencies(multiagent_collision_check_genlisp multiagent_collision_check_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multiagent_collision_check_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(multiagent_collision_check
  "/home/joaomendes/workspace1/src/multiagent_collision_check/msg/Segment.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multiagent_collision_check
)

### Generating Services

### Generating Module File
_generate_module_nodejs(multiagent_collision_check
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multiagent_collision_check
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(multiagent_collision_check_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(multiagent_collision_check_generate_messages multiagent_collision_check_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joaomendes/workspace1/src/multiagent_collision_check/msg/Segment.msg" NAME_WE)
add_dependencies(multiagent_collision_check_generate_messages_nodejs _multiagent_collision_check_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multiagent_collision_check_gennodejs)
add_dependencies(multiagent_collision_check_gennodejs multiagent_collision_check_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multiagent_collision_check_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(multiagent_collision_check
  "/home/joaomendes/workspace1/src/multiagent_collision_check/msg/Segment.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiagent_collision_check
)

### Generating Services

### Generating Module File
_generate_module_py(multiagent_collision_check
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiagent_collision_check
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(multiagent_collision_check_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(multiagent_collision_check_generate_messages multiagent_collision_check_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joaomendes/workspace1/src/multiagent_collision_check/msg/Segment.msg" NAME_WE)
add_dependencies(multiagent_collision_check_generate_messages_py _multiagent_collision_check_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multiagent_collision_check_genpy)
add_dependencies(multiagent_collision_check_genpy multiagent_collision_check_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multiagent_collision_check_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multiagent_collision_check)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multiagent_collision_check
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(multiagent_collision_check_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(multiagent_collision_check_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multiagent_collision_check)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multiagent_collision_check
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(multiagent_collision_check_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(multiagent_collision_check_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multiagent_collision_check)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multiagent_collision_check
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(multiagent_collision_check_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(multiagent_collision_check_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multiagent_collision_check)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multiagent_collision_check
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(multiagent_collision_check_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(multiagent_collision_check_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiagent_collision_check)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiagent_collision_check\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiagent_collision_check
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(multiagent_collision_check_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(multiagent_collision_check_generate_messages_py std_msgs_generate_messages_py)
endif()
