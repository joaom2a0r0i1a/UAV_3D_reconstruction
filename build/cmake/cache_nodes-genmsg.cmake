# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cache_nodes: 1 messages, 3 services")

set(MSG_I_FLAGS "-Icache_nodes:/home/joaomendes/workspace1/src/cache_nodes/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cache_nodes_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/msg/Node.msg" NAME_WE)
add_custom_target(_cache_nodes_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cache_nodes" "/home/joaomendes/workspace1/src/cache_nodes/msg/Node.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/Query.srv" NAME_WE)
add_custom_target(_cache_nodes_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cache_nodes" "/home/joaomendes/workspace1/src/cache_nodes/srv/Query.srv" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/BestNode.srv" NAME_WE)
add_custom_target(_cache_nodes_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cache_nodes" "/home/joaomendes/workspace1/src/cache_nodes/srv/BestNode.srv" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/Reevaluate.srv" NAME_WE)
add_custom_target(_cache_nodes_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cache_nodes" "/home/joaomendes/workspace1/src/cache_nodes/srv/Reevaluate.srv" "geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/msg/Node.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cache_nodes
)

### Generating Services
_generate_srv_cpp(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/Query.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cache_nodes
)
_generate_srv_cpp(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/BestNode.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cache_nodes
)
_generate_srv_cpp(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/Reevaluate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cache_nodes
)

### Generating Module File
_generate_module_cpp(cache_nodes
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cache_nodes
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cache_nodes_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cache_nodes_generate_messages cache_nodes_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/msg/Node.msg" NAME_WE)
add_dependencies(cache_nodes_generate_messages_cpp _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/Query.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_cpp _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/BestNode.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_cpp _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/Reevaluate.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_cpp _cache_nodes_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cache_nodes_gencpp)
add_dependencies(cache_nodes_gencpp cache_nodes_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cache_nodes_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/msg/Node.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cache_nodes
)

### Generating Services
_generate_srv_eus(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/Query.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cache_nodes
)
_generate_srv_eus(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/BestNode.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cache_nodes
)
_generate_srv_eus(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/Reevaluate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cache_nodes
)

### Generating Module File
_generate_module_eus(cache_nodes
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cache_nodes
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(cache_nodes_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(cache_nodes_generate_messages cache_nodes_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/msg/Node.msg" NAME_WE)
add_dependencies(cache_nodes_generate_messages_eus _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/Query.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_eus _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/BestNode.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_eus _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/Reevaluate.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_eus _cache_nodes_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cache_nodes_geneus)
add_dependencies(cache_nodes_geneus cache_nodes_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cache_nodes_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/msg/Node.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cache_nodes
)

### Generating Services
_generate_srv_lisp(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/Query.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cache_nodes
)
_generate_srv_lisp(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/BestNode.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cache_nodes
)
_generate_srv_lisp(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/Reevaluate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cache_nodes
)

### Generating Module File
_generate_module_lisp(cache_nodes
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cache_nodes
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cache_nodes_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cache_nodes_generate_messages cache_nodes_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/msg/Node.msg" NAME_WE)
add_dependencies(cache_nodes_generate_messages_lisp _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/Query.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_lisp _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/BestNode.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_lisp _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/Reevaluate.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_lisp _cache_nodes_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cache_nodes_genlisp)
add_dependencies(cache_nodes_genlisp cache_nodes_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cache_nodes_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/msg/Node.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cache_nodes
)

### Generating Services
_generate_srv_nodejs(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/Query.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cache_nodes
)
_generate_srv_nodejs(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/BestNode.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cache_nodes
)
_generate_srv_nodejs(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/Reevaluate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cache_nodes
)

### Generating Module File
_generate_module_nodejs(cache_nodes
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cache_nodes
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(cache_nodes_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(cache_nodes_generate_messages cache_nodes_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/msg/Node.msg" NAME_WE)
add_dependencies(cache_nodes_generate_messages_nodejs _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/Query.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_nodejs _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/BestNode.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_nodejs _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/Reevaluate.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_nodejs _cache_nodes_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cache_nodes_gennodejs)
add_dependencies(cache_nodes_gennodejs cache_nodes_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cache_nodes_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/msg/Node.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cache_nodes
)

### Generating Services
_generate_srv_py(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/Query.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cache_nodes
)
_generate_srv_py(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/BestNode.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cache_nodes
)
_generate_srv_py(cache_nodes
  "/home/joaomendes/workspace1/src/cache_nodes/srv/Reevaluate.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cache_nodes
)

### Generating Module File
_generate_module_py(cache_nodes
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cache_nodes
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cache_nodes_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cache_nodes_generate_messages cache_nodes_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/msg/Node.msg" NAME_WE)
add_dependencies(cache_nodes_generate_messages_py _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/Query.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_py _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/BestNode.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_py _cache_nodes_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/joaomendes/workspace1/src/cache_nodes/srv/Reevaluate.srv" NAME_WE)
add_dependencies(cache_nodes_generate_messages_py _cache_nodes_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cache_nodes_genpy)
add_dependencies(cache_nodes_genpy cache_nodes_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cache_nodes_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cache_nodes)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cache_nodes
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(cache_nodes_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(cache_nodes_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cache_nodes)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cache_nodes
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(cache_nodes_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(cache_nodes_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cache_nodes)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cache_nodes
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(cache_nodes_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(cache_nodes_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cache_nodes)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cache_nodes
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(cache_nodes_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(cache_nodes_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cache_nodes)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cache_nodes\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cache_nodes
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(cache_nodes_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(cache_nodes_generate_messages_py geometry_msgs_generate_messages_py)
endif()
