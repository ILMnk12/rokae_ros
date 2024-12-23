# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rokae_msgs: 4 messages, 0 services")

set(MSG_I_FLAGS "-Irokae_msgs:/home/ubuntu/caktin_ws/src/rokae_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rokae_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/ExternalForce.msg" NAME_WE)
add_custom_target(_rokae_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rokae_msgs" "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/ExternalForce.msg" "geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/JointPosVel.msg" NAME_WE)
add_custom_target(_rokae_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rokae_msgs" "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/JointPosVel.msg" ""
)

get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotMode.msg" NAME_WE)
add_custom_target(_rokae_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rokae_msgs" "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotMode.msg" ""
)

get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotState.msg" NAME_WE)
add_custom_target(_rokae_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rokae_msgs" "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotState.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/ExternalForce.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_msgs
)
_generate_msg_cpp(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/JointPosVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_msgs
)
_generate_msg_cpp(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_msgs
)
_generate_msg_cpp(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(rokae_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rokae_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rokae_msgs_generate_messages rokae_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/ExternalForce.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_cpp _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/JointPosVel.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_cpp _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotMode.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_cpp _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_cpp _rokae_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_msgs_gencpp)
add_dependencies(rokae_msgs_gencpp rokae_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/ExternalForce.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_msgs
)
_generate_msg_eus(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/JointPosVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_msgs
)
_generate_msg_eus(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_msgs
)
_generate_msg_eus(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(rokae_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(rokae_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(rokae_msgs_generate_messages rokae_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/ExternalForce.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_eus _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/JointPosVel.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_eus _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotMode.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_eus _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_eus _rokae_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_msgs_geneus)
add_dependencies(rokae_msgs_geneus rokae_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/ExternalForce.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_msgs
)
_generate_msg_lisp(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/JointPosVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_msgs
)
_generate_msg_lisp(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_msgs
)
_generate_msg_lisp(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(rokae_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rokae_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rokae_msgs_generate_messages rokae_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/ExternalForce.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_lisp _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/JointPosVel.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_lisp _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotMode.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_lisp _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_lisp _rokae_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_msgs_genlisp)
add_dependencies(rokae_msgs_genlisp rokae_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/ExternalForce.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_msgs
)
_generate_msg_nodejs(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/JointPosVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_msgs
)
_generate_msg_nodejs(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_msgs
)
_generate_msg_nodejs(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(rokae_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(rokae_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(rokae_msgs_generate_messages rokae_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/ExternalForce.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_nodejs _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/JointPosVel.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_nodejs _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotMode.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_nodejs _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_nodejs _rokae_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_msgs_gennodejs)
add_dependencies(rokae_msgs_gennodejs rokae_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/ExternalForce.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_msgs
)
_generate_msg_py(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/JointPosVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_msgs
)
_generate_msg_py(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotMode.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_msgs
)
_generate_msg_py(rokae_msgs
  "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(rokae_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rokae_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rokae_msgs_generate_messages rokae_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/ExternalForce.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_py _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/JointPosVel.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_py _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotMode.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_py _rokae_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ubuntu/caktin_ws/src/rokae_msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(rokae_msgs_generate_messages_py _rokae_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rokae_msgs_genpy)
add_dependencies(rokae_msgs_genpy rokae_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rokae_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rokae_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rokae_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(rokae_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/rokae_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(rokae_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(rokae_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rokae_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rokae_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(rokae_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/rokae_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(rokae_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(rokae_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rokae_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rokae_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(rokae_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
