# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "waffle_lab: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iwaffle_lab:/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Itf:/opt/ros/noetic/share/tf/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(waffle_lab_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/msg/my_msg.msg" NAME_WE)
add_custom_target(_waffle_lab_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "waffle_lab" "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/msg/my_msg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(waffle_lab
  "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/msg/my_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/waffle_lab
)

### Generating Services

### Generating Module File
_generate_module_cpp(waffle_lab
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/waffle_lab
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(waffle_lab_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(waffle_lab_generate_messages waffle_lab_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/msg/my_msg.msg" NAME_WE)
add_dependencies(waffle_lab_generate_messages_cpp _waffle_lab_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(waffle_lab_gencpp)
add_dependencies(waffle_lab_gencpp waffle_lab_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS waffle_lab_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(waffle_lab
  "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/msg/my_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/waffle_lab
)

### Generating Services

### Generating Module File
_generate_module_eus(waffle_lab
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/waffle_lab
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(waffle_lab_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(waffle_lab_generate_messages waffle_lab_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/msg/my_msg.msg" NAME_WE)
add_dependencies(waffle_lab_generate_messages_eus _waffle_lab_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(waffle_lab_geneus)
add_dependencies(waffle_lab_geneus waffle_lab_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS waffle_lab_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(waffle_lab
  "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/msg/my_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/waffle_lab
)

### Generating Services

### Generating Module File
_generate_module_lisp(waffle_lab
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/waffle_lab
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(waffle_lab_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(waffle_lab_generate_messages waffle_lab_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/msg/my_msg.msg" NAME_WE)
add_dependencies(waffle_lab_generate_messages_lisp _waffle_lab_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(waffle_lab_genlisp)
add_dependencies(waffle_lab_genlisp waffle_lab_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS waffle_lab_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(waffle_lab
  "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/msg/my_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/waffle_lab
)

### Generating Services

### Generating Module File
_generate_module_nodejs(waffle_lab
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/waffle_lab
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(waffle_lab_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(waffle_lab_generate_messages waffle_lab_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/msg/my_msg.msg" NAME_WE)
add_dependencies(waffle_lab_generate_messages_nodejs _waffle_lab_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(waffle_lab_gennodejs)
add_dependencies(waffle_lab_gennodejs waffle_lab_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS waffle_lab_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(waffle_lab
  "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/msg/my_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/waffle_lab
)

### Generating Services

### Generating Module File
_generate_module_py(waffle_lab
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/waffle_lab
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(waffle_lab_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(waffle_lab_generate_messages waffle_lab_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ryven/D-Star-Lite-Implementation-on-Turtlebot3-Waffle-/D_star_lite/src/waffle_lab/msg/my_msg.msg" NAME_WE)
add_dependencies(waffle_lab_generate_messages_py _waffle_lab_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(waffle_lab_genpy)
add_dependencies(waffle_lab_genpy waffle_lab_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS waffle_lab_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/waffle_lab)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/waffle_lab
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(waffle_lab_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(waffle_lab_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET tf_generate_messages_cpp)
  add_dependencies(waffle_lab_generate_messages_cpp tf_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(waffle_lab_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/waffle_lab)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/waffle_lab
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(waffle_lab_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(waffle_lab_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET tf_generate_messages_eus)
  add_dependencies(waffle_lab_generate_messages_eus tf_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(waffle_lab_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/waffle_lab)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/waffle_lab
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(waffle_lab_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(waffle_lab_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET tf_generate_messages_lisp)
  add_dependencies(waffle_lab_generate_messages_lisp tf_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(waffle_lab_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/waffle_lab)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/waffle_lab
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(waffle_lab_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(waffle_lab_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET tf_generate_messages_nodejs)
  add_dependencies(waffle_lab_generate_messages_nodejs tf_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(waffle_lab_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/waffle_lab)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/waffle_lab\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/waffle_lab
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(waffle_lab_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(waffle_lab_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET tf_generate_messages_py)
  add_dependencies(waffle_lab_generate_messages_py tf_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(waffle_lab_generate_messages_py geometry_msgs_generate_messages_py)
endif()
