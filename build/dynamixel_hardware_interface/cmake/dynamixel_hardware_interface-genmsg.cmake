# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dynamixel_hardware_interface: 3 messages, 8 services")

set(MSG_I_FLAGS "-Idynamixel_hardware_interface:/home/dave/ros/clam/src/dynamixel_hardware_interface/msg;-Istd_msgs:/opt/ros/groovy/share/std_msgs/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

#better way to handle this?
set (ALL_GEN_OUTPUT_FILES_cpp "")

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/msg/JointState.msg
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_msg_cpp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/msg/MotorState.msg
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_msg_cpp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/msg/MotorStateList.msg
  "${MSG_I_FLAGS}"
  "/home/dave/ros/clam/src/dynamixel_hardware_interface/msg/MotorState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_hardware_interface
)

### Generating Services
_generate_srv_cpp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/RestartController.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_cpp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/SetComplianceMargin.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_cpp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/SetComplianceSlope.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_cpp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/SetTorqueLimit.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_cpp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/SetVelocity.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_cpp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/StartController.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_cpp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/StopController.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_cpp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/TorqueEnable.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_hardware_interface
)

### Generating Module File
_generate_module_cpp(dynamixel_hardware_interface
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_hardware_interface
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dynamixel_hardware_interface_gencpp ALL
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/msg/JointState.msg
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_msg_lisp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/msg/MotorState.msg
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_msg_lisp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/msg/MotorStateList.msg
  "${MSG_I_FLAGS}"
  "/home/dave/ros/clam/src/dynamixel_hardware_interface/msg/MotorState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_hardware_interface
)

### Generating Services
_generate_srv_lisp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/RestartController.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_lisp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/SetComplianceMargin.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_lisp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/SetComplianceSlope.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_lisp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/SetTorqueLimit.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_lisp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/SetVelocity.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_lisp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/StartController.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_lisp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/StopController.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_lisp(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/TorqueEnable.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_hardware_interface
)

### Generating Module File
_generate_module_lisp(dynamixel_hardware_interface
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_hardware_interface
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dynamixel_hardware_interface_genlisp ALL
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/msg/JointState.msg
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_msg_py(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/msg/MotorState.msg
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_msg_py(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/msg/MotorStateList.msg
  "${MSG_I_FLAGS}"
  "/home/dave/ros/clam/src/dynamixel_hardware_interface/msg/MotorState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_hardware_interface
)

### Generating Services
_generate_srv_py(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/RestartController.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_py(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/SetComplianceMargin.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_py(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/SetComplianceSlope.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_py(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/SetTorqueLimit.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_py(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/SetVelocity.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_py(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/StartController.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_py(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/StopController.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_hardware_interface
)
_generate_srv_py(dynamixel_hardware_interface
  /home/dave/ros/clam/src/dynamixel_hardware_interface/srv/TorqueEnable.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_hardware_interface
)

### Generating Module File
_generate_module_py(dynamixel_hardware_interface
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_hardware_interface
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dynamixel_hardware_interface_genpy ALL
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)


debug_message(2 "dynamixel_hardware_interface: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dynamixel_hardware_interface
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(dynamixel_hardware_interface_gencpp std_msgs_gencpp)

if(genlisp_INSTALL_DIR)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dynamixel_hardware_interface
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(dynamixel_hardware_interface_genlisp std_msgs_genlisp)

if(genpy_INSTALL_DIR)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_hardware_interface\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dynamixel_hardware_interface
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(dynamixel_hardware_interface_genpy std_msgs_genpy)
