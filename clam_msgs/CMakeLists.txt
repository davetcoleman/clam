cmake_minimum_required(VERSION 2.8.3)
project(clam_msgs)

# List message types we depend on in our messages
set(MSG_DEPS
  std_msgs
  actionlib_msgs
  geometry_msgs
  trajectory_msgs
)

find_package(catkin 
  REQUIRED COMPONENTS
    actionlib_msgs
    genmsg 
    ${MSG_DEPS}
)

set(MSG_FILES

)

set(SRV_FILES

)

add_action_files(
  DIRECTORY 
    action 
  FILES 
    ClamArm.action
    ClamGripperCommand.action
    BlockPerception.action
    BlockLogic.action
    PickPlace.action
)

#add_message_files(DIRECTORY msg FILES ${MSG_FILES})
#add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(
  DEPENDENCIES 
    actionlib_msgs 
    ${MSG_DEPS}
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
)
