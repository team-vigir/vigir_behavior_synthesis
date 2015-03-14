cmake_minimum_required(VERSION 2.8.3)
project(vigir_bs_msgs)

set(MSG_DEPS
  std_msgs
  actionlib_msgs)

find_package(catkin REQUIRED genmsg ${MSG_DEPS})

set(MSG_FILES
	AutomatonState.msg
	SynthesizedAutomaton.msg
)

set(SRV_FILES
)

set(ACT_FILES
)

add_action_files(DIRECTORY action FILES ${ACT_FILES})
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(DEPENDS ${MSG_DEPS})