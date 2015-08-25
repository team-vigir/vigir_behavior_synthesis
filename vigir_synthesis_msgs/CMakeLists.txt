cmake_minimum_required(VERSION 2.8.3)
project(vigir_synthesis_msgs)

set(MSG_DEPS
	std_msgs
	actionlib_msgs
	vigir_be_msgs
)

find_package(catkin REQUIRED genmsg ${MSG_DEPS})
# message_generation does not need to be listed explicitly, it is referenced implicitly by actionlib_msgs

set(MSG_FILES
	SynthesisErrorCodes.msg
	BehaviorSynthesisRequest.msg
	SynthesisOptions.msg
	LTLSpecification.msg
	AutomatonState.msg
	FSAutomaton.msg
)

set(SRV_FILES
	GenerateLTLSpecification.srv
	SynthesizeAutomaton.srv
	GenerateFlexBESM.srv
)

set(ACT_FILES
	BehaviorSynthesis.action
)

add_action_files(DIRECTORY action FILES ${ACT_FILES})
add_message_files(DIRECTORY msg FILES ${MSG_FILES})
add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(
	DEPENDENCIES
	${MSG_DEPS}
	vigir_be_msgs
)

catkin_package(DEPENDS ${MSG_DEPS})
# The transitive dependency on message_runtime is happening automatically
