#! /usr/bin/env python

import rospy
import actionlib

from vigir_synthesis_msgs.msg import *

import ltl_compilation_client
import ltl_synthesis_client
import sm_generate_client

class BehaviorSynthesisActionServer(object):
    '''ROS Action server that handles the following processes:

    * LTL Specification Compilation
    * LTL Synthesis (resulting in an automaton)
    * State Machine Generation/Instantiation

    Depending on the synthesis request's options, all 
    or a subset of the above step will be carried out.
    '''

    # Messages that are used to publish feedback/result
    _feedback = BehaviorSynthesisFeedback()
    _result   = BehaviorSynthesisResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                BehaviorSynthesisAction,
                                                execute_cb = self.execute_cb,
                                                auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        '''The code to be executed when a BehaviorSynthesisActionGoal is received.'''

        r = rospy.Rate(1)   # FIX: What should this be?
        success = True      # start optimistically

        # Acknowledge goal reception
        self.set_and_publish_feedback("Received behavior synthesis request.")

        # Examine the goal message
        synthesis_goal = goal.request
        synthesis_options = goal.synthesis_options

        #TODO: receive a callback when a preempt request is received
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False

        if success:  
            # Request LTL Specification Compilation from the corresponding server
            # and also update and publish the appropriate feedback
            ltl_spec, error_code_value, success = self.handle_ltl_specification_request(synthesis_goal)

        if success:
            # Request LTL Synthesis from the corresponding server
            # and also update and publish the appropriate feedback
            automaton, error_code_value, success = self.handle_ltl_synthesis_request(ltl_spec)

        if success:
            # Request State Machine Generation from the corresponding server
            # and also update and publish the appropriate feedback
            # TODO: how to get the the yaml_config file?
            sm, error_code_value, success = self.handle_sm_generation_request(automaton, synthesis_goal.system)

        if success:
            self._result.error_code = BSErrorCodes(BSErrorCodes.SUCCESS)
            self._result.states = sm
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            self._result.error_code = BSErrorCodes(error_code_value)
            rospy.logerr('%s: Failed' % self._action_name)
            self._as.set_aborted(self._result)
    
    def handle_ltl_specification_request(self, synthesis_goal):
        '''
        Makes a LTL Specification Compilation request 
        to the corresponding service and handles the response.

        synthesis_goal: BehaviorSynthesisRequest    A partial specification.
        '''

        system = synthesis_goal.system
        goals = synthesis_goal.goals
        initial_conditions = synthesis_goal.initial_conditions
        custom_ltl = synthesis_goal.ltl_specification #TODO: handle this (currently ignored)

        response = ltl_compilation_client.ltl_compilation_client(system,
                                                                 goals,
                                                                 initial_conditions)
        
        # Update success and publish feedback based on response
        if response.error_code.value is BSErrorCodes.SUCCESS:
            self.set_and_publish_feedback("Received LTL specification")
            success = True
        else:
            self.set_and_publish_feedback("Did not receive LTL specification")
            success = False

        return response.ltl_specification, response.error_code.value, success

    def handle_ltl_synthesis_request(self, ltl_spec):
        '''
        Makes a LTL Synthesis request 
        to the corresponding service and handles the response.

        ltl_spec:   LTLSpecification    A complete LTL specification.
        '''

        response = ltl_synthesis_client.ltl_synthesis_client(ltl_spec)

        if response.synthesizable:
            self.set_and_publish_feedback("The LTL Specification is synthesizable")
            success = True
        else:
            self.set_and_publish_feedback("The LTL Specification is unsynthesizable")
            success = False

        return response.automaton, response.error_code, success

    def handle_sm_generation_request(self, synthesized_automata, system):
        '''
        Generate State Machine definitions for a given 
        robotic system based on a synthesized automaton.

        @param synthesized_automata SynthesizedAutomaton    The automaton to instantiate as a SM.
        @param system               string                  System name. e.g. "atlas"
        '''
        response = sm_generate_client.sm_generate_client(synthesized_automata, system)

        # Update success and publish feedback based on response
        if response.error_code.value is BSErrorCodes.SUCCESS:
            self.set_and_publish_feedback("Generated State Machine definitions")
            success = True
        else:
            self.set_and_publish_feedback("Unable to generate State Machine.")
            success = False

        return response.state_definition, response.error_code.value, success

    def set_and_publish_feedback(self, status):
        '''Helper method for updating and publishing feedback.''' 
        
        self._feedback.status = status
        self._as.publish_feedback(self._feedback)

if __name__ == '__main__':
    rospy.init_node('vigir_behavior_synthesis')
    BehaviorSynthesisActionServer(rospy.get_name())
    rospy.spin()
