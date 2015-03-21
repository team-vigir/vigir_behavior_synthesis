#! /usr/bin/env python

import roslib; roslib.load_manifest('vigir_bs_manager')

import rospy
import actionlib

from vigir_bs_msgs.msg import BehaviorSynthesisAction, BehaviorSynthesisFeedback, BehaviorSynthesisResult, BSErrorCodes

import ltl_compilation_client

class BehaviorSynthesisActionServer(object):
    '''...'''

    # Messages that are used to publish feedback/result
    _feedback = BehaviorSynthesisFeedback()
    _result   = BehaviorSynthesisResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, BehaviorSynthesisAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        '''...'''

        r = rospy.Rate(1)   #FIX: What should this be?
        success = True      # start optimistically

        # Acknowledge goal reception
        self.set_and_publish_feedback("Received behavior synthesis request.")

        # Examine the goal message
        bs_goal = goal.request
        bs_options = goal.synthesis_options

        #TODO: receive a callback when a preempt request is received
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False

        if success:  
            # Request LTL Specification Compilation from the corresponding server
            # and also update and publish the appropriate feedback
            ltl_spec, error_code_value, success = self.handle_ltl_specification_request()

        if success: pass
            # Request LTL Synthesis from the corresponding server
            # and also update and publish the appropriate feedback
            # automaton, error_code_value, success = self.handle_ltl_synthesis_request()

        if success: pass
            # Request State Machine Generation from the corresponding server
            # and also update and publish the appropriate feedback
            # sm, error_code_value, success = self.handle_sm_generation_request()

        if success:
            self._result.error_code = BSErrorCodes(BSErrorCodes.SUCCESS)
            # self._result.states = sm
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            self._result.error_code = BSErrorCodes(error_code_value)
            rospy.logerr('%s: Failed' % self._action_name)
            self._as.set_failed(self._result)
    
    def handle_ltl_specification_request(self):
        '''...'''

        response = ltl_compilation_client.ltl_compilation_client()
        
        # Update success and publish feedback based on response
        if response.error_code.value is BSErrorCodes.SUCCESS:
            self.set_and_publish_feedback("Received LTL specification")
            success = True
        else:
            self.set_and_publish_feedback("Did not receive LTL specification")
            success = False

        return response.ltl_specification, response.error_code.value, success

    def handle_ltl_synthesis_request(self):
        '''...'''

        pass

    def handle_sm_generation_request(self):
        '''...'''

        pass

    def set_and_publish_feedback(self, status):
        '''Helper method for updating and publishing feedback.''' 
        
        self._feedback.status = status
        self._as.publish_feedback(self._feedback)

if __name__ == '__main__':
    rospy.init_node('behavior_synthesis')
    BehaviorSynthesisActionServer(rospy.get_name())
    rospy.spin()