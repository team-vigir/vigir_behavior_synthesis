#! /usr/bin/env python

import roslib; roslib.load_manifest('vigir_bs_manager')

import rospy
import actionlib

from vigir_bs_msgs.msg import *

import ltl_compilation_client

class BehaviorSynthesisActionServer(object):
    # create messages that are used to publish feedback/result
    _feedback = BehaviorSynthesisFeedback()
    _result   = BehaviorSynthesisResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, BehaviorSynthesisAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # append the seeds for the fibonacci sequence
        self._feedback.status = "Executing behavior synthesis"

        # start executing the action
        # check that preempt has not been requested by the client
        #TODO: receive a callback when a preempt request is received
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False

        # Request an LTL Specification from the corresponding server
        ltl_compilation_client.ltl_compilation_client()
        self._feedback.status = "Received LTL Specification"

        # publish the feedback
        self._as.publish_feedback(self._feedback)
          
        if success:
            self._result.error_code = BSErrorCodes(BSErrorCodes.SUCCESS)
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
    rospy.init_node('behavior_synthesis')
    BehaviorSynthesisActionServer(rospy.get_name())
    rospy.spin()