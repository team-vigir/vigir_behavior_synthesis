#! /usr/bin/env python

import roslib; roslib.load_manifest('vigir_synthesis_manager')

import rospy
import actionlib

from vigir_synthesis_msgs.msg import *

def behavior_synthesis_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('vigir_behavior_synthesis', BehaviorSynthesisAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = BehaviorSynthesisGoal()
    goal.request = BehaviorSynthesisRequest()
    goal.request.objective = 'manipulate'
    goal.synthesis_options = SynthesisOptions()
    goal.synthesis_options.something = False

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    
    try:
        # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
        rospy.init_node('behavior_synthesis_client_py')
        result = behavior_synthesis_client()
        print('Result: %s' % str(result.error_code.value))
    except rospy.ROSInterruptException:
        print "Program interrupted before completion"

    rospy.sleep(1.0) # Dirty way of avoiding a ROSHandshakeException
