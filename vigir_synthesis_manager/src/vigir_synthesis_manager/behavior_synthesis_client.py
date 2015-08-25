#! /usr/bin/env python

import rospy
import actionlib

from vigir_synthesis_msgs.msg import *

def behavior_synthesis_client(system, goals, initial_conditions):
    '''...'''

    # Create the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('vigir_behavior_synthesis', BehaviorSynthesisAction)

    # Wait until the action server has started up.
    client.wait_for_server()

    # Create a goal to send to the action server.
    action_goal = BehaviorSynthesisGoal()

    # Fill out the request part of the message.
    action_goal.request = BehaviorSynthesisRequest()
    action_goal.request.system = system
    action_goal.request.goals = goals
    action_goal.request.initial_conditions = initial_conditions
    action_goal.request.name = 'client_request'
    
    # Fill ot any options (all False by default).
    action_goal.synthesis_options = SynthesisOptions()

    # Send the goal to the action server.
    client.send_goal(action_goal)

    # Wait for the server to finish performing the action.
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    
    try:
        # Initialize a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
        rospy.init_node('behavior_synthesis_client_py')
        
        # Request arguments:
        system = BehaviorSynthesisRequest.ATLAS
        goals = ['pickup']
        initial_conditions = ['stand_prep']
        
        result = behavior_synthesis_client(system, goals, initial_conditions)

        print('Behavior Synthesis result: %s \n' % str(result.error_code.value))
        print('State Instantiation:')
        print(result.states)
    except rospy.ROSInterruptException:
        print "Client interrupted before completion"

    rospy.sleep(1.0) # Dirty way of avoiding a ROSHandshakeException
