#! /usr/bin/env python

# import roslib; roslib.load_manifest('vigir_synthesis_manager')

import rospy
import actionlib

from vigir_synthesis_msgs.msg import BehaviorSynthesisAction, BehaviorSynthesisGoal, BehaviorSynthesisRequest, SynthesisOptions

def behavior_synthesis_client():
    '''...'''

    # Create the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('vigir_behavior_synthesis', BehaviorSynthesisAction)

    # Wait until the action server has started up.
    client.wait_for_server()

    # Create a goal to send to the action server.
    goal = BehaviorSynthesisGoal()

    # Fill out the request part of the message.
    goal.request = BehaviorSynthesisRequest()
    goal.request.system = BehaviorSynthesisRequest.ATLAS
    goal.request.goals = ['manipulate']
    goal.request.initial_conditions = ['stand_prep']
    
    # Fill ot any options (all False by default).
    goal.synthesis_options = SynthesisOptions()

    # Send the goal to the action server.
    client.send_goal(goal)

    # Wait for the server to finish performing the action.
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    
    try:
        # Initialize a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
        rospy.init_node('behavior_synthesis_client_py')
        result = behavior_synthesis_client()
        print('Behavior Synthesis result: %s \n' % str(result.error_code.value))
        print('State Instantiation:')
        print(result.states)
    except rospy.ROSInterruptException:
        print "Client interrupted before completion"

    rospy.sleep(1.0) # Dirty way of avoiding a ROSHandshakeException
