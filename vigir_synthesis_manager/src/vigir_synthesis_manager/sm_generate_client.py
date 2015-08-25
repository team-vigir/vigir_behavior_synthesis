#!/usr/bin/env python

import rospy

from vigir_synthesis_msgs.srv import GenerateFlexBESM

def sm_generate_client(synthesized_automata, system):
    '''A wrapper for a call to the GenerateFlexBESM service.'''

    rospy.wait_for_service('sm_generate')
    
    try:
        sm_generate_srv = rospy.ServiceProxy('sm_generate', GenerateFlexBESM)
        response = sm_generate_srv(synthesized_automata, system)
        
        #DEBUG
        for si in response.state_definition:
            print(si)
        
        return response
    
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    sm_generate_client()
