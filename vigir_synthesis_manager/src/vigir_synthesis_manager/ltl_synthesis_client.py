#!/usr/bin/env python

import rospy

from vigir_synthesis_msgs.srv import SynthesizeAutomaton

def ltl_synthesis_client(ltl_spec, name = ''):
    '''Client'''

    rospy.wait_for_service('ltl_synthesis')
    
    try:
        ltl_synthesis_srv = rospy.ServiceProxy('ltl_synthesis', SynthesizeAutomaton)
        response = ltl_synthesis_srv(ltl_spec, name)
        
        #DEBUG
        print 'LTL Synthesis client reporting:'
        print response.automaton
        print 'LTL Synthesis error code: ', response.error_code
        
        return response
    
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    ltl_synthesis_client()