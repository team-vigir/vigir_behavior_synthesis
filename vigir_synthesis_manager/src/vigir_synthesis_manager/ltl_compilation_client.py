#!/usr/bin/env python

import rospy

from vigir_synthesis_msgs.srv import GenerateLTLSpecification


def ltl_compilation_client(system, goals, initial_conditions, custom_ltl = None):
    '''Client'''

    rospy.wait_for_service('ltl_compilation')
    
    try:
        ltl_compilation_srv = rospy.ServiceProxy('ltl_compilation', GenerateLTLSpecification)
        response = ltl_compilation_srv(system, goals, initial_conditions)
        
        #DEBUG
        # print response.ltl_specification
        print 'LTL Compilation error code: ', response.error_code
        
        return response
    
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    ltl_compilation_client('atlas', ['pickup'], ['stand'])
