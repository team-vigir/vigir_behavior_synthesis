#!/usr/bin/env python

import rospy

from vigir_synthesis_msgs.srv import LTLCompilation
# from vigir_synthesis_msgs.msg import LTLSpecification, BSErrorCodes

def ltl_compilation_client():
    '''Client'''

    rospy.wait_for_service('ltl_compilation')
    
    try:
        ltl_compulation_srv = rospy.ServiceProxy('ltl_compilation', LTLCompilation)
        response = ltl_compulation_srv(['stand'], ['manipulate'], 'atlas')
        
        #DEBUG
        print response.ltl_specification.sys_init
        print response.ltl_specification.env_init
        print response.ltl_specification.sys_trans
        print response.ltl_specification.env_trans
        print response.ltl_specification.sys_liveness
        print response.ltl_specification.env_liveness
        print response.error_code
        
        return response
    
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    ltl_compilation_client()
