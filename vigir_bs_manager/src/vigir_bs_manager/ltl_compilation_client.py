#!/usr/bin/env python

import rospy

from vigir_bs_msgs.srv import CompileLTL
from vigir_bs_msgs.msg import LTLFormula, BSErrorCodes

def ltl_compilation_client():
    '''Client'''

    rospy.wait_for_service('ltl_compilation')
    
    try:
        ltl_compulation_srv = rospy.ServiceProxy('ltl_compilation', CompileLTL)
        resp = ltl_compulation_srv(['stand'], ['manipulate'])
        
        #DEBUG
        print resp.error_code.value
        print resp.ltl_formula.sys_init
        print resp.ltl_formula.sys_liveness
        
        return resp.ltl_formula
    
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    ltl_compilation_client()
