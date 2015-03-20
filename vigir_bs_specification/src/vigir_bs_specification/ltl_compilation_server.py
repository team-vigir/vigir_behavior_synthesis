#!/usr/bin/env python

import rospy

from vigir_bs_msgs.srv import LTLCompilation
from vigir_bs_msgs.msg import LTLFormula, BSErrorCodes

def handle_ltl_compilation(request):
    '''Responsible for putting together a complete LTL specification.'''
    
    initial_conditions = request.initial_conditions
    goal = request.goal

    ltl_specification = LTLFormula()
    # temp
    ltl_specification.sys_init = initial_conditions
    ltl_specification.sys_liveness = goal

    # Behavior Synthesis error code
    error_code = BSErrorCodes(BSErrorCodes.SUCCESS)

    return LTLCompilationResponse(ltl_specification, error_code)

def ltl_compilation_server():
    ''''Server'''
    
    rospy.init_node('ltl_compilation_server')
    
    s = rospy.Service('ltl_compilation', LTLCompilation, handle_ltl_compilation)
    
    rospy.loginfo("Ready to receive LTL Specification Compilation requests.")
    rospy.spin()

if __name__ == "__main__":
    ltl_compilation_server()