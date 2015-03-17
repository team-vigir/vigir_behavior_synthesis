#!/usr/bin/env python

import rospy

from vigir_bs_msgs.srv import CompileLTL
from vigir_bs_msgs.msg import LTLFormula, BSErrorCodes

def handle_ltl_compilation(req):
    '''Responsible for putting together a complete LTL specification.'''
    
    initial_conditions = req.initial_conditions
    goal = req.goal

    ltl_specification = LTLFormula()
    # temp
    ltl_specification.sys_init = initial_conditions
    ltl_specification.sys_liveness = goal

    return LTLCompilationResponse(ltl_specification)

def ltl_compilation_server():
    ''''Server'''
    
    rospy.init_node('ltl_compilation_server')
    
    s = rospy.Service('add_two_ints', CompileLTL, handle_ltl_compilation)
    
    rospy.loginfo("LTL Compilation Request received")
    rospy.spin()

if __name__ == "__main__":
    ltl_compilation_server()