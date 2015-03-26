#!/usr/bin/env python

import rospy

from vigir_synthesis_msgs.srv import LTLCompilation, LTLCompilationResponse
from vigir_synthesis_msgs.msg import LTLSpecification, BSErrorCodes
from vigir_ltl_specification.atlas_specification import ControlModeSpecification

def handle_ltl_compilation(request):
    '''Responsible for putting together a complete LTL specification.'''

    if request.system:
        if request.system == 'atlas':
            ltl_specification, error_code = gen_ltl_spec_for_atlas('atlas_spec', request.initial_conditions, request.goals)
        else:
            error_code = BSErrorCodes(BSErrorCodes.LTL_SPEC_COMPILATION_FAILED)
            rospy.logerr('LTL Specification Compilation does not support %s' % request.system)

    return LTLCompilationResponse(ltl_specification, error_code)

def gen_ltl_spec_for_atlas(name, initial_conditions, goals):
    ''''''

    spec = ControlModeSpecification(name, initial_mode = initial_conditions[0])
    # Optional argument example: modes_of_interest = ['stand_prep', 'stand', 'manipulate']

    # Add control mode goals (system liveness requirements)
    for goal in goals:
        spec.add_control_mode_goal(goal)

    # Compile an LTL specification that fulfills the request
    ltl_specification_msg = gen_msg_from_specification(spec)

    # Behavior Synthesis error code
    error_code = BSErrorCodes(BSErrorCodes.SUCCESS)

    return ltl_specification_msg, error_code

def gen_msg_from_specification(spec):
    '''...'''

    ltl_specification_msg = LTLSpecification()

    # The atomic propositions
    ltl_specification_msg.sys_props = spec.sys_props
    ltl_specification_msg.env_props = spec.env_props

    # The 6 formulas that make up a GR(1) specification
    ltl_specification_msg.sys_init = spec.sys_init
    ltl_specification_msg.env_init = spec.env_init
    ltl_specification_msg.sys_trans = spec.sys_trans
    ltl_specification_msg.env_trans = spec.env_trans
    ltl_specification_msg.sys_liveness = spec.sys_liveness
    ltl_specification_msg.env_liveness = spec.env_liveness

    return ltl_specification_msg

def ltl_compilation_server():
    ''''Server'''
    
    rospy.init_node('vigir_ltl_specification')
    
    s = rospy.Service('ltl_compilation', LTLCompilation, handle_ltl_compilation)
    
    rospy.loginfo("Ready to receive LTL Specification Compilation requests.")
    rospy.spin()

if __name__ == "__main__":
    ltl_compilation_server()