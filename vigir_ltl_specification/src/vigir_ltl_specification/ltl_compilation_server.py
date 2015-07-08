#!/usr/bin/env python

import rospy

from vigir_synthesis_msgs.srv import LTLCompilation, LTLCompilationResponse
from vigir_synthesis_msgs.msg import LTLSpecification, BSErrorCodes
from vigir_ltl_specification.atlas_specification import ControlModeSpecification, VigirSpecification
from vigir_ltl_specification.gr1_specification import GR1Specification

def handle_ltl_compilation(request):
    """Responsible for putting together a complete LTL specification."""

    if request.system:
        if request.system == 'atlas':
            ltl_specification, error_code = gen_ltl_spec_for_atlas('atlas_spec', request.initial_conditions, request.goals)
        else:
            error_code = BSErrorCodes(BSErrorCodes.LTL_SPEC_COMPILATION_FAILED)
            rospy.logerr('LTL Specification Compilation does not support %s' % request.system)

    return LTLCompilationResponse(ltl_specification, error_code)

def gen_ltl_spec_for_atlas(name, initial_conditions, goals):
    """OBSOLETE: This method is using the old atlas_specification module!"""

    cm_spec = ControlModeSpecification('atlas_cm', initial_mode = initial_conditions[0],
                                       modes_of_interest = ['stand_prep', 'stand', 'manipulate'])
    # Optional argument example: modes_of_interest = ['stand_prep', 'stand', 'manipulate']
    
    vigir_spec = VigirSpecification()

    for goal in goals:
        # FIX: Not all goals are going to be action!
        # For control modes, check 'if goal in spec.control_modes'
        # spec.add_control_mode_goal(goal)
        vigir_spec.handle_new_action_goal(goal)

    complete_spec = GR1Specification(name)
    individual_specs = [cm_spec, vigir_spec]

    complete_spec.merge_gr1_specifications(individual_specs)

    # Compile an LTL specification that fulfills the request
    ltl_specification_msg = gen_msg_from_specification(complete_spec)

    # Behavior Synthesis error code
    error_code = BSErrorCodes(BSErrorCodes.SUCCESS)

    return ltl_specification_msg, error_code

def gen_msg_from_specification(spec):
    """Creates an LTLSpecification message from a structured slugs formatted specification."""

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
    """The LTL Specification Compilation server/node."""
    
    rospy.init_node('vigir_ltl_specification')
    
    s = rospy.Service('ltl_compilation', LTLCompilation, handle_ltl_compilation)
    
    rospy.loginfo("Ready to receive LTL Specification Compilation requests.")
    rospy.spin()

if __name__ == "__main__": #pragma: no cover
    ltl_compilation_server()
