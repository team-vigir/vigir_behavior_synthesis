#!/usr/bin/env python
"""
Some terminology used in this file (hopefully consistently)

SM = State Machine, refers to the final big SM that will be loaded into FlexBE.
State/substate = a substate of the main SM.
out = output as defined by SMACH
(fake) output = What a substate thinks the final transition out of the entire SM.
              However, we have one more level of indirection since we can
              reconfigure "done" to "finished". In this case, "done" is the
              fake output. Sometimes just referred to as SM output.
real output = From above, it would be "finished". Always referred to as real
              output
"""

import os
import yaml
import rospy
import sys

from vigir_synthesis_msgs.srv import SMGenerate, SMGenerateResponse
from vigir_synthesis_msgs.msg import BSErrorCodes
from vigir_be_msgs.msg import StateInstantiation
from concurrent_state_generator import ConcurrentStateGenerator
from sm_gen_util import new_si, class_decl_to_string
from sm_gen_helper import SMGenHelper

vigir_repo = os.environ['VIGIR_ROOT_DIR']


def modify_names(automata):
    """Make the state names human readable."""
    for state in automata:
        state.name = "State{0}".format(state.name)
        state.transitions = ["State{0}".format(n) for n in state.transitions]
    return automata

def generate_sm_wrapper(request):
    """
    A wrapper around generate_sm. This catches any exception and
    converts it to an appropriate BSErrorCodes.
    """
    try:
        return generate_sm(request)
    except SmgError as e:
        return SMGenerateResponse([], e.args[0])

def generate_sm(request):
    """
    This method takes in a JSON file describe an automaton and a YAML file
    describing how the automaton names corresponds to real state machine names
    and produced class parameters for FlexBE.

    Broadly speaking, there are two types of input variables, and two types
    of output variables.

    Input Variables
        - Sensor     - variables that are the results of a sensor reading
        - Completion - variables that say whether or not something activated
    Output Variables
        - Activation - activate something
        - Perform    - perform something other than activate (e.g. "beep" or
                       "print")

    This distinction is important because Completion and Activation variables
    are related. Specifically, only one state machine should be created for a
    given completion-activation pair. The Activation variable tells what to
    activate, and the Completion variable tells where to go once that thing is
    activated.

    Sensor and Perform variables require their own state machines.

    The algorithm is as follows:
    1. For each state X, list all the outputs of X, and find the corresponding
       state machines in the YAML file.
        a. For each SM, find (in YAML file) what each state machine output
           mean in terms of input variables. (e.g. "changed" from the
           ChangeControlModeActionState means an input variable of "step_c".)
        b. If none of the outputs of the state machine correspond to inputs,
           then that output is a Perform variable. (e.g.
    2. For each transition (X -> Y on input A),
        a. Find what SM + outputs corresponds to A.
        b. If no SM's output maps to input A, then A must come from an external
           sensor. In this case, look up the definitions in the YAML file.
        c. Since A could be multiple conditions (e.g. "found object AND stand
           up complete"), sort these conditions based on the order of the input
           variable in the JSON file.

    @param request An instance of SMGenerateRequest
    @return A SMGenerateResponse with generated StateInstantiation.
    """
    yaml_file = os.path.join(vigir_repo, 'catkin_ws/src/vigir_behavior_synthesis/vigir_sm_generation/src/vigir_sm_generation/configs/systems.yaml')
    try:
        with open(yaml_file) as yf:
            systems = yaml.load(yf)
    except IOError:
        error_code = BSErrorCodes(BSErrorCodes.NO_SYSTEMS_FILE)
        return SMGenerateResponse([], error_code)
        
    sa = request.automaton # SynthesizedAutomaton
    all_out_vars = sa.output_variables
    all_in_vars = sa.input_variables
    automata = modify_names(sa.automaton)

    system_name = request.system
    if system_name in systems:
        yaml_file = os.path.join(vigir_repo, systems[system_name])
        try:
            with open(yaml_file) as yf:
                config = yaml.load(yf)
        except IOError:
            error_code = BSErrorCodes(BSErrorCodes.SYSTEM_CONFIG_NOT_FOUND)
            return SMGenerateResponse([], error_code)
    else:
        error_code = BSErrorCodes(BSErrorCodes.NO_SYSTEM_CONFIG)
        return SMGenerateResponse([], error_code)

    helper = SMGenHelper(config, all_in_vars, all_out_vars, automata)

    # Initialize list of StateInstantiation's with parent SI.
    SIs = [new_si("/", StateInstantiation.CLASS_STATEMACHINE,
           helper.get_sm_real_outputs(), [], "State0", [], [])]

    for state in automata:
        name = state.name
        if helper.is_fake_state(name):
            continue

        csg = ConcurrentStateGenerator(name)

        # Add an internal state for each output.
        curr_state_output_vars = helper.get_state_output_vars(state)
        for out_var in curr_state_output_vars:
            decl = helper.get_class_decl(out_var)
            csg.add_internal_state(out_var, decl)

        transitions = helper.get_transitions(state, automata)
        for next_state, conditions in transitions.items():
            substate_name_to_out = {} # i.e. condition mapping
            for in_var in conditions:
                ss_name = helper.get_substate_name(in_var)
                # go from input variable -> class declaration -> out map
                # to get what this input variable (e.g. 'stand_c') maps to in
                # the class declared (e.g. 'changed').
                decl = helper.get_class_decl(in_var)
                out_map = helper.get_out_map(decl)
                substate_name_to_out[ss_name] = out_map[in_var]
                # need to add internal state for sensor input variables
                if not helper.is_response_var(in_var):
                    csg.add_internal_state(ss_name, decl)

            csg.add_internal_outcome(helper.get_real_name(next_state))
            csg.add_internal_outcome_maps({
                'outcome': helper.get_real_name(next_state),
                'condition': substate_name_to_out
            })
            rospy.logdebug("{0} -> {1} if: {2}".format(name, next_state,
                                              substate_name_to_out))

        SIs.append(csg.gen())

    return SMGenerateResponse(SIs, BSErrorCodes(BSErrorCodes.SUCCESS))

def sm_gen_server():
    '''Start the SM Generation server.'''
    rospy.init_node('sm_generation')
    s = rospy.Service('sm_generate', SMGenerate, generate_sm)
    rospy.loginfo("Ready to receive SM Generation requests.")
    rospy.spin()

if __name__ == "__main__":
    sm_gen_server()
