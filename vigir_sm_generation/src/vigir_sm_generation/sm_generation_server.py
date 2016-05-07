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
import sys
import yaml

import rospy
import rospkg

from vigir_synthesis_msgs.srv import GenerateFlexBESM, GenerateFlexBESMResponse
from vigir_synthesis_msgs.msg import SynthesisErrorCodes
from flexbe_msgs.msg import StateInstantiation
from concurrent_state_generator import ConcurrentStateGenerator
from sm_gen_config import SMGenConfig
from sm_gen_util import (
    new_si,
    class_decl_to_string,
    clean_variable
)
from sm_gen_error import SMGenError

INIT_STATE_NAME = "ROOT_OF_INIT_STATES"

def modify_names(all_out_vars, automata):
    """
    Make the state names human readable.

    We append "_X" to the state name for each output variable X that is true.
    """
    for state in automata:
        state.name = str(state.name) # sometimes they're ints

    old_name_to_new_name = {}
    for state in automata:
        new_name = state.name
        for i, val in enumerate(state.output_valuation):
            if val:
                new_name += "_" + clean_variable(all_out_vars[i])
        old_name_to_new_name[state.name] = new_name

    for state in automata:
        state.name = old_name_to_new_name[state.name]
        state.transitions = [old_name_to_new_name[n]
                             for n in state.transitions]

    return automata

def get_init_temp_state(init_states):
    """
    Return a temporary initial state that goes to every real initial state.

    init_states: a list of initial states to go to
    """
    init_state_names = [s.name for s in init_states]
    return new_si(
        state_path = "/{0}".format(INIT_STATE_NAME),
        state_class = "LogState",
        outcomes = ["done" for s in init_state_names], # outcomes and
        transitions = init_state_names, # transitions are the same
        initial_state = None,
        p_names = ["text"],
        p_vals = ['"Initial state"'],
        autonomy = [0 for s in init_states]
    )

def generate_sm(request):
    """
    A wrapper around generate_sm_handle. This catches any exception and
    converts it to an appropriate SynthesisErrorCodes.
    """
    try:
        return generate_sm_handle(request)
    except SMGenError as e:
        rospy.logerr("There was an SMGenError: {0}".format(e.error_code))
        return GenerateFlexBESMResponse([], SynthesisErrorCodes(e.error_code))
    except Exception as e:
        rospy.logerr("Something went wrong:\n\t{0}\n\t{1}"\
            .format(e.__doc__, e.message))
        return GenerateFlexBESMResponse([],
            SynthesisErrorCodes(SynthesisErrorCodes.SM_GENERATION_FAILED))

def generate_sm_handle(request):
    """
    This method handles the GenerateFlexBESM service. It creates StateInstantiations
    given a system name and an automaton.

    Broadly speaking, there are two types of input variables, and two types
    of output variables.

    Input Variables
        - Sensor     - variables that are the results of a sensor reading
        - Completion - variables that say whether or not something activated
    Output Variables
        - Activation - activate something
        - Perform    - perform something other than activate (e.g. "beep" or
                       "print") TODO: implement this

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
           then that output is a Perform variable. (TODO: implement)
    2. For each transition (X -> Y on input A),
        a. Find what SM + outputs corresponds to A.
        b. If no SM's output maps to input A, then A must come from an external
           sensor. In this case, look up the definitions in the YAML file.
        c. Otherwise, it's a completion variable. Connect it to the
           corresponding SM.

    @param request An instance of GenerateFlexBESMRequest
    @return A GenerateFlexBESMResponse with generated StateInstantiation.
    """

    sm_generation_pkg_dir = rospkg.RosPack().get_path('vigir_sm_generation')
    config_dir = os.path.join(sm_generation_pkg_dir,
                             'src/vigir_sm_generation/configs')
    yaml_file = os.path.join(config_dir, 'systems.yaml')

    try:
        with open(yaml_file) as yf:
            systems = yaml.load(yf)
    except IOError:
        rospy.logerr("System file could not be loaded from {0}."
            .format(yaml_file))
        raise SMGenError(SynthesisErrorCodes(error_code))

    sa = request.automaton # FSAutomaton
    all_out_vars = sa.output_variables
    all_in_vars = sa.input_variables
    automata = modify_names(all_out_vars, sa.automaton)

    # Load the config file
    system_name = request.system
    if system_name not in systems:
        rospy.logerr("System {0} is not in the systems file ({1})."\
            .format(system_name, yaml_file))
        raise SMGenError(SynthesisErrorCodes.NO_SYSTEM_CONFIG)

    yaml_sys_file = os.path.join(config_dir, systems[system_name])

    try:
        with open(yaml_sys_file) as yf:
            config = yaml.load(yf)
    except IOError:
        rospy.logerr("System {0} could not be loaded from {1}."\
            .format(system_name, yaml_sys_file))
        raise SMGenError(SynthesisErrorCodes.SYSTEM_CONFIG_NOT_FOUND)

    helper = SMGenConfig(config, all_in_vars, all_out_vars, automata)

    # Initialize list of StateInstantiation's with parent SI.
    # SIs = [new_si("/", StateInstantiation.CLASS_STATEMACHINE,
    #        helper.get_sm_real_outputs(), [], INIT_STATE_NAME, [], [])]
    init_states = helper.get_init_states()
    # SIs.append(get_init_temp_state(init_states))

    #TEMP: Assume single initial state for July experiments:
    SIs = [new_si("/", StateInstantiation.CLASS_STATEMACHINE,
           helper.get_sm_real_outputs(), [], init_states[0], [], [])]

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
            csg.add_internal_userdata(helper.get_userdata_keys(out_var),
                                      helper.get_userdata_remapping(out_var))

        transitions = helper.get_transitions(state)
        #TEMP: Ignore states where everything is False [?]
        if not transitions:
            continue

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

            is_concurrent = csg.is_concurrent()
            csg.add_internal_outcome_and_transition(
                helper.get_outcome_name(is_concurrent, next_state,
                                        substate_name_to_out),
                helper.get_real_name(next_state),
                helper.get_autonomy_list(substate_name_to_out)
            )
            csg.add_internal_outcome_maps({
                'outcome': helper.get_outcome_name(is_concurrent, next_state,
                                                   substate_name_to_out),
                'condition': substate_name_to_out
            })
            rospy.logdebug("{0} -> {1} if: {2}".format(name, next_state,
                                              substate_name_to_out))

        SIs.append(csg.gen())

    return GenerateFlexBESMResponse(SIs, SynthesisErrorCodes(SynthesisErrorCodes.SUCCESS))

def sm_gen_server():
    '''Start the SM Generation server.'''
    rospy.init_node('vigir_sm_generation')
    s = rospy.Service('sm_generate', GenerateFlexBESM, generate_sm)
    rospy.loginfo("Ready to receive SM Generation requests.")
    rospy.spin()

if __name__ == "__main__":
    sm_gen_server()
