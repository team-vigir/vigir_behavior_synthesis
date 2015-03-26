#!/usr/bin/env python

import json
import yaml
import rospy
import sys
import logging

from vigir_bs_msgs.srv import *
from vigir_bs_msgs.msg import *
from vigir_be_core.msg import StateInstantiation

logging.basicConfig(stream=sys.stderr, level=logging.INFO)

# Fake implementation for testing
#class StateInstantiation():
#    def __init__(self, state_path, state_class, outcomes, transitions,
#        initial_state=""):
#        self.state_path = state_path
#        self.state_class = state_class
#        self.outcomes = outcomes
#        self.transitions = transitions
#        self.initial_state = initial_state
#        self.parameter_name = []
#        self.parameter_value = []
#
#    def __str__(self):
#        lines = []
#        if len(self.state_path) > 0:
#            lines.append("state_path = {0}".format(self.state_path))
#        if len(self.state_class) > 0:
#            lines.append("state_class = {0}".format(self.state_class))
#        if len(self.outcomes) > 0:
#            lines.append("outcomes = {0}".format(self.outcomes))
#        if len(self.transitions) > 0:
#            lines.append("transitions = {0}".format(self.transitions))
#        if len(self.initial_state) > 0:
#            lines.append("initial_state = {0}".format(self.initial_state))
#        if len(self.parameter_name) > 0:
#            lines.append("parameter_name = {0}".format(self.parameter_name))
#        if len(self.parameter_value) > 0:
#            lines.append("parameter_value = {0}".format(self.parameter_value))
#        return "\n".join(lines)

def get_automaton(name, automata):
    """
    Returns the automaton of a given automaton name in the list of automata.

    @param name The name of the automaton of interest.
    @param automata List of AutomatonStates.
    """
    i = [a.name for a in automata].index(name)
    return automata[i]

def get_transitions(name, automata):
    """
    Deduce the transition needed to go to the next states.

    @param name The name of the state to transitions away from
    @param automata List of all AutomatonState's.
    @returns A dictionary, that maps next state to indices of the input
            variables that need to be true to go to that next state.

             For example:
        {
            'State1': [1, 2], # input var 1 & 2 need to be true
            'State2': [1, 3, 4],
            ...
        }
    """
    state = get_automaton(name, automata)
    next_states = [str(x) for x in state.transitions]
    next_states = list(set(next_states) - set([name])) # remove self loop

    if len(next_states) == 0:
        raise Exception("This state has no exit!")
    if len(next_states) > 1:
        logging.debug("Multiple next states for {0}".format(name))

    transitions = {}
    for next_state in next_states:
        input_vals = get_automaton(next_state, automata).input_valuation
        transitions[next_state] = [idx for (idx, v) in
                                   enumerate(input_vals)
                                   if v == 1]
    return transitions

def reformat(nodes, n_in_vars):
    """
    Reformat automaton dictionary so that input/output variables are
    separated.  For example, if before the dictionary for a node was
    {
        ...
        "states" : [a, b, c, d, e, f, g] # a-d are input, e-g are output
        ...
    }
    where a-d corresponds to input variables and e-g corresponds to output
    variables, now it looks like
    {
        ...
        "in_vars" : [a, b, c, d]
        "out_vars" : [e, f, g]
        ...
    }
    """
    for name, dic in nodes.items():
        in_vars = dic['state'][:n_in_vars]
        out_vars = dic['state'][n_in_vars:]
        dic['in_vars'] = in_vars
        dic['out_vars'] = out_vars
        dic.pop('state', None)
    return nodes

def add_subsubstate(name, var_config, SIs):
    """
    Add a substate to the [name] concurrent substate.
    @param name       The name of the substate.
    @param var_config The configuration for this variable.
    @param SIs        The global list of state instantiations.
    """
    state_path = "/State{0}/State{1}".format(name, name)
    state_class = var_config['class']
    # For Substates within a ConcurrentState, you don't need to remap their
    # outcomes.
    sub_si = StateInstantiation(state_path, state_class, [], [])
    if 'params' in var_config:
        sub_si.parameter_name = var_config['params']['names']
        sub_si.parameter_value = var_config['params']['values']
    SIs.append(sub_si)

def get_substate_name(in_var, config):
    """
    Return the readable neam of the substate associated with an input variable.
    For now, jsut use that input variable as the name.

    @param in_var Input variable associated with this substate.
    @param config Configuration dictionary from YAML.
    """
    return in_var

#def generate_sm(json_file, yaml_file):
def generate_sm(data):
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

    @param data An instance of SMGenerateRequest with a SynthesizedAutomaton.
    @return A SMGenerateResponse with generated StateInstantiation.
    """
    json_file = data.json_file
    all_out_vars = data.output_variables
    all_in_vars = data.input_variables
    automata = data.automaton
    yaml_file = data.yaml_file

    with open(yaml_file) as yf:
        config = yaml.load(yf)

    # Two short helper functions
    def get_in_var_name(i):
        """Get the input variable name at index [i] in [in_vars] dict."""
        return all_in_vars[i]
    def get_out_var_name(i):
        """Get the output variable name at index [i] in [out_vars] dict."""
        return all_out_vars[i]

    input_vars = config['input']

    # Initialize list of StateInstantiation's with parent SI.
    SIs = [StateInstantiation("/", "CLASS_STATEMACHINE", ['done', 'failed'],
                              [], initial_state = "/State0")]
    for state in automata:
        name = state.name
        out_vals = state.output_valuations
        logging.debug("Data for state {0}".format(name))
        transitions = get_transitions(name, automata)

        # extract what output variables the current state is outputting
        curr_state_output_vars = [get_out_var_name(i)
                                  for i, v in enumerate(out_vals)
                                  if v == 1]
        perform_sms = set()
        class_decl_to_out_map = {} # the map from class declaration to its out_map
        in_var_to_class_decl = {} # what state machine goes with an input variable?

        # Internal parameters of the ConcurrentState that we need to build.
        # See 'states', 'outcomes' and 'outcome_mapping' in the documentation
        # of ConcurrentState for more detail.
        internal_state_names = {}
        internal_outcomes = []
        internal_maps = []
        for out_var in curr_state_output_vars:
            var_config = config[out_var]
            class_decl = var_config['class']
            out_map = var_config['output_mapping']
            class_decl_to_out_map[class_decl] = out_map

            is_activation = False
            for in_var, state_outcome in out_map.items():
                if in_var in input_vars: # This is an Activation variable
                    is_activation = True
                    in_var_to_class_decl[in_var] = class_decl

                    add_subsubstate(name, var_config, SIs)
                    break
            if not is_activation:
                perform_sms.add(class_decl)

        concurrent_si_outcomes = []
        concurrent_si_transitions = []
        for next_state, condition_idxs in transitions.items():
            conditions = [get_in_var_name(i) for i in condition_idxs]
            substate_name_to_out = {}
            for in_var in conditions:
                ss_name = get_substate_name(in_var, config)
                if in_var in in_var_to_class_decl: # Completion variable
                    class_decl = in_var_to_class_decl[in_var]
                    substate_name_to_out[ss_name] =\
                        class_decl_to_out_map[class_decl][in_var]
                else: # Sensor variable
                    var_config = config[in_var]
                    class_decl = var_config['class']
                    out_map = var_config['output_mapping']
                    class_decl_to_out_map[class_decl] = out_map
                    in_var_to_class_decl[in_var] = class_decl

                    substate_name_to_out[ss_name] =\
                        class_decl_to_out_map[class_decl][in_var]

                    add_subsubstate(name, var_config, SIs)
                internal_state_names[ss_name] = in_var_to_class_decl[in_var]

            next_state_name = "State{0}".format(next_state)
            internal_outcomes.append(next_state_name)
            internal_maps.append({
                'outcome': next_state_name,
                'condition': substate_name_to_out
            })

            # Not really needed, but it's good to be explicit
            concurrent_si_outcomes.append(next_state_name)
            concurrent_si_transitions.append(next_state_name)
            logging.debug("{0} -> {1} if: {2}".format(name, next_state,
                                              substate_name_to_out))

        si = StateInstantiation("/State{0}".format(name), "ConcurrentState",
                                concurrent_si_outcomes,
                                concurrent_si_transitions)
        si.parameter_name = ["states", "outcomes", "outcome_mapping"]
        si.parameter_value = [str(internal_state_names),
                              str(internal_outcomes), str(internal_maps)]
        SIs.append(si)

    return SMGenerateResponse(SIs)

if __name__ == "__main__":
    rospy.init_node('bs_sm_generate')
    s = rospy.Service('sm_generate', SMGenerate, generate_sm)

    #json_file = "examples/all_modes_pickup/pickup.json"
    #yaml_file = "examples/all_modes_pickup/pickup.yaml"
    #json_file = "examples/object_pickup/object_pickup.json"
    #yaml_file = "examples/object_pickup/object_pickup.yaml"
    #SIs = generate_sm(json_file, yaml_file)
    #for si in SIs:
    #    print("")
    #    print(si)
