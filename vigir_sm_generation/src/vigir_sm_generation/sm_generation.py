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
import logging

from vigir_synthesis_msgs.srv import SMGenerate, SMGenerateResponse
from vigir_synthesis_msgs.msg import BSErrorCodes
from vigir_be_msgs.msg import StateInstantiation

logging.basicConfig(stream=sys.stderr, level=logging.INFO)

vigir_repo = os.environ['VIGIR_ROOT_DIR']

def new_si(state_path, state_class, outcomes, transitions, initial_state,
    p_names, p_vals):
    """ Create a new SI with optional parameters.  """
    si = StateInstantiation()
    si.state_path = state_path
    si.state_class = state_class
    si.outcomes = outcomes
    si.transitions = transitions
    if initial_state != None:
        si.initial_state_name = initial_state
    si.parameter_names = p_names
    si.parameter_values = p_vals

    return si

class SMGenerator():
    """A class used to help generate state machines."""
    def __init__(self, config, all_in_vars, all_out_vars,
            sm_fake_out_to_real_out):
        self.config = config

        # List of in/out vars of the substates
        self.all_in_vars = all_in_vars
        self.all_out_vars = all_out_vars

        # Map from what a substate thinks is the final transition to what it
        # actually should be (e.g. "done" -> "finished")
        self.sm_fake_out_to_real_out = sm_fake_out_to_real_out

        # List of outputs of the entire SM
        self.sm_fake_outputs = sm_fake_out_to_real_out.keys()

        # To exit this SM, we find states that should exit. At the end, we'll
        # make any transition that goes to one of these states go to the real
        # output.
        self.state_name_to_sm_output = {}

    def get_automaton(self, name, automata):
        """
        Returns the automaton of a given automaton name in the list of automata.

        @param name The name of the automaton of interest.
        @param automata List of AutomatonStates.
        """
        i = [a.name for a in automata].index(name)
        return automata[i]

    def get_transitions(self, name, automata):
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
        state = self.get_automaton(name, automata)
        next_states = [str(x) for x in state.transitions]
        next_states = list(set(next_states) - set([name])) # remove self loop

        if len(next_states) == 0:
            print("State {s} has no transitions out of it!".format(s = name))
            #TODO: Report an appropriate error code instead of raising and exception
        if len(next_states) > 1:
            logging.debug("Multiple next states for {0}".format(name))

        transitions = {}
        for next_state in next_states:
            input_vals = self.get_automaton(next_state, automata).input_valuation
            transitions[next_state] = [idx for (idx, v) in
                                       enumerate(input_vals)
                                       if v == 1]
        return transitions

    def get_substate_name(self, in_var):
        """
        Return the readable neam of the substate associated with an input variable.
        For now, jsut use that input variable as the name.

        @param in_var Input variable associated with this substate.
        @param config Configuration dictionary from YAML.
        """
        return in_var

    def get_in_var_name(self, i):
        """Get the input variable name at index [i] in [in_vars] dict."""
        return self.all_in_vars[i]

    def get_out_var_name(self, i):
        """Get the output variable name at index [i] in [out_vars] dict."""
        return self.all_out_vars[i]

    def get_state_output_vars(self, state):
        """Extract what output variables a state is outputting."""
        out_vals = state.output_valuation
        return [self.get_out_var_name(i) for i, v in enumerate(out_vals)
                                         if v == 1]

    def is_sm_output(self, outputs):
        """Return true iff this state's output valuations indicate that this
        state should transition out of the SM completely."""
        check = set(outputs)
        return any(k in check for k in self.sm_fake_outputs)

    def update_out_to_sm_out(self, name, outputs):
        """Updates the mapping from the name of a substate that represents
        and exit, to the specific output. E.g. "State5" -> "finished"
        """
        if self.is_sm_output(outputs):
            in_both = [k for k in self.sm_fake_outputs if k in outputs]
            if len(in_both) > 1:
                raise Exception("Substate has more than one output for the"\
                              + "entire state machine.")
            if len(in_both) == 0:
                raise Exception("Substate has no output for the entire state"\
                              + "machine, but one was expected.")
            self.state_name_to_sm_output[name] = in_both[0]

    def get_real_name(self, name):
        """Returns the real name of this state. It might be different because
        a state may represent a SM exit (e.g. "State 5" -> "failed")
        """
        if name in self.state_name_to_sm_output:
            # Could actually be "State 5" -> "done" -> "finished"
            fake_output = self.state_name_to_sm_output[name]
            return self.sm_fake_out_to_real_out[fake_output]
        else:
            return name

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
    sa = request.automaton # SynthesizedAutomaton
    all_out_vars = sa.output_variables
    all_in_vars = sa.input_variables
    automata = sa.automaton
    system_name = request.system

    if system_name == 'atlas':
        yaml_file = os.path.join(vigir_repo, 'catkin_ws/src/vigir_behavior_synthesis/vigir_sm_generation/src/vigir_sm_generation/configs/atlas.yaml')
        with open(yaml_file) as yf:
            config = yaml.load(yf)
    else:
        error_code = BSErrorCodes(BSErrorCodes.NO_SYSTEM_CONFIG)
        return SMGenerateResponse([], error_code)

    # Map from what a substate thinks is the final transition to what it
    # actually should be (e.g. "done" -> "finished")
    sm_fake_out_to_real_out = config['output']
    smg = SMGenerator(config, all_in_vars, all_out_vars,
                      sm_fake_out_to_real_out)

    # Initialize list of StateInstantiation's with parent SI.
    SIs = [new_si("/", StateInstantiation.CLASS_STATEMACHINE,
           sm_fake_out_to_real_out.values(), [], "/State0", [], [])]
    for state in automata:
        curr_state_output_vars = smg.get_state_output_vars(state)
        smg.update_out_to_sm_out(state.name, curr_state_output_vars)

    for state in automata:
        name = state.name
        logging.debug("Data for state {0}".format(name))
        transitions = smg.get_transitions(name, automata)
        curr_state_output_vars = smg.get_state_output_vars(state)

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
            class_decl = var_config['class_decl']
            out_map = var_config['output_mapping']
            class_decl_to_out_map[class_decl] = out_map

            # Decide if this is an activation output or not.
            is_activation = False
            for in_var, state_outcome in out_map.items():
                if in_var in all_in_vars: # This is an Activation variable
                    is_activation = True
                    in_var_to_class_decl[in_var] = class_decl
                    break
            if not is_activation:
                perform_sms.add(class_decl)

        concurrent_si_outcomes = []
        concurrent_si_transitions = []
        for next_state, condition_idxs in transitions.items():
            conditions = [smg.get_in_var_name(i) for i in condition_idxs]
            substate_name_to_out = {}
            for in_var in conditions:
                ss_name = smg.get_substate_name(in_var)
                if in_var in in_var_to_class_decl: # Completion variable
                    class_decl = in_var_to_class_decl[in_var]
                    substate_name_to_out[ss_name] =\
                        class_decl_to_out_map[class_decl][in_var]
                else: # Sensor variable
                    var_config = config[in_var]
                    class_decl = var_config['class_decl']
                    out_map = var_config['output_mapping']
                    class_decl_to_out_map[class_decl] = out_map
                    in_var_to_class_decl[in_var] = class_decl

                    substate_name_to_out[ss_name] =\
                        class_decl_to_out_map[class_decl][in_var]

                internal_state_names[ss_name] = in_var_to_class_decl[in_var]

            next_state_name = "State{0}".format(next_state)
            internal_outcomes.append(smg.get_real_name(next_state_name))
            internal_maps.append({
                'outcome': smg.get_real_name(next_state_name),
                'condition': substate_name_to_out
            })

            # Not really needed, but it's good to be explicit
            concurrent_si_outcomes.append(next_state_name)
            concurrent_si_transitions.append(next_state_name)
            logging.debug("{0} -> {1} if: {2}".format(name, next_state,
                                              substate_name_to_out))

        p_names = ["states", "outcomes", "outcome_mapping"]
        p_vals = [str(internal_state_names), str(internal_outcomes),
                  str(internal_maps)]
        si = new_si("/State{0}".format(name), "ConcurrentState",
                    concurrent_si_outcomes, concurrent_si_transitions,
                    None, p_names, p_vals)
        SIs.append(si)

    return SMGenerateResponse(SIs, BSErrorCodes(BSErrorCodes.SUCCESS))

def sm_gen_server():
    '''Start the SM Generation server.'''
    rospy.init_node('sm_generation')
    s = rospy.Service('sm_generate', SMGenerate, generate_sm)
    rospy.loginfo("Ready to receive SM Generation requests.")
    rospy.spin()

if __name__ == "__main__":
    sm_gen_server()
