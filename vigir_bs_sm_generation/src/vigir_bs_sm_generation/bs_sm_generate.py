import json
import yaml
import rospy
import sys
import logging
# [BEGIN] ROS CODE
#from vigir_bs_msgs.srv import *
#from vigir_bs_msgs.msg import *
# [END] ROS CODE

logging.basicConfig(stream=sys.stderr, level=logging.INFO)

# Fake implementation for testing
class StateInstantiation():
    def __init__(self):
        self.state_path = "State path not set"
        self.state_class = "State class not set"
        self.outcomes = []
        self.transitions = []
        self.autonomy = "Autonomy not set"
    def __str__(self):
        s = "state_path = {0}\n".format(self.state_path)
        s += "state_class = {0}\n".format(self.state_class)
        s += "outcomes = {0}\n".format(self.outcomes)
        s += "transitions = {0}\n".format(self.transitions)
        s += "autonomy = {0}".format(self.autonomy)
        return s

def get_transitions(name, nodes):
    """
    Deduce the transition needed to go to the next states.

    @param name The name of the state to transitions away from
    @param nodes The dictionary containing all of the node information.
    @returns A dictionary, that maps next state to indices of the input
             variables that need to be true to go to that next state.

             For example:
        {
            'State1': [1, 2], # input var 1 & 2 need to be true
            'State2': [1, 3, 4],
            ...
        }
    """
    node_info = nodes[name]
    next_states = [str(x) for x in node_info['trans']]
    next_states = list(set(next_states) - set([name])) # remove self loop

    if len(next_states) == 0:
        raise Exception("This state has no exit!")
    if len(next_states) > 1:
        print("[INFO] Multiple next states for {0}".format(name))

    transitions = {}
    for next_state in next_states:
        transitions[next_state] = [idx for (idx, v) in
                                   enumerate(nodes[next_state]['in_vars'])
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
    sub_si = StateInstantiation()
    sub_si.state_path = "/State{0}/State{1}".format(name, name)
    sub_si.state_class = var_config['class']
    # I'm not sure how concurrent things are going to work, but for
    # not just don't remap the internal sm's outputs
    sub_si.outcomes = var_config['output_mapping'].values()
    sub_si.transitions = sub_si.outcomes
    sub_si.autonomy = var_config['autonomy']
    SIs.append(sub_si)

# [BEGIN] ROS CODE
#def generate_sm(data):
#    json_file = data.json_file
#    yaml_file = data.yaml_file # the block that is the destination of the arm's motion
# [END] ROS CODE
def generate_sm(json_file, yaml_file):
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

    @param json_file Path to file containing the automaton specifications.
    @param yaml_file Path to file containing automaton configuration.
    @param outfile Path to file to save results to.
    """
        
    with open(yaml_file) as yf:
        config = yaml.load(yf)

    with open(json_file) as data_file:    
        info = json.load(data_file)
    variables = info['variables']
    n_in_vars = len(config['input'])

    # Two short helper functions
    def get_in_var_name(i):
        """Get the input variable name at index [i] in [in_vars] dict."""
        return variables[i]
    def get_out_var_name(i):
        """Get the output variable name at index [i] in [out_vars] dict."""
        return variables[n_in_vars + i]

    nodes = reformat(info['nodes'], n_in_vars)
    input_vars = config['input']

    SIs = []
    parent_si = StateInstantiation()
    parent_si.state_path = "/" 
    parent_si.state_class = "CLASS_STATEMACHINE"
    parent_si.initial_state_name = "/State0" 
    parent_si.outcomes = ['finished', 'failed']
    SIs.append(parent_si)
    for name, data in sorted(nodes.items(), key=lambda x: x[0]):
        logging.debug("Data for state {0}".format(name))
        si = StateInstantiation()
        si.state_path = "/State{0}".format(name)
        si.state_class = "Concurrent_TODO_State"
        si.autonomy = config['default_autonomy']
        transitions = get_transitions(name, nodes)

        # extract what output variables the current state is outputting
        curr_state_output_vars = [get_out_var_name(i)
                                  for i, v in enumerate(data['out_vars'])
                                  if v == 1]
        perform_sms = set()
        sm_maps = {} # the map from variable to SM output name
        in_var_to_sm = {} # what state machine goes with an input variable?
        sm_to_in_var = {} # vice versa for fast lookup
        for out_var in curr_state_output_vars:
            var_config = config[out_var]
            class_name = var_config['class']
            out_map = var_config['output_mapping']
            sm_maps[class_name] = out_map

            is_activation = False
            for in_var, state_outcome in out_map.items():
                if in_var in input_vars: # This is an Activation variable
                    is_activation = True
                    in_var_to_sm[in_var] = class_name
                    sm_to_in_var[class_name] = in_var

                    add_subsubstate(name, var_config, SIs)
                    break
            if not is_activation:
                perform_sms.add(class_name)

        for next_state, condition_idxs in transitions.items():
            conditions = [get_in_var_name(i) for i in condition_idxs]
            sm_out_data = []
            for in_var in conditions:
                if in_var in in_var_to_sm: # Completion variable
                    sm = in_var_to_sm[in_var]
                    sm_out_data.append((in_var, sm_maps[sm][in_var]))
                else: # Sensor variable
                    var_config = config[in_var]
                    class_name = var_config['class']
                    out_map = var_config['output_mapping']
                    sm_maps[class_name] = out_map
                    in_var_to_sm[v] = class_name
                    sm_to_in_var[v] = class_name

                    sm_out_data.append((in_var, out_map[in_var]))

                    add_subsubstate(name, var_config, SIs)

                #TODO: figure out if there's a way to set the order or preserve
                # the mapping
            sm_out_data.sort(key=lambda (var, x): input_vars.index(var))
            sorted_sm_out_vars = [sm_out for in_var, sm_out in sm_out_data]

            si.outcomes.append(str(sorted_sm_out_vars))
            si.transitions.append("State{0}".format(next_state))
            logging.debug("{0} -> {1} if: {2}".format(name, next_state,
                                              sorted_sm_out_vars))

        logging.debug("Concurrent(")
        sorted_sms = sm_maps.keys()
        sorted_sms.sort(key=lambda x: input_vars.index(sm_to_in_var[sm]))
        for sm in sorted_sms:
            logging.debug("  {0}".format(sm))
        logging.debug(")")
        SIs.append(si) 
        for sm in perform_sms:
            logging.debug(sm)

    # [BEGIN] ROS CODE
    #return SMGenerateResponse(SIs)
    # [END] ROS CODE
    return SIs

if __name__ == "__main__":
    # [BEGIN] ROS CODE
    #rospy.init_node('bs_sm_generate')
    #s = rospy.Service('sm_generate', SMGenerate, generate_sm)
    # [END] ROS CODE

    #json_file = "examples/all_modes_pickup/pickup.json"
    #yaml_file = "examples/all_modes_pickup/pickup.yaml"
    json_file = "examples/object_pickup/object_pickup.json"
    yaml_file = "examples/object_pickup/object_pickup.yaml"
    SIs = generate_sm(json_file, yaml_file)
    for si in SIs:
        print(si)
