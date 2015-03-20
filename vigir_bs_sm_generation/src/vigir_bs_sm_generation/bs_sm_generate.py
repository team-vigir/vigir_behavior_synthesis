import json
import yaml

def get_class_params_from_json(json_file, yaml_file, outfile):
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

    parent_si = StateInstanstiation()
    parent_si.state_path = "/" 
    parent_si.state_path = "CLASS_STATEMACHINE"

    for name, data in sorted(nodes.items(), key=lambda x: x[0]):
        print("\nData for state {0}".format(name))
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
                if in_var in input_vars:
                    is_activation = True
                    in_var_to_sm[in_var] = class_name
                    sm_to_in_var[class_name] = in_var
                    break
            if not is_activation:
                perform_sms.add(class_name)

        for next_state, condition_idxs in transitions.items():
            conditions = [get_in_var_name(i) for i in condition_idxs]
            sm_out_data = []
            for in_var in conditions:
                if in_var in in_var_to_sm: # _c type of variable
                    sm = in_var_to_sm[in_var]
                    sm_out_data.append((in_var, sm_maps[sm][in_var]))
                else: # sensor needed
                    var_config = config[in_var]
                    class_name = var_config['class']
                    out_map = var_config['output_mapping']
                    sm_maps[class_name] = out_map
                    in_var_to_sm[v] = class_name
                    sm_to_in_var[v] = class_name

                    sm_out_data.append((in_var, out_map[in_var]))
            sm_out_data.sort(key=lambda (var, x): input_vars.index(var))
            sorted_sm_out_vars = [v for v, x in sm_out_data]

            print("{0} -> {1} if: {2}".format(name, next_state,
                                              sorted_sm_out_vars))

        print("Concurrent(")
        sorted_sms = sm_maps.keys()
        sorted_sms.sort(key=lambda x: input_vars.index(sm_to_in_var[sm]))
        for sm in sorted_sms:
            print("  {0}".format(sm))
        print(")")
        for sm in perform_sms:
            print(sm)

    return "Complete"

if __name__ == "__main__":
    #json_file = "examples/all_modes_pickup/pickup.json"
    #yaml_file = "examples/all_modes_pickup/pickup.yaml"
    json_file = "examples/object_pickup/object_pickup.json"
    yaml_file = "examples/object_pickup/object_pickup.yaml"
    out_file =  "output/test.json"
    ps = get_class_params_from_json(json_file, yaml_file, out_file)
    print(ps)
