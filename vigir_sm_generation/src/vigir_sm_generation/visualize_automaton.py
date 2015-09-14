import argparse
import yaml
import json
import pickle
import os
from graphviz import Digraph
from sm_gen_config import SMGenConfig
from vigir_synthesis_msgs.msg import AutomatonState, FSAutomaton

def get_DOT_from_fsautomaton(fsautomaton, config):
    """
    Given an FSAutomaton instance and config dictionary, return a DOT file
    describing the FSAutomaton.
    """
    dot = Digraph(comment='The Round Table')
    out_vars = fsautomaton.output_variables
    in_vars = fsautomaton.input_variables
    automata = fsautomaton.automaton
    helper = SMGenConfig(config, in_vars, out_vars, automata)

    # A tricky part is that some nodes (mainly just the finish and failed ones)
    # are duplicated. So, if two nodes have the same label (e.g. "finished") we
    # want to combine them. To do so, map all nodes names to a label, and then
    # map each label to a canonical node name.
    node_name_to_label = {}
    for a in helper.automata:
        name = str(a.name)
        out_val = a.output_valuation
        true_out_val = [out_vars[i] for i, v in enumerate(out_val)
                                  if v or out_vars[i][-2:] == "_m"]
        node_name_to_label[name] = "\n".join(true_out_val)

    label_to_canonical_node_name = {}
    for name, label in node_name_to_label.items():
        # Use the last name in list as the canonical name
        label_to_canonical_node_name[label] = name

    for label, name in label_to_canonical_node_name.items():
        dot.node(name, label)

    # Add edges
    def get_name(node_name):
        label = node_name_to_label[node_name]
        return label_to_canonical_node_name[label]

    for a in automata:
        parent = str(a.name)
        trans = helper.get_transitions(a)
        for next_state, trans_condition in trans.items():
            label = "\n".join(trans_condition)
            n1 = get_name(parent)
            n2 = get_name(next_state)
            dot.edge(n1, n2, label)

    return dot

# Copied from ltl_synthesis_server.py
def automaton_state_from_node_info(name, info, n_in_vars, mem_idxs):
    '''
    Generate an AutomatonState from the info of a node (from the json automaton
    synthesized file.
    @param name      string     The name of this state.
    @param info      dictionary As of the writing of this doc, this dictionary
                                has the rank, state, and transitions.
    @param n_in_vars int        How many input variables there are
    @param mem_idxs  list       Indices of where an output variable is a memory
                                proposition (i.e. remove it)
    '''
    state = AutomatonState()
    state.name = str(name) # convert integer to string

    state.output_valuation = info['state'][n_in_vars:]
    ## Only keep an output variable if it is not a memory proposition
    #state.output_valuation = [x for i, x in enumerate(state.output_valuation)
    #                            if i not in mem_idxs]
    state.input_valuation = info['state'][:n_in_vars]
    state.transitions = [str(t) for t in info['trans']] # convert integers to strings
    state.rank  = info['rank']

    return state

# Copied from ltl_synthesis_server.py
def gen_automaton_msg_from_json(json_file):
    '''
    Generate a FSAutomaton file from a synthesized json automaton
    file.
    @param json_file   string The synthesized json automaton description.
    '''

    with open(json_file) as data_file:
        data = json.load(data_file)

    #mem_idxs = [i for i, x in enumerate(output_vars) if "_m" == x[-2:]]
    
    automaton = FSAutomaton()
    vs = data['variables']
    automaton.input_variables = [v for v in vs
                                    if v[-2:] == "_c" or v[-2:] == "_f"]
    automaton.output_variables = [v for v in vs
                                   if v not in automaton.input_variables]
    ## Only keep an output variable if it is not a memory proposition
    #automaton.output_variables = [x for i, x in enumerate(output_vars)
    #                                if i not in mem_idxs]
    #automaton.input_variables = input_vars
    n_in_vars = len(automaton.input_variables)

    states = data['nodes'] # The automaton's states are called nodes in the synthesizer's output

    automaton.automaton = [automaton_state_from_node_info(i, states[i],
                           n_in_vars, None) for i in states]

    return automaton

def dict_to_automaton(d):
    a = AutomatonState()
    a.name = d['name']
    a.output_valuation = d['output_valuation']
    a.input_valuation = d['input_valuation']
    a.transitions = d['transitions']
    a.rank = d['rank']
    return a

def dict_to_fsautomaton(d):
    fsa = FSAutomaton()
    fsa.output_variables = d['output_variables']
    fsa.input_variables = d['input_variables']
    fsa.automaton = [dict_to_automaton(a) for a in d['automaton']]
    return fsa

def load_fsautomaton(path):
    extension = path.split(".")[-1]
    if extension == "pickle":
        with open(path, 'r') as f:
            fsa = pickle.load(f)
        return fsa

    if extension == "yaml":
        with open(path, 'r') as f:
            data = yaml.load(f)
    elif extension == "json":
        with open(path, 'r') as f:
            data = json.load(f)
    else:
        raise Exception("Extension {0} not recognized".format(extension))
    return dict_to_fsautomaton(data)

def get_DOT_from_fsautomaton_simple(fsautomaton):
    """
    Given an FSAutomaton instance, return a DOT file describing the FSAutomaton.
    """
    dot = Digraph(comment='The Round Table')
    out_vars = fsautomaton.output_variables
    in_vars = fsautomaton.input_variables
    automata = fsautomaton.automaton

    node_to_true_in_vals = {}
    # Add nodes
    for a in automata:
        name = str(a.name)
        out_val = a.output_valuation

        true_out_val = [out_vars[i] for i, v in enumerate(out_val)
                            if v or out_vars[i][-2:] == "_m"]
        label = "\n".join(true_out_val)
        if label == "":
            label = "INIT"
        dot.node(name, label)

        in_val = a.input_valuation
        node_to_true_in_vals[name] = [in_vars[i] for i, v in enumerate(in_val)
                                        if v or in_vars[i][-2:] == "_m"]

    # Add edges
    for a in automata:
        in_val = a.input_valuation
        parent = str(a.name)
        for next_state in a.transitions:
            label = "\n".join(node_to_true_in_vals[next_state])
            dot.edge(parent, next_state, label)

    return dot

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("fs_automaton_path",
                        help="Path to the fs_automaton (.pickle, .yaml, or "
                             ".json) file")
    parser.add_argument("save_dir",
                        help="Path to the directory where files are saved.")
    parser.add_argument("--config_path",
                        help="Path to the system config (.yaml) file")
    parser.add_argument("-v", "--verbose",
                        action="store_true",
                        help="Increase verbosity.")
    parser.add_argument("-u", "--use_old",
                        action="store_true",
                        help="Use old implementation.")
    parser.add_argument("-uj", "--use_old_json",
                        action="store_true",
                        help="Use old implementation.")
    args = parser.parse_args()

    fs_automaton_path = args.fs_automaton_path
    save_dir = args.save_dir

    if not os.path.exists(save_dir):
        os.mkdir(save_dir)

    if args.use_old_json:
        fsa = gen_automaton_msg_from_json(fs_automaton_path)
    else:
        fsa = load_fsautomaton(fs_automaton_path)
    if args.verbose:
        print("\n*** Input FSAutomaton ***")
        print(fsa)

    if args.use_old:
        config_path = args.config_path
        with open(config_path, 'r') as f:
            config = yaml.load(f)
        dot = get_DOT_from_fsautomaton(fsa, config)
    else:
        dot = get_DOT_from_fsautomaton_simple(fsa)

    dot.render(os.path.join(save_dir, "result"), view=args.verbose)
    if args.verbose:
        print("\n*** Output DOT file ***")
        print(dot)

if __name__ == '__main__':
    main()
