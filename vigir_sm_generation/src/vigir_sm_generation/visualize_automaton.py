import argparse
import yaml
import pickle
import os
from graphviz import Digraph
from sm_gen_config import SMGenConfig

def get_DOT_from_fsautomaton(fsautomaton, config):
    dot = Digraph(comment='The Round Table')
    out_vars = fsautomaton.output_variables
    in_vars = fsautomaton.input_variables
    automata = fsautomaton.automaton

    helper = SMGenConfig(config, in_vars, out_vars, automata)

    for a in automata:
        name = str(a.name)
        out_val = a.output_valuation
        in_val = a.input_valuation
        transitions = a.transitions
        rank = a.rank

        true_in_val = [in_vars[i] for i, v in enumerate(in_val) if v]
        dot.node(name, "\n".join(true_in_val))

    return dot

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("fs_automaton_path",
                        help="Path to the fs_automaton .pickle file")
    parser.add_argument("config_path",
                        help="Path to the system config .yaml file")
    parser.add_argument("save_dir",
                        help="Path to the directory where files are saved.")
    args = parser.parse_args()

    fs_automaton_path = args.fs_automaton_path
    config_path = args.config_path
    save_dir = args.save_dir

    if not os.path.exists(save_dir):
        os.mkdir(save_dir)

    with open(fs_automaton_path, 'r') as f:
        fsa = pickle.load(f)

    with open(config_path, 'r') as f:
        config = yaml.load(f)

    dot = get_DOT_from_fsautomaton(fsa, config)
    print(dot)
    dot.render(os.path.join(save_dir, "result.png"), view=True)

if __name__ == '__main__':
    main()
