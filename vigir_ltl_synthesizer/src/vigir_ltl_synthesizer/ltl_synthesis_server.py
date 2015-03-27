#!/usr/bin/env python

import os, sys, subprocess

import rospy
import json

from vigir_synthesis_msgs.srv import LTLSynthesis, LTLSynthesisResponse
from vigir_synthesis_msgs.msg import AutomatonState, SynthesizedAutomaton, BSErrorCodes
from vigir_ltl_synthesizer.StructuredSlugsParser import compiler as slugs_compiler

VIGIR_ROOT_DIR = os.environ['VIGIR_ROOT_DIR']

def handle_ltl_synthesis(request):
    '''Handles a request to synthesize an automaton from a LTL specification.'''
    
    ltl_spec = request.ltl_specification
    output_vars = ltl_spec.sys_props
    input_vars = ltl_spec.env_props

    if request.spec_name:
        spec_name = request.spec_name # Optional name (just a convenience)
    else:
        # Make something up
        spec_name = 'nfrewhgdn' #FIX

    # Parse LTL specification msg and write .structuredslugs file
    structured_slugs_file, folder_path = write_structured_slugs_from_msg(ltl_spec, spec_name)
    
    # First, step inside the specification's directory
    initial_dir = os.getcwd()
    os.chdir(folder_path)

    # Perform conversion: .structuredslugs --> .slugsin
    convert_structured_slugs_to_slugsin(spec_name)

    try:
        # Call slugs executable on .slugsin file and return output
        synthesizable, automaton_file = call_slugs_synthesizer(spec_name)
        automaton, error_code = handle_slugs_output(synthesizable, automaton_file, input_vars, output_vars)
    
    except Exception as e:
        synthesizable = False
        automaton = SynthesizedAutomaton() # Return empty automaton
        error_code = BSErrorCodes(BSErrorCodes.SYNTHESIS_FAILED)
        rospy.logerr('Could not infer synthesizability from SLUGS output!\n%s' % str(e))

    #TODO: Find a way to visualize automata without relying on LTLMoP

    # Finally, step out of the specification's / automaton's directory
    os.chdir(initial_dir)

    #FIX: Having synthesizable as a separate field is redundant since there's an error_code for that

    return LTLSynthesisResponse(synthesizable, automaton, error_code)

def handle_slugs_output(synthesizable, automaton_file, input_vars, output_vars):
    '''...'''

    if synthesizable:
        # Parse JSON into SynthesizedAutomaton msg
        automaton = gen_automaton_msg_from_json(automaton_file, input_vars, output_vars)
        error_code = BSErrorCodes(BSErrorCodes.SUCCESS)
        rospy.loginfo('Successfully synthesized an automaton from the LTL specification.')
    
    elif not synthesizable:  
        automaton = SynthesizedAutomaton() # Return empty automaton
        error_code = BSErrorCodes(BSErrorCodes.SPEC_UNSYNTHESIZABLE)
        rospy.logwarn('The LTL specification was unsynthesizable!')

    return automaton, error_code

def convert_structured_slugs_to_slugsin(name):
    '''Call function from StructuredSlugsParser to get slugsin file.'''
    
    slugsin_file = name + ".slugsin"

    with open(slugsin_file, "w") as f:

        #TODO: update performConversion so we don't have to do stdout redirection
        #TODO:         -//-        so that it doesn't output formulas in terminal
        sys.stdout = f
        slugs_compiler.performConversion(name + ".structuredslugs", thoroughly = True)
        sys.stdout = sys.__stdout__

    return slugsin_file

def call_slugs_synthesizer(name):
    '''...'''

    # Synthesize automaton from .slugsin input
    #FIX: Do we need the '--sysInitRoboticsSemantics' option?
    slugs_cmd = ['slugs', "--jsonOutput", name + ".slugsin", name + ".json"]

    #TODO: Check for synthesizability based on slugs output in terminal
    # For example, RESULT: Specification is realizable.
    synthesizable = True

    synthesis_process = subprocess.Popen(slugs_cmd, stdout=subprocess.PIPE, preexec_fn=os.setsid)
    os.waitpid(synthesis_process.pid, 0)

    return synthesizable, name + ".json"

def automaton_state_from_node_info(name, info, n_in_vars):
    '''
    Generate an AutomatonState from the info of a node (from the json automaton
    synthesized file.

    @param name      string     The name of this state.
    @param info      dictionary As of the writing of this doc, this dictionary
                                has the rank, state, and transitions.
    @param n_in_vars int        How many input variables there are
    '''
    state = AutomatonState()
    state.name = name
    state.output_valuation = info['state'][n_in_vars:]
    state.input_valuation = info['state'][:n_in_vars]
    state.transitions = info['trans']
    state.rank  = info['rank']

    return state

def gen_automaton_msg_from_json(json_file, input_vars, output_vars):
    '''
    Generate a SynthesizedAutomaton file from a synthesized json automaton
    file.
    @param json_file   string The synthesized json automaton description.
    @param input_vars  list   Strings of the name of the input variables.
    @param output_vars list   Strings of the name of the output variables.
    '''

    with open(json_file) as data_file:
        data = json.load(data_file)
    automaton = SynthesizedAutomaton()
    automaton.output_variables = output_vars
    automaton.input_variables = input_vars
    automaton.automaton = [automaton_state_from_node_info(i) for i in data['nodes']]

    return automaton

def write_structured_slugs_from_msg(ltl_spec, name):
    '''...'''
    
    # The directory where specs and automata are saved:
    specs_folder_path = os.path.join(VIGIR_ROOT_DIR, 'catkin_ws/src/vigir_behavior_synthesis/synthesis_byproducts') 

    # The directory where this spec will be saved:
    this_folder_path = os.path.join(specs_folder_path, name)
    if not os.path.exists(this_folder_path):
        os.makedirs(this_folder_path)

    """Create, or open, the structuredslugs file and write the 8 sections."""
    structured_slugs_file = name + ".structuredslugs"

    full_file_path = os.path.join(this_folder_path, structured_slugs_file)
    
    with open(full_file_path, 'w') as spec_file:
        # System and environment propositions
        _write_input(ltl_spec, spec_file)
        _write_output(ltl_spec, spec_file)
        # Initial Conditions
        _write_sys_init(ltl_spec, spec_file)
        _write_env_init(ltl_spec, spec_file)
        # Safety Requirements & Assumptions
        _write_sys_trans(ltl_spec, spec_file)
        _write_env_trans(ltl_spec, spec_file)
        # Liveness Requirements & Assumptions
        _write_sys_liveness(ltl_spec, spec_file)
        _write_env_liveness(ltl_spec, spec_file)

    rospy.loginfo("\nCreated specification file %s in %s \n" % (structured_slugs_file, this_folder_path))

    return structured_slugs_file, this_folder_path

def _write_input(ltl_spec, spec_file):
    spec_file.write("[INPUT]\n")
    for prop in ltl_spec.env_props:
        spec_file.write(prop + "\n")
    spec_file.write("\n")

def _write_output(ltl_spec, spec_file):
    spec_file.write("[OUTPUT]\n")
    for prop in ltl_spec.sys_props:
        spec_file.write(prop + "\n")
    spec_file.write("\n")

def _write_sys_init(ltl_spec, spec_file):
    spec_file.write("[SYS_INIT]\n")
    for formula in ltl_spec.sys_init:
        spec_file.write(formula + "\n")
    spec_file.write("\n")

def _write_env_init(ltl_spec, spec_file):
    spec_file.write("[ENV_INIT]\n")
    for formula in ltl_spec.env_init:
        spec_file.write(formula + "\n")
    spec_file.write("\n")

def _write_sys_trans(ltl_spec, spec_file):
    spec_file.write("[SYS_TRANS]\n")
    for formula in ltl_spec.sys_trans:
        spec_file.write(formula + "\n")
    spec_file.write("\n")

def _write_env_trans(ltl_spec, spec_file):
    spec_file.write("[ENV_TRANS]\n")
    for formula in ltl_spec.env_trans:
        spec_file.write(formula + "\n")
    spec_file.write("\n")

def _write_sys_liveness(ltl_spec, spec_file):
    spec_file.write("[SYS_LIVENESS]\n")
    for formula in ltl_spec.sys_liveness:
        spec_file.write(formula + "\n")
    spec_file.write("\n")

def _write_env_liveness(ltl_spec, spec_file):
    spec_file.write("[ENV_LIVENESS]\n")
    for formula in ltl_spec.env_liveness:
        spec_file.write(formula + "\n")
    spec_file.write("\n")

def ltl_synthesis_server():
    ''''Server'''
    
    rospy.init_node('vigir_ltl_synthesizer')
    
    s = rospy.Service('ltl_synthesis', LTLSynthesis, handle_ltl_synthesis)
    
    rospy.loginfo("Ready to receive LTL Synthesis requests.")
    rospy.spin()

if __name__ == "__main__":
    ltl_synthesis_server()
