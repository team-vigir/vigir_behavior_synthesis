#!/usr/bin/env python

import os, sys
import commands
from distutils.spawn import find_executable

import json

import rospy
import rospkg

from vigir_synthesis_msgs.srv import SynthesizeAutomaton, SynthesizeAutomatonResponse
from vigir_synthesis_msgs.msg import AutomatonState, FSAutomaton, SynthesisErrorCodes
from vigir_ltl_synthesizer.StructuredSlugsParser import compiler as slugs_compiler


def handle_ltl_synthesis(request):
    '''Handles a request to synthesize an automaton from a LTL specification.'''

    ltl_spec = request.ltl_specification
    output_vars = ltl_spec.sys_props
    input_vars = ltl_spec.env_props

    if request.spec_name:
        if request.spec_name[0] == '/':  # FlexBE requests start with a /, the SM's path
            path_name = request.spec_name[1:] # Strip the first slash to make it a relative path
        else:
            path_name = request.spec_name
        spec_name = os.path.split(path_name)[-1] # Extract the name from the path
    else:
        #TODO: If no name is provided, generate one based on system goals
        spec_name = 'default_spec_name'
        path_name = spec_name

    # Parse LTL specification msg and write .structuredslugs file
    structured_slugs_file, folder_path = write_structured_slugs_from_msg(ltl_spec, path_name, spec_name)

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
        automaton = FSAutomaton() # Return empty automaton
        error_code = SynthesisErrorCodes(SynthesisErrorCodes.SYNTHESIS_FAILED)
        rospy.logerr('Could not infer synthesizability from SLUGS output!\n %s' % str(e))

    #TODO: Find a way to visualize automata without relying on LTLMoP

    # Finally, step out of the specification's / automaton's directory
    os.chdir(initial_dir)

    #FIX: Having synthesizable as a separate field is redundant since there's an error_code for that

    return SynthesizeAutomatonResponse(synthesizable, automaton, error_code)

def handle_slugs_output(synthesizable, automaton_file, input_vars, output_vars):
    '''...'''
    # TODO: The output is handled elsewhere. Make this be about the Automaton msg only (get name.json here instead)
    if synthesizable:
        # Parse JSON into FSAutomaton msg
        automaton = gen_automaton_msg_from_json(automaton_file, input_vars, output_vars)
        error_code = SynthesisErrorCodes(SynthesisErrorCodes.SUCCESS)
        rospy.loginfo('\033[92mSuccessfully created Automaton msg '\
                      'from the synthesized automaton.\033[0m')

    elif not synthesizable:
        automaton = FSAutomaton() # Return empty automaton
        error_code = SynthesisErrorCodes(SynthesisErrorCodes.SPEC_UNSYNTHESIZABLE)
        rospy.logwarn('The LTL specification was unsynthesizable!')

    return automaton, error_code

def call_slugs_synthesizer(name):
    '''
    Calls SLUGS in order to synthesize automaton from .slugsin input.

    Handles potential failures, such as the specification being unrealizable.
    '''

    options = ["--jsonOutput"]  # Do we need the '--sysInitRoboticsSemantics' option?
    slugs_cmd = ['slugs'] + options + [name + ".slugsin", name + ".json"]
    slugs_cmd = ' '.join(slugs_cmd) # convert to string for use with the commands module

    (status, slugs_output) = commands.getstatusoutput(slugs_cmd)

    if status == 0:
        # The command ran successfully. Analyze output.
        synthesizable = determine_synthesizability(slugs_output)
        automaton_file = name + ".json"
    else:
        # The command did not even run.
        rospy.logerr("""SLUGS command failed with status: {0}
                     \nHave you installed slugs? Output: {1}"""
                     .format(status, slugs_output))
        automaton_file = ''
        synthesizable = False

    return synthesizable, automaton_file

def determine_synthesizability(slugs_output):
    '''Determine synthesizability based on the terminal output of SLUGS.'''

    synthesizable = False

    if 'RESULT: Specification is realizable.' in slugs_output:
        synthesizable = True
        rospy.loginfo('\033[92mSuccessfully synthesized an automaton '\
                      'from the LTL specification.\033[0m')
    else:
        rospy.logwarn('The LTL specification was unsynthesizable!\n{}'
                      .format(slugs_output))

    return synthesizable

def convert_structured_slugs_to_slugsin(name):
    '''Call function from StructuredSlugsParser to get slugsin file.'''

    slugsin_file = name + ".slugsin"

    #TODO: update performConversion so we don't have to do stdout redirection
    with open(slugsin_file, "w") as sys.stdout:

        slugs_compiler.performConversion(name + ".structuredslugs",
                                         thoroughly = True)

    sys.stdout = sys.__stdout__

    return slugsin_file

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
    # Only keep an output variable if it is not a memory proposition
    state.output_valuation = [x for i, x in enumerate(state.output_valuation)
                                if i not in mem_idxs]
    state.input_valuation = info['state'][:n_in_vars]
    state.transitions = [str(t) for t in info['trans']] # convert integers to strings
    state.rank  = info['rank']

    return state

def gen_automaton_msg_from_json(json_file, input_vars, output_vars):
    '''
    Generate a FSAutomaton file from a synthesized json automaton
    file.
    @param json_file   string The synthesized json automaton description.
    @param input_vars  list   Strings of the name of the input variables.
    @param output_vars list   Strings of the name of the output variables.
    '''

    with open(json_file) as data_file:
        data = json.load(data_file)

    mem_idxs = [i for i, x in enumerate(output_vars) if "_m" == x[-2:]]

    automaton = FSAutomaton()
    # Only keep an output variable if it is not a memory proposition
    automaton.output_variables = [x for i, x in enumerate(output_vars)
                                    if i not in mem_idxs]
    automaton.input_variables = input_vars
    n_in_vars = len(input_vars)

    states = data['nodes'] # The automaton's states are called nodes in the synthesizer's output

    automaton.automaton = [automaton_state_from_node_info(i, states[i],
                           n_in_vars, mem_idxs) for i in states]

    return automaton

def write_structured_slugs_from_msg(ltl_spec, path, name):
    """Create the structuredslugs file and write the 8 sections."""

    # The directory where specs and automata are saved:
    synthesizer_pkg_dir = rospkg.RosPack().get_path('vigir_ltl_synthesizer')
    specs_dir_path = os.path.join(synthesizer_pkg_dir, 'synthesis_byproducts')

    # The directory where this spec will be saved:
    this_folder_path = os.path.join(specs_dir_path, path)
    if not os.path.exists(this_folder_path):
        os.makedirs(this_folder_path)

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

    rospy.loginfo(' \033[92mSuccessfully created specification file:\033[0m\n'\
                  '{} in {}\n'.format(structured_slugs_file, this_folder_path))

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

    # Check whether the synthesizer (slugs executable) is installed
    slugs_exec_path = find_executable('slugs')

    if not slugs_exec_path:
        rospy.logwarn('The synthesizer (SLUGS) is NOT installed. '\
                      'Please use the install_slugs.sh script. '\
                      'The LTL Synthesis service will not be available.')
    else:
        rospy.loginfo('The synthesizer (SLUGS) is installed: {dir}'
                      .format(dir = slugs_exec_path))

        s = rospy.Service('ltl_synthesis', SynthesizeAutomaton, handle_ltl_synthesis)

        rospy.loginfo("Ready to receive LTL Synthesis requests.")

    rospy.spin()

if __name__ == "__main__":
    ltl_synthesis_server()
