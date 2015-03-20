#!/usr/bin/env python

import os, sys, subprocess

import rospy

from vigir_bs_msgs.srv import LTLSynthesis
from vigir_bs_msgs.msg import AutomatonState, SynthesizedAutomaton, BSErrorCodes, LTLSynthesisResponse
from vigir_bs_synthesizer.StructuredSlugsParser.compiler import performConversion

vigir_repo = os.environ['VIGIR_ROOT_DIR']

def handle_ltl_synthesis(request):
    '''Handles a request to synthesize an automaton from a LTL specification.'''
    
    specification = request.ltl_specification

    if request.spec_name:
        spec_name = request.spec_name # Optional name (just a convenience)
    else:
        # Make something up
        spec_name = 'FYDS-FY_DS' #FIX

    # Parse LTL specification msg and write .structuredslugs file
    structured_slugs_file, folder_path = write_structured_slugs_from_msg(specification)
    
    # First, step inside the specification's directory
    initial_dir = os.getcwd()
    os.chdir(folder_path)

    # Perform conversion: .structuredslugs --> .slugsin
    convert_structured_slugs_to_slugsin(spec_name)

    try:
        # Call slugs executable on .slugsin file and return output
        synthesizable, automaton_file = call_slugs_synthesizer(spec_name)
        automaton, error_code = handle_slugs_output(synthesizable, automaton_file)
    except Exception as e:
        synthesizable = False
        automaton = SynthesizedAutomaton() # Return empty automaton
        error_code = BSErrorCodes(BSErrorCodes.SYNTHESIS_FAILED)
        rospy.logerr('Could not infer synthesizability from SLUGS output!\n%s' % str(e))

    #TODO: Find a way to visualize automata without relying on LTLMoP

    # Finally, step out of the specification's / automaton's directory
    os.chdir(initial_dir)

    return LTLSynthesisResponse(synthesizable, automaton, error_code)

def handle_slugs_output(synthesizable, automaton_file):
    '''...'''

    if synthesizable:
        error_code = BSErrorCodes(BSErrorCodes.SUCCESS)
        # Parse JSON into SynthesizedAutomaton msg
        automaton = gen_automaton_msg_from_json(automaton_file)
        rospy.loginfo('Successfully synthesized an automaton from LTL specification.')
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
        performConversion(name + ".structuredslugs", thoroughly = True)
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

def write_structured_slugs_from_msg(specification):
    '''...'''

    specs_folder = os.path.join(vigir_repo, 'catkin_ws/src/vigir_behavior_synthesis/temp_bs_files')

    pass

def gen_automaton_msg_from_json(json_file):
    '''...'''

    automaton = SynthesizedAutomaton()

    return automaton

def ltl_synthesis_server():
    ''''Server'''
    
    rospy.init_node('ltl_synthesis_server')
    
    s = rospy.Service('ltl_synthesis', LTLSynthesis, handle_ltl_synthesis)
    
    rospy.loginfo("Ready to receive LTL Synthesis requests.")
    rospy.spin()

if __name__ == "__main__":
    ltl_synthesis_server()
