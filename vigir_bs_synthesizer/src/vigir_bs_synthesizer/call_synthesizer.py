import os, sys
import subprocess

# The directory where specs and automata are saved:
vigir_bs_specification = os.path.abspath(os.path.join(os.pardir, os.pardir, os.pardir, 'vigir_bs_specification', 'src'))
sys.path.insert(0, vigir_bs_specification)

from vigir_bs_specification.atlas_specification import ControlModeSpecification

# =====================================================
# Parse structured SLUGS file and synthesize automaton
# =====================================================

name = "go_to_manipulate"

my_mode_spec = ControlModeSpecification(name, initial_mode = 'stand_prep',
											  modes_of_interest = ['stand_prep', 'stand', 'manipulate'])

# Add manipulate as a goal (system liveness requirement)
my_mode_spec.add_control_mode_goal("manipulate")

# Write specification in .structuredslugs file
structured_slugs_file, folder_path = my_mode_spec.write_structured_slugs_file()

# Prepare for conversion and synthesis using slugs
slugs_path = os.path.abspath(os.path.join(os.pardir, os.pardir, os.pardir, os.pardir, 'slugs'))
slugs_exec_path = os.path.join(slugs_path, 'build', 'slugs') #FIX: This will either have to change or it will be redundant (slugs will be in path)
slugs_structured_parser_path = os.path.join(slugs_path, "tools", "StructuredSlugsParser")
sys.path.insert(0, slugs_structured_parser_path)

# import conversion function from slugs' compiler.py
from compiler import performConversion

# Step inside the specification's directory
initial_dir = os.getcwd()
os.chdir(folder_path)

with open(name + ".slugsin", "w") as f:

	#TODO: update performConversion so we don't have to do stdout redirection
	#TODO:         -//-			 so that it doesn't output formulas in terminal
	sys.stdout = f
	performConversion(name + ".structuredslugs", thoroughly = True)
	sys.stdout = sys.__stdout__

# Synthesize automaton from .slugsin input
slugs_cmd = [slugs_exec_path, "--sysInitRoboticsSemantics", name + ".slugsin", name + ".aut"] #TODO: --jsonOutput

synthesis_process = subprocess.Popen(slugs_cmd, stdout=subprocess.PIPE, preexec_fn=os.setsid)
os.waitpid(synthesis_process.pid, 0)

#TODO: Check for synthesizability based on slugs output in terminal (RESULT: Specification is realizable.)

# =====================================================
# Visualize automaton (Graphviz .dot file)
# =====================================================

# Add LTLMoP src folder to Python's path in order to use Strategy
ltlmop_path = os.path.expanduser("~/Cornell/My Research/LTLMoP/src/")
sys.path.insert(0, ltlmop_path)
from lib.strategy import createStrategyFromFile
			
def visualize_automaton(name):
	automaton = createStrategyFromFile(name + ".aut", my_mode_spec.env_props, my_mode_spec.sys_props)
	# init_prop_assignment = dict(sys_init.items() + env_init.items()) #TODO: I don't know if that's what the 'start_state' thing means ...
	# start_state = automaton.searchForOneState(init_prop_assignment)  #TODO: Is this important? I had been populating it accidentally ...
	# automaton.exportAsDotFile(name + ".dot", start_state)
	automaton.exportAsDotFile(name + ".dot", {})

	return "done"

visualize_automaton(name)

# Step out of the specification's / strategy's directory
os.chdir(initial_dir)