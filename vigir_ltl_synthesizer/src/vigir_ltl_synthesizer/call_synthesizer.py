#!/usr/bin/env python

import os, sys
import subprocess

from vigir_ltl_specification.atlas_specification import ControlModeSpecification
from vigir_ltl_synthesizer.StructuredSlugsParser.compiler import performConversion

vigir_repo = os.environ['VIGIR_ROOT_DIR']

# =====================================================
# Parse structured SLUGS file and synthesize automaton
# =====================================================

name = "go_to_manipulate"

my_mode_spec = ControlModeSpecification(name, initial_mode = 'stand_prep',
											  modes_of_interest = ['stand_prep', 'stand', 'manipulate'])

# Add manipulate as a goal (system liveness requirement)
my_mode_spec.add_control_mode_goal("manipulate")

# Write specification in .structuredslugs file in synthesis_byproducts folder
specs_folder = os.path.join(vigir_repo, 
							'catkin_ws/src/vigir_behavior_synthesis/vigir_ltl_synthesizer/synthesis_byproducts')
structured_slugs_file, folder_path = my_mode_spec.write_structured_slugs_file(specs_folder)

# Prepare for conversion and synthesis using slugs
# structured_parser_path = os.path.join(vigir_repo, 'catkin_ws/src/vigir_behavior_synthesis/vigir_ltl_synthesizer', "StructuredSlugsParser")
# sys.path.insert(0, structured_parser_path)
# import conversion function from slugs' compiler.py
# from compiler import performConversion

# First, step inside the specification's directory
initial_dir = os.getcwd()
os.chdir(folder_path)

with open(name + ".slugsin", "w") as f:

	#TODO: update performConversion so we don't have to do stdout redirection
	#TODO:         -//-			 so that it doesn't output formulas in terminal
	sys.stdout = f
	performConversion(name + ".structuredslugs", thoroughly = True)
	sys.stdout = sys.__stdout__

# Synthesize automaton from .slugsin input
slugs_cmd = ['slugs', "--sysInitRoboticsSemantics", name + ".slugsin", name + ".aut"] #TODO: --jsonOutput

synthesis_process = subprocess.Popen(slugs_cmd, stdout=subprocess.PIPE, preexec_fn=os.setsid)
os.waitpid(synthesis_process.pid, 0)

#TODO: Check for synthesizability based on slugs output in terminal (RESULT: Specification is realizable.)

#TODO: Find a way to visualize automata without relying on LTlMoP

# Finally, step out of the specification's / strategy's directory
os.chdir(initial_dir)