#!/usr/bin/env python

import os
import pprint

import preconditions as precond
from gr1_specification import GR1Specification
from gr1_formulas import GR1Formula, FastSlowFormula

"""
Module's docstring #TODO
"""

VIGIR_ROOT_DIR = os.environ['VIGIR_ROOT_DIR']

class VigirSpecification(GR1Specification):
	"""
	The class encodes requirements related to Team ViGIR's software as GR(1) formulas, written in the structured slugs format. """
	
	def __init__(self, spec_name = '', env_props = [], sys_props = []):
		super(VigirSpecification, self).__init__(spec_name, env_props, sys_props)
		
		config_files = ['atlas_preconditions.yaml', 'vigir_preconditions.yaml']
		self.handle_atlas_preconditions(config_files)
		# self.gen_formulas_for_actions(self.sys_props)

	def handle_atlas_preconditions(self, files):
		'''Loads precondtions, converts to fast-slow, adds propositions and formulas to specification.'''
		
		location = os.path.join(VIGIR_ROOT_DIR, 'catkin_ws/src/vigir_behavior_synthesis/vigir_ltl_specification/src/vigir_ltl_specification/config')

		preconditions = precond.load_preconditions_from_config_files(location, files)

		# Assume that all preconditions are completion (env) propositions
		self.preconditions, new_env_props, new_sys_props = precond.convert_preconditions_to_fastslow(preconditions)
		self.env_props = self.merge_env_propositions(new_env_props)
		self.sys_props = self.merge_sys_propositions(new_sys_props)

		precondition_formulas = precond.gen_formulas_from_preconditions(self.preconditions, fast_slow = True)

		self.add_to_sys_trans(precondition_formulas)

	def add_action_goal(self, action):
		'''Generate (fast-slow) system liveness requirement from an action.'''

		#TODO: Extend to handle multiple goals (memory part is problematic atm)

		action_a, action_c = self.convert_action_to_props(action)

		self.gen_formulas_for_actions([action])

		# Add action as goal
		# self.add_to_sys_liveness(action_c)

		# Create a memory proposition for the action
		memory = GR1Formula()

		mem_prop, mem_formula = memory.gen_goal_memory_formula(action_c)

		success_condition = memory.gen_success_condition([mem_prop], 'finished')

		# Add the new propositions to the specification
		self.sys_props.extend([mem_prop, 'finished'])

		# Add the memory and success -related formulas to the specification
		self.add_to_sys_trans(mem_formula)
		self.add_to_sys_trans(success_condition)
		self.add_to_sys_liveness('finished')

		# And also set action and memory props to False at the beginning
		self.add_false_initial_conditions(action_a, action_c)
		self.add_false_initial_conditions(mem_prop)
		self.add_false_initial_conditions('finished')

	def gen_formulas_for_actions(self, actions):
		
		action_formula = FastSlowFormula([], actions)

		# Generate and add fast-slow formulas that govern action completion
		action_completion = action_formula.gen_action_completion_formula()
		self.add_to_env_trans(action_completion)

		# Generate and add fairness conditions for the actions
		action_fairness = action_formula.gen_generic_fairness_formula()
		self.add_to_env_liveness(action_fairness)

	def add_false_initial_conditions(self, pi_a = '', pi_c = ''):
		
		if pi_a:
			self.add_to_sys_init('! ' + pi_a)
		if pi_c:
			self.add_to_env_init('! ' + pi_c)

	def convert_action_to_props(self, action):
		
		fs_formula = FastSlowFormula([], [action])

		# Return the activation and completion props corresponding to this action
		return fs_formula.activation[0], fs_formula.completion[0]

class ControlModeSpecification(GR1Specification):
	"""
	The class encodes ATLAS' control modes as GR(1) formulas, written in the structured slugs format.

	The formulas encoding the control mode constraints are of the "fast-slow" (activation-competion) type. See (V. Raman, et al. ICRA 2013)
	"""

	def __init__(self, spec_name, initial_mode, modes_of_interest = []):

		# Get the control mode transition system corresponding to the modes of interest
		self.control_modes = ControlModeTransitionSystem(modes_of_interest)

		modes_of_interest = self.control_modes.ts.keys() # Refresh

		if initial_mode not in modes_of_interest:
			raise ValueError("The initial control mode, %s, is not one of the modes of interest: %s" % (initial_mode, str(modes_of_interest)))

		# Add the control modes of interest as sys props
		sys_props = modes_of_interest 
		env_props = []	# There are no environment proposition (yet)
		
		super(ControlModeSpecification, self).__init__(spec_name, env_props, sys_props) 

		# First, generate the control mode initial conditions and add them to the specification
		self.add_control_mode_initial_conditions(initial_mode)

		# Then, create formulas out of the control mode transition system
		self.control_mode_formula = FastSlowFormula(env_props, sys_props, self.control_modes.ts)

		# and add them to the specification as sys_trans formulas
		control_mode_sys_trans = self.control_mode_formula.gen_fs_sys_transitions()
		self.add_to_sys_trans(control_mode_sys_trans)

		# At this point, replace the original propositions with the activation/completion ones,
		# since a control mode specification is by definition of the "fast-slow" type
		self.env_props = self.control_mode_formula.completion
		self.sys_props = self.control_mode_formula.activation

		# Next, add control mode mutual exclusion formulas to env_trans
		mutex_formulas = self.control_mode_formula.gen_mutex_formulas(self.env_props, future = True)
		self.add_to_env_trans(mutex_formulas)

		# Also add formulas for how the control mode can change in one time step
		single_step_formulas = self.control_mode_formula.gen_single_step_change_formula()
		self.add_to_env_trans(single_step_formulas)

		# Finally, add environment fairness formulas
		fairness_formula = self.control_mode_formula.gen_mutex_fairness_formula()
		self.add_to_env_liveness(fairness_formula)

	def add_control_mode_initial_conditions(self, mode):
		'''Generate system and environment initial conditions from a control mode.'''

		mode_a, mode_c = self.convert_mode_to_props(mode) 	# activation/completion props ('fast-slow')
		
		fs_formula = FastSlowFormula(self.env_props, self.sys_props)
		sys_init, env_init = fs_formula.gen_init_fs_from_props([mode_a, mode_c])

		self.add_to_sys_init(sys_init)
		self.add_to_env_init(env_init)

	def add_control_mode_goal(self, mode):
		'''Generate (fast-slow) system liveness requirement from a control mode.'''

		_ , mode_c = self.convert_mode_to_props(mode)

		self.add_to_sys_liveness(mode_c)

	def convert_mode_to_props(self, mode):
		
		fs_formula = FastSlowFormula([], [mode])

		# Return the activation and completion props corresponding to the control mode
		return fs_formula.activation[0], fs_formula.completion[0]

	def replace_props_with_fs():
		'''Replace the original propositions with the activation/completion ones'''

		pass
		

class ControlModeTransitionSystem(object):
	"""BDI Control Mode Transition System encoded as a dictionary."""

	ts = {"stand_prep" 	: ["stand_prep", "stand"],
		  "stand"		: ["stand_prep", "stand", "manipulate", "step", "walk"],
		  "manipulate" 	: ["manipulate", "stand"],
		  "step" 		: ["step", "stand"],
		  "walk" 		: ["walk", "stand"]
		 }
	
	def __init__(self, modes_of_interest = []):

		if not modes_of_interest:
			self.ts = ControlModeTransitionSystem.ts
		else:
			self.ts = dict()
			for mode in modes_of_interest:
				# Cherry-pick the modes of interest from the dictionary
				self.ts[mode] = [m for m in ControlModeTransitionSystem.ts[mode] if m in modes_of_interest]

# =========================================================
# Entry point
# =========================================================

def main():
	
	cm_spec = ControlModeSpecification('go_to_manipulate', initial_mode = 'stand_prep',
											modes_of_interest = ['stand_prep', 'stand', 'manipulate'])

	# Add manipulate as a goal (system liveness requirement)
	# cm_spec.add_control_mode_goal("manipulate")

	vigir_spec = VigirSpecification('preconditions')

	print "[PRECONDITIONS]"
	pprint.pprint(vigir_spec.preconditions)
	pprint.pprint(vigir_spec.sys_trans)

	vigir_spec.add_action_goal('pickup')

	complete_spec = GR1Specification('pickup')
	individual_specs = [cm_spec, vigir_spec]

	complete_spec.merge_gr1_specifications(individual_specs)

	print "[SYS_INIT]"
	pprint.pprint(complete_spec.sys_init)
	print "[ENV_INIT]"
	pprint.pprint(complete_spec.env_init)
	print "[SYS_TRANS]"
	pprint.pprint(complete_spec.sys_trans)
	print "[ENV_TRANS]"
	pprint.pprint(complete_spec.env_trans)
	print "[SYS_LIVENESS]"
	pprint.pprint(complete_spec.sys_liveness)
	print "[ENV_LIVENESS]"
	pprint.pprint(complete_spec.env_liveness)

	# Write the specification to a file

	# The directory where specs and automata are saved:
	synthesis_byproducts = os.path.join(VIGIR_ROOT_DIR, 'catkin_ws/src/vigir_behavior_synthesis/synthesis_byproducts')

	complete_spec.write_structured_slugs_file(synthesis_byproducts)

if __name__ == "__main__":
	main()
