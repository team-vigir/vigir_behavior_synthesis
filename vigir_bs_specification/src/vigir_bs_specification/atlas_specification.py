#!/usr/bin/env python

from gr1_specification import GR1Specification
from gr1_formulas import GR1Formula, FastSlowFormula

import pprint

"""
Module's docstring #TODO
"""

class AtlasSpecification(object):
	"""docstring for ClassName"""
	def __init__(self, arg):
		super(ClassName, self).__init__()
		self.arg = arg


class ControlModeSpecification(GR1Specification):
	"""
	The class encodes ATLAS' control modes as GR(1) formulas, written in the structured slugs format.

	The formulas encoding the control mode constraints are of the "fast-slow" (activation-competion) type. See (V. Raman, et al. ICRA 2013)
	"""

	def __init__(self, spec_name, initial_mode, modes_of_interest = []):
		
		if initial_mode not in modes_of_interest:
			raise ValueError("The initial control mode, %s, is not one of the modes of interest: %s" % (initial_mode, str(modes_of_interest)))

		# Get the control mode transition system corresponding to the modes of interest
		self.control_modes = ControlModeTransitionSystem(modes_of_interest)
		# And add the control modes of interest as sys props
		sys_props = self.control_modes.ts.keys() 
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
	
	my_mode_spec = ControlModeSpecification('go_to_manipulate', initial_mode = 'stand_prep',
											modes_of_interest = ['stand_prep', 'stand', 'manipulate'])

	# Add manipulate as a goal (system liveness requirement)
	my_mode_spec.add_control_mode_goal("manipulate")

	print "[SYS_INIT]"
	pprint.pprint(my_mode_spec.sys_init)
	print "[ENV_INIT]"
	pprint.pprint(my_mode_spec.env_init)
	print "[SYS_TRANS]"
	pprint.pprint(my_mode_spec.sys_trans)
	print "[ENV_TRANS]"
	pprint.pprint(my_mode_spec.env_trans)
	print "[SYS_LIVENESS]"
	pprint.pprint(my_mode_spec.sys_liveness)
	print "[ENV_LIVENESS]"
	pprint.pprint(my_mode_spec.env_liveness)

	my_mode_spec.write_structured_slugs_file()

if __name__ == "__main__":
	main()
