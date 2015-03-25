#!/usr/bin/env python
"""Formulas for the GR(1) fragment of Linear Temporal Logic

This module contains two main classes:
  * GR1Formula for vanilla GR(1) formulas
  * FastSlowFormula for extended GR(1) formulas following the paradigm 
    in Vasumathi Raman and Hadas Kress-Gazit (ICRA 2013)

The syntax used is that of .structuredslugs, 
which is meant for use with the SLUGS synthesis tool.

"""

class GR1Formula(object):
	"""

	Arguments:
	  env_props (list of str)	Environment propositions (strings)
	  sys_props (list of str)	System propositions (strings)
	  ts 		(dict of str)	Transition system, TS (e.g. workspace topology)
	  							Implicitly contains some propositions in the keys.

	Attributes:
	  formula 	(list of str)	...

	Raises:
	  ValueError:				When a proposition is neither a system or an environment proposition

	"""
	
	def __init__(self, env_props, sys_props, ts = {}):
		
		self.sys_props = sys_props
		self.env_props = env_props
		self.ts = ts

		self._add_props_from_ts()

		self.formula = [] #TODO: The formula(s) should be stored in the object, not just returned by methods
	
	# =====================================================
	# System and environment initial conditions
	# =====================================================

	def gen_sys_init_from_prop_assignment(self, prop_assignment, props = "sys_props"):
		"""Set the given propositions to the desired truth value in the system initial conditions formula. Set all the others to False."""
		
		sys_init = list()

		props_of_type = getattr(self, props)

		for prop in props_of_type:
			if prop in prop_assignment.keys():
				if prop_assignment[prop] is True:
					sys_init.append(prop)
				else:
					sys_init.append(self.neg(prop))
			else:
				sys_init.append(self.neg(prop))
		
		return sys_init

	def gen_env_init_from_prop_assignment(self, prop_assignment, props = "env_props"):
		"""Set the given propositions to the desired truth value in the environment initial conditions formula. Set all the others to False."""
		
		env_init = list()

		props_of_type = getattr(self, props)

		for prop in props_of_type:
			if prop in prop_assignment.keys():
				if prop_assignment[prop]:
					env_init.append(prop)
				else:
					env_init.append(self.neg(prop))
			else:
				env_init.append(self.neg(prop))
		
		return env_init

	# =====================================================
	# System transition formulas (e.g., workspace topology)
	# =====================================================

	def gen_sys_trans_formulas(self, future = True):
		"""Generate system requirement formulas that encode the transition system (e.g. workspace topology).

		The transition system TS, 'self.ts', is provided in the form of a dictionary.
		"""
		sys_trans_formulas = list()
		for prop in self.ts.keys():
			left_hand_side = prop
			right_hand_side = list()
			
			for adj_prop in self.ts[prop]:
				adj_phi_prop = self.gen_phi_prop(adj_prop)
				disjunct = self.prime(adj_phi_prop) if future else adj_phi_prop
				right_hand_side.append(disjunct)

			right_hand_side = self.disj(right_hand_side)
			sys_trans_formulas.append(self.implication(left_hand_side, right_hand_side))

		return sys_trans_formulas

	# =====================================================
	# Various formulas
	# =====================================================

	def gen_mutex_formulas(self, mutex_props, future):
		"""Eq. (1) 
		Create a set of formulas that enforce mutual exclusion between the given propositions.

		The argument 'future' dictates whether the propositions will be primed (True) or not (False).
		'future' should be set to True in fast-slow formulas.
		"""

		mutex_formulas = list()

		for prop in mutex_props:
			other_props = [p for p in mutex_props if p != prop]
			negated_props = list()
			for prop_prime in other_props:
				if future:
					left_hand_side = self.prime(prop)
					neg_prop = self.neg(self.prime(prop_prime)) # not(next(p'))
				else:
					left_hand_side = prop
					neg_prop = self.neg(prop_prime)
				negated_props.append(neg_prop)
			right_hand_side = self.conj(negated_props)
			
			mutex_formulas.append(self.iff(left_hand_side, right_hand_side)) # left -> right

		return mutex_formulas

	def gen_precondition_formula(self, action, preconditions):
		'''Conditions that have to hold for an action (prop) to be allowed.'''

		neg_preconditions = map(self.neg, preconditions)
		left_hand_side = self.disj(neg_preconditions)
		right_hand_side = self.neg(action)

		precondition_formula = self.implication(left_hand_side, right_hand_side)

		return precondition_formula

	# =====================================================
	# Various helper methods
	# =====================================================

	def _add_props_from_ts(self):
		"""Reads the items in the TS dictionary and adds them to the system propositions, if they are not already there."""

		props_to_add = self.ts.keys()
		for v in self.ts.values():
			for prop in v:
				props_to_add.append(prop)

		self.sys_props = list(set(self.sys_props + props_to_add))

	def gen_phi_prop(self, prop):
		"""Generate (non-atomic) proposition of the form \phi_r, i.e., mutex version of \pi_r (where prop = \pi_r)"""
		
		props_in_phi = [prop] # Initialize with pi_r

		other_props = self.get_other_trans_props(prop)
		for other_prop in other_props:
			props_in_phi.append(self.neg(other_prop)) # All pi_r' are negated

		phi_prop = self.conj(props_in_phi)

		return '(' + phi_prop + ')'

	def get_other_trans_props(self, prop):
		"""For some proposition \pi_r, get all propositions \pi_r' such that r' =/= r."""
		
		# return [p for p in self.ts.keys() if p != prop] # This will return completion props instead of activation in fs

		if prop in self.sys_props:
			return [p for p in self.sys_props if p != prop]
		elif prop in self.env_props:
			return [p for p in self.env_props if p != prop]
		else:
			raise ValueError("Unknown type for proposition: %s" % prop)

	# =====================================================
	# LTL Operators
	# =====================================================

	def conj(self, terms):
		return " & ".join(terms)

	def disj(self, terms):
		return " | ".join(terms)

	def neg(self, term):
		return "! " + term

	def prime(self, proposition):
		# return proposition + "'"
		return "next(" + proposition + ")"

	def implication(self, left_hand_side, right_hand_side):
		return left_hand_side + " -> " + right_hand_side 	#TODO: add parentheses to right side ?

	def iff(self, left_hand_side, right_hand_side):
		return left_hand_side + " <-> " + right_hand_side #TODO: add parentheses to right side ?

	def par(self, term):
		return "(" + term + ")"


class FastSlowFormula(GR1Formula):

	"""

	Arguments:
	  env_props (list of str)	Environment propositions (strings)
	  sys_props (list of str)	System propositions (strings)
	  ts 		(dict of str)	Transition system, TS (e.g. workspace topology)
	  							Implicitly contains some propositions in the keys.

	Attributes:
	  activation (list of str)	...
	  completion (list of str)	...
	  ts 		 (dict of str)	...

	Raises:
	  ValueError:				When a proposition is neither an activation or a completion proposition

	"""

	def __init__(self, env_props, sys_props, ts = {}):
		super(FastSlowFormula, self).__init__(env_props, sys_props, ts) # Initializes a bunch of attributes

		self.activation = []
		self.completion = []

		self._gen_a_c_propositions()

		# Save the original (non fast-slow) TS, then convert to a "fast-slow" transition system
		self.ts_original = ts
		self._convert_ts_to_fs()

	# =====================================================
	# Activation / Competion (fast-slow) formulas
	# Equation and section numbers are w.r.t. Vasumathi Raman and Hadas Kress-Gazit (ICRA 2013)
	# =====================================================

	def _convert_ts_to_fs(self):
		"""..."""
		new_ts = {}

		for k in self.ts.keys():		
			k_c = self._get_c_prop(k) 		# completion prop

			k_c_values = list()
			for v in self.ts[k]:			
				v_a = self._get_a_prop(v) 	# activation prop

				k_c_values.append(v_a)

			new_ts[k_c] = k_c_values

		self.ts = new_ts

	def gen_init_fs_from_props(self, props):
		'''
		Given a list of propositions that are initially True, this method:	
		  * converts them to actication/completion props (if necessary) 
		  * prepares the corresponding prop assignment (dictionary)
		  * calls the two methods for getting the sys and env initial condition formulas
		  * returns both of them	
		'''

		prop_assignment_sys = dict()
		prop_assignment_env = dict()

		for p in props:
			
			if p in self.sys_props:
				p_init = self._get_a_prop(p)
				prop_assignment_sys[p_init] = True
			elif p in self.activation:
				p_init = p
				prop_assignment_sys[p_init] = True		
			elif p in self.env_props:
				p_init = self._get_c_prop(p)
				prop_assignment_env[p_init] = True
			elif p in self.completion:
				p_init = p
				prop_assignment_env[p_init] = True
			else:
				raise ValueError("Unknown type for proposition (fast-slow): %s" % p)

		sys_init = self.gen_sys_init_from_prop_assignment(prop_assignment_sys, 'activation')
		env_init = self.gen_env_init_from_prop_assignment(prop_assignment_env, 'completion')

		return sys_init, env_init

	def gen_fs_sys_transitions(self):
		"""
		Section V-B-2
		
		Encodes a transition system in terms of 'fast-slow' safety requirement formulas.
		"""

		return self.gen_sys_trans_formulas(future = False) # In present tense for fast-slow transition system formulas

	def gen_action_competion_formula(self):
		"""Eq. (4)"""
		pass

	def gen_generic_fairness_formula(self):
		"""Section V-B (4)"""
		pass

	def gen_mutex_fairness_formula(self):
		"""
		Section V-B (4)

		Generates environment fairness conditions for the special case of 
		mutually exclusive propositions, such as regions of the workspace.
		Specifically, these propositions are encoded as a transition system.
		"""
		
		completion_terms = list()
		change_terms = list()

		for pi in self.ts_original.keys():

			pi_a = self._get_a_prop(pi)
			phi = self.gen_phi_prop(pi_a)

			pi_c = self._get_c_prop(pi)
			next_pi_c = self.prime(pi_c)
			not_next_phi = self.neg(self.prime(phi))

			completion_term = self.par(self.conj([phi, next_pi_c]))
			completion_terms.append(completion_term)
			
			change_term = self.par(self.conj([phi, not_next_phi]))
			change_terms.append(change_term)

		completion_formula = self.disj(completion_terms)
		change_formula = self.disj(change_terms)
		fairness_formula = self.disj([completion_formula, change_formula])

		return fairness_formula

	def gen_single_step_change_formula(self):
		"""Eq. (2)"""

		all_formulas = list()

		for pi in self.ts_original.keys():
			
			for pi_prime in self.ts_original[pi]:
				pi_c = self._get_c_prop(pi)
				pi_prime_a = self._get_a_prop(pi_prime)
				phi = self.gen_phi_prop(pi_prime_a)

				left_hand_side = self.conj([pi_c, phi])

				next_pi_c = self.prime(pi_c)
				pi_prime_c = self._get_c_prop(pi_prime)
				next_pi_prime_c = self.prime(pi_prime_c)

				if next_pi_c == next_pi_prime_c:
					right_hand_side = next_pi_prime_c
				else:
					right_hand_side = self.par(self.disj([next_pi_c, next_pi_prime_c]))

				implication = self.implication(left_hand_side, right_hand_side)

				all_formulas.append(implication)

		return all_formulas

	def _gen_a_c_propositions(self):
		"""
		For each system proposition (action, region ,etc.), create the corresponding activation and completion propositions.		
		"""

		original_props = list()
		for sys_prop in self.sys_props:		
			if sys_prop[-2:] != "_a": 	# activation props always end with '_a'
				
				# Add activation proposition and mark original system proposition for removal
				self.activation.append(self._get_a_prop(sys_prop))
				original_props.append(sys_prop)
				# Also add the corresponding completion proposition
				self.completion.append(self._get_c_prop(sys_prop))

		for sys_prop in original_props:
			self.sys_props.remove(sys_prop)

	def get_other_trans_props(self, prop):
		"""For some proposition \pi_r, get all propositions \pi_r' such that r' =/= r."""
		
		# return [p for p in self.ts.keys() if p != prop] # This will return completion props instead of activation in fs

		if prop in self.activation:
			return [p for p in self.activation if p != prop]
		elif prop in self.completion:
			return [p for p in self.completion if p != prop]
		else:
			raise ValueError("Unknown type for proposition (fast-slow): %s" % prop)

	def _get_a_prop(self, prop):
		return prop + "_a"

	def _get_c_prop(self, prop):
		return prop + "_c"

# =========================================================
# Entry point
# =========================================================

def main():
	
	my_gr1_formula = GR1Formula([], [])
	my_fs_formula = FastSlowFormula([], [])

if __name__ == "__main__":
	main()