#!/usr/bin/env python

from ltl import LTL
from gr1_formulas import GR1Formula

"""
The activation-outcomes paradigm generalizes the activation-completion paradigm
in Vasumathi Raman and Hadas Kress-Gazit (ICRA 2013).

The class ActivationOutcomesFormula is a generalization of FastSlowFormula.

"""

class ActivationOutcomesFormula(GR1Formula):

	"""

	Arguments:
	  env_props (list of str)	Environment propositions (strings)
	  sys_props (list of str)	System propositions (strings)
	  ts 		(dict of str)	Transition system, TS (e.g. workspace topology)
	  							Implicitly contains some propositions in the keys
	  outcomes  (list of str)	The possible outcomes of an action proposition
	  							Example: ['completed', 'failed', 'preempted']
	  							Defaults to the singleton ['completed']
	  							Ideally, the first character of outcomes will
	  							be unique, such as in the example above (c,f,p)

	Attributes:
	  activation (list of str)		Activation propositions (subset of system)
	  outcomes 	 (dict of str:list)	The propositions corresponding to each
	  								possible outcome

	Raises:
	  ValueError:	When a proposition is neither activation nor an outcome

	"""

	def __init__(self, env_props = [], sys_props = [], outcomes = ['completed']):
		super(ActivationOutcomesFormula, self).__init__(env_props, sys_props, ts = {})

		self.activation = list()
		self.outcomes 	= outcomes
		self.outcome_props = dict({outcome: list() for outcome in outcomes})

		# Populate the lists of activation and outcome propositions
		# from the provided lists of environment and system propositions.
		self._gen_activation_outcome_propositions() #FIX: Do I want this to happen here?

		self.formulas = list()

	def _gen_activation_outcome_propositions(self):
		"""
		For each system proposition (action, region ,etc.), create the
		corresponding activation and outcome propositions (e.g. completion).		
		"""

		for sys_prop in self.sys_props:
			if not _is_activation(sys_prop):
				# Add activation proposition
				self.activation.append(_get_act_prop(sys_prop))
				# Also add the corresponding outcome propositions
				for outcome in self.outcomes:
					self.outcome_props[outcome].append(_get_out_prop(sys_prop, outcome))

class ActionOutcomeConstraintsFormula(ActivationOutcomesFormula):
	"""docstring for ClassName"""
	def __init__(self, actions, outcomes):
		super(ActionOutcomeConstraintsFormula, self).__init__(env_props = [], sys_props = actions, outcomes = outcomes)
		
		self.formulas = self.gen_action_outcomes_formulas()

	def gen_action_outcomes_formulas(self):

		eq3_formulas = list()
		eq4_formulas = list()

		for pi in self.sys_props:

			pi_c = _get_com_prop(pi)
			pi_a = _get_act_prop(pi)
			pi_outcomes = [_get_out_prop(pi, out) for out in self.outcomes]

			# Generate Eq. (3)
			left_hand_side = LTL.conj([pi_c, pi_a])

			rhs_props = [LTL.next(pi_out) for pi_out in pi_outcomes]
			right_hand_side = LTL.disj(rhs_props)
			
			formula = LTL.implication(left_hand_side, right_hand_side)
			eq3_formulas.append(formula)

			# Generate Eq. (4)
			not_pi_c = LTL.neg(pi_c)
			not_pi_a = LTL.neg(pi_a)
			left_hand_side = LTL.conj([not_pi_c, not_pi_a])
			
			rhs_props = [LTL.next(LTL.neg(pi_out)) for pi_out in pi_outcomes]
			right_hand_side = LTL.conj(rhs_props)
			formula = LTL.implication(left_hand_side, right_hand_side)
			eq4_formulas.append(formula)

		return eq3_formulas + eq4_formulas

	def gen_deactivation_formula(self):
		"""Eq. (4)"""
		
		actions = self.sys_props

		formulas = list()

		for pi in actions:

			pi_c = _get_com_prop(pi)
			pi_a = _get_act_prop(pi)
			pi_outcomes = [_get_out_prop(pi, out) for out in self.outcomes]

			# Generate Eq. (3)
			left_hand_side = LTL.conj([pi_c, pi_a])

			rhs_props = [LTL.next(pi_out) for pi_out in pi_outcomes]
			right_hand_side = LTL.disj(rhs_props)
			
			formula = LTL.implication(left_hand_side, right_hand_side)
			formulas.append(formula)

		return formulas

# =========================================================
# Module-level helper functions
# =========================================================

def _get_act_prop(prop):
	return prop + "_a" # 'a' stands for activation

def _get_com_prop(prop):
	return prop + "_c" # 'c' stands for completion

def _get_out_prop(prop, outcome):
	# Use first character of outcome (string) as the subscript
	return prop + "_" + outcome[0]

def _is_activation(prop):
	return prop[-2:] == "_a"

# =========================================================
# Entry point
# =========================================================

def main():
	
	formula = ActivationOutcomesFormula(env_props = [], sys_props = ['dance'],
										outcomes = ['completed', 'failed'])

	formula = ActionOutcomeConstraintsFormula(actions = ['dance'],
									outcomes = ['completed', 'failed'])

	print 'Activation:\t', formula.activation
	print 'Outcomes:\t', formula.outcomes
	print 'Formula:\t', formula.formulas

if __name__ == "__main__":
	main()
