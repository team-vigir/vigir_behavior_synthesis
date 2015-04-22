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

	def __init__(self, env_props = [], sys_props = [], ts = {}, outcomes = ['completed']):
		super(ActivationOutcomesFormula, self).__init__(env_props, sys_props, ts)

		self.activation = list()
		self.outcomes 	= dict({outcome: list() for outcome in outcomes})

		# Populate the lists of activation and outcome propositions
		self._gen_activation_outcome_propositions()

		# Store the original (non fast-slow) TS,
		# then convert to a "fast-slow" transition system
		# self.ts_original = ts
		# self._convert_ts_to_fs()

	def _gen_activation_outcome_propositions(self):
		'''...'''

		pass


# =========================================================
# Entry point
# =========================================================

def main():
	
	formula = ActivationOutcomesFormula(env_props = [], sys_props = ['dance'],
										outcomes = ['completed', 'failed'])

	print 'Activation:\t', formula.activation
	print 'Outcomes:\t', formula.outcomes

if __name__ == "__main__":
	main()
