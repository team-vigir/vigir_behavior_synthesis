#!/usr/bin/env python

from activation_outcomes import *

import unittest

class FormulaGenerationTests(unittest.TestCase):

	# =========================================================================
	# Test the generation of ActivationOutcomes formulas
	# =========================================================================

	def setUp(self):
		"""Gets called before every test case."""

		self.sys_props = ['dance', 'sleep']
		self.outcomes  = ['completed', 'failed', 'preempted']

		print("Setting up a new formula test.")

	def tearDown(self):
		"""Gets called after every test case."""

		print("Cleaning up after latest test ...")

		del self.sys_props
		del self.outcomes

	def test_base_class(self):
		"""..."""

		formula = ActivationOutcomesFormula(self.sys_props, self.outcomes)

		# Test whether the obvious things are working as expected
		self.assertSetEqual(set(self.outcomes), set(formula.outcomes))
		self.assertSetEqual(set(self.sys_props), set(formula.sys_props))
		self.assertSetEqual(set(), set(formula.env_props))
		self.assertSetEqual(set(), set(formula.formulas))

		# Test whether the activation propositions are generated correctly
		expected_act_props = ['dance_a', 'sleep_a']
		self.assertSetEqual(set(expected_act_props), set(formula.activation))

		# Test whether the outcome propositions are generated correctly
		expected_out_props = {'dance': ['dance_c', 'dance_f', 'dance_p'],
							  'sleep': ['sleep_c', 'sleep_f', 'sleep_p']}
		self.assertDictEqual(expected_out_props, formula.outcome_props)


# =============================================================================
# Entry point
# =============================================================================

if __name__ == '__main__':
	# Run all tests
	unittest.main()