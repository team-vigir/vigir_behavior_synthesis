#!/usr/bin/env python

from vigir_ltl_specification.activation_outcomes import *

import unittest


class FormulaGenerationTests(unittest.TestCase):
	"""Test the generation of Activation-Outcomes formulas"""

	def setUp(self):
		"""Gets called before every test case."""

		self.sys_props = ['dance', 'sleep']
		self.outcomes  = ['completed', 'failed', 'preempted']

		self.ts = {'r1': ['r1', 'r2', 'r3'],
               	   'r2': ['r2'],
               	   'r3': ['r3', 'r1']}

	def tearDown(self):
		"""Gets called after every test case."""

		del self.sys_props
		del self.outcomes

	def test_base_class(self):

		formula = ActivationOutcomesFormula(self.sys_props, self.outcomes)

		# Test whether the obvious things are working as expected
		self.assertItemsEqual(self.outcomes, formula.outcomes)
		self.assertEqual(list(), formula.formulas)

		# Test whether the activation propositions are generated correctly
		expected_act_props = ['sleep_a', 'dance_a']
		self.assertItemsEqual(expected_act_props, formula.sys_props)

		# Test whether the outcome propositions are generated correctly
		expected_out_props = {'dance': ['dance_c', 'dance_f', 'dance_p'],
							  'sleep': ['sleep_c', 'sleep_f', 'sleep_p']}
		self.assertItemsEqual(expected_out_props, formula.outcome_props)

		# Test whether the environment propositions are generated correctly
		expected_env_props = ['dance_c', 'dance_f', 'dance_p',
							  'sleep_c', 'sleep_f', 'sleep_p']
		self.assertItemsEqual(expected_env_props, formula.env_props)

	def test_constructor_raises_exceptions(self):

		self.assertRaises(TypeError,  ActivationOutcomesFormula, ['dance', 1.0])
		self.assertRaises(ValueError, ActivationOutcomesFormula, ['dance'],[])
		self.assertRaises(TypeError,  ActivationOutcomesFormula, ['dance'],[2])
		self.assertRaises(ValueError, ActivationOutcomesFormula, ['dance'],
						  ['completed', 'capitalized', 'called', 'calculated'])

	def test_system_initial_conditions(self):
		
		self.fail('Incomplete test!')

	def test_mutex_formula(self):

		formula = OutcomeMutexFormula(
						['dance'],
						outcomes = ['completed', 'failed', 'preempted'])

		expected_formula_c = 'next(dance_c) -> (next(! dance_f) & next(! dance_p))'
		expected_formula_f = 'next(dance_f) -> (next(! dance_c) & next(! dance_p))'
		expected_formula_p = 'next(dance_p) -> (next(! dance_c) & next(! dance_f))'

		expected_formulas = [expected_formula_c, expected_formula_f, expected_formula_p]

		self.assertEqual('env_trans', formula.type)
		self.assertItemsEqual(expected_formulas, formula.formulas)

	def test_mutex_single_outcome(self):

		formula = OutcomeMutexFormula(['dance'], outcomes = ['completed'])

		self.assertItemsEqual(list(), formula.formulas)

	def test_transition_relation_formula(self):

		formula = TransitionRelationFormula(self.ts)

		self.assertEqual('sys_trans', formula.type)

		self.fail('Incomplete test!')

	def test_single_step_change_formula_without_outcomes(self):

		formula = SingleStepChangeFormula(self.ts) # 'completed' used by default

		self.assertEqual('env_trans', formula.type)

		self.fail('Incomplete test!')

	def test_single_step_change_formula_with_extra_outcomes(self):
		
		formula = SingleStepChangeFormula(self.ts, ['completed', 'failed'])

		self.assertEqual('env_trans', formula.type)

		self.fail('Incomplete test!')

	def test_preconditions_formula(self):
		
		formula = PreconditionsFormula(action = 'run',
									   preconditions = ['step', 'walk'])

		expected_formula = '(! step_c | ! walk_c) -> ! run_a'

		self.assertEqual('sys_trans', formula.type)
		self.assertItemsEqual([expected_formula], formula.formulas)


# =============================================================================
# Entry point
# =============================================================================

if __name__ == '__main__':
	# Run all tests
	unittest.main()
