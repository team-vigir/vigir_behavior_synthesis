#!/usr/bin/env python

from vigir_ltl_specification.activation_outcomes import *

import unittest


class FormulaGenerationTests(unittest.TestCase):
	"""Test the generation of Activation-Outcomes formulas"""

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
		self.assertEqual(set(self.outcomes), set(formula.outcomes))
		self.assertEqual(set(self.sys_props), set(formula.sys_props))
		self.assertEqual(set(), set(formula.env_props))
		self.assertEqual(set(), set(formula.formulas))

		# Test whether the activation propositions are generated correctly
		expected_act_props = ['dance_a', 'sleep_a']
		self.assertEqual(set(expected_act_props), set(formula.activation))

		# Test whether the outcome propositions are generated correctly
		expected_out_props = {'dance': ['dance_c', 'dance_f', 'dance_p'],
							  'sleep': ['sleep_c', 'sleep_f', 'sleep_p']}
		self.assertEqual(expected_out_props, formula.outcome_props)

	def test_formulas_raise_exceptions(self):
		"""Test whether the formula constructors raise Exceptions"""

		self.assertRaises(TypeError, ActivationOutcomesFormula, ['dance', 1.0])
		self.assertRaises(TypeError,ActivationOutcomesFormula, ['dance'],[2.0])

# =============================================================================
# The following code is not actually useful. It will be removed promptly.
# =============================================================================

# docs.python.org/2/library/unittest.html#skipping-tests-and-expected-failures

@unittest.skip("Demonstrating class skipping")
class MySkippedTestCase(unittest.TestCase):
	
	@unittest.skip("Demonstrating skipping of a test")
	def test_nothing(self):
		self.fail("Shouldn't happen!")

class ExpectedFailureTestCase(unittest.TestCase):
    
    @unittest.expectedFailure
    def test_fail(self):
        self.assertEqual(True, False, "Broken test")

# =============================================================================
# Entry point
# =============================================================================

if __name__ == '__main__':
	# Run all tests
	unittest.main()

### Organization (docs.python.org/2/library/unittest.html#organizing-tests)

# def suite():
# 	tests = ['test_default_size', 'test_resize']

# 	return unittest.TestSuite(map(WidgetTestCase, tests))

# suite = unittest.TestLoader().loadTestsFromTestCase(WidgetTestCase)

# suite1 = module1.TheTestSuite()
# suite2 = module2.TheTestSuite()
# alltests = unittest.TestSuite([suite1, suite2])
