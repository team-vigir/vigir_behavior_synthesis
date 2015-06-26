#!/usr/bin/env python

import unittest

from vigir_ltl_specification.robot_specification import *

class SpecificationConstructionTests(unittest.TestCase):
    """Test the construction of the ActionSpecification class."""

    def setUp(self):
        """Gets called before every test case."""

        self.spec_name = 'test'
        self.preconditions = {'a': ['b']}

        self.spec = ActionSpecification(name = self.spec_name,
                                        preconditions = self.preconditions)

    def tearDown(self):
        """Gets called after every test case."""

        del self.spec

    def test_input_to_object(self):

        self.assertEqual(self.spec_name, self.spec.spec_name)
        self.assertEqual(self.preconditions, self.spec.preconditions)


class ActionHandlingTests(unittest.TestCase):
    """The calls to the method(s) handling new actions."""

    def setUp(self):
        """Gets called before every test case."""

        self.spec_name = 'test'
        self.preconditions = {'run': ['step', 'walk'],
                              'bar': ['foo'],
                              'fu' : ['bar'],
                              'foo': None}

        self.spec = ActionSpecification(name = self.spec_name,
                                        preconditions = self.preconditions)

    def tearDown(self):
        """Gets called after every test case."""

        del self.spec

    def test_formulas_from_preconditions(self):
        
        action = 'bar'
        formula = self.spec._gen_preconditions_formula(action, act_out = True)

        self.spec.handle_new_action(action)

        expected_formula = '! foo_c -> ! bar_a'

        self.assertIsInstance(formula, PreconditionsFormula)
        self.assertEqual(expected_formula, formula.formulas[0])

    def test_not_act_out_raises_exception(self):

        action = self.preconditions.keys()[0]
        self.assertRaises(NotImplementedError,
                          self.spec._gen_preconditions_formula, action, False)

    def test_action_not_in_preconditions_config(self):
        
        action = 'i_am_not_in_the_dict'
        self.spec.handle_new_action(action)
        #FIX: This assertion will fail once more formulas are added
        self.assertEqual(len(self.spec.sys_trans), 1) # 1 for deactivation

    def test_action_without_preconditions(self):
        
        action = 'foo'
        self.spec.handle_new_action(action)
        #FIX: This assertion will fail once more formulas are added
        self.assertEqual(len(self.spec.sys_trans), 1) # 1 for deactivation

    def test_recursive_preconditions(self):

        action = 'fu'
        self.spec.handle_new_action(action)

        expected_formula_0 = '! foo_c -> ! bar_a'
        expected_formula_1 = '! bar_c -> ! fu_a'

        self.assertIn(expected_formula_0, self.spec.sys_trans)
        self.assertIn(expected_formula_1, self.spec.sys_trans)
        #FIX: This assertion will fail once more formulas are added
        self.assertEqual(len(self.spec.sys_trans), 5) # 2 preconditions + 3 deactivation
        
    def test_handle_new_action_with_multiple_preconditions(self):
        
        action = 'run'
        self.spec.handle_new_action(action)

        expected_formula_0 = '(run_c & run_a) -> next(run_c)' # outcome
        expected_formula_1 = '(! run_c & ! run_a) -> next(! run_c)' # outcome
        expected_formula_2 = '(run_a & next(run_c)) -> next(! run_a)' # deactivation
        expected_formula_3 = '(! step_c | ! walk_c) -> ! run_a' # preconditions
        expected_formula_4a = '((run_a & next(run_c)) | (! run_a & next(! run_c)))'
        expected_formula_4b = '((run_a & next(! run_a)) | (! run_a & next(run_a)))'
        expected_formula_4 = '(' + expected_formula_4a + ' | ' + \
                              expected_formula_4b + ')' # fairness condition
         
                             

        self.assertIn(expected_formula_0, self.spec.env_trans)
        self.assertIn(expected_formula_1, self.spec.env_trans)
        self.assertIn(expected_formula_2, self.spec.sys_trans)
        self.assertIn(expected_formula_3, self.spec.sys_trans)
        self.assertIn(expected_formula_4, self.spec.env_liveness)

    def test_handle_multiple_outcomes(self):
        
        self.fail('Incomplete test!')

class ConfigurationTests(unittest.TestCase):
    """Test the construction of the RobotConfiguration class."""

    def setUp(self):
        """Gets called before every test case."""

        self.robot = 'atlas'

        self.config = RobotConfiguration(robot = self.robot)

    def tearDown(self):
        """Gets called after every test case."""

        del self.config

    def test_input_to_object(self):

        self.assertEqual(self.robot, self.config._robot)

    def test_load_config_file(self):
    	
    	self.assertIsInstance(self.config._full_config, dict)
    	self.assertIsInstance(self.config.ts, dict)
    	self.assertIsInstance(self.config.preconditions, dict)

    def test_incorrect_input(self):
    	
    	self.config = RobotConfiguration(robot = 'bad_robot_name')

    	self.assertItemsEqual(dict(), self.config._full_config)
    	self.assertItemsEqual(dict(), self.config.ts)
    	self.assertItemsEqual(dict(), self.config.preconditions)

# =============================================================================
# Entry point
# =============================================================================

if __name__ == '__main__':
    # Run all tests
    unittest.main()
