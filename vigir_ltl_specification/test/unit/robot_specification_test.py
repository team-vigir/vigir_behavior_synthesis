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
        self.preconditions = {'a': ['a1', 'a2'],
                              'b': ['b1']}

        self.spec = ActionSpecification(name = self.spec_name,
                                        preconditions = self.preconditions)

    def tearDown(self):
        """Gets called after every test case."""

        del self.spec

    def test_formulas_from_preconditions(self):
        
        formula = self.spec._gen_preconditions_formula('a',
                                                       self.preconditions['a'],
                                                       act_out = True)

        self.assertIsInstance(formula, PreconditionsFormula)

    def test_not_act_out_raises_exception(self):

        self.assertRaises(NotImplementedError,
                          self.spec._gen_preconditions_formula,
                          '', [], False)

    def test_handle_new_action(self):

        self.spec.handle_new_action('b')

        expected_formula = '! b1 -> ! b'

        self.assertEqual(expected_formula, self.spec.sys_trans[0]) # preconditions
        # self.assertNotEqual(list(), self.spec.env_trans) # activation
        # self.assertNotEqual(list(), self.spec.env_liveness) # fairness


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
