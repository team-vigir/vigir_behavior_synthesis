#!/usr/bin/env python

import unittest

from vigir_ltl_specification.robot_specification import *

class SpecificationConstructionTests(unittest.TestCase):
    """Test the generation of Activation-Outcomes formulas"""

    def setUp(self):
        """Gets called before every test case."""

        self.spec_name = 'test'

        self.spec = ActionSpecification(name = self.spec_name)

        print("Setting up a new specification construction test.")

    def tearDown(self):
        """Gets called after every test case."""

        print("Cleaning up after latest test ...")

        del self.spec

    def test_input_to_object(self):

        self.assertEqual(self.spec_name, self.spec.spec_name)

class ConfigurationTests(unittest.TestCase):
    """Test the generation of Activation-Outcomes formulas"""

    def setUp(self):
        """Gets called before every test case."""

        self.robot = 'atlas'

        self.config = RobotConfiguration(robot = self.robot)

        print("Setting up a new robot configuration test.")

    def tearDown(self):
        """Gets called after every test case."""

        print("Cleaning up after latest test ...")

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
