#!/usr/bin/env python

import unittest

from vigir_ltl_specification.goal_specification import *

class SpecificationConstructionTests(unittest.TestCase):
    """Test the generation of Activation-Outcomes formulas"""

    def setUp(self):
        """Gets called before every test case."""

        self.spec_name = 'test'

        self.spec = GoalSpecification(name = self.spec_name)

    def tearDown(self):
        """Gets called after every test case."""

        del self.spec

    def test_input_to_object(self):

        self.assertEqual(self.spec_name, self.spec.spec_name)

    def test_handle_single_goal(self):
        
        goal = 'dance'
        self.spec.handle_single_liveness(goals = [goal])

        expected_formula_0 = 'dance_c -> next(dance_m)'
        expected_formula_1 = 'dance_m -> next(dance_m)'
        expected_formula_2 = '(! dance_m & ! dance_c) -> next(! dance_m)'
        expected_formula_3 = 'finished <-> dance_m'

        self.assertItemsEqual(actual_seq = self.spec.sys_trans,
                              expected_seq = [expected_formula_0, expected_formula_1,
                                              expected_formula_2, expected_formula_3])
        self.assertItemsEqual(actual_seq = self.spec.sys_liveness,
                              expected_seq = ['finished'])
