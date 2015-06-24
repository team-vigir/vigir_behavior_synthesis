#!/usr/bin/env python

import unittest

from vigir_ltl_specification.ts_specification import *

class SpecificationConstructionTests(unittest.TestCase):
    """Test the generation of Activation-Outcomes formulas"""

    def setUp(self):
        """Gets called before every test case."""

        self.spec_name = 'test'
        self.ts = {'r1': ['r1', 'r2', 'r3'],
                   'r2': ['r2'],
                   'r3': ['r3', 'r1']}

        self.spec = TransitionSystemSpecification(name = self.spec_name,
                                                  ts = self.ts)

        print("Setting up a new specification construction test.")

    def tearDown(self):
        """Gets called after every test case."""

        print("Cleaning up after latest test ...")

        del self.ts, self.spec

    def test_input_to_object(self):

        self.assertEqual(self.spec_name, self.spec.spec_name)
        self.assertItemsEqual(self.ts, self.spec.ts)

    def test_ts_of_interest(self):
 
        props = ['r1', 'r3']
        ts_of_interest = {'r1': ['r1', 'r3'],
                          'r3': ['r3', 'r1']}

        self.spec = TransitionSystemSpecification(ts = self.ts,
                                                  props_of_interest = props)
        
        self.assertItemsEqual(ts_of_interest, self.spec.ts)


# =============================================================================
# Entry point
# =============================================================================

if __name__ == '__main__':
    # Run all tests
    unittest.main()