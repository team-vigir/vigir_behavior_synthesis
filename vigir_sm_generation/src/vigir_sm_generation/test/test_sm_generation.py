#!/usr/bin/env python

from vigir_sm_generation.sm_generation import generate_sm
from vigir_synthesis_msgs.srv import GenerateFlexBESMRequest
from vigir_synthesis_msgs.msg import (
    SynthesisErrorCodes,
    FSAutomaton,
    AutomatonState
)
from flexbe_msgs.msg import StateInstantiation

import unittest
import yaml
import os
import ast
vigir_repo = os.environ['VIGIR_ROOT_DIR']

def load_synthesized_automaton(fpath):
    """
    Give the file path to a request yaml file, this returns a tuple with
    parameters to make a synthesized parameter. The tuple has the following:
        - A list of output variables
        - A list of input variables
        - A list of AutomatonState definitions (dictionary form)

    These request parameters are based on a sample automata generated on
    5/14/15. In this automata...
    Goal: 'pickup'
    Initial Condition: 'stand_prep'
    Preconditions:
        pickup:
            - grasp                                                                     
            - manipulate                                                                
        grasp:                                                                          
            - manipulate
    """
    yaml_file = os.path.join(vigir_repo, fpath)
    with open(yaml_file) as yf:
        sa = yaml.load(yf)
    automaton = [AutomatonState(a['name'],
                                a['output_valuation'],
                                a['input_valuation'],
                                a['transitions'],
                                a['rank'])
                 for a in sa['automaton']]
    return (sa['output_variables'], sa['input_variables'], automaton)

class TestGenerateFlexBESM(unittest.TestCase):
    """
    Test the generation of a simple state machine. Throughout this program,
    SI = StateInstantiation.
    """
    def setUp(self):
        fpath = 'catkin_ws/src/vigir_behavior_synthesis/vigir_sm_generation/'+\
                'src/vigir_sm_generation/test/test_sm_gen_request_multiple_output.yaml'
        #fpath = 'catkin_ws/src/vigir_behavior_synthesis/vigir_sm_generation/'+\
        #        'src/vigir_sm_generation/test/test_sm_gen_request.yaml'
        (self.out_vars, self.in_vars, self.automata) =\
            load_synthesized_automaton(fpath)
        automaton = FSAutomaton(self.out_vars,
                                         self.in_vars,
                                         self.automata)
        self.request = GenerateFlexBESMRequest(automaton, "test")

        response = generate_sm(self.request)
        self.SIs = response.state_definition
        self.error_code = response.error_code

    ## Test Error Codes ###
    def test_success(self):
        """ Test that a generation succeeds.  """
        self.assertEqual(self.error_code.value, SynthesisErrorCodes.SUCCESS,
            "Generation did not succeed as expected. Error code: {0}"\
                .format(self.error_code))

    def test_no_config_file(self):
        self.request.system = "non-existing system"
        response = generate_sm(self.request)
        SIs = response.state_definition
        error_code = response.error_code

        self.assertEqual(error_code.value, SynthesisErrorCodes.NO_SYSTEM_CONFIG,
            "Unhandled case: No system configuration in system.yaml."+\
            "Error code: {0}".format(error_code.value))

    def test_config_file_not_found(self):
        self.request.system = "test_nonexistent"
        response = generate_sm(self.request)
        SIs = response.state_definition
        error_code = response.error_code

        self.assertEqual(error_code.value,
            SynthesisErrorCodes.SYSTEM_CONFIG_NOT_FOUND,
            "Unhandled case: System configuration not found after looking "+\
            "up in system.yaml. Error code: {0}".format(error_code.value))

    ### Test that SIs contents have valid types ###
    def test_si_types(self):
        """ Test that each SI outputted are the correct type. """
        for si in self.SIs:
            self.assertTrue(isinstance(si, StateInstantiation))

    def test_param_name_and_value_len(self):
        """ Test that each SI's param name and value are the same length. """
        for si in self.SIs:
            self.assertEqual(len(si.parameter_names), len(si.parameter_values))

    def test_states_param_type(self):
        """ Test that the states parameter (if set) has valid types. """
        for si in self.SIs:
            if si.state_class != "ConcurrentState":
                continue
            if 'states' not in si.parameter_names:
                continue
            self.assertEqual(1, si.parameter_names.count('states'),
                "Only one parameter can be called 'states'.")
            idx = si.parameter_names.index('states')

            val_str = si.parameter_values[idx]
            # Converts string -> dict.
            self.assertEqual(val_str[0], "{",
                "state parameter is not a dictionary string")
            self.assertEqual(val_str[-1], "}",
                "state parameter is not a dictionary string")

    def test_outcomes_param_type(self):
        """ Test that the outcome parameter (if set) has valid types. """
        for si in self.SIs:
            if si.state_class != "ConcurrentState":
                continue
            if 'outcomes' not in si.parameter_names:
                continue
            self.assertEqual(1, si.parameter_names.count('outcomes'),
                "Only one parameter can be called 'outcomes'.")
            idx = si.parameter_names.index('outcomes')

            val_str = si.parameter_values[idx]
            # Converts string -> list
            outcomes = ast.literal_eval(val_str)
            self.assertTrue(type(outcomes) is list,
                "'outcomes' value is not a dictionary.")

    def test_outcome_mapping_param_type(self):
        """
        Test that the outcome_mapping parameter (if set) has valid types.
        """
        for si in self.SIs:
            if si.state_class != "ConcurrentState":
                continue
            if 'outcome_mapping' not in si.parameter_names:
                continue
            self.assertEqual(1, si.parameter_names.count('outcome_mapping'),
                "Only one parameter can be called 'outcome_mapping'.")
            idx = si.parameter_names.index('outcome_mapping')

            val_str = si.parameter_values[idx]
            # Converts string -> list of dicts.
            outcome_mappings = ast.literal_eval(val_str)

            for outcome_mapping in outcome_mappings:
                self.assertTrue(type(outcome_mapping) is dict,
                    "'outcome_mapping' value is not a dictionary.")

                # outcome mapping should be of the format:
                # 'outcome': <string>
                # 'condition': <dict of string -> string>
                self.assertEqual(len(outcome_mapping), 2,
                    "Outcome mapping should only have two entries. Found "+\
                    "{0} entries.".format(len(outcome_mapping)))
                for k, v in outcome_mapping.items():
                    self.assertTrue(type(k) is str,
                        "outcome_mapping dictionary key is not a string.")

                # Checkout 'outcome'
                self.assertTrue('outcome' in outcome_mapping,
                    "'outcome' not in outcome_mapping")
                self.assertTrue(type(outcome_mapping['outcome']) is str,
                    "outcome_mapping['outcome'] should be a string.")

                # Checkout 'condition'
                self.assertTrue('condition' in outcome_mapping,
                    "'condition' not in outcome_mapping")
                condition = outcome_mapping['condition']
                self.assertTrue(type(condition) is dict,
                    "outcome_mapping['condition'] value is not a dictionary.")
                for k, v in condition.items():
                    self.assertTrue(type(k) is str,
                        "outcome_mapping['condition'] key is not a string.")
                    self.assertTrue(type(v) is str,
                        "outcome_mapping['condition'] value is not a string.")

    ### Test that SIs contents are logically consistent. ###
    def test_root_exists(self):
        """ Test that exactly one SI has the root path. """
        root_SIs = [si for si in self.SIs if si.state_path == "/"]
        self.assertEqual(len(root_SIs), 1,
            "There must be one root. Found {0} roots".format(len(root_SIs)))

    def test_transitions_and_output_len(self):
        """ Test that each SI's param name and value are the same length. """
        for si in self.SIs:
            self.assertTrue(len(si.transitions) <= len(si.outcomes))

    def test_outcome_param_logic(self):
        """
        Test that the outcome parameter (if set) are logically consistent.
        """
        for si in self.SIs:
            if si.state_class != "ConcurrentState":
                continue
            if 'outcome' not in si.parameter_names:
                continue
            idx = si.parameter_names.index('outcome')
            val_str = si.parameter_values[idx]
            # Converts string -> list
            outcomes = ast.literal_eval(val_str)

            # Check that these outcomes are a subset of the possible outcomes
            extra_outcomes = set(outcomes) - set(si.outcomes)
            self.assertEqual(len(extra_outcomes), 0,
                "The following substate outcomes are not outcomes of the "+\
                "entire state machine: {0}".format(extra_outcomes))

if __name__ == '__main__':
    unittest.main()
