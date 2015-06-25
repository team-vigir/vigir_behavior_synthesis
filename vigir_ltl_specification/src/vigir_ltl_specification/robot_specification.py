#!/usr/bin/env python

import os
import yaml

from gr1_specification import GR1Specification
from activation_outcomes import *

"""
Module's docstring #TODO

* Generate formulas
* Load formulas in specification

"""

class ActionSpecification(GR1Specification):
    """
    docstring for ActionSpecification

    Arguments:
      ts            dict    Dictionary encoding a transition system (TS).
      preconditions dict    Dictionary encoding action preconditions.

    """
    def __init__(self, name = '', preconditions = {}):
        super(ActionSpecification, self).__init__(spec_name = name,
                                                  env_props = [],
                                                  sys_props = [])

        self.preconditions = preconditions

    def handle_new_action(self, action, act_out = True):
        """
        ...

        Arguments:
          action    string  ...
          act_out   bool    Whether to use activation_outcomes paradigm

        """
        
        action_formulas = list()

        if action in self.preconditions.keys() and self.preconditions[action]:
            preconditions_formula = self._gen_preconditions_formula(action,
                                                                    act_out) 
            action_formulas.append(preconditions_formula)

        #TODO: Generate all action formulas ...

        self.load_formulas(action_formulas)

    def _gen_preconditions_formula(self, action, act_out = True):
        """..."""

        action_preconditions = self.preconditions[action]

        #Recursively get preconditions for this action's preconditions
        for pc in action_preconditions:
            # Check whether this precondition has preconditions of its own
            if pc in self.preconditions.keys():
                #FIX: Actions that don't have preconditions should also be handled!
                self.handle_new_action(pc)

        if act_out:
            formula = PreconditionsFormula(action, action_preconditions)
        else:
            raise NotImplementedError('Preconditions for the vanilla GR(1) ' +
                                      'paradigm have not been implemented yet!')

        return formula


class RobotConfiguration(object):
    """
    docstring for RobotConfiguration

    Arguments:
      robot     string  The system / robot whose configuration will be loaded.

    """
    def __init__(self, robot):
        self._robot = robot
        
        self._full_config = self._load_config_from_file(robot)
        self.ts, self.preconditions = self._extract_configs()

    @staticmethod
    def _load_config_from_file(robot):
        """..."""
        
        # Get absolute path to this module
        module_path = os.path.dirname(__file__)
        config_file = ('%s_config.yaml' % robot)
        rel_config_path = 'config/' + config_file

        config_file_path = os.path.join(module_path, rel_config_path)

        try:
            with open(config_file_path, 'r') as stream:
                config = yaml.load(stream)
        except IOError as e:
            print('Failed to load {0}! {1}'.format(config_file, e))
            config = dict()
        
        return config

    def _extract_configs(self):
        """Extract the individual elements of a robot configuration file."""

        try:
            ts = self._full_config['transition_system']
            preconditions = self._full_config['action_preconditions']
        except KeyError as e:
            print('Failed to extract configuration element {}!'.format(e))
            ts, preconditions = {}, {}

        return (ts, preconditions)
