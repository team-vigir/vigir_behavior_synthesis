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
      ts    dict    Dictionary encoding a transition system (TS).

    """
    def __init__(self, name = ''):
        super(ActionSpecification, self).__init__(spec_name = name,
                                                  env_props = [],
                                                  sys_props = [])


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
