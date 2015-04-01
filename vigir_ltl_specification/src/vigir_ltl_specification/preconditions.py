#!/usr/bin/env python

import yaml

from gr1_formulas import GR1Formula, FastSlowFormula

"""
Module's docstring #TODO
"""

def recurse_action_preconditions(prop):
	'''
	Recursively get the preconditions of the given proposition,
	as well as the preconditions of those preconditions, etc.
	'''

	pass

def get_action_preconditions(prop):
	'''Given a proposition, find its preconditions.'''

	pass

def gen_formulas_from_preconditions(preconditions, fast_slow = True):
	'''Generates LTL formulas of the precondition format from a dictionary of preconditions.'''
	#TODO: Assume preconditions are completion props and action is an activation prop.

	formula = FastSlowFormula() if fast_slow else GR1Formula()

	precondition_formulas = list()

	for prop, preconditions in preconditions.items():
		
		formula = formula.gen_precondition_formula(prop, preconditions)
		precondition_formulas.append(formula)

	return precondition_formulas

def load_preconditions_from_config_files(files):
	'''Loads preconditions as a single dictionary from one or more configuration YAML files'''

	preconditions = dict()

	for filename in files:
		with open(filename, 'r') as stream:
			contents = yaml.load(stream)
			# Add the preconditions from this file to the rest
			# This will overwrite any duplicates (there shouldn't be any)
			preconditions = dict(preconditions, **contents) if contents else preconditions

	return preconditions