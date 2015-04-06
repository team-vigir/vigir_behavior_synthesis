#!/usr/bin/env python

import os, yaml

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

def get_action_preconditions(prop, preconditions):
	'''Given a proposition, find its preconditions.'''

	action_preconditions = dict()

	if preconditions[prop]:
		action_preconditions[prop] = preconditions[prop]
	else:
		action_preconditions[prop] = []
		print('Action {a} does not have any preconditions.'.format(a = prop))

	return action_preconditions

def gen_formulas_from_preconditions(preconditions, fast_slow):
	'''Generates LTL formulas of the precondition format from a dictionary of preconditions.'''
	#TODO: Assume preconditions are completion props and action is an activation prop.

	formula = FastSlowFormula() if fast_slow else GR1Formula()

	precondition_formulas = list()

	for prop, preconditions in preconditions.items():
		
		formula = formula.gen_precondition_formula(prop, preconditions)
		precondition_formulas.append(formula)

	return precondition_formulas

def convert_preconditions_to_fastslow(preconditions):
	'''
	Converts the elements of a preconditions dictionary to activation and completion propositions (fast-slow).

	In addition to the revised dictionary, it returns the activation and completion propositions.
	'''

	preconditions_fs = dict()
	sys_props = list()
	env_props = list()

	for key, values in preconditions.items():

		v_c = [v + '_c' for v in values] 	# completion propositions
		env_props.extend(v_c)

		k_a = key + '_a' 					# activation proposition
		sys_props.append(k_a)
		
		k_c = key + '_c'					# proposition for the completion of this action		
		env_props.append(k_c)

		# The activation of k is preconditioned on the completion of vs:
		preconditions_fs[k_a] = v_c

	return preconditions_fs, env_props, sys_props

def load_preconditions_from_config_files(location, files):
	'''Loads preconditions as a single dictionary from one or more configuration YAML files'''

	preconditions = dict()

	for filename in files:
		
		full_file_path = os.path.join(location, filename)

		with open(full_file_path, 'r') as stream:
			contents = yaml.load(stream)
			# Add the preconditions from this file to the rest
			# This will overwrite any duplicates (there shouldn't be any)
			preconditions = dict(preconditions, **contents) if contents else preconditions

	return preconditions