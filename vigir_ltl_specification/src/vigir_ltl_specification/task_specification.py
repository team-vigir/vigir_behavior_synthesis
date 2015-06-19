#!/usr/bin/env python

import os
import pprint

import preconditions as precond
from gr1_specification import GR1Specification
from gr1_formulas import GR1Formula, FastSlowFormula

"""
Module's docstring #TODO
"""

VIGIR_ROOT_DIR = os.environ['VIGIR_ROOT_DIR']

class TaskSpecification(GR1Specification):
	"""..."""
	
	def __init__(self, spec_name = '', env_props = [], sys_props = []):
		super(TaskSpecification, self).__init__(spec_name, env_props, sys_props)

# =========================================================
# Entry point
# =========================================================

def main(): #pragma: no cover
	
	my_spec = TaskSpecification()

	print 'Environment props:\t', my_spec.env_props
	print 'System props:\t\t', my_spec.sys_props

if __name__ == "__main__": #pragma: no cover
	main()
	