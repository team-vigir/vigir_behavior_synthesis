#!/usr/bin/env python
"""Specification written in the GR(1) fragment of Linear Temporal Logic

This module contains one main class, GR1Specification.
The class has methods for populating the 6 parts of a GR(1) formula:
  * Environment initial conditions
  * Environment safety assumptions
  * Environment fairness assumptions
  * System initial conditions
  * System safety requirements
  * System liveness requirements (aka, goals)

There are also methods for writing the specification in 
a .structuredslugs file for use with the SLUGS synthesis tool.

"""

import os

class GR1Specification(object):
	"""
	The class encodes the GR(1) fragment of LTL formulas, written in the structured slugs format.

	* All methods that generate LTL formulas return a list of formulas (each one a string), not one big string.
	* There are separate methods for adding those lists to the various LTL subformulas (requirements, liveness, etc.)
	* There are separate methods for writing those lists of subformulas to a .structuredslugs file.

	Arguments:
	  spec_name	(str)			The name of the specification. The spec file will be named like that.
	  env_props	(list of str)	Environment propositions
	  sys_props	(list of str)	System propositions
	
	"""

	def __init__(self, spec_name, env_props, sys_props):
		self.spec_name = spec_name

		self.env_props = env_props	
		self.sys_props = sys_props

		# Initialize the six GR(1) subformulas
		self.sys_init = list()
		self.env_init = list()
		self.sys_trans = list()
		self.env_trans = list()
		self.sys_liveness = list()
		self.env_liveness = list()

	# =====================================================
	# Load a GR(1) formula
	# =====================================================

	def load_gr1_formula(self, gr1_formula):
		"""Load an object of type GR1Formula into the GR(1) specification."""
		
		#TODO: Ensure we don't duplicate propositions, e.g., using sets
		pass

	# =====================================================
	# "Setter"-type methods for the 6 types of subformulas
	# =====================================================

	def add_to_sys_init(self, formulas):	
			self.add_to_list('sys_init', formulas)

	def add_to_env_init(self, formulas):
		self.add_to_list('env_init', formulas)

	def add_to_sys_trans(self, formulas):	
		self.add_to_list('sys_trans', formulas)

	def add_to_env_trans(self, formulas):
		self.add_to_list('env_trans', formulas)

	def add_to_sys_liveness(self, formulas):
		self.add_to_list('sys_liveness', formulas)

	def add_to_env_liveness(self, formulas):
		self.add_to_list('env_liveness', formulas)

	def add_to_list(self, desired_list, thing_to_add):
		"""Generic method for appending to or extending a list attribute (e.g. self.sys_liveness)"""

		if type(thing_to_add) == str:
			getattr(self, desired_list).append(thing_to_add)
		elif type(thing_to_add) == list:
			getattr(self, desired_list).extend(thing_to_add)
		elif thing_to_add is None:
			print("Warning: Nothing was added to %s!" % desired_list)
		else:
			raise ValueError("Invalid input: %s | Add either a string or a list of strings." % str(thing_to_add))

	# =====================================================
	# Composition of the Structured SLUGS file
	# =====================================================

	def write_structured_slugs_file(self, folder_path = ""):
		"""Create, or open, a structuredslugs file and write the 8 sections."""	
		
		filename = self.spec_name + ".structuredslugs"

		# The directory where specs and automata are saved:
		temp_bs_files = os.path.abspath(os.path.join(os.pardir, os.pardir, os.pardir, 'temp_bs_files'))

		if not folder_path:
			folder_path = os.path.join(temp_bs_files, self.spec_name)
		else:
			folder_path = os.path.join(folder_path, self.spec_name)

		if not os.path.exists(folder_path):
			os.makedirs(folder_path)

		full_file_path = os.path.join(folder_path, filename)
		
		with open(full_file_path, 'w') as spec_file:
			# System and environment propositions
			self._write_input(spec_file)
			self._write_output(spec_file)
			# Initial Conditions
			self._write_sys_init(spec_file)
			self._write_env_init(spec_file)
			# Safety Requirements & Assumptions
			self._write_sys_trans(spec_file)
			self._write_env_trans(spec_file)
			# Liveness Requirements & Assumptions
			self._write_sys_liveness(spec_file)
			self._write_env_liveness(spec_file)

		print("\nCreated specification file %s in %s \n" % (filename, folder_path))

		return full_file_path, folder_path

	def _write_input(self, spec_file):
		spec_file.write("[INPUT]\n")
		for prop in self.env_props:
			spec_file.write(prop + "\n")
		spec_file.write("\n")

	def _write_output(self, spec_file):
		spec_file.write("[OUTPUT]\n")
		for prop in self.sys_props:
		    spec_file.write(prop + "\n")
		spec_file.write("\n")

	def _write_sys_init(self, spec_file):
		spec_file.write("[SYS_INIT]\n")
		for formula in self.sys_init:
			spec_file.write(formula + "\n")
		spec_file.write("\n")

	def _write_env_init(self, spec_file):
		spec_file.write("[ENV_INIT]\n")
		for formula in self.env_init:
			spec_file.write(formula + "\n")
		spec_file.write("\n")

	def _write_sys_trans(self, spec_file):
		spec_file.write("[SYS_TRANS]\n")
		for formula in self.sys_trans:
			spec_file.write(formula + "\n")
		spec_file.write("\n")

	def _write_env_trans(self, spec_file):
		spec_file.write("[ENV_TRANS]\n")
		for formula in self.env_trans:
			spec_file.write(formula + "\n")
		spec_file.write("\n")

	def _write_sys_liveness(self, spec_file):
		spec_file.write("[SYS_LIVENESS]\n")
		for formula in self.sys_liveness:
			spec_file.write(formula + "\n")
		spec_file.write("\n")

	def _write_env_liveness(self, spec_file):
		spec_file.write("[ENV_LIVENESS]\n")
		for formula in self.env_liveness:
			spec_file.write(formula + "\n")
		spec_file.write("\n")


# =====================================================
# Custom Exceptions
# =====================================================

class SpecificationTypeException(Exception):
    pass

# =========================================================
# Entry point
# =========================================================

def main():
	
	my_spec = GR1Specification('test_refactoring', ['a'], ['b'])
	my_spec.write_structured_slugs_file()

if __name__ == "__main__":
	main()