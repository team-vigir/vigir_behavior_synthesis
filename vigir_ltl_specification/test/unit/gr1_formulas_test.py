#!/usr/bin/env python

import os, sys
import subprocess

from vigir_ltl_specification.gr1_specification import GR1Specification
from vigir_ltl_specification.gr1_formulas import GR1Formula, FastSlowFormula

import unittest

class FormulaGenerationTests(unittest.TestCase):

	# ==========================================
	# Tests for generic GR(1) formulas
	# ==========================================

	def setUp(self):
		"""Gets called before every test case."""

		self.env_props = ['x1', 'x2']
		self.sys_props = ['y1', 'y2', 'y3']

		print("Setting up a new formula test.")

	def tearDown(self):
		"""Gets called after every test case."""

		print("Cleaning up after latest test ...")

		del self.env_props
		del self.sys_props

	def test_props_from_ts(self):
		
		ts = {
			'a1': ['a1', 'a2'],
			'a2': ['a2', 'a3'],
			'a3': ['a3', 'a2']
		}

		expected_props = set(self.sys_props + ts.keys())

		formula = GR1Formula(self.env_props, self.sys_props, ts)

		self.assertSetEqual(set(formula.sys_props), expected_props)

	def test_mutex(self):
		"""Test whether mutual exclusion formulas are generated correctly."""

		formula = GR1Formula(self.env_props, self.sys_props)

		mutex = formula.gen_mutex_formulas(self.sys_props, False)

		expected_mutex = ["y1 <-> ! y2 & ! y3", "y2 <-> ! y1 & ! y3", "y3 <-> ! y1 & ! y2"]

		self.assertSetEqual(set(mutex), set(expected_mutex))

	def test_sys_trans(self):
		"""Test the adjacency relation that encodes the transition system."""

		ts = {
			'y1': ['y1', 'y2'],
			'y2': ['y2', 'y3'],
			'y3': ['y3', 'y2']
		}

		formula = GR1Formula(self.env_props, self.sys_props ,ts)

		adj_relation = formula.gen_sys_trans_formulas()
		# print adj_relation

		expected_adj_relation = [
			"y1 -> (y1 & ! y2 & ! y3)' | (y2 & ! y1 & ! y3)'",
			"y2 -> (y2 & ! y1 & ! y3)' | (y3 & ! y1 & ! y2)'",
			"y3 -> (y3 & ! y1 & ! y2)' | (y2 & ! y1 & ! y3)'"
		]

		# self.assertSetEqual(set(adj_relation), set(expected_adj_relation))

	# ==========================================
	# Fast-slow specific tests
	# ==========================================

	def test_activation_completion_props(self):
		"""Test whether activation and completion propositions were properly generated."""

		formula = FastSlowFormula(self.env_props, self.sys_props)

		expected_c_props = set(['y1_c', 'y2_c', 'y3_c'])
		expected_a_props = set(['y1_a', 'y2_a', 'y3_a'])

		self.assertSetEqual(set(formula.completion), expected_c_props)
		self.assertSetEqual(set(formula.activation), expected_a_props)
		self.assertSetEqual(set(formula.sys_props), set([]))

	def test_fs_mutex(self):
		"""Test whether mutual exclusion formulas are generated correctly in the fast-slow case."""

		formula = FastSlowFormula(self.env_props, self.sys_props)

		mutex = formula.gen_mutex_formulas(formula.completion, True)

		expected_mutex = ["y1_c' <-> ! y3_c' & ! y2_c'", "y2_c' <-> ! y1_c' & ! y3_c'", "y3_c' <-> ! y1_c' & ! y2_c'"]

		self.assertSetEqual(set(mutex), set(expected_mutex))

	def test_fs_ts(self):
		
		ts = {
			'm1': ['m1', 'm2'],
			'm2': ['m1', 'm2', 'm3'],
			'm3': ['m3', 'm2']
		}

		formula = FastSlowFormula([], [], ts)

		for k in formula.ts:
			for v in formula.ts[k]:
				assert (k in formula.completion) & (v in formula.activation)

	def test_fs_sys_trans(self):

		ts = {
			'm1': ['m1', 'm2'],
			'm2': ['m1', 'm2', 'm3'],
			'm3': ['m3', 'm2']
		}

		formula = FastSlowFormula([], [] ,ts)

		adj_relation = formula.gen_sys_trans_formulas()

		expected_adj_relation = [
			"m1_c -> (m1_a & ! m3_a & ! m2_a)' | (m2_a & ! m1_a & ! m3_a)'",
			"m2_c -> (m1_a & ! m3_a & ! m2_a)' | (m2_a & ! m1_a & ! m3_a)' | (m3_a & ! m1_a & ! m2_a)'",
			"m3_c -> (m3_a & ! m1_a & ! m2_a)' | (m2_a & ! m1_a & ! m3_a)'"
		]

		self.assertSetEqual(set(adj_relation), set(expected_adj_relation))

class SpecificationTests(unittest.TestCase):

	NAME = "test_refactoring" # without file extension

	def setUp(self):
		"""Gets called before every test case."""
		
		self.name = SpecificationTests.NAME

		print("Setting up a new specification: %s" % self.name)

		self.env_props = ['x']
		self.sys_props = ['y1', 'y2']

		# self.sys_init = dict()
		# self.sys_init = {'y1': True, 'y2': False}
		# self.env_init = {'x': False}

		self.spec = GR1Specification(self.name, self.env_props, self.sys_props)

	def tearDown(self):
		"""Gets called after every test case."""

		print("Cleaning up after latest test ...")

		test_dir = os.path.join(os.getcwd(), self.name)
		files = [f for f in os.listdir(test_dir) if os.path.isfile(os.path.join(test_dir, f))]

		extensions = ['.structuredslugs', '.slugsin', '.aut', '.dot']
		
		for f in files:
			if any(f.lower().endswith(ext) for ext in extensions):
				os.remove(os.path.join(test_dir, f))
				print("Deleting file: %s" % str(f))

		del self.name
		del self.env_props
		del self.sys_props
		del self.spec

	def test_for_spec_file(self):
		"""Test whether specification object was created."""
		
		self.failUnless(self.spec)

	def test_for_duplicate_env_props(self):
		duplicate_env_props = len(self.spec.env_props) != len(set(self.spec.env_props))
		self.failIf(duplicate_env_props is True, "Some environment propositions appear more than once.")

	def test_for_duplicate_sys_props(self):
		duplicate_sys_props = len(self.spec.sys_props) != len(set(self.spec.sys_props))
		self.failIf(duplicate_sys_props is True, "Some system propositions appear more than once.")

	def test_spec_file_generation(self):
		
		self.spec.write_structured_slugs_file()

		full_file_path = os.path.join(self.name, self.name + ".structuredslugs")

		self.failUnless(os.path.isfile(full_file_path))


if __name__ == '__main__':
	# Run all tests
	unittest.main()