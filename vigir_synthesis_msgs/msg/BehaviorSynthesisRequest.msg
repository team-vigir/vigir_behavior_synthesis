# A synthesis request consists of the system (robot)
# for which to synthesize a behavior for, the desired
# objective(s), and the system's initial conditions.

# The system for which to synthesize for
string system

# The system's objective(s)
string[] goals

# The system's initial conditions
# Provided by the user when offline.
# Can be inferred when the system is online.
string[] initial_conditions

# Optional, user-specified LTL formulas to
# be included in the LTL compilation process
LTLSpecification ltl_specification

# System constants
# Currently only Atlas is supported

string ATLAS = 'atlas'
string OTHER = 'other'