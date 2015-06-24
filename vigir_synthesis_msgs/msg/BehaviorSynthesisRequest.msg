# A synthesis request consists of the system (robot)
# for which to synthesize a behavior, the desired
# objective(s), and the system's initial conditions.

# The system / robot for which to synthesize
# The value has to be one of the constants below
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

# Optional. Useful for locating the generated files.
# Does not affect the server's response.
string name

# System constants
# Currently only Atlas is supported

string ATLAS = atlas
string YOUBOT = youbot
string BAXTER = baxter
string OTHER = other