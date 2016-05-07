# A synthesis request consists of the system (robot)
# for which to synthesize a behavior, the system's initial conditions,
# the desired objective(s) and the resulting state machine's outcomes.

# The system's initial conditions
# Provided by the user when offline.
# Can be inferred when the system is online (not implemented)
string initial_condition

# The system's objective(s)
# All have to be achieved for a state machine's run to be successsful.
string goal

# The outcomes of the resulting synthesized state machine
# Empty for vanilla synthesized automata (infinite execution)
# By convention, 'finished' and 'failed' are used for success and failure
string[] sm_outcomes

# The system / robot for which to synthesize
#-------------------------------------------

string ATLAS = atlas
string YOUBOT = youbot
string BAXTER = baxter
string OTHER = other

# The value has to be one of the constants.
# Currently, only ATLAS is supported.
string system

# All fields below are OPTIONAL
#------------------------------

# The compiled specification's name
# Useful for locating the generated files.
# Does not affect the server's response.
string name

# User-specified LTL formulas to be
# included in the LTL compilation process
LTLSpecification ltl_specification
