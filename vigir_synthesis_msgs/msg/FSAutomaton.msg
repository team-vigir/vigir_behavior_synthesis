# This message specifies a finite-state automaton.
# Specifically, a Mealy machine (the output depends on input and state)
# This is the kind of automaton that results from reactive LTL synthesis

# Variables controlled by the system/robot
string[] output_variables

# Variables controlled by the environment
string[] input_variables

# The automaton itself, consisting of states
AutomatonState[] automaton