# This messages specifies a state of an automaton.
# A state consists of the truth value of atomic propositions,
# the possible transitions out of the state, and the goal rank.

#
string state_name

# Truth value (True/False, 0/1) of the propositions in this state.
bool[] proposition_valuation

# The states that this state can transition to
uint16[] transitions

# The state's goal rank (which liveness is being pursued)
uint8 rank