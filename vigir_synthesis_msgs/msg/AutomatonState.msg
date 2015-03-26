# This messages specifies a state of an automaton.
# A state consists of the truth value of atomic propositions,
# the possible transitions out of the state, and the goal rank.

# The state's identifier / number
# In an automaton, the first state is 0, etc.
string name

# Truth value (True/False, 0/1) of the output propositions in this state.
bool[] output_valuation

# Truth value (True/False, 0/1) of the input propositions in this state.
bool[] input_valuation

# The states that this state can transition to, referenced by state name
string[] transitions

# The state's goal rank (which liveness is being pursued)
uint8 rank