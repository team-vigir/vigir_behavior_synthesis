# This messages specifies a state of an automaton.
# A state consists of the truth value of atomic propositions,
# the possible transitions out of the state, and the goal rank.

# The state's identifier / number
# In an automaton, the first state is 0, etc.
uint16 state_id

# Truth value (True/False, 0/1) of the propositions in this state.
bool[] proposition_valuation

# The states that this state can transition to
uint16[] transitions

# The state's goal rank (which liveness is being pursued)
uint8 rank