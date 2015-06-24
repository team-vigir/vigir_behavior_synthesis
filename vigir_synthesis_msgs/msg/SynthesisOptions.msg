# Behavior Synthesis options (default to False)

# The following three options are mutually exclusive

# (1) LTL Compilation only
bool ltl_compilation_only

# (2) LTL Synthesis only
# The synthesis request must include a full LTL
# formula, since compilation won't take place
bool ltl_synthesis_only

# (3) LTL Compilation and LTL Synthesis only
# This will return an automaton,
# not a state machine instantiation 
bool ltl_steps_only