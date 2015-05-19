# Behavior Synthesis error code

int32 value

# Error code constants:

# Overall result
int32 SUCCESS = 1
int32 FAILURE = 99999

# Synthesis management errors
int32 BEHAVIOR_SYNTHESIS_FAILED = -1
int32 PREEMPTED = -9
# ...

# Specification compilation errors
int32 LTL_SPEC_COMPILATION_FAILED = -10
# ...

# LTL Synthesis errors
int32 SYNTHESIS_FAILED = -20
int32 SPEC_UNSYNTHESIZABLE = -21
# ... 

# State machine generation errors
int32 SM_GENERATION_FAILED = -30
int32 NO_SYSTEMS_FILE = -31         # There's not system.yaml file
int32 NO_SYSTEM_CONFIG = -32        # This system is not in system.yaml
int32 SYSTEM_CONFIG_NOT_FOUND = -33 # This system is in system.yaml but not found
int32 NO_INITIAL_STATE = -34        # Unable to deduce the initial state
int32 MULTIPLE_INITIAL_STATE = -35  # Multiple initial states deduced
int32 CONFIG_FILE_INVALID = -36     # The config file is invalid
int32 CONFIG_AUTONOMY_INVALID = -37     # The config file is invalid
int32 CONFIG_VARIABLE_CONFIG_INVALID = -38     # The config file is invalid
int32 AUTOMATON_INVALID = -39       # Something is wrong about the automaton
int32 AUTOMATON_NEXT_STATE_INVALID = -40       # The next states doesn't map to anything
int32 AUTOMATON_INPUT_VALUATION_INVALID = -41  # Something is wrong with the input valuations
int32 AUTOMATON_OUTPUT_VALUATION_INVALID = -42  # Something is wrong with the output valuations
