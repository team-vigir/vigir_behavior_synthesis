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
