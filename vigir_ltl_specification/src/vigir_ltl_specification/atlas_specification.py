#!/usr/bin/env python

import pprint

from gr1_specification import GR1Specification
from robot_specification import RobotConfiguration, ActionSpecification
from ts_specification import TransitionSystemSpecification
from goal_specification import GoalSpecification
from ic_specification import InitialConditionsSpecification

"""
Module's docstring #TODO
"""


class CompleteSpecification(GR1Specification):
    """
    ...
    """
    
    def __init__(self, name, initial_conditions, goals):
        super(CompleteSpecification, self).__init__(spec_name = name,
                                                	env_props = [],
                                                	sys_props = [])

        # Load transition system and action preconditions from config file
        atlas_config = RobotConfiguration('atlas')
        control_mode_ts = atlas_config.ts
        atlas_preconditions = atlas_config.preconditions

        # Generate a LTL specification governing BDI control modes
        #FIX: infer control modes of interest from input arguments
        modes_of_interest = ['stand_prep', 'stand', 'manipulate']
        ts_spec = TransitionSystemSpecification(ts = control_mode_ts,
        							  			props_of_interest = modes_of_interest)

        # Generate LTL specification governing action and preconditions
        #FIX: This needs to be executed for each goal in goals
        action_spec = ActionSpecification(preconditions = atlas_preconditions)
        action_spec.handle_new_action(action = goals[0],
        							  act_out = True,
                          			  outcomes = ['completed'])

        # Generate LTL specification governing the achievement of goals
        goal_spec = GoalSpecification()
        goal_spec.handle_single_liveness(goals = goals, success = 'finished')

        # Merge these specifications. Initial conditions are still missing.
        self.merge_gr1_specifications([ts_spec, action_spec, goal_spec])

        # Now generate LTL specification encoding the initial conditions
        ic_spec = InitialConditionsSpecification()
        ic_spec.set_ics_from_spec(spec = self,
        						  true_props = initial_conditions)

        # Finally, also merge the initial conditions specification
        self.merge_gr1_specifications([ic_spec])


# =========================================================
# Entry point
# =========================================================

def main(): #pragma: no cover
	
	specification = CompleteSpecification('test', ['stand'], ['grasp'])
	
	print "[INPUT]"
	pprint.pprint(specification.env_props)
	print "[OUTPUT]"
	pprint.pprint(specification.sys_props)
	print "[SYS_INIT]"
	pprint.pprint(specification.sys_init)
	print "[ENV_INIT]"
	pprint.pprint(specification.env_init)
	print "[SYS_TRANS]"
	pprint.pprint(specification.sys_trans)
	print "[ENV_TRANS]"
	pprint.pprint(specification.env_trans)
	print "[SYS_LIVENESS]"
	pprint.pprint(specification.sys_liveness)
	print "[ENV_LIVENESS]"
	pprint.pprint(specification.env_liveness)

if __name__ == "__main__": #pragma: no cover
	main()
