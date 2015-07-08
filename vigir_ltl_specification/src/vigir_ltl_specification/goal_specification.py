#!/usr/bin/env python

from gr1_specification import GR1Specification
from activation_outcomes import *

"""
Module's docstring #TODO
"""


class GoalSpecification(GR1Specification):
    """
    A specificaton containing formulas that encode the conditions under which 
    the system/robot wins. Includes both safety and livenesss requirements.
    """
    
    def __init__(self, name = ''):
        super(GoalSpecification, self).__init__(spec_name = name,
                                                env_props = [],
                                                sys_props = [])

    def handle_single_liveness(self, goals, success = 'finished'):
        """
        Create a single system liveness requirement (e.g. []<> finished) 
        from one or more goals. The method also generates the necessary 
        formulas for triggerring this liveness.
        """

        #FIX: This cannot handle failure: []<> (finished | failed)

        success_formula = SuccessfulOutcomeFormula(conditions = goals,
                                                   success = success)
        liveness_formula = SystemLivenessFormula(goals = [success])

        goal_formulas = [success_formula, liveness_formula]

        # Finally, load the formulas (and props) into the GR1 Specification
        self.load_formulas(goal_formulas)

    def handle_liveness_conjunction(self):
        #TODO
        pass
