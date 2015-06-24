#!/usr/bin/env python

from gr1_specification import GR1Specification
from activation_outcomes import *

"""
Module's docstring #TODO

* Generate formulas
* Load formulas in specification

"""

class TransitionSystemSpecification(GR1Specification):
    """
    docstring for TransitionSystemSpecification

    Arguments:
      ts    dict    Dictionary encoding a transition system (TS).

    """
    def __init__(self, name = '', ts = {}, props_of_interest = []):
        super(TransitionSystemSpecification, self).__init__(spec_name = name,
                                                            env_props = [],
                                                            sys_props = [])
        
        self.ts = self._get_ts_of_interest(ts, props_of_interest)

        #TODO: Also handle initial condition here [?] Maybe in other module ...

        #TODO: Generate the necessary ActivationOutcomes formulas
        #TODO: Load formulas into specification

    @staticmethod
    def _get_ts_of_interest(original_ts, props_of_interest):
        """..."""

        if not props_of_interest:
            ts = original_ts
        else:
            ts = dict()
            for pi in props_of_interest:
                # Cherry-pick the props of interest from the dictionary
                transitions = original_ts[pi]
                ts[pi] = [t for t in transitions if t in props_of_interest]
        return ts
