#!/usr/bin/env python

from ltl import LTL
from gr1_formulas import GR1Formula

"""
The activation-outcomes paradigm generalizes the activation-completion paradigm
in Vasumathi Raman and Hadas Kress-Gazit (ICRA 2013).

The base class ActivationOutcomesFormula is a generalization of FastSlowFormula

"""

class ActivationOutcomesFormula(GR1Formula):
    """
    Arguments:
      env_props (list of str)   Environment propositions (strings)
      sys_props (list of str)   System propositions (strings)
      ts        (dict of str)   Transition system, TS (e.g. workspace topology)
                                Implicitly contains some props in the keys.
      outcomes  (list of str)   The possible outcomes of activating actions.
                                Example: ['completed', 'failed', 'preempted']
                                Defaults to the singleton ['completed']
                                Ideally, the first character of outcomes will
                                be unique, such as in the example above (c,f,p)

    Attributes:
      activation (list of str)  Activation propositions (subset of system)
      outcomes   (list of str)  The possible outcomes of an action
      outcome_props (dict of str:list)  The propositions corresponding
                                        to each possible activation outcome
    """

    def __init__(self, sys_props, outcomes = ['completed']):
        super(ActivationOutcomesFormula, self).__init__(env_props = [],
                                                        sys_props = sys_props)

        self.activation = list()
        self.outcomes   = outcomes

        # Check that the arguments (now attributes) are of the correct type, etc.
        self._check_input_arguments()
        #TODO: Check that first character of outcomes is unique
        
        self.outcome_props = dict({pi: list() for pi in sys_props})

        # Populate the list of activation and dict of outcome propositions
        self._gen_activation_outcome_propositions()

    def _gen_activation_outcome_propositions(self):
        """
        For each system proposition (action, region ,etc.), create the
        corresponding activation and outcome propositions (e.g. completion).        
        """

        for sys_prop in self.sys_props:
            if not _is_activation(sys_prop):
                # Add activation proposition
                self.activation.append(_get_act_prop(sys_prop))
                # Also add the corresponding outcome propositions
                for outcome in self.outcomes:
                    self.outcome_props[sys_prop].append(_get_out_prop(sys_prop,
                                                                      outcome))

    def _check_input_arguments(self):
        """..."""

        if any([type(p) != str for p in self.sys_props]):
            raise TypeError('Invalid type of system propositions: {}'.format(map(type, self.sys_props)))

        if any([type(o) != str for o in self.outcomes]):
            raise TypeError('Invalid type of outcomes: {}'.format(map(type, self.outcomes)))

class OutcomeMutexFormula(ActivationOutcomesFormula):
    """The outcomes of an action are mutually exclusive."""
    
    def __init__(self, sys_props, outcomes = ['completed']):
        super(OutcomeMutexFormula, self).__init__(sys_props = sys_props,
                                                  outcomes = outcomes)

        self.formulas = self._gen_outcome_mutex_formulas()
        self.type = 'env_trans'

    def _gen_outcome_mutex_formulas(self):
        """Generate the formulas establishing mutual exclusion."""
        
        mutex_formulas = list()

        for pi in self.sys_props:

            # Use the mutex formula method of the GR1Formula class
            pi_outs = self.outcome_props[pi]
            formula = self.gen_mutex_formulas(pi_outs, future = True)
            mutex_formulas.extend(formula)

        return mutex_formulas
        

class ActionOutcomeConstraintsFormula(ActivationOutcomesFormula):
    """Safety formulas that constrain the outcomes of actions."""

    def __init__(self, actions, outcomes):
        super(ActionOutcomeConstraintsFormula, self).__init__(sys_props = actions, outcomes = outcomes)
        
        self.formulas = self._gen_action_outcomes_formulas()
        self.type = 'env_trans'

    def _gen_action_outcomes_formulas(self):
        """Equivalent of Equations (3) and (4)"""

        #TODO: Eq. (3) treats completion outcome in a special way. Rethink.

        eq3_formulas = list()
        eq4_formulas = list()

        for pi in self.sys_props:

            pi_c = _get_com_prop(pi)
            pi_a = _get_act_prop(pi)
            pi_outcomes = [_get_out_prop(pi, out) for out in self.outcomes]

            # Generate Eq. (3)
            left_hand_side = LTL.conj([pi_c, pi_a])

            rhs_props = [LTL.next(pi_out) for pi_out in pi_outcomes]
            right_hand_side = LTL.disj(rhs_props)
            
            formula = LTL.implication(left_hand_side, right_hand_side)
            eq3_formulas.append(formula)

            # Generate Eq. (4)
            not_pi_c = LTL.neg(pi_c)
            not_pi_a = LTL.neg(pi_a)
            left_hand_side = LTL.conj([not_pi_c, not_pi_a])
            
            rhs_props = [LTL.next(LTL.neg(pi_out)) for pi_out in pi_outcomes]
            right_hand_side = LTL.conj(rhs_props)
            formula = LTL.implication(left_hand_side, right_hand_side)
            eq4_formulas.append(formula)

        return eq3_formulas + eq4_formulas


# =========================================================
# Module-level helper functions
# =========================================================

def _get_act_prop(prop):
    return prop + "_a" # 'a' stands for activation

def _get_com_prop(prop):
    #FIX: Completion shouldn't require special treatment
    return prop + "_c" # 'c' stands for completion

def _get_out_prop(prop, outcome):
    # Use first character of outcome (string) as the subscript
    return prop + "_" + outcome[0]

def _is_activation(prop):
    return prop[-2:] == "_a"

# =========================================================
# Entry point
# =========================================================

def main(): #pragma: no cover
    
    formulas  = list()
    sys_props = ['dance', 'sleep']
    outcomes  = ['completed', 'failed', 'preempted']

    formulas.append(ActivationOutcomesFormula(sys_props, outcomes)) # empty

    formulas.append(ActionOutcomeConstraintsFormula(sys_props, outcomes))

    formulas.append(OutcomeMutexFormula(sys_props, outcomes))

    print 'Activation:\t', formulas[-1].activation
    print 'Outcomes:\t', formulas[-1].outcome_props

    for formula in formulas:
        print '---'
        print 'Formula:\t', formula.formulas
        print 'Type:\t', formula.type

if __name__ == "__main__": #pragma: no cover
    main()
