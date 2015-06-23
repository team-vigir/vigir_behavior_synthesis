#!/usr/bin/env python

from ltl import LTL
from gr1_formulas import GR1Formula

"""
The activation-outcomes paradigm generalizes the activation-completion paradigm
in Vasumathi Raman and Hadas Kress-Gazit (ICRA 2013).

The base class ActivationOutcomesFormula is a generalization of FastSlowFormula.

"""

class ActivationOutcomesFormula(GR1Formula):
    """
    
    Arguments:
      env_props (list of str)   Environment propositions (strings)
      sys_props (list of str)   System propositions (strings)
      outcomes  (list of str)   The possible outcomes of activating actions.
                                Example: ['completed', 'failed', 'preempted']
                                Defaults to the singleton ['completed']
                                Ideally, the first character of outcomes will
                                be unique, such as in the example above (c,f,p)
      ts        (dict of str)   Transition system, TS (e.g. workspace topology)
                                Implicitly contains some props in its keys.

    Attributes:
      activation (list of str)  Activation propositions (subset of system)
      outcomes   (list of str)  The possible outcomes of an action
      outcome_props (dict of str:list)  The propositions corresponding
                                        to each possible activation outcome

    Raises:
      TypeError
      ValueError
    
    """

    def __init__(self, sys_props, outcomes = ['completed'], ts = dict()):

        # Check whether the input arguments are of the correct type, etc.
        self._check_input_arguments(sys_props, outcomes, ts)
        
        self.outcomes = outcomes

        # Generate activation (list) and outcome (dict) propositions
        act_props = self._gen_activation_propositions(sys_props)
        self.outcome_props = self._gen_outcome_propositions(sys_props)
        # Get the outcome props (environment) as a list
        env_props = self._get_env_props_from_outcome_props()

        super(ActivationOutcomesFormula, self).__init__(env_props = env_props,
                                                        sys_props = act_props,
                                                        ts = ts)

    @staticmethod
    def _check_input_arguments(sys_props, outcomes, ts):
        """Check type of input arguments as well as adherence to conventions."""

        # Check types
        if any([type(pi) != str for pi in sys_props]):
            raise TypeError('Invalid type of system props (expected str): {}'
                            .format(map(type, sys_props)))

        if not outcomes:
            raise ValueError('No outcomes where provided! ' +
                             'At least "completed" (or equivalent) is required.')

        if any([type(out) != str for out in outcomes]):
            raise TypeError('Invalid type of outcomes (expected str): {}'
                            .format(map(type, outcomes)))

        # Check convention of outcome names
        out_first_chars = [out[0] for out in outcomes]
        if any([True for c in out_first_chars if out_first_chars.count(c) > 1]):
            raise ValueError('The outcomes do not adhere to the convention ' +
                             'that their first character is unique: {}'
                             .format(outcomes))

        #TODO: Check transition system (ts)

    @staticmethod
    def _gen_activation_propositions(sys_props):
        """
        For each system proposition (action, region ,etc.), create the
        corresponding activation and outcome propositions (e.g. completion).        
        """

        act_props = list()

        for pi in sys_props:
            if not _is_activation(pi):
                # Add activation proposition
                pi_a = _get_act_prop(pi)
                act_props.append(pi_a)

        return act_props

    def _gen_outcome_propositions(self, sys_props):
        """
        For each system proposition (action, region ,etc.), create the
        corresponding activation and outcome propositions (e.g. completion).        
        """

        outcome_props = dict()

        for pi in sys_props:
            outcome_props[pi] = list()
            # Add outcome propositions
            for outcome in self.outcomes:
                outcome_props[pi].append(_get_out_prop(pi, outcome))

        return outcome_props

    def _get_env_props_from_outcome_props(self):
        """..."""

        env_props = reduce(lambda acc, ele: acc + ele,
                           self.outcome_props.values(), [])

        return env_props


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

        for pi in self.outcome_props.keys():

            # Use the mutex formula method of the GR1Formula class
            pi_outs = self.outcome_props[pi]
            formula = self.gen_mutex_formulas(pi_outs, future = True)
            mutex_formulas.extend(formula)

        return mutex_formulas

class TransitionRelationFormula(ActivationOutcomesFormula):
    """
    Generate system requirement formulas that
    encode the transition system (e.g. workspace topology).

    The transition system TS, is provided in the form of a dictionary.
    """
    
    def __init__(self, ts):
        super(TransitionRelationFormula, self).__init__(sys_props = [],
                                                        ts = ts)

        self.formulas = self._gen_trans_relation_formulas()
        self.type = 'sys_trans'

    def _gen_trans_relation_formulas(self):
        """Safety requirements from Section V-B (2)"""

        sys_trans_formulas = list()
        for prop in self.ts.keys():
            left_hand_side = prop
            right_hand_side = list()
            
            for adj_prop in self.ts[prop]:
                adj_phi_prop = self._gen_phi_prop(adj_prop)
                disjunct = LTL.next(adj_phi_prop) if future else adj_phi_prop
                right_hand_side.append(disjunct)

            right_hand_side = LTL.disj(right_hand_side)
            sys_trans_formulas.append(LTL.implication(left_hand_side,
                                                      right_hand_side))

        return sys_trans_formulas

class ActionOutcomeConstraintsFormula(ActivationOutcomesFormula):
    """Safety formulas that constrain the outcomes of actions."""

    def __init__(self, actions, outcomes = ['completed']):
        super(ActionOutcomeConstraintsFormula, self).__init__(sys_props = actions,
                                                              outcomes = outcomes)
        
        self.formulas = self._gen_action_outcomes_formulas()
        self.type = 'env_trans'

    def _gen_action_outcomes_formulas(self):
        """Equivalent of Equations (3) and (4)"""

        #TODO: Eq. (3) treats completion outcome in a special way. Rethink.

        eq3_formulas = list()
        eq4_formulas = list()

        for pi in self.outcome_props.keys():

            pi_a = _get_act_prop(pi)
            pi_outcomes = self.outcome_props[pi]

            # Generate Eq. (3)
            lhs_disjunct = LTL.paren(LTL.disj(pi_outcomes))
            left_hand_side = LTL.conj([lhs_disjunct, pi_a])

            rhs_props = map(LTL.next, pi_outcomes)
            right_hand_side = LTL.disj(rhs_props)
            
            formula = LTL.implication(left_hand_side, right_hand_side)
            eq3_formulas.append(formula)

            # Generate Eq. (4)
            not_pi_a = LTL.neg(pi_a)

            for pi_out in pi_outcomes:
                
                not_pi_out = LTL.neg(pi_out)
                left_hand_side = LTL.conj([not_pi_out, not_pi_a])
                right_hand_side = LTL.next(not_pi_out)
                
                formula = LTL.implication(left_hand_side, right_hand_side)
                eq4_formulas.append(formula)

        return eq3_formulas + eq4_formulas

class PropositionDeactivationFormula(ActivationOutcomesFormula):
    """
    Turn action proposition OFF once an outcome is returned.
    """
    
    def __init__(self, sys_props, outcomes = ['completed']):
        super(PropositionDeactivationFormula, self).__init__(sys_props = sys_props,
                                                             outcomes = outcomes)

        self.formulas = self._gen_proposition_deactivation_formulas()
        self.type = 'sys_trans'

    def _gen_proposition_deactivation_formulas(self):
        """
        Generate a safety requirement that turns an activation proposition
        off once a corresponding action outcome has become True.
        """

        deactivation_formulas = list()

        for pi in self.outcome_props.keys():

            pi_outs = self.outcome_props[pi]
            next_pi_outs = map(LTL.next, pi_outs)
            out_disjunct = LTL.paren(LTL.disj(next_pi_outs))

            pi_a = _get_act_prop(pi)
            next_not_pi_a = LTL.next(LTL.neg(pi_a))

            left_hand_side = LTL.conj([pi_a, out_disjunct])

            formula = LTL.implication(left_hand_side, next_not_pi_a)
            deactivation_formulas.append(formula)

        return deactivation_formulas

class ActionFairnessConditionsFormula(ActivationOutcomesFormula):
    """
    Environment liveness formulas that ensure that every action proposition (not
    topology) eventually returns an outcome (or that the robot changes its mind)
    
    Arguments:
      mutex    (bool)   Whether the action props are mutually exclusive or not

    """

    def __init__(self, actions, outcomes = ['completed'], mutex = False):
        super(ActionFairnessConditionsFormula, self).__init__(sys_props = actions,
                                                              outcomes = outcomes)
        
        self.formulas = self._gen_action_fairness_formulas()
        self.type = 'env_liveness'

    def _gen_action_fairness_formulas(self):
        """Fairness conditions (for actions) from Section V-B (4)"""

        #TODO: Be more efficient if props are mutually exclusive

        fairness_formulas = list()

        for pi in self.outcome_props.keys():

            pi_a = _get_act_prop(pi)
            not_pi_a = LTL.neg(pi_a)
            pi_outcomes = self.outcome_props[pi]

            next_pi_outs = map(LTL.next, pi_outcomes)
            out_disjunct = LTL.paren(LTL.disj(next_pi_outs))
            next_not_pi_outs = map(LTL.next, map(LTL.neg, pi_outcomes))
            out_conjunct = LTL.paren(LTL.conj(next_not_pi_outs))
            
            outcomes_disjunct_1 = LTL.conj([pi_a, out_disjunct])
            outcomes_disjunct_2 = LTL.conj([not_pi_a, out_conjunct])
            outcomes_formula = LTL.disj([outcomes_disjunct_1, outcomes_disjunct_2])

            change_disjunt_1 = LTL.paren(LTL.conj([pi_a, LTL.next(not_pi_a)]))
            change_disjunt_2 = LTL.paren(LTL.conj([not_pi_a, LTL.next(pi_a)]))
            change_formula = LTL.disj([change_disjunt_1, change_disjunt_2])

            fairness_condition = LTL.disj([outcomes_formula, change_formula])

            fairness_formulas.append(fairness_condition)

        return fairness_formulas

class TopologyFairnessConditionsFormula(ActivationOutcomesFormula):
    """
    Environment liveness formulas that ensure that every transition on the
    TS eventually returns an outcome (or that the robot changes its mind).
    The possible outcomes are all adjacent states in the transition system.
    """

    #TODO: Rethink topology propositions in the Activation-Outcomes framework

    def __init__(self, ts):
        super(TopologyFairnessConditionsFormula, self).__init__(sys_props = [],
                                                                outcomes = [],
                                                                ts = ts)
        
        self.formulas = self._gen_ts_fairness_formulas(ts)
        self.type = 'env_liveness'

    def _gen_ts_fairness_formulas(self, ts):
        """Fairness conditions (for regions) from Section V-B (4)"""
        
        completion_terms = list()
        change_terms = list()

        for pi in ts.keys():

            pi_a = _get_act_prop(pi)
            phi = self._gen_phi_prop(pi_a)

            pi_c = _get_com_prop(pi)
            next_pi_c = LTL.next(pi_c)
            not_next_phi = LTL.neg(LTL.next(phi))

            completion_term = LTL.paren(LTL.conj([phi, next_pi_c]))
            completion_terms.append(completion_term)
            
            change_term = LTL.paren(LTL.conj([phi, not_next_phi]))
            change_terms.append(change_term)

        completion_formula = LTL.disj(completion_terms)
        change_formula = LTL.disj(change_terms)
        fairness_formula = LTL.disj([completion_formula, change_formula])

        return fairness_formula


# =========================================================
# Module-level helper functions
# =========================================================

def _get_act_prop(prop):
    return prop + "_a" # 'a' stands for activation

def _get_com_prop(prop):
    #FIX: Delete after removing all cases of "special treatment" of completion
    return prop + "_c" # 'c' stands for completion or completed

def _get_out_prop(prop, outcome):
    # If an activation proposition was passed, strip the '_a' suffix
    if _is_activation(prop):
        prop = prop[:-2]
    # Use first character of the outcome's name (string) as the subscript
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

    ts = {'r1' : ['r2'], 'r2': ['r1']}

    formulas.append(OutcomeMutexFormula(sys_props, outcomes))

    formulas.append(ActionOutcomeConstraintsFormula(sys_props, outcomes))

    formulas.append(PropositionDeactivationFormula(sys_props, outcomes))

    formulas.append(ActionFairnessConditionsFormula(sys_props, outcomes))

    formulas.append(TransitionRelationFormula({})) # Pass TS here

    # formulas.append(TopologyFairnessConditionsFormula(ts))

    for formula in formulas:
        print '---'
        print 'Formula Class:\t',   formula.__class__.__name__ # prints class name
        print 'GR(1) Type:\t',      formula.type
        print 'System props:\t',    formula.sys_props
        print 'Env props:\t',       formula.env_props
        print 'Outcomes:\t',        formula.outcome_props
        print 'Formula(s):\t',      formula.formulas

if __name__ == "__main__": #pragma: no cover
    main()
