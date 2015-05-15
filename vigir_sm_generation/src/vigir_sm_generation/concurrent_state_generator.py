from sm_gen_util import (
    new_si,
    class_decl_to_string
)
class ConcurrentStateGenerator():
    """
    This class is used to generate a StateInstantiation instance for a
    ConcurrentState.

    Usage:
    > csg = ConcurrentState('foo')
    > csg.add_internal_state(...)
    > csg.add_internal_outcome(...)
    > csg.add_internal_outcome_maps(...)
    > si = csg.gen()

    If this generator realizes that there's only one internal state, it
    only generates that state instead of wrapping it around a concurrent state.
    """
    def __init__(self, name):
        self.name = name
        # Internal parameters of the ConcurrentState that we need to build.
        # See 'states', 'outcomes' and 'outcome_mapping' in the documentation
        # of ConcurrentState for more detail.
        self.internal_states = {}
        self.internal_outcomes = []
        self.internal_outcome_maps = []

    def clean_variable(self, name):
        """Given a input or output variable, remove the _* ending."""
        return name[:-2]

    def add_internal_state(self, label, class_decl):
        """
        Add an internal state to this concurrent state.

        Internal states are the classes that run in parallel.

        label: the name by which this internal state is referred to.
        class_decl: the class declaration for code generation. e.g. 'Foo()'
        """
        clean_label = self.clean_variable(label)
        if clean_label not in self.internal_states:
            self.internal_states[clean_label] = class_decl

    def add_internal_outcome(self, outcome):
        """
        Add an internal outcome of the concurrent machine. Note that this can
        be remapped with the 'outcomes' and 'transitions' parameter.

        outcome: a string
        """
        if outcome not in self.internal_outcomes:
            self.internal_outcomes.append(outcome)

    def clean_out_map(self, out_map):
        clean_out_map = {}
        clean_out_map['outcome'] = out_map['outcome']

        clean_conditions = dict((self.clean_variable(k), v)
                                for k, v in out_map['condition'].items())
        clean_out_map['condition'] = clean_conditions

        return clean_out_map

    def add_internal_outcome_maps(self, out_map):
        clean_out_map = self.clean_out_map(out_map)
        if clean_out_map not in self.internal_outcome_maps:
            self.internal_outcome_maps.append(clean_out_map)

    def gen_single(self):
        """
        Generate the state instantiation for a single state. This should only
        be called in the degenerate case when there's only one internal state,
        so it doesn't make sense to use a concurrent state.
        """
        label, decl = self.internal_states.items()[0]
        outcomes = []
        transitions = []
        for out_map in self.internal_outcome_maps:
            transitions.append(out_map['outcome'])
            outcomes.append(out_map['condition'][label])

        return new_si("/" + self.name,
                       decl['name'],
                       outcomes,
                       transitions,
                       None,
                       decl['param_names'],
                       decl['param_values'])

    def gen_states(self):
        """Turn return states parameters for the ConcurrentState."""
        states = {}
        for label, class_decl in self.internal_states.items():
            states[label] = class_decl_to_string(class_decl)
        return states

    def gen(self):
        """Generate the state instantiation for this concurrent state."""
        if len(self.internal_states) == 1:
            return self.gen_single()

        p_names = ["states", "outcomes", "outcome_mapping"]
        states = self.gen_states()
        p_vals = [str(states),
                  str(self.internal_outcomes),
                  str(self.internal_outcome_maps)]

        # Not really needed, but it's good to be explicit
        concurrent_si_outcomes = self.internal_outcomes
        concurrent_si_transitions = self.internal_outcomes
        
        return new_si("/" + self.name,
                      "ConcurrentState",
                      concurrent_si_outcomes,
                      concurrent_si_transitions,
                      None,
                      p_names,
                      p_vals)
