from sm_gen_util import class_decl_to_string, clean_variable
import rospy

class SMGenHelper():
    """
    A class used to help generate state machines. This class basically
    creates a bunch of helper methods for navigating the configurations as
    well as the given automata.
    """
    def __init__(self, config, all_in_vars, all_out_vars, automata):
        self.config = config

        # List of in/out vars of the substates
        self.all_in_vars = all_in_vars
        self.all_out_vars = all_out_vars

        self.automata = automata

        # Map from what a substate thinks is the final transition to what it
        # actually should be (e.g. "done" -> "finished")
        self.sm_fake_out_to_real_out = config['output']

        # List of outputs of the entire SM
        self.sm_fake_outputs = self.sm_fake_out_to_real_out.keys()

        # To exit this SM, we find states that should exit. At the end, we'll
        # make any transition that goes to one of these states go to the real
        # output.
        self.state_name_to_sm_output = self.get_state_name_to_sm_output()

        """
        Populate various dictionaries to future functions. These lines
        should should be at the end of the initializer
        """
        # what class declaration goes with an input variable?
        self.in_var_to_class_decl = {}
        # to map response variables back to their activation variable
        self.in_var_to_out_var = {}
        # the map from class declaration to its out_map
        self.class_decl_to_out_map = {}

        for out_var, var_config in self.config.items():
            # If class_decl isn't in var_config, it's not configuration for a
            # state
            if 'class_decl' not in var_config:
                continue
            class_decl = var_config['class_decl']
            out_map = var_config['output_mapping']
            class_decl_str = class_decl_to_string(class_decl)
            self.class_decl_to_out_map[class_decl_str] = out_map
            for in_var, state_outcome in out_map.items():
                # check if this is a response variable
                if in_var in self.all_in_vars and out_var in self.all_out_vars:
                    self.in_var_to_class_decl[in_var] = class_decl
                    self.in_var_to_out_var[in_var] = out_var

        # Make one more pass to handle sensor variables
        for in_var in self.all_in_vars:
            if not self.is_response_var(in_var): # it's a sensor variable
                var_config = self.config[in_var]
                class_decl = var_config['class_decl']
                out_map = var_config['output_mapping']
                class_decl_str = class_decl_to_string(class_decl)
                self.class_decl_to_out_map[class_decl_str] = out_map

    def get_init_state_name(self):
        """ Return the name of the initial state.
        For now, naively choose the the state corresponding to a name of 0. """
        for state in self.automata:
            if state.name[0] == "0":
                return state.name
        raise SMGenError(BSErrorCodes.NO_INITIAL_STATE)

    def get_state_name_to_sm_output(self):
        """Create the mapping from the name of a substate that represents
        and exit, to the specific output. E.g. "State5" -> "finished"
        """
        d = {}
        for state in self.automata:
            outputs = self.get_state_output_vars(state)
            if self.is_sm_output(outputs):
                in_both = [k for k in self.sm_fake_outputs if k in outputs]
                if len(in_both) > 1:
                    raise Exception("Substate has more than one output for"\
                                  + " the entire state machine.")
                if len(in_both) == 0:
                    raise Exception("Substate has no output for the entire "\
                                  + "state machine, but one was expected.")
                d[state.name] = in_both[0]

        return d

    def get_sm_real_outputs(self):
        return self.sm_fake_out_to_real_out.values()

    def get_automaton(self, name, automata):
        """
        Returns the automaton of a given automaton name in the list of automata.

        @param name The name of the automaton of interest.
        @param automata List of AutomatonStates.
        """
        i = [a.name for a in automata].index(name)
        return automata[i]

    def get_transitions(self, state, automata):
        """
        Deduce the transition needed to go to the next states.

        @param state The state to transitions away from
        @param automata List of all AutomatonState's.
        @returns A dictionary, that maps next state to the input variable
                that need to be true to go to that next state.

                 For example:
            {
                'State1': ['stand_prep_c'], # input var 1 & 2 need to be true
                'State2': ['stand_prep_c', 'stand_c'],
                ...
            }
        """
        next_states = [str(x) for x in state.transitions]
        next_states = list(set(next_states) - set([state.name])) # remove self loop

        if len(next_states) == 0:
            print("{0} has no transitions out of it!".format(state.name))
        if len(next_states) > 1:
            rospy.logdebug("Multiple next states for {0}".format(state.name))

        transitions = {}
        for next_state in next_states:
            input_vals = self.get_automaton(next_state, automata).input_valuation
            conditions = []
            for idx, val in enumerate(input_vals):
                if val == 1: # only look at active inputs
                    in_var = self.get_in_var_name(idx)
                    if self.is_response_var(in_var):
                        # only consider this response variable if the source
                        # state activated this response variable.
                        if self.does_state_activate(state, in_var):
                            conditions.append(in_var)
                    else:
                        conditions.append(in_var)
            if len(conditions) > 0:
                transitions[next_state] = conditions
        return transitions

    def get_substate_name(self, in_var):
        """
        Return the readable name of the substate associated with an input
        variable.

        If the input variable is associated with an activation variable,
        use the activation variable. Otherwise, just use the input variable.

        @param in_var Input variable associated with this substate.
        """
        if self.is_response_var(in_var):
            return self.in_var_to_out_var[in_var]
        return in_var

    def get_in_var_name(self, i):
        """Get the input variable name at index [i] in [in_vars] dict."""
        return self.all_in_vars[i]

    def get_out_var_name(self, i):
        """Get the output variable name at index [i] in [out_vars] dict."""
        return self.all_out_vars[i]

    def get_state_output_vars(self, state):
        """Extract what output variables a state is outputting."""
        out_vals = state.output_valuation
        return [self.get_out_var_name(i) for i, v in enumerate(out_vals)
                                         if v == 1]

    def is_sm_output(self, outputs):
        """Return true iff this state's output valuations indicate that this
        state should transition out of the SM completely."""
        check = set(outputs)
        return any(k in check for k in self.sm_fake_outputs)

    def get_outcome_name(self, state_name, condition):
        """Returns a name needed to transition to the next_state.

        state_name: The name of the state in the automaton (e.g. "0")
        condition: A map from proposition to value needed for this outcome
                   to happen.
        If it's a fake state name representing a final outcome, just return
        that outcome.
        Otherwise, combine the conditions in a meaningful way.
        """
        if self.is_fake_state(state_name):
            # Could actually be "State 5" -> "done" -> "finished"
            fake_output = self.state_name_to_sm_output[state_name]
            return self.sm_fake_out_to_real_out[fake_output]
        else:
            # Output format is "key1_value1__key2_value2__key3_value3__..."
            outcome_str = "__".join(["{0}_{1}".format(clean_variable(k), v)
                                     for (k, v) in condition.items()])
            return outcome_str

    def get_real_name(self, state_name):
        """Returns the real name of a state. The only time a name isn't real
        is when this is a fake state, at which point this returns the
        corresponding output.
        """
        if self.is_fake_state(state_name):
            # Could actually be "State 5" -> "done" -> "finished"
            fake_output = self.state_name_to_sm_output[state_name]
            return self.sm_fake_out_to_real_out[fake_output]
        else:
            return state_name

    def is_fake_state(self, name):
        """Returns if a state is a placeholder for an output."""
        return name in self.state_name_to_sm_output

    def is_response_var(self, in_var):
        """Returns if an input variable is a response variable.
        A response variable is the counter part to an activation variable
        (e.g. completion, failure, etc.). """
        return in_var in self.in_var_to_class_decl

    def is_activation_var(self, out_var):
        """Returns if an output variable is an activation variable."""
        var_config = config[out_var]
        class_decl = var_config['class_decl']
        out_map = var_config['output_mapping']
        return any([in_var in self.all_in_vars
                    for in_var, _ in out_map.items()])

    def get_out_map(self, class_decl):
        """Get the output mapping associated with a class declaration."""
        class_decl_str = class_decl_to_string(class_decl)
        return self.class_decl_to_out_map[class_decl_str]

    def get_class_decl(self, var):
        """Get the class declaration associated with a variable."""
        if var in self.config:
            return self.config[var]['class_decl']
        elif var in self.in_var_to_class_decl:
            return self.in_var_to_class_decl[var]

    def does_state_activate(self, state, in_var):
        """Returns if [state] activate something that has [in_var] as a
        potential response."""
        # If in_var is not in this directionary, then it's not even a response
        # variable.
        if in_var not in self.in_var_to_out_var:
            return False
        out_var = self.in_var_to_out_var[in_var]
        state_out_vars = self.get_state_output_vars(state)
        return out_var in state_out_vars

