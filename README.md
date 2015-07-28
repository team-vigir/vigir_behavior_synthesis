# Team ViGIR's Behavior Synthesis stack

ROS Python packages for automatically synthesizing *behaviors* (executable state machines) from a high-level user specification via reactive Linear Temporal Logic (LTL) synthesis.

## About

### Workflow

The nominal workflow is depicted in the diagram below:

![img](https://dl.dropboxusercontent.com/s/qi8zcg187lal0td/behavior_synthesis_packages.png)

### Dependecies

For `vigir_ltl_specification`, you need:
- [`ReSpeC`](https://github.com/team-vigir/ReSpeC) for compiling LTL specifications from the high-level user requests

For `vigir_ltl_synthesizer`, you need:
- [`slugs`](https://github.com/LTLMoP/slugs) synthesizer for LTL synthesis (an [installation script](https://github.com/team-vigir/vigir_behavior_synthesis/blob/master/vigir_ltl_synthesizer/install_slugs.sh) is provided)

For using the full stack, you also need:
- [`flexbe_behavior_engine`](https://github.com/team-vigir/flexbe_behavior_engine) for sending synthesis requests via the FlexBE app's GUI and for access to the `StateInstantiation.msg`
- [`vigir_behaviors`](https://github.com/team-vigir/vigir_behaviors) for access to the library of states and for saving the synthesized behaviors

### Maintainers:
- Spyros Maniatopoulos ([@spmaniato](https://github.com/spmaniato), sm2296@cornell.edu)

## License
[BSD-3](http://opensource.org/licenses/BSD-3-Clause) (see [`LICENSE`](https://raw.githubusercontent.com/tea-vigir/vigir_behavior_synthesis/master/LICENSE) file)