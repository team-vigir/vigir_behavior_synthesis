# Team ViGIR's Behavior Synthesis stack

ROS Python packages for automatically synthesizing *high-level robot behaviors* (in the form of executable SMACH-based state machines) from the user's task specifications via reactive Linear Temporal Logic (LTL) synthesis.

## About

### Experimental Demonstration on ATLAS

Click on the image below to watch a demonstration of Behavior Synthesis on ATLAS.

[![ICRA 2016 video attachment](https://dl.dropboxusercontent.com/s/tqggkyx7e1ryabj/ATLAS_synthesis_demo.png)](http://www.youtube.com/watch?v=mez-7pegxuE)

### Synthesis Workflow

The nominal workflow is depicted in the diagram below:

![img](https://dl.dropboxusercontent.com/s/ctxjcrm061ouu5o/behavior_synthesis_packages.png)

### Dependencies

For `vigir_ltl_specification`, you need:
- [`ReSpeC`](https://github.com/team-vigir/ReSpeC) for compiling LTL specifications from the high-level user requests

For `vigir_ltl_synthesizer`, you need:
- [`slugs`](https://github.com/LTLMoP/slugs) synthesizer for LTL synthesis (an [installation script](https://github.com/team-vigir/vigir_behavior_synthesis/blob/master/vigir_ltl_synthesizer/install_slugs.sh) is provided)

For using the full stack, you also need:
- [`flexbe_behavior_engine`](https://github.com/team-vigir/flexbe_behavior_engine) for sending synthesis requests via the FlexBE app's GUI and for access to the `StateInstantiation.msg`
- [`vigir_behaviors`](https://github.com/team-vigir/vigir_behaviors) for access to the library of states and for saving the synthesized behaviors

### Publications

- Spyros Maniatopoulos, Philipp Schillinger, Vitchyr Pong, David C. Conner, and Hadas Kress-Gazit, "Reactive High-level Behavior Synthesis for an Atlas Humanoid Robot", IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016 (to appear).

### Maintainers:
- Spyros Maniatopoulos ([@spmaniato](https://github.com/spmaniato), sm2296@cornell.edu)

## License
[BSD-3](http://opensource.org/licenses/BSD-3-Clause) (see [`LICENSE`](https://raw.githubusercontent.com/tea-vigir/vigir_behavior_synthesis/master/LICENSE) file)
