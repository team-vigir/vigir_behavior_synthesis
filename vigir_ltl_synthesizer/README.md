vigir_ltl_synthesizer
=====================

This package acts as an interface between other ROS packages and the LTL Synthesis tool(s).
Currently, it only supports SLUGS, a synthesis tool for the GR(1) fragment of LTL:

slugs - SmalL bUt Complete GROne Synthesizer
--------------------------------------------
Slugs is a stand-alone reactive synthesis tool for generalized reactivity(1) synthesis.
https://github.com/LTLMoP/slugs
Copyright (c) 2013, Ruediger Ehlers, Vasumathi Raman, and Cameron Finucane

The source code of SLUGS is not included in this package.
It is assumed that SLUGS is downloaded, compiled, and copied to /usr/bin/local/.
The package does include SLUGS' StructuredSlugsParser tool as well its LICENSE.