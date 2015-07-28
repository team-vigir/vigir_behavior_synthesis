vigir_ltl_synthesizer
=====================

This package acts as an interface between other ROS packages and the LTL Synthesis tool(s).
Currently, it only supports `slugs`, a synthesis tool for the GR(1) fragment of LTL:

> slugs - SmalL bUt Complete GROne Synthesizer
> --------------------------------------------

> Slugs is a stand-alone reactive synthesis tool for generalized reactivity(1) synthesis.

> ['https://github.com/LTLMoP/slugs'](https://github.com/LTLMoP/slugs)

> Copyright (c) 2013, Ruediger Ehlers, Vasumathi Raman, and Cameron Finucane

The source code of `slugs` is not included in this package.
You can use the installation script, `install_slugs.sh`, provided here or download and compile `slugs` manually.
If you install it manually, make sure the `slugs` executable is in the `PATH`.

This package does include `slugs`' `StructuredSlugsParser` tool as well its `LICENSE`.