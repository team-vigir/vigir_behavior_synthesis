# Synthesis Byroducts

The `slugs` synthesizer needs to generate a few files each time it synthesizes.
File formats include `.json`, `.slugsin`, and `.structuredslugs`.
These are saved in a folder named after the specification and can be useful for debugging the LTL specifications and resulting automata.

The repository's `.gitignore` is set up to ignore everything in the `synthesis_byproducts` folder except for this `README` file.