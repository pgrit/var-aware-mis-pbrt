Stratification-aware MIS weights for BDPT in PBRTv3
===================================================

This repository implements a simplified and cleaned-up version of the approach discussed in the
"Stratification-Aware Multiple Importance Sampling" paper. This is not the code that was used
for the paper. The results will be similar but not identical.

The implementation required minor changes in the bidirectional path tracer integrator
(`src/integrators/bdpt.cpp` and `bdpt.h`) to separate the rendering into multiple iterations
and to look-up and multiply with the propper stratification factors during the MIS computation.
The respecitive lines of code are tagged with a "for stratification-aware MIS" comment.
Furthermore, the weighted combination of the first iteration with the following one also
required a small addition to the `Film` class in `src/core/film.cpp` and `src/core/film.h`.

The core of the implementation is in `src/integrators/samis.h` and `src/integrators/samis.cpp`:
The class `SAMISRectifier` implements the rectification as a black box: given the outcome of paths
sampled from the first iteration, it computes the required variance estimates and the stratification
factors.

For build instructions, documentation, test scenes, etc., refer to [the original PBRTv3 repository](https://github.com/mmp/pbrt-v3/)
