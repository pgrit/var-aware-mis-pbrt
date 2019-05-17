Variance-aware MIS weights in PBRTv3
===================================================

This repository implements the approach discussed in the
"Variance-Aware Multiple Importance Sampling" paper.

The core of the implementation is in `src/integrators/samis.h` and `src/integrators/samis.cpp`:
The class `SAMISRectifier` implements the rectification as a black box: given the outcome of paths
sampled from the first iteration, it computes the required variance estimates and the resulting factors.

The implementation required minor changes in the bidirectional path tracer integrator
(`src/integrators/bdpt.cpp` and `bdpt.h`) to separate the rendering into multiple iterations
and to look-up and multiply with the propper stratification factors during the MIS computation.
Furthermore, the weighted combination of the first iteration with the following one also
required a small addition to the `Film` class in `src/core/film.cpp` and `src/core/film.h`.

The defensive sampling application is implemented in a new integrator, see `src/integrators/guideddi.cpp` and
`src/integrators/guideddi.h`

For build instructions, documentation, test scenes, etc., refer to [the original PBRTv3 repository](https://github.com/mmp/pbrt-v3/)
or the repository created by [Benedikt Bitterli](https://benedikt-bitterli.me/resources/).
All results in the paper were generated from (sometimes slightly modified versions of) the scenes from those repositories.
