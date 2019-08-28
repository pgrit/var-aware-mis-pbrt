Variance-aware MIS weights in PBRTv3
===================================================

This repository implements the approach discussed in the
"Variance-Aware Multiple Importance Sampling" paper.

The core of the implementation is in `src/util/samis.h` and `src/util/samis.cpp`:
The class `SAMISRectifier` implements the rectification as a black box: given the outcome of paths
sampled from the first iteration, it computes the required variance estimates and the resulting factors.

For reference value computations, the files `src/util/varestim.h` and `src/util/varestim.cpp` provide a utility
class to compute accurate estimates of the variance factors given a large number of samples.

The implementation required minor changes in the bidirectional path tracer integrator
(`src/integrators/bdpt.cpp` and `bdpt.h`) to separate the rendering into multiple iterations
and to look-up and multiply with the propper variance factors during the MIS computation.
Furthermore, the weighted combination of the first iteration with the following one also
required a small addition to the `Film` class in `src/core/film.cpp` and `src/core/film.h`.
The defensive sampling application is implemented in a new integrator, see `src/integrators/guideddi.cpp` and
`src/integrators/guideddi.h`. The implementation is analogous to the one from the "Optimal Multiple Importance Sampling"
paper by Kondapaneni et al.

For build instructions, documentation, test scenes, etc., refer to [the original PBRTv3 repository](https://github.com/mmp/pbrt-v3/)
or the repository created by [Benedikt Bitterli](https://benedikt-bitterli.me/resources/).
All results in the paper were generated from (sometimes slightly modified versions of) the scenes from those repositories.

Testing
---------

The folder `test` contains a number of Python scripts to render comparison images with various approaches.

The `test/runtests_bdpt.py` script loads a number of PBRT scene files (specified in the script) and replaces
the integrator and sampler definitions by the appropriate methods, as given in the beginning of the script.

The `test/runtests_di.py` script does the same for the defensive sampling application. For the comparison to the
optimal MIS weights, the path to the source code from that paper needs to be hard-coded in the script (by modifying
the `optimal_mis_executable` variable in the beginning.)
