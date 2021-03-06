#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_UTIL_SAMIS_H
#define PBRT_UTIL_SAMIS_H

#include "pbrt.h"
#include "geometry.h"
#include "spectrum.h"
#include "parallel.h"
#include "film.h"

#include "atomicimg.h"

namespace pbrt {

// Manages the stratification factor estimates for the techniques of a bidirectional path tracer
class SAMISRectifier {
    // TODO
    // - support lower resolution images (parameter controlled)
    // - replace by constant of one if no factor significantly larger
    // - manually modify the parameters for filtering etc
public:
    // Function that returns the weighting factor for a given path length and technique.
    // Computation should depend solely on the provided variance and mean.
    using ComputeFactorFn = std::function<Float(int, int, Float, Float)>;

    // TODO / REFACTOR instead of film, pass width and height. Instead of min and maxDepth, pass number of techniques
    // bdpt will then create multiple rectifier objects, one for each path length
    SAMISRectifier(const Film *film, int minDepth, int maxDepth, int downsamplingFactor,
                   bool considerMis, const ComputeFactorFn& computeFactor, bool loadRefs, bool loadVariance);

    void AddEstimate(const Point2f& pixel, int pathLen, int technique,
                     const Spectrum &unweightedEstimate, const Spectrum &weightedEstimate);

    // Fixes the buffers and prepares them for use during MIS computation.
    // Applies filtering, outlier removal, etc.
    // For on-line refinement, this could be implemented via double-buffering.
    void Prepare(int sampleCount, Float threshold);

    // Writes images with the stratification factors to .exr files for debugging purposes
    void WriteImages();

    Float Get(const Point2i &pixel, int pathLen, int technique) const;

    // Returns true if the pixel value from the prepass should be discarded
    bool IsMasked(const Point2i &pixel) const;

private:
    const int minDepth;
    const int maxDepth;
    const int downsamplingFactor;
    const int width;
    const int height;
    const int reducedWidth;
    const int reducedHeight;
    const Film *film;
    const bool considerMis;

    ComputeFactorFn computeFactor;

    std::vector<std::vector<AtomicImage>> techImages;
    std::vector<std::vector<std::vector<float>>> stratFactors;

    std::vector<bool> prepassMask;
};

} // namespace pbrt

#endif // PBRT_UTIL_SAMIS_H