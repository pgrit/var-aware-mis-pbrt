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

namespace pbrt {

// Manages the stratification factor estimates for the techniques of a bidirectional path tracer
class SAMISRectifier {
    // TODO
    // - support lower resolution images (parameter controlled)
    // - replace by constant of one if no factor significantly larger
    // - manually modify the parameters for filtering etc
public:
    SAMISRectifier(const Film *film, int minDepth, int maxDepth, int downsamplingFactor, bool considerMis);

    void AddEstimate(const Point2f& pixel, int pathLen, int technique,
                     const Spectrum &unweightedEstimate, const Spectrum &weightedEstimate);

    // Fixes the buffers and prepares them for use during MIS computation.
    // Applies filtering, outlier removal, etc.
    // For on-line refinement, this could be implemented via double-buffering.
    void Prepare(int sampleCount);

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

    // A datatype that allows copying std::atomic
    // The copy procedure itself is not atomic
    template <typename T>
    struct CopyableAtomic : public std::atomic<T> {
        CopyableAtomic() { }
        CopyableAtomic(T v) { this->store(v); }
        CopyableAtomic& operator= (const CopyableAtomic& a) { this->store(a.load()); return *this; }
        CopyableAtomic(const CopyableAtomic& a) { this->store(a.load()); }
        CopyableAtomic& operator= (T i) { this->store(i); return *this; }
    };

    template <typename T>
    static T AtomicAdd(std::atomic<T>& a, T b) {
        T old_val = a.load();
        T desired_val = old_val + b;
        while(!a.compare_exchange_weak(old_val, desired_val))
            desired_val = old_val + b;
        return desired_val;
    }

    using AtomicImage = std::vector<CopyableAtomic<float>>;

    std::vector<std::vector<AtomicImage>> techImages;
    std::vector<std::vector<std::vector<float>>> stratFactors;

    std::vector<bool> prepassMask;
};

} // namespace pbrt

#endif // PBRT_UTIL_SAMIS_H