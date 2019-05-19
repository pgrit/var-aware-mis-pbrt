#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_UTIL_VARESTIM_H
#define PBRT_UTIL_VARESTIM_H

#include "pbrt.h"
#include "geometry.h"
#include "spectrum.h"
#include "parallel.h"
#include "film.h"

#include "atomicimg.h"

namespace pbrt {

class VarianceEstimator {
public:
    VarianceEstimator(const Film *film);
    void AddEstimate(const Point2f& pixel, const Spectrum &estimate);

    // Number of samples is a parameter here to avoid having to call AddEstimate() for samples with zero contribution
    void WriteToFile(const std::string& filename, int numSamples, bool useLtHack, bool storeOurFactor);

private:
    int width, height;
    AtomicImage squares, means;

    int ToIdx(const Point2f& pixel) {
        const int x = std::max(std::min(int(pixel.x), width - 1), 0);
        const int y = std::max(std::min(int(pixel.y), height - 1), 0);
        return y * width + x;
    }
};

} // namespace pbrt

#endif // PBRT_UTIL_VARESTIM_H