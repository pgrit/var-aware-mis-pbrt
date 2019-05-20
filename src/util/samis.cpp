#include "samis.h"
#include "imageio.h"

namespace pbrt {

SAMISRectifier::SAMISRectifier(const Film *film, int minDepth, int maxDepth, int downsamplingFactor,
                               bool considerMis, const ComputeFactorFn& computeFactor, bool loadRefs, bool loadVariance)
: film(film), minDepth(minDepth), maxDepth(maxDepth), downsamplingFactor(downsamplingFactor)
, width(film->croppedPixelBounds.Diagonal().x), height(film->croppedPixelBounds.Diagonal().y)
, reducedWidth(width / downsamplingFactor), reducedHeight(height / downsamplingFactor)
, considerMis(considerMis), computeFactor(computeFactor)
{
    // TODO pass lambda functions to constructor that return the number of techniques for a given path length

    int numPixels = width * height;
    for (int d = minDepth; d <= maxDepth; ++d) {
        // depth is in number of vertices, which is the number of techniques in BDPT
        techImages.emplace_back(d, AtomicImage(numPixels, 0.0f));
    }

    // TODO hacky...
    if (loadRefs) {
        for (int d = minDepth; d <= maxDepth; ++d) {
            stratFactors.emplace_back(d, std::vector<float>(width * height, 0.0f));
            for (int t = 1; t <= d; ++t) {
                Point2i res;
                std::string filename = loadVariance ? StringPrintf("variance-d%d-t%d.exr",d-2,t) : StringPrintf("factor-d%d-t%d.exr",d-2,t);
                std::unique_ptr<RGBSpectrum[]> img(ReadImageEXR(filename,&res.x, &res.y));
                for (int y = 0; y < res.y; ++y) for (int x = 0; x < res.x; ++x) {
                    Float val = img[y * res.x + x][0];
                    if (loadVariance) {
                        val = val == 0.0 ? 1.0 : (1.0 / val);
                        if (std::isinf(val) || std::isnan(val) || val < 0.0)
                            val = 1.0;
                    }
                    stratFactors[d-minDepth][t-1][y * res.x + x] = val;
                }
            }
        }
    }
}

void SAMISRectifier::AddEstimate(const Point2f &pixel, int pathLen, int technique,
                                 const Spectrum &unweightedEstimate, const Spectrum &weightedEstimate)
{
    // TODO assert that the factors are not yet finalized!
    if (pathLen < minDepth || pathLen > maxDepth)
        return;

    const int x = std::max(std::min(int(pixel.x), width - 1), 0);
    const int y = std::max(std::min(int(pixel.y), height - 1), 0);
    const int index = y * width + x;

    const Spectrum& estim = considerMis ? weightedEstimate : unweightedEstimate;
    AtomicAdd(techImages[pathLen - minDepth][technique - 1][index], estim.y());
}

void SAMISRectifier::Prepare(int sampleCount, Float threshold) {
    const Float invSamples = 1.0f / sampleCount;
    for (int d = minDepth; d <= maxDepth; ++d) {
        stratFactors.emplace_back(d, std::vector<float>(reducedWidth * reducedHeight, 0.0f));
        for (int t = 1; t <= d; ++t) {
            auto &tech = techImages[d - minDepth][t - 1];

            // Estimate the variances
            // See Knuth TAOCP vol 2, 3rd edition, page 232
            ParallelFor([&](int y){
                for (int x = 0; x < reducedWidth; ++x) {
                    int n = 0;
                    float mean = 0.0f;
                    float var = 0.0f;
                    for (int yfull = y * downsamplingFactor; yfull < height && yfull < (y+1) * downsamplingFactor; ++yfull) {
                        for (int xfull = x * downsamplingFactor; xfull < width && xfull < (x+1) * downsamplingFactor; ++xfull) {
                            auto idx = yfull * width + xfull;
                            n++;
                            if (n == 1) {
                                mean = tech[idx];
                            } else {
                                auto newMean = mean + (tech[idx] - mean) / n;
                                var += (tech[idx] - mean) * (tech[idx] - newMean);
                                mean = newMean;
                            }
                        }
                    }
                    var /= (n - 1);
                    stratFactors[d - minDepth][t - 1][y * reducedWidth + x] = computeFactor(d, t, var, mean);
                }
            }, reducedHeight, 1);
        }
    }
    techImages.clear();

    prepassMask.resize(reducedWidth * reducedHeight);

    for (int y = 0; y < reducedHeight; ++y)
    for (int x = 0; x < reducedWidth; ++x) {
        float maxval = 1;
        for (int d = minDepth; d <= maxDepth; ++d)
        for (int t = 1; t <= d; ++t) {
            auto &s = stratFactors[d - minDepth][t - 1][y * reducedWidth + x];
            maxval = std::max(maxval, float(s));
        }
        prepassMask[y * reducedWidth + x] = maxval > threshold;
    }
}

void SAMISRectifier::WriteImages() {
    std::vector<Float> rgb(3 * reducedWidth * reducedHeight);
    for (int d = minDepth; d <= maxDepth; ++d) {
        for (int t = 1; t <= d; ++t) {
            auto &tech = stratFactors[d - minDepth][t - 1];
            int offset = 0;
            for (int k = 0; k < tech.size(); ++k) {
                rgb[offset++] = tech[k];
                rgb[offset++] = tech[k];
                rgb[offset++] = tech[k];
            }
            Bounds2i cropWnd(Point2i(0, 0), Point2i(reducedWidth, reducedHeight));
            pbrt::WriteImage(StringPrintf("stratfactor-d%d-t%d.exr", d, t), rgb.data(), cropWnd, Point2i(reducedWidth, reducedHeight));
        }
    }
}

Float SAMISRectifier::Get(const Point2i &pixel, int pathLen, int technique) const {
    if (pathLen < minDepth || pathLen > maxDepth)
        return 1.0f;

    const int x = std::max(std::min(int(pixel.x / downsamplingFactor), reducedWidth - 1), 0);
    const int y = std::max(std::min(int(pixel.y / downsamplingFactor), reducedHeight - 1), 0);
    auto& buffer = stratFactors[pathLen - minDepth][technique - 1];
    return buffer[y * reducedWidth + x];
}

bool SAMISRectifier::IsMasked(const Point2i &pixel) const {
    const int x = std::max(std::min(int(pixel.x / downsamplingFactor), reducedWidth - 1), 0);
    const int y = std::max(std::min(int(pixel.y / downsamplingFactor), reducedHeight - 1), 0);
    const int index = y * reducedWidth + x;
    return prepassMask[index];
}

} // namespace pbrt