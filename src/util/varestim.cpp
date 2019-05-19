#include "varestim.h"
#include "imageio.h"

namespace pbrt {

VarianceEstimator::VarianceEstimator(const Film *film)
: width(film->croppedPixelBounds.Diagonal().x), height(film->croppedPixelBounds.Diagonal().y)
, squares(width * height, 0.0f), means(width * height, 0.0f)
{
}

void VarianceEstimator::AddEstimate(const Point2f& pixel, const Spectrum &estimate) {
    Float val = estimate.y();
    AtomicAdd(squares[ToIdx(pixel)], val * val);
    AtomicAdd(means[ToIdx(pixel)], val);
}

void VarianceEstimator::WriteToFile(const std::string& filename, int numSamples, bool useLtHack, bool storeOurFactor) {
    // write values to buffer, store different information in each color channel
    std::vector<Float> rgb(3 * width * height);
    int offset = 0;
    for (int k = 0; k < squares.size(); ++k) {
        Float moment = squares[k] / numSamples;
        Float mean = means[k] / numSamples;
        Float meansqr = mean * mean;

        if (useLtHack) {
            // Because of design decisions in PBRT, the light tracer estimates already contain a division by the
            // number of samples (= number of pixels)
            // In order to compute the correct variance estimate, this needs to be accounted for!
            moment *= width * height;
        }

        Float var = moment - meansqr;

        // our variance factor
        Float factor = var == 0.0f ? 1.0f : (moment / var);
        // factor = factor < 1.0f ? 1.0f : factor;

        Float val = storeOurFactor ? factor : var;

        rgb[offset++] = val;
        rgb[offset++] = val;
        rgb[offset++] = val;
    }

    Bounds2i cropWnd(Point2i(0, 0), Point2i(width, height));
    pbrt::WriteImage(filename, rgb.data(), cropWnd, Point2i(width, height));
}

} // namespace pbrt