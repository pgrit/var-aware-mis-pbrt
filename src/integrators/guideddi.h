#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_INTEGRATORS_GUIDEDDI_H
#define PBRT_INTEGRATORS_GUIDEDDI_H

// integrators/guideddi.h*
#include "pbrt.h"
#include "integrator.h"
#include "scene.h"
#include "lightdistrib.h"

#include <unordered_map>

namespace pbrt {

// Same as DirectLighting integrator, but able to combine
// multiple light selection strategies via MIS.
// Mimics the implementation of the Optimal MIS paper [Kondapaneni et al. 2019]
// Supports only direct lighting, no media, and no delta light sources or specular surfaces.
class GuidedDirectIllum : public Integrator {
public:
    GuidedDirectIllum(std::shared_ptr<Sampler> sampler,
                      std::shared_ptr<const Camera> camera)
    : sampler(sampler), camera(camera)
    {
    }

    void Render(const Scene &scene) override;

    virtual void SetUp(const Scene &scene);
    virtual void PrepareIteration(const Scene &scene, const int iter);
    virtual void RenderIteration(const Scene &scene, const int iter);
    virtual void ProcessIteration(const Scene &scene, const int iter);
    virtual void WriteFinalImage();

    virtual Spectrum Li(const RayDifferential &ray, const Scene &scene,
                        Sampler &sampler, MemoryArena &arena, const Point2i& pixel,
                        const int iter);

protected:
    enum SamplingTech {
        SAMPLE_UNIFORM = 0,
        SAMPLE_GUIDED = 1,
        SAMPLE_BSDF = 2
    };

    virtual Spectrum SampleLightSurface(const Point2i& pixel, const Scene &scene, const Distribution1D *lightDistrib,
        const Interaction &it, Sampler &sampler, SamplingTech tech);

    virtual Spectrum SampleBsdf(const Point2i& pixel, const Scene &scene, const Distribution1D *lightDistrib, const Interaction &it, Sampler &sampler);

    virtual Float MisWeight(const Scene& scene, const Point2i& pixel, const Light* light, const Distribution1D *lightDistrib, SamplingTech tech, Float pdfBsdf, Float pdfLight);

    // Callback function invoked whenever an MC estimate is computed from any technique
    virtual void LogContrib(const Point2i& pixel, const Spectrum& value, Float misWeight, SamplingTech tech);

private:
    std::shared_ptr<Sampler> sampler;
    std::shared_ptr<const Camera> camera;

    std::unique_ptr<LightDistribution> guidedLightDistrib;

    std::unordered_map<const Light *, size_t> lightToIdx;
};

GuidedDirectIllum *CreateGuidedDiIntegrator(
	const ParamSet &params, std::shared_ptr<Sampler> sampler,
	std::shared_ptr<const Camera> camera);

} // namespace pbrt

#endif // PBRT_INTEGRATORS_GUIDEDDI_H