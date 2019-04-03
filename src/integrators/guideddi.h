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
#include "util/samis.h"

#include <unordered_map>

namespace pbrt {

enum OurMode {
    OUR_DISABLED,
    OUR_VARIANCE,
    OUR_MOMENT
};

enum MisMode {
    MIS_BALANCE,
    MIS_POWER,
    MIS_UNIFORM
};

// Same as DirectLighting integrator, but able to combine
// multiple light selection strategies via MIS.
// Mimics the implementation of the Optimal MIS paper [Kondapaneni et al. 2019]
// Supports only direct lighting, no media, and no delta light sources or specular surfaces.
class GuidedDirectIllum : public Integrator {
public:
    GuidedDirectIllum(std::shared_ptr<Sampler> sampler,
                      std::shared_ptr<const Camera> camera,
                      OurMode ourMode,
                      MisMode misMode,
                      bool enableBsdfSamples,
                      bool enableGuided,
                      bool enableUniform)
    : sampler(sampler), camera(camera)
    , ourMode(ourMode), misMode(misMode)
    , enableBsdfSamples(enableBsdfSamples)
    , enableGuided(enableGuided)
    , enableUniform(enableUniform)
    {
    }

    void Render(const Scene &scene) override;

    virtual void SetUp(const Scene &scene);
    virtual void PrepareIteration(const Scene &scene, const int iter);
    virtual void RenderIteration(const Scene &scene, const int iter);
    virtual void ProcessIteration(const Scene &scene, const int iter);
    virtual void WriteFinalImage();

    virtual Spectrum Li(const RayDifferential &ray, const Scene &scene,
                        Sampler &sampler, MemoryArena &arena, const Point2f& pixel,
                        const int iter);

protected:
    enum SamplingTech {
        SAMPLE_UNIFORM = 0,
        SAMPLE_GUIDED = 1,
        SAMPLE_BSDF = 2
    };

    OurMode ourMode;
    MisMode misMode;
    bool enableBsdfSamples;
    bool enableGuided;
    bool enableUniform;

    virtual Spectrum SampleLightSurface(const Point2f& pixel, const Scene &scene, const Distribution1D *lightDistrib,
        const Interaction &it, Sampler &sampler, SamplingTech tech);

    virtual Spectrum SampleBsdf(const Point2f& pixel, const Scene &scene, const Distribution1D *lightDistrib, const Interaction &it, Sampler &sampler);

    virtual Float MisWeight(const Scene& scene, const Point2f& pixel, const Light* light, const Distribution1D *lightDistrib, SamplingTech tech, Float pdfBsdf, Float pdfLight);

    // Callback function invoked whenever an MC estimate is computed from any technique
    virtual void LogContrib(const Point2f& pixel, const Spectrum& value, Float misWeight, SamplingTech tech);

private:
    std::shared_ptr<Sampler> sampler;
    std::shared_ptr<const Camera> camera;

    std::unique_ptr<LightDistribution> guidedLightDistrib;

    std::unordered_map<const Light *, size_t> lightToIdx;

    std::unique_ptr<SAMISRectifier> rectifier;

    int numIterations;
    int currentIteration;

    std::vector<Float> prepassBuffer;
};

GuidedDirectIllum *CreateGuidedDiIntegrator(
	const ParamSet &params, std::shared_ptr<Sampler> sampler,
	std::shared_ptr<const Camera> camera);

} // namespace pbrt

#endif // PBRT_INTEGRATORS_GUIDEDDI_H