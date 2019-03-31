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

namespace pbrt {

// Same as DirectLighting integrator, but able to combine 
// multiple light selection strategies via MIS.
// Mimics the implementation of the Optimal MIS paper [Kondapaneni et al. 2019]
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
                        const int iter) const;

private:
    std::shared_ptr<Sampler> sampler;
    std::shared_ptr<const Camera> camera;
};

GuidedDirectIllum *CreateGuidedDiIntegrator(
	const ParamSet &params, std::shared_ptr<Sampler> sampler,
	std::shared_ptr<const Camera> camera);

} // namespace pbrt

#endif // PBRT_INTEGRATORS_GUIDEDDI_H