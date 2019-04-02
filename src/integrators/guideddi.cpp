#include "integrators/guideddi.h"
#include "film.h"
#include "filters/box.h"
#include "integrator.h"
#include "lightdistrib.h"
#include "paramset.h"
#include "progressreporter.h"
#include "sampler.h"
#include "camera.h"

namespace pbrt {

void GuidedDirectIllum::Render(const Scene &scene) {
    SetUp(scene);

    // Determine the number of iterations based on the number of samples per pixel.
    int numIter = sampler->samplesPerPixel;

    ProgressReporter reporter(numIter, "Rendering");
    for (int iter = 0; iter < numIter; ++iter) {
        PrepareIteration(scene, iter);
        RenderIteration(scene, iter);
        ProcessIteration(scene, iter);
        reporter.Update();
    }
    reporter.Done();
    WriteFinalImage();
}

void GuidedDirectIllum::SetUp(const Scene &scene) {
    guidedLightDistrib.reset(new SpatialLightDistribution(scene));
}

void GuidedDirectIllum::PrepareIteration(const Scene &scene, const int iter) {

}

/// Returns the initializer for the FNV hash function
inline uint32_t fnv_init() { return 0x811C9DC5; }

/// Hashes 4 bytes using FNV
inline uint32_t fnv_hash(uint32_t h, uint32_t d) {
    h = (h * 16777619) ^ ( d        & 0xFF);
    h = (h * 16777619) ^ ((d >>  8) & 0xFF);
    h = (h * 16777619) ^ ((d >> 16) & 0xFF);
    h = (h * 16777619) ^ ((d >> 24) & 0xFF);
    return h;
}

/// Returns a seed for a sampler object, based on the current pixel id and iteration count
inline uint32_t sampler_seed(uint32_t pixel, uint32_t iter) {
    return fnv_hash(fnv_hash(fnv_init(), pixel), iter);
}

void GuidedDirectIllum::RenderIteration(const Scene &scene, const int iter) {
    // Compute number of tiles, _nTiles_, to use for parallel rendering
	Bounds2i sampleBounds = camera->film->GetSampleBounds();
	Vector2i sampleExtent = sampleBounds.Diagonal();
	const int tileSize = 16;
	Point2i nTiles((sampleExtent.x + tileSize - 1) / tileSize,
		(sampleExtent.y + tileSize - 1) / tileSize);

    ParallelFor2D([&](Point2i tile) {
        // Render section of image corresponding to _tile_

        // Allocate _MemoryArena_ for tile
        MemoryArena arena;

        // Get sampler instance for tile
        int seed = sampler_seed(tile.y * nTiles.x + tile.x, iter);
        std::unique_ptr<Sampler> tileSampler = sampler->Clone(seed);

        // Compute sample bounds for tile
        int x0 = sampleBounds.pMin.x + tile.x * tileSize;
        int x1 = std::min(x0 + tileSize, sampleBounds.pMax.x);
        int y0 = sampleBounds.pMin.y + tile.y * tileSize;
        int y1 = std::min(y0 + tileSize, sampleBounds.pMax.y);
        Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));
        LOG(INFO) << "Starting image tile " << tileBounds;

        // Get _FilmTile_ for tile
        std::unique_ptr<FilmTile> filmTile =
            camera->film->GetFilmTile(tileBounds);

        // Loop over pixels in tile to render them
        for (Point2i pixel : tileBounds) {
            tileSampler->StartPixel(pixel);

            Bounds2i pixelBounds = camera->film->GetSampleBounds();
            if (!InsideExclusive(pixel, pixelBounds))
                continue;

            // Sample a ray from the camera
            CameraSample cameraSample = tileSampler->GetCameraSample(pixel);
            RayDifferential ray;
            Float rayWeight =
                camera->GenerateRayDifferential(cameraSample, &ray);
            ray.ScaleDifferentials(
                1 / std::sqrt((Float)tileSampler->samplesPerPixel));

            Spectrum L(0.f);
            if (rayWeight > 0)
                L = Li(ray, scene, *tileSampler, arena, pixel, iter);

            filmTile->AddSample(cameraSample.pFilm, L, rayWeight);

            arena.Reset();
        }

        // Merge image tile into _Film_
        camera->film->MergeFilmTile(std::move(filmTile));
    }, nTiles);
}

void GuidedDirectIllum::ProcessIteration(const Scene &scene, const int iter) {
    if (iter == 0) {
        // At the end of the first iteration, we finalize the variance estimates for MIS
    }
}

void GuidedDirectIllum::WriteFinalImage() {
    camera->film->WriteImage();
}

Spectrum GuidedDirectIllum::Li(const RayDifferential &ray, const Scene &scene,
            Sampler &sampler, MemoryArena &arena, const Point2i& pixel,
            const int iter) const
{
    Spectrum L(0.f);

    // Find closest ray intersection or return background radiance
    SurfaceInteraction isect;
    if (!scene.Intersect(ray, &isect)) {
        for (const auto &light : scene.lights) L += light->Le(ray);
        return L;
    }

    // Compute scattering functions for surface interaction
    isect.ComputeScatteringFunctions(ray, arena);
    if (!isect.bsdf)
        return Li(isect.SpawnRay(ray.d), scene, sampler, arena, pixel, iter);
    Vector3f wo = isect.wo;

    // Compute emitted light if ray hit an area light source
    L += isect.Le(wo);

    if (scene.lights.size() > 0) {
        const Distribution1D *lightDistr = guidedLightDistrib->Lookup(isect.p);
        Float uniformSelPdf = 1 / Float(scene.lights.size());

        int lightIdx;
        Light* lightUniform = SampleLight(scene, sampler, nullptr, &lightIdx);
        L += SampleLightSurface(scene, *lightUniform, isect, sampler, SAMPLE_UNIFORM) / uniformSelPdf;

        Light* lightGuided = SampleLight(scene, sampler, lightDistr, &lightIdx);
        Float guidedSelPdf = lightDistr->DiscretePDF(lightIdx);
        L += SampleLightSurface(scene, *lightGuided, isect, sampler, SAMPLE_GUIDED) / guidedSelPdf;

        L += SampleBsdf(scene, isect, sampler);

        L += Spectrum(0);
    }

    return L;
}

Light* GuidedDirectIllum::SampleLight(const Scene &scene, Sampler &sampler,
    const Distribution1D *lightDistrib, int* lightNum) const {
    // Randomly choose a single light to sample
    int nLights = int(scene.lights.size());
    if (nLights == 0) return nullptr;
    if (lightDistrib)
        *lightNum = lightDistrib->SampleDiscrete(sampler.Get1D());
    else
        *lightNum = std::min((int)(sampler.Get1D() * nLights), nLights - 1);
    return scene.lights[*lightNum].get();
}

Spectrum GuidedDirectIllum::SampleLightSurface(const Scene &scene, const Light& light,
    const Interaction &it, Sampler &sampler, SamplingTech tech) const
{
    Spectrum L(0.0f);

    // sample point on the light
    Point2f uLight = sampler.Get2D();
    Vector3f wi;
    Float lightPdf = 0, scatteringPdf = 0;
    VisibilityTester visibility;
    Spectrum Li = light.Sample_Li(it, uLight, &wi, &lightPdf, &visibility);

    if (lightPdf > 0 && !Li.IsBlack()) {
        // Compute BSDF or phase function's value for light sample
        Spectrum f;
        if (it.IsSurfaceInteraction()) {
            // Evaluate BSDF for light sampling strategy
            const SurfaceInteraction &isect = (const SurfaceInteraction &)it;
            f = isect.bsdf->f(isect.wo, wi, BSDF_ALL) *
                AbsDot(wi, isect.shading.n);
            scatteringPdf = isect.bsdf->Pdf(isect.wo, wi, BSDF_ALL);
        } else {
            // Evaluate phase function for light sampling strategy
            const MediumInteraction &mi = (const MediumInteraction &)it;
            Float p = mi.phase->p(mi.wo, wi);
            f = Spectrum(p);
            scatteringPdf = p;
        }
        if (!f.IsBlack()) {
            // Compute effect of visibility for light source sample
            // if (handleMedia) {
            //     Li *= visibility.Tr(scene, sampler);
            // } else
            if (!visibility.Unoccluded(scene)) {
                Li = Spectrum(0.f);
            }

            // Add light's contribution to reflected radiance
            if (!Li.IsBlack()) {
                if (IsDeltaLight(light.flags))
                    L = f * Li / lightPdf;
                else {
                    Float weight = MisWeight(&light, tech, scatteringPdf, lightPdf);
                    L = f * Li * weight / lightPdf;
                }
            }
        }
    }
    return L;
}

Spectrum GuidedDirectIllum::SampleBsdf(const Scene &scene, const Interaction &it, Sampler &sampler) const {
    Point2f uScattering = sampler.Get2D();

    Spectrum f;
    Vector3f wi;
    Float scatteringPdf;
    Float lightPdf;
    bool sampledSpecular = false;
    if (it.IsSurfaceInteraction()) {
        // Sample scattered direction for surface interactions
        BxDFType sampledType;
        const SurfaceInteraction &isect = (const SurfaceInteraction &)it;
        f = isect.bsdf->Sample_f(isect.wo, &wi, uScattering, &scatteringPdf,
                                 BSDF_ALL, &sampledType);
        f *= AbsDot(wi, isect.shading.n);
        sampledSpecular = (sampledType & BSDF_SPECULAR) != 0;
    } else {
        // Sample scattered direction for medium interactions
        const MediumInteraction &mi = (const MediumInteraction &)it;
        Float p = mi.phase->Sample_p(mi.wo, &wi, uScattering);
        f = Spectrum(p);
        scatteringPdf = p;
    }

    if (!f.IsBlack() && scatteringPdf > 0) {
        // Account for light contributions along sampled direction _wi_

        // Find intersection and compute transmittance
        SurfaceInteraction lightIsect;
        Ray ray = it.SpawnRay(wi);
        bool foundSurfaceInteraction = scene.Intersect(ray, &lightIsect);

        // Add light contribution from material sampling
        Spectrum Li(0.f);
        if (foundSurfaceInteraction) {
            Li = lightIsect.Le(-wi);
        }

        Float weight = MisWeight(foundSurfaceInteraction ? lightIsect.primitive->GetAreaLight() : nullptr,
                                 SAMPLE_BSDF, scatteringPdf, lightPdf);
        return f * Li * weight / scatteringPdf;
    }

    return Spectrum(0.f);
}

Float GuidedDirectIllum::MisWeight(const Light* light, SamplingTech tech, Float pdfBsdf, Float pdfLight) const {
    // TODO compute balance heuristic weights
    // TODO implement logging of unweighted contribution
    // TODO implement variance aware weights
    // TODO implement optimal weights?

    return 1.0f / 3.0f;
}

GuidedDirectIllum *CreateGuidedDiIntegrator(const ParamSet &params, std::shared_ptr<Sampler> sampler,
                                            std::shared_ptr<const Camera> camera)
{
    return new GuidedDirectIllum(sampler, camera);
}


} // namespace pbrt