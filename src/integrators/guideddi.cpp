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
        // TODO apply our MIS scheme to BSDF + light samples

        // TODO combine multiple light sampling strategies

        L += UniformSampleOneLight(isect, scene, arena, sampler);
    }

    return L;
}

GuidedDirectIllum *CreateGuidedDiIntegrator(const ParamSet &params, std::shared_ptr<Sampler> sampler,
                                            std::shared_ptr<const Camera> camera) 
{
    return new GuidedDirectIllum(sampler, camera);
}


} // namespace pbrt