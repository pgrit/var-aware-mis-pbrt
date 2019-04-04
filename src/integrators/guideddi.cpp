#include "integrators/guideddi.h"
#include "film.h"
#include "filters/box.h"
#include "integrator.h"
#include "lightdistrib.h"
#include "paramset.h"
#include "progressreporter.h"
#include "sampler.h"
#include "camera.h"
#include "imageio.h"

namespace pbrt {

void GuidedDirectIllum::Render(const Scene &scene) {
    currentIteration = 0;
    SetUp(scene);

    // Determine the number of iterations based on the number of samples per pixel.
    numIterations = sampler->samplesPerPixel;

    ProgressReporter reporter(numIterations, "Rendering");
    for (int iter = 0; iter < numIterations; ++iter) {
        currentIteration = iter; // TODO remove this from the parameters of the following functions
        PrepareIteration(scene, iter);
        RenderIteration(scene, iter);
        ProcessIteration(scene, iter);
        reporter.Update();
    }
    reporter.Done();
    WriteFinalImage();

    if (ourMode != OUR_DISABLED && visWeights)
        rectifier->WriteImages();
}

void GuidedDirectIllum::SetUp(const Scene &scene) {
    guidedLightDistrib.reset(new SpatialLightDistribution(scene));

    for (size_t i = 0; i < scene.lights.size(); ++i)
        lightToIdx[scene.lights[i].get()] = i;

    if (ourMode != OUR_DISABLED)
        rectifier.reset(new SAMISRectifier(camera->film, 3, 3, 16, false, // TODO this is ugly, only works because the number of techniques here is also 3, as in bdpt
            [&](int d, int t, Float var, Float mean) {
                if (var != 0 && mean != 0)
                    return ourMode == OUR_VARIANCE ? (1 / var) : (1 + mean * mean / var);
                else
                    return Float(1);
            }));
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
                L = Li(ray, scene, *tileSampler, arena, cameraSample.pFilm, iter);

            filmTile->AddSample(cameraSample.pFilm, L, rayWeight);

            arena.Reset();
        }

        // Merge image tile into _Film_
        camera->film->MergeFilmTile(std::move(filmTile));
    }, nTiles);
}

void GuidedDirectIllum::ProcessIteration(const Scene &scene, const int iter) {
    if (ourMode != OUR_DISABLED && iter == 0) {
        // At the end of the first iteration, we finalize the variance estimates for MIS
        rectifier->Prepare(1);

        // Store the rendered image from the first iteration separately, it will be re-weighted!
        prepassBuffer = camera->film->WriteImageToBuffer(1.0f);
        camera->film->Clear();
    }
}

void GuidedDirectIllum::WriteFinalImage() {
    // TODO if our is enabled
    if (ourMode != OUR_DISABLED) {
        std::vector<Float> rectified = camera->film->WriteImageToBuffer(Float(1) / (numIterations - 1));

        Float invSampleCount = Float(1) / numIterations;
        Float weightPrepass = invSampleCount;
        Float weightRectified =  (numIterations - 1) * invSampleCount;

        size_t offset = 0;
        for (Point2i px : camera->film->croppedPixelBounds) {
            if (rectifier->IsMasked(px)) { // ignore the prepass
                // out[offset + 0] = rectified[offset + 0];
                // out[offset + 1] = rectified[offset + 1];
                // out[offset + 2] = rectified[offset + 2];
            } else { // average the two based on sample count
                rectified[offset + 0] = prepassBuffer[offset + 0] * weightPrepass + rectified[offset + 0] * weightRectified;
                rectified[offset + 1] = prepassBuffer[offset + 1] * weightPrepass + rectified[offset + 1] * weightRectified;
                rectified[offset + 2] = prepassBuffer[offset + 2] * weightPrepass + rectified[offset + 2] * weightRectified;
            }
            offset += 3;
        }

        pbrt::WriteImage(camera->film->filename, rectified.data(), camera->film->croppedPixelBounds, camera->film->fullResolution);
    } else
        camera->film->WriteImage();
}

Spectrum GuidedDirectIllum::Li(const RayDifferential &ray, const Scene &scene,
            Sampler &sampler, MemoryArena &arena, const Point2f& pixel,
            const int iter)
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
        L += SampleLightSurface(pixel, scene, lightDistr, isect, sampler, SAMPLE_UNIFORM);
        L += SampleLightSurface(pixel, scene, lightDistr, isect, sampler, SAMPLE_GUIDED);
        L += SampleBsdf(pixel, scene, lightDistr, isect, sampler);
    }

    return L;
}

Spectrum GuidedDirectIllum::SampleLightSurface(const Point2f& pixel, const Scene &scene, const Distribution1D *lightDistrib,
    const Interaction &it, Sampler &sampler, SamplingTech tech)
{
    Spectrum L(0.0f);

    // select a light source
    int nLights = int(scene.lights.size());
    int lightIdx;
    Float lightSelectPdf;
    if (tech == SAMPLE_GUIDED)
        lightIdx = lightDistrib->SampleDiscrete(sampler.Get1D(), &lightSelectPdf);
    else {
        lightIdx = std::min((int)(sampler.Get1D() * nLights), nLights - 1);
        lightSelectPdf = Float(1) / nLights;
    }
    Light& light = *(scene.lights[lightIdx].get());

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
                    L = f * Li / (lightPdf * lightSelectPdf);
                else {
                    Float weight = MisWeight(scene, pixel, &light, lightDistrib, tech, scatteringPdf, lightPdf);
                    Spectrum estimate = f * Li / (lightPdf * lightSelectPdf);
                    L = weight * estimate;
                    LogContrib(pixel, estimate, weight, tech);
                }
            }
        }
    }
    return L;
}

Spectrum GuidedDirectIllum::SampleBsdf(const Point2f& pixel, const Scene &scene, const Distribution1D *lightDistr, const Interaction &it, Sampler &sampler) {
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
        // Find intersection and check whether there is a light source
        SurfaceInteraction lightIsect;
        Ray ray = it.SpawnRay(wi);
        if (!scene.Intersect(ray, &lightIsect)) return Spectrum(0.f);
        const Light* light = lightIsect.primitive->GetAreaLight();
        if (light == nullptr) return Spectrum(0.f);

        // Compute the contribution and MIS weights
        Spectrum Li = lightIsect.Le(-wi);
        lightPdf = light->Pdf_Li(it, wi);
        Float weight = MisWeight(scene, pixel, light, lightDistr,
                                 SAMPLE_BSDF, scatteringPdf, lightPdf);
        Spectrum estimate = f * Li / scatteringPdf;
        LogContrib(pixel, estimate, weight, SAMPLE_BSDF);
        return weight * estimate;
    }

    return Spectrum(0.f);
}

Float GuidedDirectIllum::MisWeight(const Scene &scene, const Point2f& pixel, const Light* light, const Distribution1D *lightDistr,
    SamplingTech tech, Float pdfBsdf, Float pdfLight) {
    if (light == nullptr) return 0.; // needed for optimal mis

    // compute light selection probabilities
    Float uniformSelPdf = 1 / Float(scene.lights.size());
    Float guidedSelPdf = lightDistr->DiscretePDF(lightToIdx[light]);

    // compute effective sampling densities
    Float effDensUni = pdfLight * uniformSelPdf;
    Float effDensGuided = pdfLight * guidedSelPdf;
    Float effDensBsdf = pdfBsdf;

    // if power: square
    if (misMode == MIS_POWER) {
        effDensUni *= effDensUni;
        effDensGuided *= effDensGuided;
        effDensBsdf *= effDensBsdf;
    }

    // if uniform: set all to one
    if (misMode == MIS_UNIFORM) {
        effDensUni = 1;
        effDensGuided = 1;
        effDensBsdf = 1;
    }

    if (!enableUniform) effDensUni = 0;
    if (!enableGuided) effDensGuided = 0;
    if (!enableBsdfSamples) effDensBsdf = 0;

    // if our: multiply by relative moments
    if (ourMode != OUR_DISABLED && currentIteration > 0) {
        Point2i pixelInt(pixel.x, pixel.y);
        effDensUni    *= rectifier->Get(pixelInt, 3, SAMPLE_UNIFORM + 1);
        effDensGuided *= rectifier->Get(pixelInt, 3, SAMPLE_GUIDED  + 1);
        effDensBsdf   *= rectifier->Get(pixelInt, 3, SAMPLE_BSDF    + 1);
    }

    Float sum = effDensUni + effDensGuided + effDensBsdf;

    if (tech == SAMPLE_UNIFORM) {
        return effDensUni / sum;
    } else if (tech == SAMPLE_GUIDED) {
        return effDensGuided / sum;
    } else if (tech == SAMPLE_BSDF) {
        return effDensBsdf / sum;
    } else return 0.0f;
}

void GuidedDirectIllum::LogContrib(const Point2f& pixel, const Spectrum& value, Float misWeight, SamplingTech tech) {
    // log the contribution if this is the first iteration using our weights
    if (ourMode != OUR_DISABLED && currentIteration == 0)
        rectifier->AddEstimate(pixel, 3, tech + 1, value, misWeight * value); // TODO refactor in SAMISRectifier: get rid of this + 1
}

GuidedDirectIllum *CreateGuidedDiIntegrator(const ParamSet &params, std::shared_ptr<Sampler> sampler,
                                            std::shared_ptr<const Camera> camera)
{
    OurMode ourMode;
    std::string str = params.FindOneString("varmode", "moment");
    if (str == "moment") {
        ourMode = OUR_MOMENT;
    } else if (str == "variance") {
        ourMode = OUR_VARIANCE;
    } else if (str == "disabled") {
        ourMode = OUR_DISABLED;
    } else {
        ourMode = OUR_MOMENT;
        Warning("Unknown \"varmode\" specified, defaulting to \"moment\"");
    }

    MisMode misMode;
    str = params.FindOneString("mis", "balance");
    if (str == "balance") {
        misMode = MIS_BALANCE;
    } else if (str == "power") {
        misMode = MIS_POWER;
    } else if (str == "uniform") {
        misMode = MIS_UNIFORM;
    } else {
        misMode = MIS_BALANCE;
        Warning("Unknown \"varmode\" specified, defaulting to \"balance\"");
    }

    bool enableBsdfSamples = params.FindOneBool("enablebsdf", true);
    bool enableGuided = params.FindOneBool("enableguided", true);
    bool enableUniform = params.FindOneBool("enableuniform", true);
    bool visWeights = params.FindOneBool("visualizefactors", false);

    return new GuidedDirectIllum(sampler, camera, ourMode, misMode, enableBsdfSamples,
                                 enableGuided, enableUniform, visWeights);
}


} // namespace pbrt