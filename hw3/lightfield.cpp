
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


// cameras/lightfield.cpp*
#include "cameras/lightfield.h"
#include "paramset.h"
#include "sampler.h"
#include "sampling.h"
#include "light.h"
#include "stats.h"

namespace pbrt {

// LightfieldCamera Method Definitions
LightfieldCamera::LightfieldCamera(const AnimatedTransform &CameraToWorld,
                                     const Bounds2f &screenWindow,
                                     Float shutterOpen, Float shutterClose,
                                     Float lensRadius, Float focalDistance,
                                     Float fov, Film *film,
                                     const Medium *medium,
                                     Vector2i cpd, Bounds2f cgb)
    : ProjectiveCamera(CameraToWorld, Perspective(fov, 1e-2f, 1000.f),
                       screenWindow, shutterOpen, shutterClose, lensRadius,
                       focalDistance, film, medium) {

    // Initialize camerasperdim and cameragridbounds
    camerasperdim = cpd;
    cameragridbounds = cgb;

    // Set Transforms
    res = Point2f(film->fullResolution.x / camerasperdim.x, film->fullResolution.y / camerasperdim.y);
    ScreenToRaster =
        Scale(res.x, res.y, 1) *
        Scale(1 / (screenWindow.pMax.x - screenWindow.pMin.x),
              1 / (screenWindow.pMin.y - screenWindow.pMax.y), 1) *
        Translate(Vector3f(-screenWindow.pMin.x, -screenWindow.pMax.y, 0));
    RasterToScreen = Inverse(ScreenToRaster);
    RasterToCamera = Inverse(CameraToScreen) * RasterToScreen;

    // Compute differential changes in origin for perspective camera rays
    dxCamera =
        (RasterToCamera(Point3f(1, 0, 0)) - RasterToCamera(Point3f(0, 0, 0)));
    dyCamera =
        (RasterToCamera(Point3f(0, 1, 0)) - RasterToCamera(Point3f(0, 0, 0)));

    // Compute image plane bounds at $z=1$ for _LightfieldCamera_
    Point3f pMin = RasterToCamera(Point3f(0, 0, 0));
    Point3f pMax = RasterToCamera(Point3f(res.x, res.y, 0));
    pMin /= pMin.z;
    pMax /= pMax.z;
    A = std::abs((pMax.x - pMin.x) * (pMax.y - pMin.y));
}

Float LightfieldCamera::GenerateRay(const CameraSample &sample,
                                    Ray *ray) const {
    ProfilePhase prof(Prof::GenerateCameraRay);
    // Compute raster and camera sample positions
    Float pFilmX = fmod(sample.pFilm.x, res.x);
    Float pFilmY = fmod(sample.pFilm.y, res.y);
    Point3f pFilm = Point3f(pFilmX, pFilmY, 0);
    Point3f pCamera = RasterToCamera(pFilm);
    *ray = Ray(Point3f(0, 0, 0), Normalize(Vector3f(pCamera)));
    // Modify ray for depth of field
    if (lensRadius > 0) {
        // Sample point on lens
        Point2f pLens = lensRadius * ConcentricSampleDisk(sample.pLens);

        // Compute point on plane of focus
        Float ft = focalDistance / ray->d.z;
        Point3f pFocus = (*ray)(ft);

        // Update ray for effect of lens
        ray->o = Point3f(pLens.x, pLens.y, 0);
        ray->d = Normalize(pFocus - ray->o);
    }
    ray->time = Lerp(sample.time, shutterOpen, shutterClose);
    ray->medium = medium;
    *ray = CameraToWorld(*ray);
    return 1;
}

Float LightfieldCamera::GenerateRayDifferential(const CameraSample &sample,
                                                RayDifferential *rd) const {
    Float wt = GenerateRay(sample, rd);
    if (wt == 0) return 0;
    rd->hasDifferentials = false;
    return wt;
}

LightfieldCamera *CreateLightfieldCamera(const ParamSet &params,
                                         const AnimatedTransform &cam2world,
                                         Film *film, const Medium *medium) {
    // Extract common camera parameters from _ParamSet_
    int cpdi;
    const int *cpd = params.FindInt("camerasperdim", &cpdi);
    Vector2i camerasperdim = Vector2i(1, 1);
    if (cpd && cpdi == 2) {
        camerasperdim.x = cpd[0];
        camerasperdim.y = cpd[1];
    } else if (cpd) {
        Error("%d values supplied for \"camerasperdim\". Expected 2.", cpdi);
    }

    int cgbi;
    const Float *cgb = params.FindFloat("cameragridbounds", &cgbi);
    Bounds2f cameragridbounds = Bounds2f(Point2f(-1.0, 1.0), Point2f(-1.0, 1.0));
    if (cgb && cgbi == 4) {
        cameragridbounds.pMin.x = cgb[0];
        cameragridbounds.pMax.x = cgb[1];
        cameragridbounds.pMin.y = cgb[2];
        cameragridbounds.pMax.y = cgb[3];
    } else if (cgb) {
        Error("%d values supplied for \"cameragridbounds\". Expected 4.", cgbi);
    }

    Float shutteropen = params.FindOneFloat("shutteropen", 0.f);
    Float shutterclose = params.FindOneFloat("shutterclose", 1.f);
    if (shutterclose < shutteropen) {
        Warning("Shutter close time [%f] < shutter open [%f].  Swapping them.",
                shutterclose, shutteropen);
        std::swap(shutterclose, shutteropen);
    }
    Float lensradius = params.FindOneFloat("lensradius", 0.f);
    Float focaldistance = params.FindOneFloat("focaldistance", 1e6);
    Float frame = params.FindOneFloat(
        "frameaspectratio",
        Float(film->fullResolution.x) / Float(film->fullResolution.y));
    Bounds2f screen;
    if (frame > 1.f) {
        screen.pMin.x = -frame;
        screen.pMax.x = frame;
        screen.pMin.y = -1.f;
        screen.pMax.y = 1.f;
    } else {
        screen.pMin.x = -1.f;
        screen.pMax.x = 1.f;
        screen.pMin.y = -1.f / frame;
        screen.pMax.y = 1.f / frame;
    }
    int swi;
    const Float *sw = params.FindFloat("screenwindow", &swi);
    if (sw) {
        if (swi == 4) {
            screen.pMin.x = sw[0];
            screen.pMax.x = sw[1];
            screen.pMin.y = sw[2];
            screen.pMax.y = sw[3];
        } else
            Error("\"screenwindow\" should have four values");
    }
    Float fov = params.FindOneFloat("fov", 90.);
    Float halffov = params.FindOneFloat("halffov", -1.f);
    if (halffov > 0.f)
        // hack for structure synth, which exports half of the full fov
        fov = 2.f * halffov;

    return new LightfieldCamera(cam2world, screen, shutteropen, shutterclose,
                                lensradius, focaldistance, fov, film, medium,
                                camerasperdim, cameragridbounds);
}

}  // namespace pbrt
