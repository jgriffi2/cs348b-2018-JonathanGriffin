
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

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CAMERAS_LIGHTFIELD_H
#define PBRT_CAMERAS_LIGHTFIELD_H

// cameras/lightfield.h*
#include "pbrt.h"
#include "camera.h"
#include "film.h"

namespace pbrt {

// LightfieldCamera Declarations
class LightfieldCamera : public ProjectiveCamera {
  public:
    // LightfieldCamera Public Methods
    LightfieldCamera(const AnimatedTransform &CameraToWorld,
                      const Bounds2f &screenWindow, Float shutterOpen,
                      Float shutterClose, Float lensRadius, Float focalDistance,
                      Float fov, Film *film, const Medium *medium,
                      Vector2i camerasperdim, Bounds2f cameragridbounds);
    Float GenerateRay(const CameraSample &sample, Ray *) const;
    Float GenerateRayDifferential(const CameraSample &sample,
                                  RayDifferential *ray) const;

  private:
    // LightfieldCamera Private Data
    Vector3f dxCamera, dyCamera;
    Float A;
    Vector2i camerasperdim;
    Bounds2f cameragridbounds;
    Transform ScreenToRaster, RasterToScreen, RasterToCamera;
    Point2f res;
};

LightfieldCamera *CreateLightfieldCamera(const ParamSet &params,
                                         const AnimatedTransform &cam2world,
                                         Film *film, const Medium *medium);

}  // namespace pbrt

#endif  // PBRT_CAMERAS_LIGHTFIELD_H
