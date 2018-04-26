#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_MANDELDE_H
#define PBRT_SHAPES_MANDELDE_H

// shapes/spherede.h*
#include "distanceestimator.h"
#include "sphere.h"
#include "mandelbulbde.h"

namespace pbrt {

// MandelbulbDEDE Declarations
class MandelbulbDE : public DistanceEstimator {
  public:
    // MandelbulbDEDE Public Methods
    MandelbulbDE(const Transform *ObjectToWorld, const Transform *WorldToObject,
                         bool reverseOrientation, int fractalIters, int mandelbulbPower,
                         int maxIters, Float hitEpsilon, Float rayEpsilonMultiplier, Float normalEpsilon)
        : DistanceEstimator(ObjectToWorld, WorldToObject, reverseOrientation, 1.f,
                            maxIters, hitEpsilon, rayEpsilonMultiplier, normalEpsilon),
          fractalIters(fractalIters),
          mandelbulbPower(mandelbulbPower) {}

    Bounds3f ObjectBound() const;
    Float Area() const;
    Float Evaluate(const Point3f& p) const;

  private:
    // MandelbulbDEDE Private Data
    const int fractalIters;
    const int mandelbulbPower;
    const Float boundSize = Radians(360.f);
};

std::shared_ptr<Shape> CreateMandelbulbDEShape(const Transform *o2w, const Transform *w2o,
                                           bool reverseOrientation, const ParamSet &params);

}  // namespace pbrt

#endif  // PBRT_SHAPES_MANDELDE_H
