#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_SPHEREDE_H
#define PBRT_SHAPES_SPHEREDE_H

// shapes/spherede.h*
#include "distanceestimator.h"
#include "sphere.h"
#include "spherede.h"

namespace pbrt {

// SphereDE Declarations
class SphereDE : public DistanceEstimator {
  public:
    // SphereDE Public Methods
    SphereDE(const Transform *ObjectToWorld, const Transform *WorldToObject,
             bool reverseOrientation, Float radius,
             int maxIters, Float hitEpsilon, Float rayEpsilonMultiplier, Float normalEpsilon)
        : DistanceEstimator(ObjectToWorld, WorldToObject, reverseOrientation, radius,
                            maxIters, hitEpsilon, rayEpsilonMultiplier, normalEpsilon),
          radius(radius) {}

    Bounds3f ObjectBound() const;
    Float Area() const;
    Float Evaluate(const Point3f& p) const;

  private:
    // SphereDE Private Data
    const Float radius;
};

std::shared_ptr<Shape> CreateSphereDEShape(const Transform *o2w, const Transform *w2o,
                                           bool reverseOrientation, const ParamSet &params);

}  // namespace pbrt

#endif  // PBRT_SHAPES_SPHEREDE_H
