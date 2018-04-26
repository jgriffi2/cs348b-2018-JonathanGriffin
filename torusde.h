#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_TORUSDE_H
#define PBRT_SHAPES_TORUSDE_H

// shapes/spherede.h*
#include "distanceestimator.h"
#include "sphere.h"
#include "torusde.h"

namespace pbrt {

// MandelbulbDEDE Declarations
class TorusDE : public DistanceEstimator {
  public:
    // MandelbulbDEDE Public Methods
    TorusDE(const Transform *ObjectToWorld, const Transform *WorldToObject,
            bool reverseOrientation, Float r, Float R, Float degree,
            int maxIters, Float hitEpsilon, Float rayEpsilonMultiplier, Float normalEpsilon)
        : DistanceEstimator(ObjectToWorld, WorldToObject, reverseOrientation, 1.f,
                            maxIters, hitEpsilon, rayEpsilonMultiplier, normalEpsilon),
          r(r),
          R(R),
          degree(degree) {}

    Bounds3f ObjectBound() const;
    Float Area() const;
    Float Evaluate(const Point3f& p) const;

  private:
    // MandelbulbDEDE Private Data
    const Float r;
    const Float R;
    const Float degree;
    // const Float boundSize = 10000.f;
};

std::shared_ptr<Shape> CreateTorusDEShape(const Transform *o2w, const Transform *w2o,
                                           bool reverseOrientation, const ParamSet &params);

}  // namespace pbrt

#endif  // PBRT_SHAPES_TORUSDE_H
