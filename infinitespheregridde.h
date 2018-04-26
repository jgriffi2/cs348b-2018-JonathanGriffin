#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_INFINITESPHEREGRIDDE_H
#define PBRT_SHAPES_INFINITESPHEREGRIDDE_H

// shapes/spherede.h*
#include "distanceestimator.h"
#include "sphere.h"
#include "infinitespheregridde.h"

namespace pbrt {

// InfiniteSphereGridDE Declarations
class InfiniteSphereGridDE : public DistanceEstimator {
  public:
    // InfiniteSphereGridDE Public Methods
    InfiniteSphereGridDE(const Transform *ObjectToWorld, const Transform *WorldToObject,
                         bool reverseOrientation, Float cellSize,
                         int maxIters, Float hitEpsilon, Float rayEpsilonMultiplier, Float normalEpsilon)
        : DistanceEstimator(ObjectToWorld, WorldToObject, reverseOrientation, 1.f,
                            maxIters, hitEpsilon, rayEpsilonMultiplier, normalEpsilon),
          cellSize(cellSize) {}

    Bounds3f ObjectBound() const;
    Float Area() const;
    Float Evaluate(const Point3f& p) const;

  private:
    // InfiniteSphereGridDE Private Data
    const Float cellSize;
    const Float boundSize = 10000.f;
};

std::shared_ptr<Shape> CreateInfiniteSphereGridDEShape(const Transform *o2w, const Transform *w2o,
                                           bool reverseOrientation, const ParamSet &params);

}  // namespace pbrt

#endif  // PBRT_SHAPES_INFINITESPHEREGRIDDE_H
