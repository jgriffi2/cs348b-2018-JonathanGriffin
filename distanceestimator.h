#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_DISTANCE_H
#define PBRT_SHAPES_DISTANCE_H

// shapes/distanceestimator.h*
#include "distanceestimator.h"
#include "sphere.h"

namespace pbrt {

// DistanceEstimator Declarations
class DistanceEstimator : public Shape {
  public:
    // DistanceEstimator Public Methods
    DistanceEstimator(const Transform *ObjectToWorld, const Transform *WorldToObject,
           bool reverseOrientation, Float radius,
           int maxIters, Float hitEpsilon, Float rayEpsilonMultiplier, Float normalEpsilon)
        : Shape(ObjectToWorld, WorldToObject, reverseOrientation),
          radius(radius),
          maxIters(maxIters),
          hitEpsilon(hitEpsilon),
          rayEpsilonMultiplier(rayEpsilonMultiplier),
          normalEpsilon(normalEpsilon) {}

    bool Intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect,
                   bool testAlphaTexture) const;
    bool IntersectP(const Ray &ray, bool testAlphaTexture) const;
    Interaction Sample(const Point2f &u, Float *pdf) const;
    Interaction Sample(const Interaction &ref, const Point2f &u,
                       Float *pdf) const;
    Float Pdf(const Interaction &ref, const Vector3f &wi) const;
    Float SolidAngle(const Point3f &p, int nSamples) const;
    Vector3f CalculateNormal(const Point3f& pos, float eps,
                             const Vector3f& defaultNormal) const;

    virtual Float Evaluate(const Point3f& p) const = 0;

  private:
    // DistanceEstimator Private Data
    const Float radius;
    const int maxIters;
    const Float hitEpsilon;
    const Float rayEpsilonMultiplier;
    const Float normalEpsilon;
};

}  // namespace pbrt

#endif  // PBRT_SHAPES_DISTANCE_H
