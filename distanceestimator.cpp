// shapes/distanceestimator.cpp*
#include "shapes/distanceestimator.h"
#include "shapes/sphere.h"
#include "shapes/spherede.h"
#include "shapes/infinitespheregridde.h"
#include "shapes/mandelbulbde.h"
#include "shapes/torusde.h"
#include "sampling.h"
#include "paramset.h"
#include "efloat.h"
#include "stats.h"

namespace pbrt {

// DistanceEstimator Method Definitions
bool DistanceEstimator::Intersect(const Ray &r, Float *tHit, SurfaceInteraction *isect,
                       bool testAlphaTexture) const {
    // Transform _Ray_ to object space
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);

    Point3f curPos = ray.o;
    Vector3f dir = ray.d;
    Float t = ray.time / dir.Length();
    Float tMax = ray.tMax;
    int numIters = 0;

    while (numIters < maxIters) {
        // Evaluate Distance
        Float dist = Evaluate(curPos);

        if (dist < 0.0) {
            return false;
        }

        if (dist < hitEpsilon) {
            // Set tHit
            if (tHit != NULL) {
                *tHit = t;
            }
            // Set isect
            if (isect != NULL) {
                Vector3f pError = rayEpsilonMultiplier * hitEpsilon * Vector3f(1, 1, 1);
                Point2f uv = Point2f(0, 0);
                Vector3f n = CalculateNormal(curPos, normalEpsilon, -dir);
                Normal3f dndu = Normal3f(n.x, n.y, n.z);
                Normal3f dndv = Normal3f(n.x, n.y, n.z);
                Vector3f dpdu, dpdv;
                CoordinateSystem(n, &dpdu, &dpdv);
                *isect = (*ObjectToWorld)(SurfaceInteraction(curPos, pError, uv,
                                                             -dir, dpdu, dpdv, dndu, dndv,
                                                             t, this));
            }
            return true;
        }

        // Determine location of new point
        Vector3f dist_dir = dist * dir;
        Point3f newPos = Point3f(curPos.x + dist_dir.x, curPos.y + dist_dir.y, curPos.z + dist_dir.z);

        // Update time
        t += dist / dir.Length();

        // Check if t is passed tMax
        if (t > tMax) {
            return false;
        }

        curPos = newPos;
        numIters++;
    }

    return false;
}

bool DistanceEstimator::IntersectP(const Ray &r, bool testAlphaTexture) const {
    return Intersect(r, NULL, NULL, testAlphaTexture);
}

Interaction DistanceEstimator::Sample(const Point2f &u, Float *pdf) const {
    LOG(FATAL) << "DistanceEstimator::Sample not implemented.";
    return Interaction();
}

Interaction DistanceEstimator::Sample(const Interaction &ref, const Point2f &u,
                           Float *pdf) const {
    LOG(FATAL) << "DistanceEstimator::Sample not implemented.";
    return Interaction();
}

Float DistanceEstimator::Pdf(const Interaction &ref, const Vector3f &wi) const {
    Point3f pCenter = (*ObjectToWorld)(Point3f(0, 0, 0));
    // Return uniform PDF if point is inside sphere
    Point3f pOrigin =
        OffsetRayOrigin(ref.p, ref.pError, ref.n, pCenter - ref.p);
    if (DistanceSquared(pOrigin, pCenter) <= radius * radius)
        return Shape::Pdf(ref, wi);

    // Compute general sphere PDF
    Float divideVal = DistanceSquared(ref.p, pCenter);
    Float sinThetaMax2 = (divideVal == 0) ? radius * radius : radius * radius / divideVal;
    Float cosThetaMax = std::sqrt(std::max((Float)0, 1 - sinThetaMax2));
    return UniformConePdf(cosThetaMax);
}

Float DistanceEstimator::SolidAngle(const Point3f &p, int nSamples) const {
    Point3f pCenter = (*ObjectToWorld)(Point3f(0, 0, 0));
    if (DistanceSquared(p, pCenter) <= radius * radius)
        return 4 * Pi;
    Float divideVal = DistanceSquared(p, pCenter);
    Float sinTheta2 = (divideVal == 0) ? radius * radius : radius * radius / divideVal;;
    Float cosTheta = std::sqrt(std::max((Float)0, 1 - sinTheta2));
    return (2 * Pi * (1 - cosTheta));
}

Vector3f DistanceEstimator::CalculateNormal(const Point3f& pos, float eps,
       const Vector3f& defaultNormal) const {
    const Vector3f v1 = Vector3f( 1.0,-1.0,-1.0);
    const Vector3f v2 = Vector3f(-1.0,-1.0, 1.0);
    const Vector3f v3 = Vector3f(-1.0, 1.0,-1.0);
    const Vector3f v4 = Vector3f( 1.0, 1.0, 1.0);

    const Vector3f normal = v1 * Evaluate( pos + v1*eps ) +
                 v2 * Evaluate( pos + v2*eps ) +
                 v3 * Evaluate( pos + v3*eps ) +
                 v4 * Evaluate( pos + v4*eps );
    const Float length = normal.Length();

    return length > 0 ? (normal/length) : defaultNormal;
}

}  // namespace pbrt
