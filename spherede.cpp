// shapes/spherede.cpp*
#include "shapes/spherede.h"
#include "shapes/sphere.h"
#include "sampling.h"
#include "paramset.h"
#include "efloat.h"
#include "stats.h"

namespace pbrt {

// SphereDE Method Definitions
Bounds3f SphereDE::ObjectBound() const {
    return Bounds3f(Point3f(-radius, -radius, -radius),
                    Point3f(radius, radius, radius));
}

Float SphereDE::Area() const { return Radians(360.f) * radius * (radius - -radius); }

Float SphereDE::Evaluate(const Point3f& p) const { return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z) - radius; }

std::shared_ptr<Shape> CreateSphereDEShape(const Transform *o2w,
                                         const Transform *w2o,
                                         bool reverseOrientation,
                                         const ParamSet &params) {
    Float radius = params.FindOneFloat("radius", 1.f);
    int iters = params.FindOneInt("maxiters", 1000);
    Float eps = params.FindOneFloat("hitEpsilon", 1e-2);
    Float rayMult = params.FindOneFloat("rayEpsilonMultiplier", 10.f);
    Float normEps = params.FindOneFloat("normalEpsilon", 1e-2);
    return std::make_shared<SphereDE>(o2w, w2o, reverseOrientation, radius, iters, eps, rayMult, normEps);
}

}  // namespace pbrt
