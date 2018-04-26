// shapes/spherede.cpp*
#include "shapes/infinitespheregridde.h"
#include "shapes/sphere.h"
#include "sampling.h"
#include "paramset.h"
#include "efloat.h"
#include "stats.h"

namespace pbrt {

// InfiniteSphereGridDE Method Definitions
Bounds3f InfiniteSphereGridDE::ObjectBound() const {
    return Bounds3f(Point3f(-boundSize, -boundSize, -boundSize),
                    Point3f(boundSize, boundSize, boundSize));
}

Float InfiniteSphereGridDE::Area() const { return std::numeric_limits<float>::infinity(); }

Float InfiniteSphereGridDE::Evaluate(const Point3f& p) const {
    Float x = std::remainder(p.x, cellSize);
    Float y = std::remainder(p.y, cellSize);
    Float z = std::remainder(p.z, cellSize);

    return std::sqrt(x*x + y*y + z*z) - 1.f;
}

std::shared_ptr<Shape> CreateInfiniteSphereGridDEShape(const Transform *o2w,
                                         const Transform *w2o,
                                         bool reverseOrientation,
                                         const ParamSet &params) {
    Float sizeOfCell = params.FindOneFloat("cellSize", 1.f);
    int iters = params.FindOneInt("maxiters", 1000);
    Float eps = params.FindOneFloat("hitEpsilon", 1e-2);
    Float rayMult = params.FindOneFloat("rayEpsilonMultiplier", 10.f);
    Float normEps = params.FindOneFloat("normalEpsilon", 1e-2);
    return std::make_shared<InfiniteSphereGridDE>(o2w, w2o, reverseOrientation, sizeOfCell, iters, eps, rayMult, normEps);
}

}  // namespace pbrt
