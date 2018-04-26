// shapes/spherede.cpp*
#include "shapes/torusde.h"
#include "shapes/sphere.h"
#include "sampling.h"
#include "paramset.h"
#include "efloat.h"
#include "stats.h"

namespace pbrt {

// MandelbulbDEDE Method Definitions
Bounds3f TorusDE::ObjectBound() const {
    return Bounds3f(Point3f(-(R+2*r), -(R+2*r), -(R+2*r)),
                    Point3f(R+2*r, R+2*r, R+2*r));
}

Float TorusDE::Area() const { return 2.f*Radians(360.f)*R*2.f*Radians(360.f)*r; }

Float TorusDE::Evaluate(const Point3f& p) const {
    Float c = std::cos(Radians(degree) * p.y);
    Float s = std::sin(Radians(degree) * p.y);
    Float newpx = c*p.x - s*p.z;
    Float newpy = s*p.x + c*p.z;
    Point3f newp = Point3f(newpx, newpy, p.y);
    Float xzLength = std::sqrt(newp.x*newp.x + newp.z*newp.z);
    Vector2f q = Vector2f(xzLength - R, newp.y);
    return std::sqrt(q.x*q.x + q.y*q.y) - r;
}

std::shared_ptr<Shape> CreateTorusDEShape(const Transform *o2w,
                                         const Transform *w2o,
                                         bool reverseOrientation,
                                         const ParamSet &params) {
    Float r = params.FindOneFloat("tubeRadius", 1.f);
    Float R = params.FindOneFloat("centerRadius", 2.f);
    Float d = params.FindOneFloat("degree", 0.f);
    int iters = params.FindOneInt("maxiters", 1000);
    Float eps = params.FindOneFloat("hitEpsilon", 1e-5);
    Float rayMult = params.FindOneFloat("rayEpsilonMultiplier", 10.f);
    Float normEps = params.FindOneFloat("normalEpsilon", 1e-5);
    return std::make_shared<TorusDE>(o2w, w2o, reverseOrientation, r, R, d, iters, eps, rayMult, normEps);
}

}  // namespace pbrt
