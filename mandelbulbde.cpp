// shapes/spherede.cpp*
#include "shapes/mandelbulbde.h"
#include "shapes/sphere.h"
#include "sampling.h"
#include "paramset.h"
#include "efloat.h"
#include "stats.h"

namespace pbrt {

// MandelbulbDEDE Method Definitions
Bounds3f MandelbulbDE::ObjectBound() const {
    return Bounds3f(Point3f(-boundSize, -boundSize, -boundSize),
                    Point3f(boundSize, boundSize, boundSize));
}

Float MandelbulbDE::Area() const { return std::numeric_limits<float>::infinity(); }

Float MandelbulbDE::Evaluate(const Point3f& p) const {
    const float bailout = 2.0f;
    const float Power = (float)mandelbulbPower;
    Point3f z = p;
    float dr = 1.0;
    float r = 0.0;
    for (int i = 0; i < fractalIters; i++) {
        r = (z-Point3f(0,0,0)).Length();
        if (r>bailout) break;

        // convert to polar coordinates
        float theta = acos(z.z/r);
        float phi = atan2(z.y,z.x);
        dr =  pow( r, Power-1.0)*Power*dr + 1.0;

        // scale and rotate the point
        float zr = pow( r,Power);
        theta = theta*Power;
        phi = phi*Power;

        // convert back to cartesian coordinates
        z = zr*Point3f(sin(theta)*cos(phi), sin(phi)*sin(theta), cos(theta));
        z += p;
    }
    return 0.5*log(r)*r/dr;
}

std::shared_ptr<Shape> CreateMandelbulbDEShape(const Transform *o2w,
                                         const Transform *w2o,
                                         bool reverseOrientation,
                                         const ParamSet &params) {
    int iters = params.FindOneInt("maxiters", 1000);
    int mandelbulbPower = params.FindOneInt("mandelbulbPower", 8);
    Float eps = params.FindOneFloat("hitEpsilon", 1e-4);
    Float rayMult = params.FindOneFloat("rayEpsilonMultiplier", 10.f);
    Float normEps = params.FindOneFloat("normalEpsilon", 1e-4);
    return std::make_shared<MandelbulbDE>(o2w, w2o, reverseOrientation, iters, mandelbulbPower, iters, eps, rayMult, normEps);
}

}  // namespace pbrt
