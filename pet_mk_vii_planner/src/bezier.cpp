#include "pet_mk_vii_planner/bezier.hpp"

#include <ugl/math/vector.h>

namespace pet::rrt
{

Bezier<3> buildCubicBezier(double duration, const ugl::Vector3 &pos0,
                           const ugl::Vector3 &vel0, const ugl::Vector3 &pos1,
                           const ugl::Vector3 &vel1)
{
    static constexpr int kDegree = 3;

    const ugl::Vector3 p0 = pos0;
    const ugl::Vector3 p1 = duration / kDegree * vel0 + p0;

    const ugl::Vector3 p3 = pos1;
    const ugl::Vector3 p2 = -duration / kDegree * vel1 + p3;

    return Bezier<3>{duration, {p0, p1, p2, p3}};
}

Bezier<5> createPenticBezier(double duration, const ugl::Vector3 &pos0,
                             const ugl::Vector3 &vel0, const ugl::Vector3 &acc0,
                             const ugl::Vector3 &pos1, const ugl::Vector3 &vel1,
                             const ugl::Vector3 &acc1)
{
    static constexpr int kDegree = 5;

    /// TODO: Re-derive magic numbers 5 and 2.
    const ugl::Vector3 p0 = pos0;
    const ugl::Vector3 p1 = duration / kDegree * vel0 + p0;
    const ugl::Vector3 p2 = duration * duration / 20 * acc0 + 2 * p1 - p0;

    const ugl::Vector3 p5 = pos1;
    const ugl::Vector3 p4 = -duration / kDegree * vel1 + p5;
    const ugl::Vector3 p3 = duration * duration / 20 * acc1 + 2 * p4 - p5;

    return Bezier<5>{duration, {p0, p1, p2, p3, p4, p5}};
}

} // namespace pet::rrt
