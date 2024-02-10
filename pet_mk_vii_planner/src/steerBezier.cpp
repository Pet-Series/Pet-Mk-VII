#include "pet_mk_vii_planner/steerBezier.hpp"

#include "pet_mk_vii_planner/bezier.hpp"
#include "pet_mk_vii_planner/rrtDefinitions.hpp"
#include "utility/interpolation.hpp"

#include <ugl/math/vector.h>

#include <cmath>
#include <limits>

namespace pet::rrt
{
namespace
{

template <int degree> double maxSpeed(const Bezier<degree> &curve)
{
    static constexpr double kTimeStepSize = 0.05;

    const auto velocityCurve = curve.getDerivative();
    double     maxSpeed      = -std::numeric_limits<double>::infinity();
    for (double t = 0.0; t <= velocityCurve.duration(); t += kTimeStepSize)
    {
        const double speed = velocityCurve.value(t).norm();
        if (speed > maxSpeed)
        {
            maxSpeed = speed;
        }
    }
    return maxSpeed;
}

template <int degree> double maxCurvature(const Bezier<degree> &curve)
{
    static constexpr double kTimeStepSize = 0.05;

    double maxCurvature = -std::numeric_limits<double>::infinity();
    for (double t = 0.0; t <= curve.duration(); t += kTimeStepSize)
    {
        const double curvature = std::abs(curve.planarCurvature(t));
        if (curvature > maxCurvature)
        {
            maxCurvature = curvature;
        }
    }
    return maxCurvature;
}

template <int degree>
double getPlanarForwardVelocity(const Bezier<degree>     &bezier,
                                const ugl::lie::Rotation &orientation, double t)
{
    const ugl::Vector3 globalVel = bezier.velocity(t);
    const ugl::Vector3 localVel  = orientation.inverse() * globalVel;
    return localVel.x();
}

template <int degree>
Path computePath(const Bezier<degree> &bezier, double startTime, int numberOfPoints)
{
    assert(numberOfPoints > 1);
    std::vector<rrt::PoseStamped> path{};

    const double ratioDelta = 1.0 / (numberOfPoints - 1);
    double       ratio      = 0.0;
    for (int i = 0; i < numberOfPoints; ++i)
    {
        const auto relativeTimestamp = util::interpolate(0.0, bezier.duration(), ratio);
        const auto pose              = bezier.planarPose(relativeTimestamp);
        const auto forwardVelocity =
            getPlanarForwardVelocity(bezier, pose.rotation(), relativeTimestamp);
        path.push_back(
            rrt::PoseStamped{pose, forwardVelocity, startTime + relativeTimestamp});
        ratio += ratioDelta;
    }
    return path;
}

} // namespace

std::optional<std::pair<VehicleState, Path>> steerBezier(const VehicleState &start,
                                                         const VehicleState &desiredEnd,
                                                         const VehicleModel &vehicleModel)
{
    const double       duration = 1.0;
    const ugl::Vector3 startVelocity =
        start.pose.rotate(ugl::Vector3{start.velocity, 0.0, 0.0});
    const ugl::Vector3 endVelocity =
        desiredEnd.pose.rotate(ugl::Vector3{desiredEnd.velocity, 0.0, 0.0});

    const CubicBezier bezier =
        buildCubicBezier(duration, start.pose.position(), startVelocity,
                         desiredEnd.pose.position(), endVelocity);

    /// TODO: Fall back on 'steerCtrv()' if kinematic constrainst are not fulfilled fails?
    /// TODO: Collision check against map. What to do if fail?
    if (maxSpeed(bezier) > vehicleModel.maxSpeed)
    {
        return {};
    }
    if (maxCurvature(bezier) > vehicleModel.maxCurvature)
    {
        return {};
    }

    // We do exact matching, so we can re-use the desired end state.
    const auto endState = desiredEnd;

    /// TODO: Should start time always start from zero or be based on timestamp from
    /// previous path?
    const double startTime = 0.0;
    const auto   path      = computePath(bezier, startTime, 20);

    return std::pair{endState, path};
}

} // namespace pet::rrt
