#include "pet_mk_vii_planner/steerBezier.hpp"

#include "pet_mk_vii_planner/bezier.hpp"
#include "pet_mk_vii_planner/rrtDefinitions.hpp"
#include "utility/interpolation.hpp"

#include <ugl/math/vector.h>

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
        const double curvature = curve.planarCurvature(t);
        if (curvature > maxCurvature)
        {
            maxCurvature = curvature;
        }
    }
    return maxCurvature;
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

    const auto endState = VehicleState{bezier.planarPose(bezier.duration()),
                                       bezier.velocity(bezier.duration()).norm()};

    /// TODO: Should start time always start from zero or be based on timestamp from
    /// previous path?
    const double startTime = 0.0;
    const double endTime   = startTime + bezier.duration();
    const auto   path      = util::interpolatePath(
        PoseStamped{bezier.planarPose(0), startTime},
        PoseStamped{bezier.planarPose(bezier.duration()), endTime}, 5);

    return std::pair{endState, path};
}

} // namespace pet::rrt
