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

template <typename ScalarType> constexpr int sign(ScalarType x) { return x < 0 ? -1 : 1; }

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

template <int degree>
Path computePath(const Bezier<degree> &bezier, const VehicleState &startState,
                 const VehicleState &endState, double startTime, int numberOfPoints)
{
    assert(numberOfPoints > 1);
    std::vector<rrt::PoseStamped> path{};

    const double ratioDelta = 1.0 / (numberOfPoints - 1);
    double       ratio      = 0.0;
    for (int i = 0; i < numberOfPoints; ++i)
    {
        const auto relativeTimestamp = util::interpolate(0.0, bezier.duration(), ratio);
        const auto pose              = bezier.planarPose(relativeTimestamp);
        const auto velocity =
            util::interpolate(startState.velocity, endState.velocity, ratio);
        path.push_back(rrt::PoseStamped{pose, velocity, startTime + relativeTimestamp});
        ratio += ratioDelta;
    }
    return path;
}

} // namespace

std::optional<std::pair<VehicleState, Path>>
steerBezierPath(const VehicleState &start, const VehicleState &desiredEnd,
                const VehicleModel &vehicleModel)
{
    // Bezier curves cannot handle changing from forward driving to reversing in a single
    // curve. If desired end velocity is wrong sign we set it to zero, which allows for
    // switching direction if connected to later in the search.
    int    drivingDirection;
    double endVelocity;
    if (start.velocity == 0.0)
    {
        drivingDirection = sign(desiredEnd.velocity);
        endVelocity      = desiredEnd.velocity;
    }
    else
    {
        drivingDirection = sign(start.velocity);
        endVelocity =
            (drivingDirection == sign(desiredEnd.velocity)) ? desiredEnd.velocity : 0.0;
    }

    const double       duration     = 1.0;
    const ugl::Vector3 startTangent =
        start.pose.rotate(ugl::Vector3::UnitX()) * drivingDirection;
    const ugl::Vector3 endTangent =
        desiredEnd.pose.rotate(ugl::Vector3::UnitX()) * drivingDirection;

    const double minRadius = 1.0 / vehicleModel.maxCurvature;

    const ugl::Vector3 p0 = start.pose.position();
    const ugl::Vector3 p1 = start.pose.position() + startTangent * minRadius;
    const ugl::Vector3 p2 = desiredEnd.pose.position() - endTangent * minRadius;
    const ugl::Vector3 p3 = desiredEnd.pose.position();
    const CubicBezier bezier{duration, {p0, p1, p2, p3}};

    /// TODO: Collision check against map. What to do if fail?
    if (maxCurvature(bezier) > vehicleModel.maxCurvature)
    {
        return {};
    }

    /// TODO: Set velocity profile  and timestamps based max acceleration/braking and
    /// boundary velocities.
    auto endState     = desiredEnd;
    endState.velocity = endVelocity;

    /// TODO: Should start time always start from zero or be based on timestamp from
    /// previous path?
    const double startTime = 0.0;
    const auto   path      = computePath(bezier, start, endState, startTime, 10);

    return std::pair{endState, path};
}

std::optional<std::pair<VehicleState, Path>>
steerBezierKinematic(const VehicleState &start, const VehicleState &desiredEnd,
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
