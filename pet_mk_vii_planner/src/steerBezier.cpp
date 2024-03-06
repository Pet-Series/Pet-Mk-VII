#include "pet_mk_vii_planner/steerBezier.hpp"

#include "pet_mk_vii_planner/bezier.hpp"
#include "pet_mk_vii_planner/rrtDefinitions.hpp"
#include "utility/interpolation.hpp"
#include "utility/tiktok.hpp"

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

template <int degree>
bool isCurvatureHigherThan(const Bezier<degree> &curve, double maxCurvature)
{
    // Start and end often have the highest curvature.
    const double startCurvature = std::abs(curve.planarCurvature(0.0));
    if (startCurvature > maxCurvature)
    {
        return true;
    }
    const double endCurvature = std::abs(curve.planarCurvature(curve.duration()));
    if (endCurvature > maxCurvature)
    {
        return true;
    }

    static constexpr double kTimeStepSize = 0.05;
    for (double t = kTimeStepSize; t < curve.duration(); t += kTimeStepSize)
    {
        const double curvature = std::abs(curve.planarCurvature(t));
        if (curvature > maxCurvature)
        {
            return true;
        }
    }
    return false;
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
    std::vector<rrt::VehicleState> path{};

    const double ratioDelta = 1.0 / (numberOfPoints - 1);
    double       ratio      = 0.0;
    for (int i = 0; i < numberOfPoints; ++i)
    {
        const auto relativeTimestamp = util::interpolate(0.0, bezier.duration(), ratio);
        const auto pose              = bezier.planarPose(relativeTimestamp);
        const auto forwardVelocity =
            getPlanarForwardVelocity(bezier, pose.rotation(), relativeTimestamp);
        path.push_back(
            rrt::VehicleState{pose, forwardVelocity, startTime + relativeTimestamp});
        ratio += ratioDelta;
    }
    return path;
}

template <int degree>
Path computePath(const Bezier<degree> &bezier, const VehicleState &startState,
                 const VehicleState &endState, int numberOfPoints)
{
    assert(numberOfPoints > 1);
    std::vector<rrt::VehicleState> path{};

    const double ratioDelta = 1.0 / (numberOfPoints - 1);
    double       ratio      = 0.0;
    for (int i = 0; i < numberOfPoints; ++i)
    {
        const auto relativeTimestamp = util::interpolate(0.0, bezier.duration(), ratio);
        const auto pose              = bezier.planarPose(relativeTimestamp);
        const auto velocity =
            util::interpolate(startState.velocity, endState.velocity, ratio);
        path.push_back(
            rrt::VehicleState{pose, velocity, startState.timestamp + relativeTimestamp});
        ratio += ratioDelta;
    }
    return path;
}

} // namespace

std::optional<std::pair<VehicleState, Path>>
steerBezierPath(const VehicleState &start, const VehicleState &desiredEnd,
                const VehicleModel &vehicleModel)
{
    util::TikTok timer{"rrt::steerBezierPath"};
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

    std::optional<CubicBezier> bezier{};
    for (const double scaleFactor : {0.2, 0.5, 1.0, 2.0})
    {
        const double tangentFactor = scaleFactor * minRadius;

        const ugl::Vector3 p0 = start.pose.position();
        const ugl::Vector3 p1 = start.pose.position() + startTangent * tangentFactor;
        const ugl::Vector3 p2 = desiredEnd.pose.position() - endTangent * tangentFactor;
        const ugl::Vector3 p3 = desiredEnd.pose.position();
        const CubicBezier  candidate{duration, {p0, p1, p2, p3}};

        if (!isCurvatureHigherThan(candidate, vehicleModel.maxCurvature))
        {
            bezier = candidate;
            break;
        }
    }

    if (bezier.has_value())
    {
        /// TODO: Set velocity profile and timestamps based on max acceleration/braking
        /// and boundary velocities.
        VehicleState endState{};
        endState.pose      = desiredEnd.pose;
        endState.velocity  = endVelocity;
        endState.timestamp = start.timestamp + duration;

        const auto path = computePath(bezier.value(), start, endState, 10);
        return std::pair{endState, path};
    }
    else
    {
        return {};
    }
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

    if (maxSpeed(bezier) > vehicleModel.maxSpeed)
    {
        return {};
    }
    if (isCurvatureHigherThan(bezier, vehicleModel.maxCurvature))
    {
        return {};
    }

    VehicleState endState{};
    endState.pose      = desiredEnd.pose;
    endState.velocity  = desiredEnd.velocity;
    endState.timestamp = start.timestamp + duration;

    const auto path = computePath(bezier, start.timestamp, 20);
    return std::pair{endState, path};
}

} // namespace pet::rrt
