#include "pet_mk_vii_planner/rrt.hpp"

#include "pet_mk_vii_planner/bezier.hpp"
#include "pet_mk_vii_planner/rrtDefinitions.hpp"
#include "utility/interpolation.hpp"

#include <ugl/lie_group/pose.h>
#include <ugl/math/vector.h>
#include <ugl/random/uniform_distribution.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>

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

std::optional<std::vector<Node>> search(const Goal &goal, Graph &tree,
                                        const SearchContext &context)
{
    for (int i = 0; i < context.maxIterations; ++i)
    {
        const auto sampledState =
            sampleState(goal, context.vehicleModel, context.searchSpace);

        const Node &parentNode = tree.findClosest(sampledState);

        /// TODO: Perform collision check against map.
        /// TODO: Implement steering function using Bezier curves.
        const auto result =
            context.steerFunction(parentNode.state, sampledState, context.vehicleModel);
        if (result.has_value())
        {
            const auto [reachedState, pathFromParent] = result.value();
            const Node &newNode = tree.addNode(reachedState, pathFromParent, parentNode);
            /// TODO: Consider velocity when determining if goal is reached.
            if (goal.isReached(reachedState))
            {
                return tree.getPathFromRoot(newNode);
            }
        }
    }

    return {};
}

VehicleState sampleState(const Goal &goal, const VehicleModel &vehicleModel,
                         const BoundingBox &searchSpace)
{
    if (shouldSampleFromGoal())
    {
        return goal.sampleState();
    }
    else
    {
        const ugl::Vector<2> position =
            ugl::random::UniformDistribution<2>::sample(searchSpace.min, searchSpace.max);
        const double heading = ugl::random::UniformDistribution<1>::sample(0.0, 2 * M_PI);

        const ugl::UnitQuaternion quaternion{
            Eigen::AngleAxisd{heading, Eigen::Vector3d::UnitZ()}};

        VehicleState state{};
        state.pose.set_position({position.x(), position.y(), 0.0});
        state.pose.set_rotation(ugl::lie::Rotation{quaternion});
        state.velocity = ugl::random::UniformDistribution<1>::sample(
            -vehicleModel.maxSpeed, vehicleModel.maxSpeed);
        return state;
    }
}

bool shouldSampleFromGoal()
{
    static constexpr double goalProbability = 0.1;
    return ugl::random::UniformDistribution<1>::sample(0.0, 1.0) < goalProbability;
}

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

std::optional<std::pair<VehicleState, Path>> steerCtrv(const VehicleState &start,
                                                       const VehicleState &desiredEnd,
                                                       const VehicleModel &vehicleModel)
{
    const ugl::Vector<6> delta = ugl::lie::ominus(desiredEnd.pose, start.pose);

    double controlDuration = 1.0;
    double forwardVel      = delta[3] / controlDuration;
    double yawrate         = delta[2] / controlDuration;

    // If speed is too low, we will not get anywhere so adding a new node is pointless.
    if (std::abs(forwardVel) < 1e-3)
    {
        return {};
    }

    const double curvature = std::abs(yawrate / forwardVel);
    if (curvature > vehicleModel.maxCurvature)
    {
        // Adjust forward velocity such that we get maximum curvature.
        const double adjustment = curvature / vehicleModel.maxCurvature;
        forwardVel *= adjustment;
        controlDuration /= adjustment;
    }

    if (std::abs(forwardVel) > vehicleModel.maxSpeed)
    {
        // Adjust forward velocity such that we get maximum speed.
        const double adjustment = vehicleModel.maxSpeed / std::abs(forwardVel);
        forwardVel *= adjustment;
        yawrate *= adjustment;
        controlDuration /= adjustment;
    }

    ugl::Vector<6> velocity = ugl::Vector<6>::Zero();
    velocity[2]             = yawrate;
    velocity[3]             = forwardVel;

    VehicleState endState;
    endState.pose     = ugl::lie::oplus(start.pose, velocity);
    endState.velocity = velocity[3];

    /// TODO: Should start time always start from zero or be based on timestamp from
    /// previous path?
    const double startTime = 0.0;
    const double endTime   = startTime + controlDuration;
    const auto   path      = util::interpolatePath(PoseStamped{start.pose, startTime},
                                                   PoseStamped{endState.pose, endTime}, 5);

    return std::pair{endState, path};
}

} // namespace pet::rrt
