#include "pet_mk_vii_planner/rrt.hpp"

#include <ugl/random/uniform_distribution.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>

namespace pet::rrt
{

std::optional<std::vector<Node>> search(const Goal &goal, Graph &tree,
                                        const SearchContext &context)
{
    for (int i = 0; i < context.maxIterations; ++i)
    {
        const auto sampledState = sampleState(goal, context.searchSpace);

        const Node &parentNode = tree.findClosest(sampledState);

        const auto result = tryConnect(parentNode.state, sampledState, context);
        if (result.has_value())
        {
            const auto [controlInput, reachedState] = result.value();
            const Node &newNode = tree.addNode(reachedState, controlInput, parentNode);
            if (goal.isReached(reachedState))
            {
                return tree.getPathFromRoot(newNode);
            }
        }
    }

    return {};
}

ugl::lie::Pose sampleState(const Goal &goal, const BoundingBox &searchSpace)
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

        ugl::lie::Pose state{};
        state.set_position({position.x(), position.y(), 0.0});
        state.set_rotation(ugl::lie::Rotation{quaternion});

        return state;
    }
}

bool shouldSampleFromGoal()
{
    static constexpr double goalProbability = 0.1;
    return ugl::random::UniformDistribution<1>::sample(0.0, 1.0) < goalProbability;
}

std::optional<std::pair<ControlInput, ugl::lie::Pose>>
tryConnect(const ugl::lie::Pose &start, const ugl::lie::Pose &desiredEnd,
           const SearchContext &context)
{
    /// TODO: Perform collision check against map.
    const ugl::Vector<6> delta = ugl::lie::ominus(desiredEnd, start);

    double controlDuration = 1.0;
    double forwardVel = delta[3] / controlDuration;
    double yawrate = delta[2] / controlDuration;

    // If speed is too low, we will not get anywhere so adding a new node is pointless.
    if (std::abs(forwardVel) < 1e-3)
    {
        return {};
    }

    const double curvature = std::abs(yawrate / forwardVel);
    if (curvature > context.vehicleModel.maxCurvature)
    {
        // Adjust forward velocity such that we get maximum curvature.
        const double adjustment = curvature / context.vehicleModel.maxCurvature;
        forwardVel *= adjustment;
        controlDuration /= adjustment;
    }

    if (std::abs(forwardVel) > context.vehicleModel.maxSpeed)
    {
        // Adjust forward velocity such that we get maximum speed.
        const double adjustment = context.vehicleModel.maxSpeed / std::abs(forwardVel);
        forwardVel *= adjustment;
        yawrate *= adjustment;
        controlDuration /= adjustment;
    }

    ugl::Vector<6> velocity = ugl::Vector<6>::Zero();
    velocity[2] = yawrate;
    velocity[3] = forwardVel;

    const ugl::lie::Pose endState = ugl::lie::oplus(start, velocity);

    /// TODO: Verify control input against vehicle constraints.
    ControlInput controlInput{};
    controlInput.duration = controlDuration;
    controlInput.angularVelocity = velocity[2];
    controlInput.linearVelocity = velocity[3];

    return std::pair{controlInput, endState};
}

} // namespace pet::rrt
