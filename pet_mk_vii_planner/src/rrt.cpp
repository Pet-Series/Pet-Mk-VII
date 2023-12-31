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

        const auto result =
            tryConnect(parentNode.state, sampledState, context.collisionMap);
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
           const CollisionMap &map)
{
    /// TODO: Perform collision check against map.
    const auto delta = ugl::lie::ominus(desiredEnd, start);
    const auto controlDuration = 1.0;
    const auto velocity = delta / controlDuration;

    /// TODO: Verify control input against vehicle constraints.
    ControlInput controlInput{};
    controlInput.duration = controlDuration;
    controlInput.angular_velocity = velocity[2];
    controlInput.linear_velocity = velocity[3];

    /// TODO: Calculate where we actually arrive with given control inputs.
    const auto endState = desiredEnd;

    return std::pair{controlInput, endState};
}

} // namespace pet::rrt
