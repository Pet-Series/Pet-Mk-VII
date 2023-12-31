#include "pet_mk_vii_planner/rrt.hpp"

namespace pet::rrt
{

std::optional<std::vector<Node>> search(const Goal &goal, Graph &tree,
                                        const SearchContext &context)
{
    for (int i = 0; i < context.m_iterations; ++i)
    {
        /// TODO: Sample from search space or from goal space.
        const auto sampledState = goal.sampleState();

        const Node &parentNode = tree.findClosest(sampledState);

        const auto result =
            tryConnect(parentNode.m_state, sampledState, context.m_collisionMap);
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
